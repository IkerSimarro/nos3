#!/usr/bin/env python3
"""Pretend to be the MCU flight computer so the HIL bridge can be tested without hardware.

Typical use (no hardware): the fake MCU creates a pty pair and links the bridge's end to /tmp/hil_tty
    ./hil_fake_mcu.py --pty /tmp/hil_tty --test &
    nos3-hil-bridge -d /tmp/hil_tty

Or against an existing device / socat pty:
    ./hil_fake_mcu.py /dev/ttyUSB0 --listen

--test  : heartbeat round trip, then an EPS housekeeping request over i2c_1 @ 0x2B; exit 0 on success
--listen: print every frame the bridge sends (CI_PKT uplinks, UART_RX, ...) until Ctrl-C
"""
import argparse
import os
import select
import struct
import sys
import termios
import time
import tty

import hil_link

EPS_I2C_BUS = 1
EPS_I2C_ADDR = 0x2B
EPS_HK_LEN = 16 + 8 * 6  # 8 uint16 fields + 8 switches of 3 uint16 (generic_eps_device.h)

SAMPLE_UART_BUS = 16
SAMPLE_NOOP_CMD = bytes([0xDE, 0xAD, 0x00, 0, 0, 0, 0, 0xBE, 0xEF])  # components/sample/fsw/shared/sample_device.c


def eps_crc8(data: bytes) -> int:
    """Same CRC8 as components/generic_eps/fsw/shared/generic_eps_device.c"""
    crc = 0xFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x31) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


class Link:
    def __init__(self, path: str = None, pty_link: str = None):
        if pty_link:
            # We keep the master; the bridge opens the slave through the symlink. Holding the slave
            # open ourselves stops the pty from hanging up if the bridge restarts.
            self.fd, self._slave = os.openpty()
            tty.setraw(self._slave)
            if os.path.lexists(pty_link):
                os.remove(pty_link)
            os.symlink(os.ttyname(self._slave), pty_link)
            print(f"pty ready: bridge side is {pty_link} -> {os.ttyname(self._slave)}", flush=True)
        else:
            self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
        if os.isatty(self.fd):
            tty.setraw(self.fd)
            termios.tcflush(self.fd, termios.TCIOFLUSH)
        self.dec = hil_link.Decoder()
        self.seq = 0

    def send(self, frame: hil_link.Frame):
        os.write(self.fd, hil_link.encode(frame))

    def recv(self, timeout: float):
        """Yield frames until timeout expires."""
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return
            r, _, _ = select.select([self.fd], [], [], remaining)
            if r:
                yield from self.dec.feed(os.read(self.fd, 4096))

    def request(self, frame: hil_link.Frame, rsp_type: int, timeout: float = 2.0):
        """Send a request and wait for the response with the matching seq; other frames are printed."""
        self.seq = (self.seq + 1) & 0xFF
        frame.seq = self.seq
        self.send(frame)
        for rsp in self.recv(timeout):
            if rsp.type == rsp_type and rsp.seq == frame.seq:
                return rsp
            print(f"  (async) {rsp}")
        return None


def test(link: Link, wait: float, check_radio: bool, check_uart: bool) -> bool:
    ok = True

    # The bridge may still be starting; keep pinging until it answers
    rsp = None
    deadline = time.monotonic() + wait
    while rsp is None and time.monotonic() < deadline:
        rsp = link.request(hil_link.Frame(hil_link.HEARTBEAT, payload=b"ping"), hil_link.HEARTBEAT, timeout=1.0)
    if rsp and rsp.payload == b"ping":
        print("PASS heartbeat round trip")
    else:
        print(f"FAIL heartbeat: no echo from bridge within {wait:.0f} s")
        return False

    link.send(hil_link.Frame(hil_link.LOG, payload=b"fake MCU starting self-test"))

    cmd = bytes([0x70, 0x00])
    cmd += bytes([eps_crc8(cmd)])
    rsp = link.request(hil_link.Frame(hil_link.I2C_TXN, bus=EPS_I2C_BUS, addr=EPS_I2C_ADDR,
                                      payload=hil_link.txn_payload(cmd, EPS_HK_LEN + 1)),
                       hil_link.I2C_RSP, timeout=5.0)
    if rsp is None:
        print("FAIL EPS HK: no I2C_RSP")
        ok = False
    elif rsp.status != 0:
        print(f"FAIL EPS HK: bridge reported {hil_link.STATUS_NAMES.get(rsp.status, rsp.status)}")
        ok = False
    elif len(rsp.payload) != EPS_HK_LEN + 1 or eps_crc8(rsp.payload[:EPS_HK_LEN]) != rsp.payload[EPS_HK_LEN]:
        print(f"FAIL EPS HK: bad reply ({len(rsp.payload)} bytes, CRC mismatch or wrong length)")
        ok = False
    else:
        batt_v, batt_t, v33, v50, v12, eps_t, sa_v, sa_t = struct.unpack(">8H", rsp.payload[:16])
        print(f"PASS EPS HK: battery raw={batt_v} temp raw={batt_t} 3v3={v33} 5v0={v50} 12v={v12} "
              f"solar array raw={sa_v}")

    rsp = link.request(hil_link.Frame(hil_link.I2C_TXN, bus=99, addr=EPS_I2C_ADDR,
                                      payload=hil_link.txn_payload(b"\x00", 1)), hil_link.I2C_RSP)
    if rsp and rsp.status == 2:
        print("PASS invalid bus rejected with BAD_REQ")
    else:
        print(f"FAIL invalid bus: got {rsp}")
        ok = False

    if check_uart:
        # The sample sim echoes every 9-byte command back on the same USART
        link.send(hil_link.Frame(hil_link.UART_TX, bus=SAMPLE_UART_BUS, payload=SAMPLE_NOOP_CMD))
        echo = b"".join(f.payload for f in link.recv(3.0)
                        if f.type == hil_link.UART_RX and f.bus == SAMPLE_UART_BUS)
        if SAMPLE_NOOP_CMD in echo:
            print("PASS UART path: sample sim echoed NOOP on usart_16")
        else:
            print(f"FAIL UART path: received {echo.hex() or 'nothing'} on usart_16")
            ok = False

    if check_radio:
        # Expects something on the radio host that answers our telemetry with a command,
        # e.g. the stand-in radio in e2e_test.sh; the real radio sim won't do this
        link.send(hil_link.Frame(hil_link.TO_PKT, payload=b"tlm-test"))
        ci = next((f for f in link.recv(5.0) if f.type == hil_link.CI_PKT), None)
        if ci is not None and ci.payload == b"cmd-test":
            print("PASS radio path: TO_PKT reached radio host, CI_PKT came back")
        else:
            print(f"FAIL radio path: got {ci}")
            ok = False

    return ok


def listen(link: Link):
    print("listening for frames from the bridge (Ctrl-C to stop)")
    last_hb = 0.0
    try:
        while True:
            if time.monotonic() - last_hb > 1.0:
                link.send(hil_link.Frame(hil_link.HEARTBEAT))
                last_hb = time.monotonic()
            for f in link.recv(0.2):
                if f.type == hil_link.HEARTBEAT:
                    continue
                print(f"{f}  {f.payload[:32].hex()}")
    except KeyboardInterrupt:
        pass


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("device", nargs="?", help="serial device or pty connected to the bridge")
    parser.add_argument("--pty", metavar="LINK", help="create a pty pair and symlink the bridge end to LINK")
    parser.add_argument("--check-radio", action="store_true",
                        help="also test the TO/CI UDP path (needs a responder on the radio host)")
    parser.add_argument("--check-uart", action="store_true",
                        help="also test usart_16 against the sample sim")
    parser.add_argument("--wait", type=float, default=30.0, help="seconds to wait for the bridge (--test)")
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--test", action="store_true", help="run the self-test and exit")
    mode.add_argument("--listen", action="store_true", help="print frames from the bridge")
    args = parser.parse_args()
    if bool(args.device) == bool(args.pty):
        parser.error("give exactly one of DEVICE or --pty LINK")

    link = Link(args.device, args.pty)
    if args.test:
        sys.exit(0 if test(link, args.wait, args.check_radio, args.check_uart) else 1)
    listen(link)


if __name__ == "__main__":
    main()
