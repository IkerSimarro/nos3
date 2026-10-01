"""Python implementation of the hil_link serial framing (see ../protocol/hil_link.h).

Wire format: COBS(type u8 | bus u8 | seq u8 | status u8 | addr u32 LE | payload | crc16 LE) 0x00
"""
import struct
from dataclasses import dataclass, field

HEARTBEAT = 0x01
LOG = 0x02
UART_TX = 0x10
UART_RX = 0x11
UART_OPEN = 0x12
I2C_TXN = 0x20
I2C_RSP = 0x21
SPI_TXN = 0x30
SPI_RSP = 0x31
CAN_TXN = 0x40
CAN_RSP = 0x41
CI_PKT = 0x50
TO_PKT = 0x51
RADIO_RX = 0x52
RADIO_TX = 0x53

TYPE_NAMES = {v: k for k, v in globals().items() if k.isupper() and isinstance(v, int)}

STATUS_NAMES = {0: "OK", 1: "BUS_ERROR", 2: "BAD_REQ", 3: "UNKNOWN"}

HEADER = struct.Struct("<BBBBI")


@dataclass
class Frame:
    type: int
    bus: int = 0
    seq: int = 0
    status: int = 0
    addr: int = 0
    payload: bytes = field(default=b"")

    def __str__(self):
        name = TYPE_NAMES.get(self.type, f"0x{self.type:02x}")
        return (f"{name} bus={self.bus} seq={self.seq} status={STATUS_NAMES.get(self.status, self.status)} "
                f"addr=0x{self.addr:x} len={len(self.payload)}")


def crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE."""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


def cobs_encode(data: bytes) -> bytes:
    out = bytearray([0])
    code_idx = 0
    code = 1
    for b in data:
        if b == 0:
            out[code_idx] = code
            code_idx = len(out)
            out.append(0)
            code = 1
        else:
            out.append(b)
            code += 1
            if code == 0xFF:
                out[code_idx] = code
                code_idx = len(out)
                out.append(0)
                code = 1
    out[code_idx] = code
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    out = bytearray()
    i = 0
    while i < len(data):
        code = data[i]
        if code == 0 or i + code > len(data):
            raise ValueError("bad COBS data")
        out += data[i + 1:i + code]
        i += code
        if code != 0xFF and i < len(data):
            out.append(0)
    return bytes(out)


def encode(frame: Frame) -> bytes:
    raw = HEADER.pack(frame.type, frame.bus, frame.seq, frame.status, frame.addr) + frame.payload
    raw += struct.pack("<H", crc16(raw))
    return cobs_encode(raw) + b"\x00"


class Decoder:
    """Feed received bytes; yields complete, CRC-checked frames."""

    def __init__(self):
        self.buf = bytearray()
        self.errors = 0

    def feed(self, data: bytes):
        for b in data:
            if b != 0:
                self.buf.append(b)
                continue
            chunk, self.buf = bytes(self.buf), bytearray()
            if not chunk:
                continue
            try:
                raw = cobs_decode(chunk)
            except ValueError:
                self.errors += 1
                continue
            if len(raw) < HEADER.size + 2 or crc16(raw[:-2]) != struct.unpack("<H", raw[-2:])[0]:
                self.errors += 1
                continue
            t, bus, seq, status, addr = HEADER.unpack(raw[:HEADER.size])
            yield Frame(t, bus, seq, status, addr, raw[HEADER.size:-2])


def txn_payload(tx: bytes, rxlen: int) -> bytes:
    """Payload for I2C_TXN / SPI_TXN / CAN_TXN: rxlen u16 LE followed by bytes to write."""
    return struct.pack("<H", rxlen) + tx
