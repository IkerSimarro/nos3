# HIL Bridge

Hardware-in-the-loop bridge that lets an external microcontroller act as the NOS3 flight computer.

The bridge takes the place of the cFS container (`nos-fsw`) on the spacecraft network. The MCU talks to the bridge over a serial port, and the bridge acts on the MCU's behalf:
- it performs bus transactions against the NOS3 device sims over NOS Engine, using the same bus names, node name and master address as `fsw/apps/hwlib/sim`;
- it relays command and telemetry packets to and from the radio sim.

The device sims, 42, the radio sim, CryptoLib and the ground software are unchanged.

```
MCU (your FSW) ──USB serial, hil_link frames──> nos3-hil-bridge  (container: -h nos-fsw)
                                                  ├─ NOS Engine tcp://nos-engine-server:12000
                                                  │    usart_N / i2c_N / spi_N / can_N ──> device sims ──> 42
                                                  └─ UDP ──> radio-sim ──> cryptolib ──> COSMOS
                                                       CI in :5010   TO out radio-sim:5011
                                                       radio in :5015  radio out radio-sim:5014
```

## Layout

| Path | What |
|---|---|
| `protocol/hil_link.[ch]` | Serial framing shared by both ends. Portable C99 with no OS dependencies. Unit test: `test_hil_link.c`. |
| `sim/` | `nos3-hil-bridge` host process. `make sim` builds it automatically, because every `components/*/sim` is built. |
| `mcu/hil_mcu_example.c` | MCU-side example: I2C transactions, UART writes, telemetry downlink, command dispatch, EPS polling. |
| `support/hil_link.py` | Python codec for the same protocol. |
| `support/hil_fake_mcu.py` | Pretends to be the MCU, for testing without hardware. |
| `support/e2e_test.sh` | Self-contained end-to-end test: engine server, EPS and sample sims, bridge and fake MCU. |

## Protocol

Each frame is COBS-encoded and ends with a single `0x00` byte:

```
type u8 | bus u8 | seq u8 | status u8 | addr u32 LE | payload | crc16 LE   (CRC-16/CCITT-FALSE)
```

| Type | Dir | bus / addr | Payload |
|---|---|---|---|
| `HEARTBEAT` 0x01 | MCU→, echoed | – | any |
| `LOG` 0x02 | MCU→ | – | text, printed in the bridge terminal |
| `UART_TX` 0x10 | MCU→ | bus = N of `usart_N` | bytes to write |
| `UART_RX` 0x11 | →MCU | bus = N | bytes the sim wrote (unsolicited) |
| `UART_OPEN` 0x12 | MCU→ | bus = N | none. Opens the port so RX flows before the first TX. |
| `I2C_TXN` / `I2C_RSP` 0x20/0x21 | MCU→ / →MCU | bus = N of `i2c_N`, addr = 7-bit address | request: `rxlen u16 LE` + tx bytes; response: rx bytes |
| `SPI_TXN` / `SPI_RSP` 0x30/0x31 | MCU→ / →MCU | bus, addr = chip select. NOS bus is `spi_<bus*10+cs>`, as in hwlib. | same as I2C |
| `CAN_TXN` / `CAN_RSP` 0x40/0x41 | MCU→ / →MCU | bus = N of `can_N`, addr = CAN id | same as I2C. The bytes are passed through unchanged, so send what hwlib `libcan` sends. |
| `CI_PKT` 0x50 | →MCU | – | uplinked command packet |
| `TO_PKT` 0x51 | MCU→ | – | telemetry packet for the ground |
| `RADIO_RX` / `RADIO_TX` 0x52/0x53 | →MCU / MCU→ | – | radio device traffic |

Responses echo the request's `seq`. `status` is one of:
- 0 OK
- 1 bus error (the NOS Engine transaction failed)
- 2 bad request (for example a bus number of 30 or more)
- 3 unknown frame type

Bus numbers and addresses match what the cFS component drivers use. For example:

| Device | Bus |
|---|---|
| EPS | `i2c_1` @ 0x2B |
| GPS | `usart_1` |
| Reaction wheels 0–2 | `usart_2`..`usart_4` |
| Sample | `usart_16` |

See `cfg/sims/sc-1-nos3-simulator.xml` for the rest. Each component's device protocol is in `components/<name>/fsw/shared/*_device.c`. That code is the reference for what your MCU firmware has to send.

## Running with hardware

1. Build: `make config && make`. `make sim` builds `sims/build/bin/nos3-hil-bridge`.
2. **WSL2 only**: attach the USB serial device to WSL. In an admin PowerShell, run:
   ```
   usbipd list
   usbipd bind --busid <BUSID>
   usbipd attach --wsl --busid <BUSID>
   ```
   Then check that `/dev/ttyACM0` (or `/dev/ttyUSB0`) exists in WSL.
3. Launch: `HIL=1 make launch`. Optional variables:
   - `HIL_SERIAL=/dev/ttyUSB0`: the host device, default `/dev/ttyACM0`. Inside the container it always appears as `/dev/ttyHIL0`.
   - `HIL_BAUD=115200`: default 921600. It is ignored by USB CDC devices.
   - `HIL_ARGS="-v -u 1"`: extra bridge flags. `-v` logs every frame; `-u N` opens `usart_N` at startup.

   The launch script is copied into `cfg/build/` by `make config`, so re-run `make config` after pulling changes to `scripts/fsw/fsw_cfs_launch.sh`.
4. The "HIL Bridge" terminal replaces "NOS3 Flight Software". It prints `MCU link up` once frames arrive, and `MCU link down` after 3 s of silence. The MCU should send a heartbeat about once a second.

## Testing without hardware

```bash
# Protocol unit test (host or container)
gcc -std=c99 -Wall -Wextra -Icomponents/hil_bridge/protocol \
    components/hil_bridge/protocol/test_hil_link.c components/hil_bridge/protocol/hil_link.c -o /tmp/t && /tmp/t

# End-to-end: bridge + real EPS/sample sims + fake MCU over a pty (from the NOS3 root, after make sim)
docker run --rm -v $PWD:$PWD --add-host nos-engine-server:127.0.0.1 ivvitc/nos3-64:20260619 \
    $PWD/components/hil_bridge/support/e2e_test.sh
```

The fake MCU can also be pointed at a running bridge through any pty or serial device, for example `support/hil_fake_mcu.py /dev/ttyUSB1 --listen`.

## Limitations

- **The COSMOS DEBUG interface doesn't work.** It talks directly to ci_lab/to_lab on `nos-fsw:5012/5013`, and the bridge doesn't serve those ports. The RADIO path through the radio sim works.
- **The bridge is sequential.** Like hwlib, NOS Engine transactions block the bridge and have no timeout. If the engine server dies mid-transaction, the bridge hangs; a second Ctrl-C or `docker stop` exits it.
- **The MCU runs on its own clock.** NOS3 runs in real time by default (`sim-microseconds-per-tick` equals `real-microseconds-per-tick`). The MCU doesn't receive NOS time ticks, so keep the sim at 1:1 speed.
- **Large packets are dropped.** Packets over `HIL_MAX_PAYLOAD` (2048 bytes by default) are dropped with a log message. If you shrink it on the MCU to save RAM, keep it at least as large as your biggest telemetry packet.
