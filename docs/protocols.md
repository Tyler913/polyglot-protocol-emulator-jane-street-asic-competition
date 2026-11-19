# Protocol catalogue

Every protocol below is a C program in `sw/examples/` or `sw/tests/` (stock GCC, `protocol.h`) running on the RTL, and every one is judged by an independent model of the other side written from the protocol's specification or a datasheet (`test/lib/`), not from our RTL. Evidence level for all rows: **RTL simulation** (Icarus and Verilator); the rows marked GL also pass gate-level simulation of the routed netlist.

Any asynchronous serial format (5 to 9 data bits, parity, 1 or 2 stop bits, inverted line) is a `ser_*` setting in C, since the engine shifts whole frames.

Size = code + initialized data in bytes (GCC 15, `-Os`), out of 3.5 KB for program, data and stack. "Engine" means a Wire Engine; "timed GPIO" means the queue that changes pins on exact cycles; "capture" means the edge-timestamp unit.

## Standard serial buses

| Protocol | Roles | Program | Size | How it is made | Test (model) |
|---|---|---|---|---|---|
| UART 8N1, any rate (tested 115200 to 3 Mbaud) | both directions | `uart_echo` | 485 | engine, fractional bit period, RX resync on edges | `test_uart` (GL) |
| UART with automatic baud rate | receiver finds the rate | `uart_autobaud` | 198 | capture: 'U' has falling edges 2 bits apart; 8 bits measured in cycles go straight into the divider | `test_gpio_protocols` (57600, 460800, 1 Mbaud) |
| SPI, modes 0-3, up to F/4 | controller | `spi_ram` | 674 | engine, internal clock on SCK, RX sample delay | `test_spi` |
| SPI | target | `spi_target` | 246 | engine, external clock, chip-select gate, frame events | `test_spi` |
| I2C 100/400 kHz | controller | `i2c_eeprom` | 954 | engine, open drain, clock stretching | `test_i2c` |
| I2C | target | `i2c_target` | 496 | engine, START/STOP gate, SCL stretching while C decides | `test_i2c` |
| I2C bus analyzer | passive | `i2c_sniff` | 190 | the target settings with TX off and 9-bit words (byte + ACK) | `test_i2c_sniff` |
| SMBus with PEC | host | `smbus` | 1124 | I2C + CRC-8 over every byte, addresses included | `test_smbus` |
| 1-Wire | master | `onewire` | 478 | timed GPIO only | `test_onewire` |
| 1-Wire (DS18B20-style) | device | `onewire_slave` | 729 | capture stamps each slot's falling edge; read-0 pulls low at once, timed GPIO releases it; READ/MATCH/SKIP/SEARCH ROM | `test_onewire_slave` |
| MDIO (IEEE 802.3 clause 22) | management station | `mdio` | 272 | lite engine; reads release MDIO and clock in the turnaround + data | `test_mdio` |
| SD card, SPI mode | host | `sd_spi` | 1210 | SPI engine whose CRC unit appends each command's CRC7; data CRC16 in C | `test_sd_spi` |

## USB and Ethernet

| Protocol | Roles | Program | Size | How it is made | Test (model) |
|---|---|---|---|---|---|
| USB 1.1 low speed (1.5 Mbit/s) | device (HID keyboard) | `usb_hid` | 1998 | engine: NRZI, stuffing, CRC16, D+/D- with SE0, SYNC hunt, handshakes and IN data released by the engine 3.1 bit times after the host's packet | `test_usb`, `test_usb_hid` |
| USB 1.1 full speed (12 Mbit/s, 48 MHz clock) | device | `usb_hid_fs` (same source) | 1990 | the same engine settings with 4 cycles per bit and the pull-up on D+; answers at 4.5 bit times | `test_usb_hid` |
| USB low speed | host | `usb_host` | 1702 | the same line layer; bus reset, keep-alives, enumeration, interrupt polling with NAKs; the host's ACK after a device DATA packet is released by the engine | `test_usb_host` |
| 10BASE-T transmit | | `eth_tx` | 398 | engine: Manchester, differential pair, CRC-32 appended, TP_IDL | `test_eth` |
| 10BASE-T receive | | `eth_rx` | 364 | engine 1: Manchester decoded by edge timing (clock recovery), SFD hunt, CRC-32 residual | `test_eth_rx` (50 and 60 MHz, edge jitter, sender 0.1 % off; GL) |
| ARP + ICMP echo (ping) over 10BASE-T, with link pulses | node | `eth_ping` | 1138 | RX on engine 1, TX on engine 0, 60 MHz; a normal link pulse (one 100 ns 1-bit NRZ word) every 16 ms while idle | `test_eth_ping`, `test_eth_nlp` |

## Field, automotive and industrial buses

| Protocol | Roles | Program | Size | How it is made | Test (model) |
|---|---|---|---|---|---|
| CAN 2.0A | node: send with arbitration, receive with ACK | `can`, `can_rx` | 880, 678 | engine: stuffing after 5 equal bits, CRC-15, read-back arbitration, ACK slot inside the packet | `test_can` (GL) |
| CAN 2.0B (29-bit identifiers, as used by SAE J1939) | node: both frame formats, send with arbitration, receive with ACK | `can_b` | 1392 | the same engine settings; C builds the 39-bit header. RX words are 20 bits long so the DLC of either format is known before the ACK slot | `test_can` (standard beats extended with the same base ID; extended DLC 0 and 1 acknowledged) |
| LIN 2.x | master | `lin_master` | 674 | UART engine; the BREAK is one 14-bit word; every byte read back from the bus | `test_lin` |
| LIN 2.x | slave: publish, subscribe, diagnostic Read by Identifier (0x3C/0x3D) | `lin_slave` | 878 | lite engine with `ser_*`; the BREAK arrives as an all-zero frame word; sync and PID parity checked; each sent byte read back from the bus | `test_lin_slave` |
| IO-Link (IEC 61131-9) | master port: wake-up, COM3/COM2/COM1 rate search, STARTUP (Direct Parameter page 1), PREOPERATE, cyclic OPERATE | `iolink_master` | 1550 | timed GPIO for the 80 us wake-up pulse, then a lite engine with `ser_*` (8E1); M-sequence checksum (spec A.1.6) in C; every octet read back from C/Q | `test_iolink` (MasterCycleTime from the device's MinCycleTime) |
| Modbus RTU | client | `modbus_master` | 1318 | UART engine; frame end = 3.5 characters of silence via the blocking-read timeout; CRC-16 in C | `test_modbus` |
| Modbus RTU | server (16 holding registers), 19200 8E1 | `modbus_server` | 1660 | lite engine with `ser_*`; frame end by the blocking-read timeout, gaps over 1.5 characters, parity and CRC errors drop the frame; functions 03, 06, 16, exceptions 01, 02, broadcast | `test_modbus_server` |
| HDLC (ISO 13239), synchronous | transmitter | `hdlc` | 302 | engine: zero insertion after five ones, FCS-16 appended and stuffed, flags as NOSTUFF words | `test_framing` |
| SAE J2716 SENT | sensor | `sent` | 386 | lite engine, one bit per 3 us tick; a pulse of N ticks is one N-bit word | `test_pulse_codes` |
| DMX512 | controller | `dmx512` | 220 | lite engine; BREAK + MAB is one 26-bit word | `test_dmx512` |
| DALI (IEC 62386) | application controller | `dali` | 356 | engine 1: Manchester at 1200 bit/s both ways (MSTART for a bus idling high) | `test_dali` |
| Wiegand 26-bit | both: reader in, reader out | `wiegand` | 390 | GPIO polling in, timed GPIO out | `test_gpio_protocols` |

## Debug, peripherals and consumer interfaces

| Protocol | Roles | Program | Size | How it is made | Test (model) |
|---|---|---|---|---|---|
| ARM SWD | host | `swd` | 1058 | lite engine, turnarounds by rewriting CTRL | `test_swd` (GL) |
| JTAG (IEEE 1149.1) | host | `jtag` | 674 | lite engine, TMS sequences | `test_jtag` |
| PS/2 | host | `ps2_host` | 662 | lite engine clocked by the keyboard | `test_ps2` |
| PS/2 | device (keyboard) | `ps2_device` | 664 | engine with an open-drain clock out; host-to-device bits clocked in with timed GPIO | `test_ps2_device` |
| HDMI-CEC | initiator | `cec` | 527 | timed GPIO on an open-drain line; the follower's ACK read at 1.05 ms | `test_cec` |
| I2S (Philips) | transmitter | `i2s` | 320 | two lite engines in lockstep: data + BCLK, and WS | `test_framing` |
| I2S (Philips) | receiver (external BCLK/WS) | `i2s_rx` | 288 | lite engine on the external BCLK; capture of a falling WS edge aligns the words | `test_framing` |
| MIDI 1.0 | sender | `midi` | 242 | UART engine at 31250 baud, running status | `test_framing` |
| HD44780 character LCD, 4-bit | controller | `hd44780` | 493 | timed GPIO: every setup/hold/pulse width on its cycle | `test_hd44780` |
| WS2812 LEDs | controller | `ws2812` | 242 | lite engine, 3 line bits per LED bit | `test_ws2812` |
| DShot600 (drone ESCs) | controller | `dshot` | 252 | lite engine, 8 line bits per DShot bit | `test_pulse_codes` |
| Futaba S.BUS (RC receiver link) | both: decode (flight controller), encode (receiver) | `sbus` | 884 | lite engine with `ser_*`: 8E2 on an inverted line, 25-byte frames found by the gap before them, 16 x 11-bit channels | `test_sbus` (parity error rejected) |
| CRSF (ExpressLRS / TBS Crossfire) | flight controller: RC channels and link statistics in, battery telemetry out | `crsf` | 760 | lite engine with `ser_*` at 420000 baud; frames by sync byte and length, CRC-8 (0xD5) in C | `test_crsf` (500 Hz frames, bad CRC rejected) |
| NEC infrared, with 38 kHz carrier | transmitter | `ir_nec` | 424 | lite engine draws the carrier as a bit pattern | `test_ir_nec` |
| NEC and RC-5 infrared | receiver | `ir_rx` | 750 | capture timestamps, decoded in C with +/- 25 % tolerance | `test_ir_rx` |
| Quadrature encoder (x4) | counter | `quadrature` | 166 | GPIO polling, edges every 0.5 us | `test_gpio_protocols` |

## Several at once

| What | Program | Test |
|---|---|---|
| Four UARTs at 1 Mbaud, one per engine | `quad_uart` | `test_quad` (GL) |
| CAN + SPI + UART + I2C as four cooperative C tasks | `mixed4` (3360 B) | `test_mixed` |
| A USB keyboard that types what arrives on its UART | `usb_hid` | `test_usb_hid` |
| Eight protocol jobs, four at a time, loaded while the others run | `queue` + `sw/jobs/` | `test_queue` |

## Clock rates

The chip is rated at 50 MHz. Protocols with a fixed bit rate that is not a divisor of 50 MHz still work through the fractional bit period (e.g. USB low speed at 33.33 cycles per bit). Two need a different clock, which the demo board's RP2350 can supply: USB full speed needs 48 or 60 MHz (4 or 5 cycles per bit), and 10BASE-T transmit needs 40 or 60 MHz for symmetric half bits (`eth_ping` runs at 60 MHz). 10BASE-T receive works at 50 MHz with up to +/-5 ns of jitter on every edge and at 60 MHz with +/-8 ns.

## Not covered

Full-speed USB host, 10BASE-T auto-negotiation (fast link pulses), CAN FD, HDLC receive (the engine destuffs only on asynchronous RX), ISO 7816 smart cards (they need a free-running card clock, which here would have to be fed by the CPU) and MIL-STD-1553 have no program yet.
