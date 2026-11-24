# Protocol catalogue

Every protocol below is a C program in `sw/examples/` or `sw/tests/` (stock GCC, `protocol.h`) running on the RTL, and every one is judged by an independent model of the other side written from the protocol's specification or a datasheet (`test/lib/`), not from our RTL. Evidence level for all rows: **RTL simulation** (Icarus and Verilator); the rows marked GL also pass gate-level simulation of the routed netlist.

Any asynchronous serial format (5 to 9 data bits, parity, 1 or 2 stop bits, inverted line) is a `ser_*` setting in C, since the engine shifts whole frames.

Size = code + initialized data in bytes (GCC 15, `-Os`), out of about 13.5 KB for program, data and stack (14 KB SRAM; the sizes were measured when it was 4 KB, so every program also fits the old 3.5 KB). "Engine" means a Wire Engine; "timed GPIO" means the queue that changes pins on exact cycles; "capture" means the edge-timestamp unit.

## Standard serial buses

| Protocol | Roles | Program | Size | How it is made | Test (model) |
|---|---|---|---|---|---|
| UART 8N1, any rate (tested 115200 to 3 Mbaud) | both directions | `uart_echo` | 485 | engine, fractional bit period, RX resync on edges | `test_uart` (GL) |
| UART with automatic baud rate | receiver finds the rate | `uart_autobaud` | 198 | capture: 'U' has falling edges 2 bits apart; 8 bits measured in cycles go straight into the divider | `test_gpio_protocols` (57600, 460800, 1 Mbaud) |
| SPI, modes 0-3, up to F/4 | controller | `spi_ram` | 674 | engine, internal clock on SCK, RX sample delay | `test_spi` |
| SPI | target | `spi_target` | 246 | engine, external clock, chip-select gate, frame events | `test_spi` |
| I2C 100 kHz, 400 kHz, 1 MHz (Fast-mode Plus) | controller | `i2c_eeprom` | 954 | engine, open drain, clock stretching | `test_i2c` |
| I2C | target (up to 1 MHz) | `i2c_target` | 496 | engine, START/STOP gate, SCL stretching while C decides | `test_i2c` |
| I2C bus analyzer | passive | `i2c_sniff` | 190 | the target settings with TX off and 9-bit words (byte + ACK) | `test_i2c_sniff` |
| SMBus with PEC | host | `smbus` | 1124 | I2C + CRC-8 over every byte, addresses included | `test_smbus` |
| 1-Wire | master | `onewire` | 478 | timed GPIO only | `test_onewire` |
| 1-Wire (DS18B20-style) | device | `onewire_slave` | 729 | capture stamps each slot's falling edge; read-0 pulls low at once, timed GPIO releases it; READ/MATCH/SKIP/SEARCH ROM | `test_onewire_slave` |
| MDIO (IEEE 802.3 clause 22) | management station | `mdio` | 272 | lite engine; reads release MDIO and clock in the turnaround + data | `test_mdio` |
| SD card, SPI mode | host | `sd_spi` | 1210 | SPI engine whose CRC unit appends each command's CRC7; data CRC16 in C | `test_sd_spi` |
| MIPI I3C SDR (12.5 MHz push-pull data) | controller | `i3c` (test) | 1068 | full engine: address header and ACK open drain at 1 MHz, data words at the second bit rate and driven push-pull (`WIRE_A_PP`) with odd-parity T bits; START/STOP by GPIO; CCCs RSTDAA, SETDASA, GETPID, GETBCR, GETDCR; private write and target-ended read | `test_i3c` (open-drain phases never driven high, data words push-pull, T bits, dynamic address assigned and reset) |
| Quad-SPI NOR flash (W25Q-style, 0x6B Fast Read Quad Output) | host | `qspi_flash` | 548 | all four engines as one 4-bit port: engine 0 sends command and address and clocks SCK, engines 1..3 receive IO1..IO3 on SCK as an external clock; IO0 released for the dummy clocks; C interleaves the nibbles | `test_qspi_flash` (bus contention checked; GL) |

## USB and Ethernet

| Protocol | Roles | Program | Size | How it is made | Test (model) |
|---|---|---|---|---|---|
| USB 1.1 low speed (1.5 Mbit/s) | device (HID keyboard) | `usb_hid` | 1998 | engine: NRZI, stuffing, CRC16, D+/D- with SE0, SYNC hunt, handshakes and IN data released by the engine 3.1 bit times after the host's packet | `test_usb`, `test_usb_hid` |
| USB 1.1 full speed (12 Mbit/s, 48 MHz clock) | device | `usb_hid_fs` (same source) | 1990 | the same engine settings with 4 cycles per bit and the pull-up on D+; answers at 4.5 bit times | `test_usb_hid` |
| USB low speed | host | `usb_host` | 1706 | the same line layer; bus reset, keep-alives, enumeration, interrupt polling with NAKs; the host's ACK after a device DATA packet is released by the engine | `test_usb_host` |
| USB full speed (12 Mbit/s, 48 MHz clock) | host | `usb_host_fs` (same source) | 1850 | the same line layer at 4 cycles per bit; the start-of-frame token goes out on its exact cycle every 1 ms (C waits for it before a transaction that would overlap) | `test_usb_host` (SOF period 1 ms +/- 0.08 us, frame numbers) |
| USB Power Delivery (USB-C, BMC at 300 kbit/s) | sink: Source_Capabilities -> Request (highest fixed voltage up to 9 V) -> Accept -> PS_RDY | `usb_pd_sink` | 1586 | engine 1: biphase mark both ways on the CC wire (TX driven only while sending, RX through a comparator), SYNC hunt on the preamble end + Sync-1, 5-bit symbols; 4b5b, CRC-32 and GoodCRC in C | `test_usb_pd` (independent source model; every GoodCRC starts within tTransmit = 195 us) |
| 10BASE-T transmit | | `eth_tx` | 398 | engine: Manchester, differential pair, CRC-32 appended, TP_IDL | `test_eth` |
| 10BASE-T receive | | `eth_rx` | 364 | engine 1: Manchester decoded by edge timing (clock recovery), SFD hunt, CRC-32 residual | `test_eth_rx` (50 and 60 MHz, edge jitter, sender 0.1 % off; GL) |
| ARP + ICMP echo (ping) over 10BASE-T, with link pulses | node | `eth_ping` | 1138 | RX on engine 1, TX on engine 0, 60 MHz; a normal link pulse (one 100 ns 1-bit NRZ word) every 16 ms while idle | `test_eth_ping`, `test_eth_nlp` |
| 10BASE-T auto-negotiation (IEEE 802.3 clause 28, fast link pulses) | both ends of the exchange | `eth_autoneg` | 884 | FLP bursts of timed 100 ns link pulses; the partner's bursts decoded from capture time stamps; ability match, acknowledge, highest common mode | `test_eth_autoneg` (full and half duplex partners) |

## Field, automotive and industrial buses

| Protocol | Roles | Program | Size | How it is made | Test (model) |
|---|---|---|---|---|---|
| CAN 2.0A | node: send with arbitration, receive with ACK | `can`, `can_rx` | 880, 678 | engine: stuffing after 5 equal bits, CRC-15, read-back arbitration, ACK slot inside the packet | `test_can` (GL) |
| CAN FD (ISO 11898-1:2015), 500 kbit/s + 2 Mbit/s and 1 + 5 Mbit/s | node: send (0 to 64 bytes, with and without bit rate switch, base and extended IDs), receive with ACK, classic frames on the same bus | `canfd`, `canfd_rx` (tests) | 1794, 1270 | engine 0: data phase at the second bit period, switched at the sample points of BRS and of the CRC delimiter; C simulates the stuffing once to get the stuff count and CRC-17/21, then queues the frame. RX: the CAN FD unit follows FDF/BRS/DLC, switches the RX bit rate, removes the fixed stuff bits, checks CRC and stuff count, and ACKs good frames itself | `test_canfd` (independent node with its own 80/75 % sample points: stuffing, fixed stuff bits, Gray stuff count, CRC; a bad-CRC frame is not ACKed) |
| CAN 2.0B (29-bit identifiers, as used by SAE J1939) | node: both frame formats, send with arbitration, receive with ACK | `can_b` | 1392 | the same engine settings; C builds the 39-bit header. RX words are 20 bits long so the DLC of either format is known before the ACK slot | `test_can` (standard beats extended with the same base ID; extended DLC 0 and 1 acknowledged) |
| LIN 2.x | master | `lin_master` | 674 | UART engine; the BREAK is one 14-bit word; every byte read back from the bus | `test_lin` |
| LIN 2.x | slave: publish, subscribe, diagnostic Read by Identifier (0x3C/0x3D) | `lin_slave` | 878 | lite engine with `ser_*`; the BREAK arrives as an all-zero frame word; sync and PID parity checked; each sent byte read back from the bus | `test_lin_slave` |
| IO-Link (IEC 61131-9) | master port: wake-up, COM3/COM2/COM1 rate search, STARTUP (Direct Parameter page 1), PREOPERATE, cyclic OPERATE | `iolink_master` | 1550 | timed GPIO for the 80 us wake-up pulse, then a lite engine with `ser_*` (8E1); M-sequence checksum (spec A.1.6) in C; every octet read back from C/Q | `test_iolink` (MasterCycleTime from the device's MinCycleTime) |
| Modbus RTU | client | `modbus_master` | 1318 | UART engine; frame end = 3.5 characters of silence via the blocking-read timeout; CRC-16 in C | `test_modbus` |
| Modbus RTU | server (16 holding registers), 19200 8E1 | `modbus_server` | 1660 | lite engine with `ser_*`; frame end by the blocking-read timeout, gaps over 1.5 characters, parity and CRC errors drop the frame; functions 03, 06, 16, exceptions 01, 02, broadcast | `test_modbus_server` |
| HDLC (ISO 13239), synchronous | transmitter | `hdlc` | 302 | engine: zero insertion after five ones, FCS-16 appended and stuffed, flags as NOSTUFF words | `test_framing` |
| HDLC (ISO 13239), synchronous | receiver (line clock in) | `hdlc_rx` | 538 | lite engine collects raw bits on the receive clock; C removes stuffed zeros, finds flags and aborts, checks the FCS residual | `test_framing` (bad FCS and abort reported) |
| SAE J2716 SENT | sensor | `sent` | 386 | lite engine, one bit per 3 us tick; a pulse of N ticks is one N-bit word | `test_pulse_codes` |
| DMX512 | controller | `dmx512` | 220 | lite engine; BREAK + MAB is one 26-bit word | `test_dmx512` |
| DALI (IEC 62386) | application controller | `dali` | 356 | engine 1: Manchester at 1200 bit/s both ways (MSTART for a bus idling high) | `test_dali` |
| SDI-12 (environmental sensors) | data recorder: ?!, aI!, aM! + service request, aD0! | `sdi12` | 1243 | lite engine on one wire, driven only while a word shifts (`WIRE_OE_FRAME`), 1200 baud 7E1 negative logic; BREAK + marking as one 25-bit word | `test_sdi12` (break and marking lengths, line released before the sensor answers) |
| BiSS-C (absolute encoders; SSI-compatible wiring) | master | `biss` | 570 | SPI engine, mode 3 at 5 MHz, 48 clocks per frame; C finds the Start bit, so any cable delay works; CRC-6 in C | `test_biss` (cable delay 0.7 and 3.7 clock periods, CRC error found; GL) |
| K-line OBD-II, ISO 14230-4 (KWP2000, fast init) | scan tool | `kline` | 889 | timed GPIO for the 25 ms wake-up pattern, then a lite engine with `ser_*` at 10400 baud; tester bytes 6 ms apart, each read back from the single wire | `test_kline` (TiniL, TWuP, W5, P1..P4 checked) |
| MIL-STD-1553B | bus controller | `mil1553` | 1214 | Manchester II and the 3-bit sync built in C as 2 Mbit/s half bits; two lite engines in lockstep drive TXA/TXB, a full engine receives RXA as packets; sync, Manchester and odd parity checked in C | `test_mil1553` (BC-to-RT, RT-to-BC, absent RT; GL) |
| ARINC 429 (avionics) | transmitter and receiver | `arinc429` | 582 | TX: two lite engines in lockstep send half-bit chips to the line driver's DATA A / DATA B (bipolar return-to-zero); RX: C polls the line receiver's HI/LO outputs | `test_arinc429` (half-bit and bit timing, gaps, parity, label order; GL) |
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
| PDM MEMS microphone | host (recording) | `pdm_mic` | 320 | SPI engine drives the microphone clock (1.04 MHz) and samples its bit stream; a third-order CIC filter in C decimates by 64 to 16.28 kHz | `test_pdm_mic` (1 kHz sine: amplitude within 0.01 %, residual 84 dB below full scale) |
| S/PDIF / AES3 (IEC 60958), 48 and 96 kHz | transmitter and receiver | `spdif` (test) | 710 | biphase mark in the engine: TX on a full engine, preambles as RAW half-slot words at the second bit period, parity in C; RX on engine 1 by edge intervals, locks on the first preamble, 30-bit words with the preamble type | `test_spdif` (independent decoder; RX from a stream started mid-subframe with +/-1 cycle of edge jitter) |
| SMPTE linear timecode (LTC), 30 fps | generator and reader | `ltc` | 962 | biphase mark in the engine: 80-bit frames as three words with the polarity bit computed in C; the reader (engine 1) hunts the end of the sync word, then takes five 16-bit words per frame | `test_ltc` (independent decoder of the chip's output; reading a stream that starts mid-frame) |
| MIDI 1.0 | sender | `midi` | 242 | UART engine at 31250 baud, running status | `test_framing` |
| DHT22 / AM2302 temperature and humidity sensor | host | `dht22` | 458 | no engine: GPIO start pulse, then capture stamps every falling edge; a bit is 1 if its edge comes > 100 us after the previous one | `test_dht22` (negative temperature, checksum error) |
| ISO/IEC 7816-3 smart card (T=0: SIM, bank and ID cards) | reader | `iso7816` | 851 | a lite engine shifting 0xAAAAAAAA makes the free-running card clock (C tops up its FIFO in every wait loop); open-drain I/O on a second engine with 1 etu = 372 card clocks; ATR, then a T=0 command with procedure byte and status word | `test_iso7816` (card model timed in card clocks: no clock gap allowed) |
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
| UART at 1 Mbaud received and sent by DMA while the CPU does not touch the engine (120 bytes back to back each way) | `dma` (test) | `test_dma` |
| Logic analyzer (all inputs every 10 cycles) and pattern generator (uo pins every 25 cycles, exact) at the same time, by DMA | `dma` (test) | `test_dma` |
| CAN FD frames back to back at 1 + 5 Mbit/s into a DMA ring while C works 100 us per frame | `canfd_rx` (test) | `test_canfd` (the same test without DMA loses frames) |
| CAN <-> serial adapter (Lawicel/SLCAN text, as used by python-can and SocketCAN `slcand`): CAN 1 Mbit/s at full bus load to text on a 3 Mbaud UART, `t`/`T` commands sent with arbitration retry, acceptance filter, timestamps (5.9 KB) | `can_gateway` | `test_can_gateway` (3,000 frames back to back with the 3-bit intermission, all forwarded; 30 frames sent while receiving) |
| Logic analyzer: burst of 3,099 samples of all inputs at 25 MS/s (12.4 KB of SRAM), or a change log that keeps 30,000 samples at 1.67 MS/s in 6 KB | `logic_analyzer` | `test_logic_analyzer` |
| UART 5 Mbaud (DMA) + SPI 16.7 MHz + I2C 1 MHz + CAN 1 Mbit/s at host-chosen rates | `stress4` (test) | `test_stress4` (not in the default list) |

## Clock rates

The chip is rated at 50 MHz. Protocols with a fixed bit rate that is not a divisor of 50 MHz still work through the fractional bit period (e.g. USB low speed at 33.33 cycles per bit). Two need a different clock, which the demo board's RP2350 can supply: USB full speed needs 48 or 60 MHz (4 or 5 cycles per bit), and 10BASE-T transmit needs 40 or 60 MHz for symmetric half bits (`eth_ping` runs at 60 MHz). 10BASE-T receive works at 50 MHz with up to +/-5 ns of jitter on every edge and at 60 MHz with +/-8 ns.

## Experimental: 100 Mbit/s Ethernet (RMII)

The engines move at most one bit every two cycles (25 Mbit/s per lane), so 100 Mbit/s Ethernet needs hardware: `src/pg_rmii.v` is a small RMII port (two bits per 50 MHz cycle each way, preamble/SFD, CRC-32 two bits per cycle, inter-frame gap; frames stay C data). The chip clock must be the PHY's 50 MHz REF_CLK. `sw/tests/rmii_echo` echoes frames of 60 to 1514 bytes at 100 Mbit/s against an independent RMII PHY model (`make -C test RMII=yes COCOTB_TEST_MODULES=test_rmii`); C keeps up with a word every 16 cycles. `sw/examples/eth100_ping` answers ARP and ping (payloads up to 1000 bytes) at 100 Mbit/s, checked against RFC 826/791/792 (`test_eth100_ping`, also with `RMII=yes`). One CPU serves one direction at a time: a frame that arrives while C is sending is lost (the RX overflow flag reports it). Evidence level: RTL simulation only; the port is not in the hardened layout: the variant `src/variants/rmii/` (`pg_io.v` with the port, `pg_top.v` with `RMII = 1`) is used only by `make RMII=yes`. A trial place-and-route of that variant (2026-11-19) did not close: the port adds about 35k µm² (synthesis, +8 %), global routing overflowed and detailed routing stopped with 315 violations, so the 8x4 tile has no room for it next to the four engines (P&R). Trading the second lite engine for the port did route cleanly (no overflow, more setup slack; P&R trial, 2026-11-19), but we kept four engines: with 4 KB of RAM and one CPU the chip can only use 100 Mbit/s for small exchanges (ARP, ping, UDP) that 10BASE-T already covers, the port needs an external PHY whose 50 MHz REF_CLK must also clock the chip, and the fourth engine buys a fourth protocol at the same time and quad-SPI reads.

## Added with hardware (2026-11-21)

CAN FD, I3C, S/PDIF, USB Power Delivery and LTC needed small hardware additions to the engines, made with the area the place-and-route work freed (`docs/architecture.md` section 6): a second bit period per word, a programmable sample point, per-word push-pull drive, the biphase line code with an edge-interval decoder, and the CAN FD receive unit on engine 0. Evidence level: RTL simulation against independent models.
