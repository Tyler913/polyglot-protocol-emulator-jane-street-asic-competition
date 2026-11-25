# Polyglot: a protocol emulator you program in plain C

An entry for the [Jane Street protocol emulator ASIC competition](https://blog.janestreet.com/protocol-emulator-asic-competition/) (Tiny Tapeout, IHP CMOS5L, 8x4 tiles).

A small RISC-V core runs ordinary C compiled with stock GCC or Clang. Protocol-agnostic, timed *wire engines* move bits on exact clock cycles. One header, `protocol.h`, is the whole toolchain. There is no custom assembler and no custom compiler.

**Status:** about 60 protocols run as C firmware on the RTL and are checked against independent peer models written from each specification ([docs/protocols.md](docs/protocols.md)): UART, SPI, I2C (up to 1 MHz), 1-Wire, PS/2, LIN, Modbus RTU and HDLC in both roles, SMBus, MDIO, SD cards, quad-SPI flash, MIPI I3C, USB 1.1 (low- and full-speed device and host), USB Power Delivery, 10BASE-T (transmit, receive with clock recovery, an ARP/ping node, auto-negotiation), CAN 2.0A/2.0B, CAN FD, IO-Link, K-line OBD-II, SDI-12, BiSS-C encoders, MIL-STD-1553B, ARINC 429, ISO 7816 smart cards, S.BUS and CRSF, SENT, DMX512, DALI, Wiegand, JTAG, ARM SWD, HDMI-CEC, I2S, PDM microphones, S/PDIF / AES3, SMPTE LTC, MIDI, HD44780, WS2812, DShot, DHT22, NEC and RC-5 infrared, quadrature encoders, and several of them at once. The full design (with 14 KB of SRAM in three macros, four Wire Engines and two DMA channels) passes the Tiny Tapeout flow locally: LVS, KLayout DRC and antenna clean, timing met at 50 MHz, precheck and 23/23 gate-level tests pass (earlier 4 KB layouts also passed in the official GitHub GDS workflow). The same RTL builds for an FPGA board (not yet run on hardware).

## Layout

| Path | Contents |
|---|---|
| `src/` | RTL. `pg_*.v` is ours; `hazard3_*` is the [Hazard3](https://github.com/Wren6991/Hazard3) RISC-V core by Luke Wren (Apache-2.0), vendored at commit `8af992930f71` with one marked change (`hazard3_regfile_1w2r.v`: x0's entry is cleared during reset) |
| `sw/` | `protocol.h`, startup code, linker script, example programs (`examples/`), test programs (`tests/`) and queue jobs (`jobs/`) |
| `test/` | cocotb tests (they build the firmware and run it on the RTL) |
| `tools/polyglot_host.py` | host driver (MicroPython on the demo board's RP2350, or CPython through the FPGA prototype's USB UART): load, run, read/write memory |
| `fpga/` | FPGA prototype for an AMD Zynq UltraScale+ board (ALIENTEK DFZU2EG): the same RTL with a block RAM, a board top and a UART bridge so a PC can act as the host |
| `docs/` | `architecture.md`, `programming.md`, `protocols.md`, `verification.md`, `ai_use.md`, and the Tiny Tapeout datasheet `info.md` |
| `librelane_plugin_polyglot.py`, `flow/` | LibreLane steps that power the three SRAM macros (Metal4 stripes on their supply columns) and merge tie cells before placement |

## Build and test

```sh
# Toolchain: any GCC with the rv32ec/ilp32e multilib, e.g. xPack riscv-none-elf-gcc 15.2
make -C sw                       # build the example programs
pip install -r test/requirements.txt
make -C test                     # RTL simulation with Icarus Verilog + cocotb
```

## Credits

- CPU: [Hazard3](https://github.com/Wren6991/Hazard3), Luke Wren, Apache-2.0 (one marked local change, see above).
- SRAM: IHP `RM_IHPSG13_1P_1024x64_c2_bm_bist` (8 KB), `RM_IHPSG13_1P_1024x32_c2_bm_bist` (4 KB) and `RM_IHPSG13_1P_256x64_c2_bm_bist` (2 KB) macros from the IHP Open PDK: 14 KB.
- SRAM power-grid approach (stripes on the macro's Metal4 supply columns) after Ken Pettit's [IHP SRAM walkthrough](https://kdp1965.github.io/ihp-um-janestreet-prism/ihp_sram_macro.html); our step is written from scratch.
- Project template: [Tiny Tapeout IHP Verilog template](https://github.com/TinyTapeout/ttihp-verilog-template).
