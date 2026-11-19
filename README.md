# Polyglot: a protocol emulator you program in plain C

An entry for the [Jane Street protocol emulator ASIC competition](https://blog.janestreet.com/protocol-emulator-asic-competition/) (Tiny Tapeout, IHP CMOS5L, 8x4 tiles).

A small RISC-V core runs ordinary C compiled with stock GCC or Clang. Protocol-agnostic, timed *wire engines* move bits on exact clock cycles. One header, `protocol.h`, is the whole toolchain. There is no custom assembler and no custom compiler.

**Status:** about 55 protocols run as C firmware on the RTL and are checked against independent peer models written from each specification ([docs/protocols.md](docs/protocols.md)): UART, SPI, I2C (up to 1 MHz), 1-Wire, PS/2, LIN, Modbus RTU and HDLC in both roles, SMBus, MDIO, SD cards, quad-SPI flash, USB 1.1 (low- and full-speed device and host), 10BASE-T (transmit, receive with clock recovery, an ARP/ping node), CAN 2.0A/2.0B, IO-Link, K-line OBD-II, SDI-12, BiSS-C encoders, MIL-STD-1553B, ARINC 429, ISO 7816 smart cards, S.BUS and CRSF, SENT, DMX512, DALI, Wiegand, JTAG, ARM SWD, HDMI-CEC, I2S, PDM microphones, MIDI, HD44780, WS2812, DShot, DHT22, NEC and RC-5 infrared, quadrature encoders, and several of them at once. The full design (with the SRAM macro) passes the Tiny Tapeout flow: LVS and DRC clean, timing met at 50 MHz, precheck and gate-level tests pass, locally and in the official GitHub workflow. The same RTL builds for an FPGA board (not yet run on hardware).

## Layout

| Path | Contents |
|---|---|
| `src/` | RTL. `pg_*.v` is ours; `hazard3_*` is the [Hazard3](https://github.com/Wren6991/Hazard3) RISC-V core by Luke Wren (Apache-2.0), vendored at commit `8af992930f71` with one marked change (`hazard3_regfile_1w2r.v`: x0's entry is cleared during reset) |
| `sw/` | `protocol.h`, startup code, linker script, example programs (`examples/`) and test programs (`tests/`) |
| `test/` | cocotb tests (they build the firmware and run it on the RTL) |
| `tools/polyglot_host.py` | host driver (MicroPython on the demo board's RP2350, or CPython through the FPGA prototype's USB UART): load, run, read/write memory |
| `fpga/` | FPGA prototype for an AMD Zynq UltraScale+ board (ALIENTEK DFZU2EG): the same RTL with a block RAM, a board top and a UART bridge so a PC can act as the host |
| `docs/` | `architecture.md`, `programming.md`, `verification.md`, `ai_use.md`, and the Tiny Tapeout datasheet `info.md` |
| `librelane_plugin_polyglot.py`, `flow/` | LibreLane step that powers the SRAM macro (Metal4 stripes on its supply columns) |

## Build and test

```sh
# Toolchain: any GCC with the rv32ec/ilp32e multilib, e.g. xPack riscv-none-elf-gcc 15.2
make -C sw                       # build the example programs
pip install -r test/requirements.txt
make -C test                     # RTL simulation with Icarus Verilog + cocotb
```

## Credits

- CPU: [Hazard3](https://github.com/Wren6991/Hazard3), Luke Wren, Apache-2.0 (one marked local change, see above).
- SRAM: IHP `RM_IHPSG13_1P_1024x32_c2_bm_bist` macro from the IHP Open PDK.
- SRAM power-grid approach (stripes on the macro's Metal4 supply columns) after Ken Pettit's [IHP SRAM walkthrough](https://kdp1965.github.io/ihp-um-janestreet-prism/ihp_sram_macro.html); our step is written from scratch.
- Project template: [Tiny Tapeout IHP Verilog template](https://github.com/TinyTapeout/ttihp-verilog-template).
