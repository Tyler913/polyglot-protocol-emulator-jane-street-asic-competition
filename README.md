# Polyglot: a protocol emulator you program in plain C

An entry for the [Jane Street protocol emulator ASIC competition](https://blog.janestreet.com/protocol-emulator-asic-competition/) (Tiny Tapeout, IHP CMOS5L, 8x4 tiles).

A small RISC-V core runs ordinary C compiled with stock GCC or Clang. Protocol-agnostic, timed *wire engines* move bits on exact clock cycles. One header, `protocol.h`, is the whole toolchain. There is no custom assembler and no custom compiler.

**Status:** W3. The CPU, SRAM, host SPI loader, timed GPIO (cycle-exact edges from C), edge capture and Wire Engine v2 pass RTL simulation. UART, SPI and I2C, each in both roles, are built from the engines in a few lines of C and checked against independent peer models.

## Layout

| Path | Contents |
|---|---|
| `src/` | RTL. `pg_*.v` is ours; `hazard3_*` is the [Hazard3](https://github.com/Wren6991/Hazard3) RISC-V core by Luke Wren (Apache-2.0), vendored unchanged at commit `8af992930f71` |
| `sw/` | `protocol.h`, startup code, linker script, example programs (`examples/`) and test programs (`tests/`) |
| `test/` | cocotb tests (they build the firmware and run it on the RTL) |
| `tools/polyglot_host.py` | host driver (MicroPython on the demo board's RP2350): load, run, read/write memory |
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

- CPU: [Hazard3](https://github.com/Wren6991/Hazard3), Luke Wren, Apache-2.0.
- SRAM: IHP `RM_IHPSG13_1P_1024x32_c2_bm_bist` macro from the IHP Open PDK.
- SRAM power-grid approach (stripes on the macro's Metal4 supply columns) after Ken Pettit's [IHP SRAM walkthrough](https://kdp1965.github.io/ihp-um-janestreet-prism/ihp_sram_macro.html); our step is written from scratch.
- Project template: [Tiny Tapeout IHP Verilog template](https://github.com/TinyTapeout/ttihp-verilog-template).
