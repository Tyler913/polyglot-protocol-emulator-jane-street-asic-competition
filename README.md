# Polyglot: a protocol emulator you program in plain C

An entry for the [Jane Street protocol emulator ASIC competition](https://blog.janestreet.com/protocol-emulator-asic-competition/) (Tiny Tapeout, IHP CMOS5L, 8x4 tiles).

A small RISC-V core (RV32EC) runs ordinary C compiled with stock GCC or Clang. Four protocol-agnostic, timed *wire engines* move bits on exact clock cycles. One header, `protocol.h`, is the whole toolchain. There is no custom assembler and no custom compiler.

**Status:** early development. There is no RTL yet.
