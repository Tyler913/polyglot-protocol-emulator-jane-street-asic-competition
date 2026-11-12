<!---
This file is used to generate your project datasheet. Please fill in the information below and delete any unused
sections.

You can also include images in this folder and reference them in the markdown. Each image must be less than
512 kb in size, and the combined size of all images must be less than 1 MB.
-->

## How it works

Polyglot is a protocol emulator that you program in plain C. A Hazard3 RISC-V core (RV32EC, the core used in the RP2350) runs a program compiled with stock GCC or Clang from 4 KB of on-chip SRAM. The program drives the protocol pins through memory-mapped registers declared in one header, `protocol.h`.

Status: bring-up version. The CPU, SRAM, host loader and plain GPIO work in RTL simulation. Timed GPIO and the protocol-agnostic wire engines come next.

### Host interface (SPI target, mode 0, SCK <= clk/8)

| Command | Bytes | Meaning |
|---|---|---|
| WRITE | `01 AH AL D0 D1 ...` | write bytes to memory starting at byte address AH:AL |
| READ | `02 AH AL xx` then read | read bytes starting at AH:AL (after one dummy byte) |
| CTRL | `03 V` | bit 0 = RUN (1 releases the CPU from reset) |
| STATUS | `04 xx` | the second byte returns status (bit 0 = RUN) |

Load a program while RUN = 0, then set RUN = 1. The program starts at address 0.

### Pin numbering in `protocol.h`

| Pins | Number |
|---|---|
| uio[0..7] (bidirectional) | 0..7 |
| uo[0..5] (outputs) | 8..13 |
| ui[0..4] (inputs) | 16..20 |

## How to test

1. Build the firmware: `make -C sw` (needs `riscv-none-elf-gcc` or any GCC with the rv32ec/ilp32e multilib).
2. Hold RUN = 0, write `sw/build/blink.bin` to address 0 with the WRITE command, then send CTRL `01`.
3. uo[0] toggles every 100 clock cycles.

## External hardware

None for the bring-up test. A logic analyser on uo[0] shows the blink.
