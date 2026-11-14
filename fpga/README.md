# FPGA prototype

Target board: ALIENTEK DFZU2EG (AMD Zynq UltraScale+ XCZU2EG-SFVC784-1); only
the programmable logic is used. The chip RTL in `src/` is used unchanged,
except that the IHP SRAM macro is replaced by a block RAM.

| File | Purpose |
|---|---|
| `board_top.v` | 100 MHz board clock -> MMCM -> 50 MHz; reset button; chip pins on header J19; the UART bridge; LEDs |
| `pg_sram_fpga.v` | replaces `src/pg_sram.v`: one block RAM, same ports and read timing |
| `uart_host_bridge.v` | FPGA-only helper: a PC drives the chip's host SPI port through the board's PL USB UART |
| `dfzu2eg.xdc` | pins, read from the board schematic (ATK_DFZU2EG_ZU4EV V1.4) and checked against Vivado's pin functions |
| `build.tcl` | full build: `vivado -mode batch -source build.tcl` -> `out/polyglot_dfzu2eg.bit` |
| `synth.tcl` | the chip top alone, out of context (size and timing without board pins) |
| `sim/` | cocotb simulation of the bridge with the chip RTL: `make -C fpga/sim` |

## Pins

Header J19 (3.3 V): odd pins 5-19 = `ui[0..7]` (17/19/... carry HOST_MOSI and
HOST_CS_N on ui[6], ui[7]; HOST_SCK ui[5] is pin 15), even pins 6-20 =
`uo[0..7]` (HOST_IRQ uo[6] pin 18, HOST_MISO uo[7] pin 20), pins 21-28 =
`uio[0..7]` (weak pull-ups on). Pin 3 is 3.3 V, pins 2/4 GND. LED1 blinks
while the clock runs; LED2 shows HOST_IRQ. PL_RST resets the chip.

## Bring-up

1. Set the boot switch SW3 to JTAG mode (all off = 0000), connect a JTAG cable
   to J12 and a USB-C cable to J4 (the PL UART, CH340E), power on.
2. Program `out/polyglot_dfzu2eg.bit` with the Vivado Hardware Manager. LED1
   should blink at about 1.5 Hz.
3. On the PC (`pip install pyserial`), with the CH340 port name:

   ```python
   import sys; sys.path.insert(0, "tools")
   from polyglot_host import Polyglot, UartBridge
   chip = Polyglot(UartBridge("COM5"))        # Linux: /dev/ttyUSB0
   chip.run_program(open("sw/build/hello.bin", "rb").read())
   print(chip.shared(1))                      # counts up; LED2 blinks at 1 Hz
   ```

   `run_program` reads the image back before it starts the CPU.

Any other program then runs the same way, with its peers on J19 (for example a
USB-UART adapter on uo[0]/ui[4] for `uart_echo`; check each example's pin list).

## Results (Vivado 2025.2, 2026-11-14)

- Chip top alone, out of context (`synth.tcl`): 5,119 LUTs (10.8 %), 2,904
  flip-flops (3.1 %), 1 block RAM tile, no DSPs; setup slack +12.42 ns at
  50 MHz (critical path 7.56 ns, CPU register bypass to the instruction
  decompressor), hold slack +0.018 ns. *(FPGA place and route)*
- Board build (`build.tcl`, with the UART bridge): 5,275 LUTs (11.2 %), 3,091
  flip-flops, 1 block RAM tile; setup slack +12.40 ns, hold slack +0.035 ns at
  50 MHz; no critical warnings. Each port's package pin and pin function are
  listed in `out/board_pins.txt` and match the schematic. *(FPGA place and route)*
- Simulation of the bridge with the chip RTL: a program loaded, read back and
  run over the UART; HOST_IRQ and a counter read while the CPU runs.
  *(RTL simulation)*
- Not yet run on the board.
