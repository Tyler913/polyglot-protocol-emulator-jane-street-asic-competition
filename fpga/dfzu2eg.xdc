# ALIENTEK DFZU2EG (XCZU2EG-SFVC784-1), schematic ATK_DFZU2EG_ZU4EV V1.4.
# Pins read from sheets 3 (bank 44), 4 (banks 25/26), 10 (bank 64) and 30 (J19).

# 100 MHz differential PL clock, bank 64 (VCCO 1.2 V), AC-coupled
set_property PACKAGE_PIN AE5 [get_ports pl_clk_p]
set_property PACKAGE_PIN AF5 [get_ports pl_clk_n]
set_property IOSTANDARD DIFF_SSTL12 [get_ports {pl_clk_p pl_clk_n}]
create_clock -name pl_clk -period 10.000 [get_ports pl_clk_p]

# Bank 44 (3.3 V): PL_RST button (pull-up, low when pressed), PL_LED1/2 (high = on)
set_property -dict {PACKAGE_PIN AH11 IOSTANDARD LVCMOS33} [get_ports pl_rst_n]
set_property -dict {PACKAGE_PIN AE10 IOSTANDARD LVCMOS33} [get_ports led1]
set_property -dict {PACKAGE_PIN AF10 IOSTANDARD LVCMOS33} [get_ports led2]
# PL USB UART (CH340E U4 on USB-C J4, sheet 28): FPGA TX -> CH340 RXD, CH340 TXD -> FPGA RX
set_property -dict {PACKAGE_PIN AB10 IOSTANDARD LVCMOS33} [get_ports uart_tx]
set_property -dict {PACKAGE_PIN AB9  IOSTANDARD LVCMOS33 PULLUP true} [get_ports uart_rx]

# Header J19 (3.3 V, banks 25/26). Odd pins 5..19: ui[0..7]; even pins 6..20: uo[0..7];
# pins 21..28: uio[0..7]. Pins 1/2 VCC5V/GND side, 3 VCC_3V3, 4 GND.
#                                              J19  net
set_property PACKAGE_PIN D11 [get_ports {ui[0]}]  ;# 5   B25_L8_N
set_property PACKAGE_PIN E12 [get_ports {ui[1]}]  ;# 7   B25_L8_P
set_property PACKAGE_PIN A10 [get_ports {ui[2]}]  ;# 9   B25_L10_N
set_property PACKAGE_PIN B11 [get_ports {ui[3]}]  ;# 11  B25_L10_P
set_property PACKAGE_PIN C12 [get_ports {ui[4]}]  ;# 13  B25_L12_N
set_property PACKAGE_PIN D12 [get_ports {ui[5]}]  ;# 15  B25_L12_P  HOST_SCK
set_property PACKAGE_PIN A14 [get_ports {ui[6]}]  ;# 17  B26_L2_N   HOST_MOSI
set_property PACKAGE_PIN B14 [get_ports {ui[7]}]  ;# 19  B26_L2_P   HOST_CS_N
set_property PACKAGE_PIN D10 [get_ports {uo[0]}]  ;# 6   B25_L7_N
set_property PACKAGE_PIN E10 [get_ports {uo[1]}]  ;# 8   B25_L7_P
set_property PACKAGE_PIN B10 [get_ports {uo[2]}]  ;# 10  B25_L9_N
set_property PACKAGE_PIN C11 [get_ports {uo[3]}]  ;# 12  B25_L9_P
set_property PACKAGE_PIN A11 [get_ports {uo[4]}]  ;# 14  B25_L11_N
set_property PACKAGE_PIN A12 [get_ports {uo[5]}]  ;# 16  B25_L11_P
set_property PACKAGE_PIN A15 [get_ports {uo[6]}]  ;# 18  B26_L1_N   HOST_IRQ
set_property PACKAGE_PIN B15 [get_ports {uo[7]}]  ;# 20  B26_L1_P   HOST_MISO
set_property PACKAGE_PIN C13 [get_ports {uio[0]}] ;# 21  B26_L4_N
set_property PACKAGE_PIN A13 [get_ports {uio[1]}] ;# 22  B26_L3_N
set_property PACKAGE_PIN C14 [get_ports {uio[2]}] ;# 23  B26_L4_P
set_property PACKAGE_PIN B13 [get_ports {uio[3]}] ;# 24  B26_L3_P
set_property PACKAGE_PIN E13 [get_ports {uio[4]}] ;# 25  B26_L6_N
set_property PACKAGE_PIN D14 [get_ports {uio[5]}] ;# 26  B26_L5_N
set_property PACKAGE_PIN E14 [get_ports {uio[6]}] ;# 27  B26_L6_P
set_property PACKAGE_PIN D15 [get_ports {uio[7]}] ;# 28  B26_L5_P
set_property IOSTANDARD LVCMOS33 [get_ports {ui[*] uo[*] uio[*]}]
set_property DRIVE 8 [get_ports {uo[*] uio[*]}]
set_property SLEW SLOW [get_ports {uo[*] uio[*]}]
# Pull-ups on the bidirectional pins (I2C, 1-Wire and SWDIO idle high; the chip
# relies on external pull-ups for open-drain lines, as on a real board)
set_property PULLUP true [get_ports {uio[*]}]
# The host CS_N idles high when nothing is connected
set_property PULLUP true [get_ports {ui[7]}]

# The chip pins are asynchronous to the board clock (synchronized inside)
set_false_path -from [get_ports {ui[*] uio[*] pl_rst_n uart_rx}]
set_false_path -to   [get_ports {uo[*] uio[*] led1 led2 uart_tx}]

set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
