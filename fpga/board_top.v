/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * board_top: the Tiny Tapeout design on the ALIENTEK DFZU2EG board.
 *
 * - Clock: the 100 MHz PL oscillator (AE5/AF5) -> MMCM -> 50 MHz, the chip clock.
 * - Reset: the PL_RST button (active low) or the MMCM not locked, synchronized.
 * - The 24 chip pins are on the 3.3 V header J19 (see dfzu2eg.xdc).
 * - Host: either a PC through the board's PL USB UART (uart_host_bridge, FPGA
 *   only; tools/polyglot_host.py UartBridge), or an external MCU on the
 *   header (HOST_SCK/MOSI/CS_N on ui[7:5], as on the Tiny Tapeout demo board).
 *   The bridge owns the host port while its CS_N is low.
 * - LEDs: PL_LED1 blinks at ~1.5 Hz while the clock runs, PL_LED2 = HOST_IRQ.
 */

`default_nettype none

module board_top (
    input  wire       pl_clk_p,
    input  wire       pl_clk_n,
    input  wire       pl_rst_n,
    input  wire       uart_rx,
    output wire       uart_tx,
    input  wire [7:0] ui,
    output wire [7:0] uo,
    inout  wire [7:0] uio,
    output wire       led1,
    output wire       led2
);

  wire clk100, clk50_mmcm, clk50, fb, fb_buf, locked;

  IBUFDS u_clk_ibuf (.I(pl_clk_p), .IB(pl_clk_n), .O(clk100));

  // VCO = 100 MHz * 12 = 1200 MHz; 1200 / 24 = 50 MHz
  MMCME4_BASE #(
      .CLKIN1_PERIOD   (10.0),
      .CLKFBOUT_MULT_F (12.0),
      .DIVCLK_DIVIDE   (1),
      .CLKOUT0_DIVIDE_F(24.0)
  ) u_mmcm (
      .CLKIN1   (clk100),
      .CLKFBIN  (fb_buf),
      .CLKFBOUT (fb),
      .CLKFBOUTB(),
      .CLKOUT0  (clk50_mmcm),
      .CLKOUT0B (), .CLKOUT1 (), .CLKOUT1B(), .CLKOUT2(), .CLKOUT2B(),
      .CLKOUT3  (), .CLKOUT3B(), .CLKOUT4 (), .CLKOUT5(), .CLKOUT6(),
      .LOCKED   (locked),
      .PWRDWN   (1'b0),
      .RST      (1'b0)
  );
  BUFG u_fb_buf  (.I(fb), .O(fb_buf));
  BUFG u_clk_buf (.I(clk50_mmcm), .O(clk50));

  // Reset: asserted asynchronously, released synchronously, ~2.6 ms after the button
  reg [16:0] rst_cnt = 17'd0;
  reg [1:0]  btn_sync = 2'b00;
  reg        rst_n = 1'b0;
  always @(posedge clk50 or negedge locked) begin
    if (!locked) begin
      btn_sync <= 2'b00;
      rst_cnt  <= 17'd0;
      rst_n    <= 1'b0;
    end else begin
      btn_sync <= {btn_sync[0], pl_rst_n};
      if (!btn_sync[1]) begin
        rst_cnt <= 17'd0;
        rst_n   <= 1'b0;
      end else if (!rst_cnt[16]) begin
        rst_cnt <= rst_cnt + 17'd1;
      end else begin
        rst_n   <= 1'b1;
      end
    end
  end

  // Host port: PC bridge or header
  wire b_cs_n, b_sck, b_mosi;
  uart_host_bridge #(.CLK_HZ(50_000_000), .BAUD(115_200)) u_bridge (
      .clk(clk50), .rst(!rst_n), .rxd(uart_rx), .txd(uart_tx),
      .spi_cs_n(b_cs_n), .spi_sck(b_sck), .spi_mosi(b_mosi),
      .spi_miso(uo[7]), .irq(uo[6])
  );
  wire [7:0] ui_chip = b_cs_n ? ui : {1'b0, b_mosi, b_sck, ui[4:0]};

  // The chip
  wire [7:0] uio_in, uio_out, uio_oe;
  tt_um_tyler913_polyglot u_chip (
      .ui_in  (ui_chip),
      .uo_out (uo),
      .uio_in (uio_in),
      .uio_out(uio_out),
      .uio_oe (uio_oe),
      .ena    (1'b1),
      .clk    (clk50),
      .rst_n  (rst_n)
  );

  genvar i;
  generate
    for (i = 0; i < 8; i = i + 1) begin : g_uio
      IOBUF u_iobuf (.I(uio_out[i]), .T(!uio_oe[i]), .IO(uio[i]), .O(uio_in[i]));
    end
  endgenerate

  reg [24:0] blink = 25'd0;
  always @(posedge clk50) blink <= blink + 25'd1;
  assign led1 = blink[24];
  assign led2 = uo[6];

endmodule
