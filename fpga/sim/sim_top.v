// Simulation top for the FPGA bridge: uart_host_bridge driving the chip's host
// port (as in board_top, without the clock and pad primitives).
`default_nettype none
`timescale 1ns / 1ps
module sim_top (
    input  wire clk,
    input  wire rst_n,
    input  wire rxd,
    output wire txd,
    output wire [7:0] uo_out
);
  wire cs_n, sck, mosi;
  wire [7:0] uio_out, uio_oe;
  tt_um_tyler913_polyglot u_chip (
      .ui_in  ({cs_n, mosi, sck, 5'd0}),
      .uo_out (uo_out),
      .uio_in (8'hFF),
      .uio_out(uio_out),
      .uio_oe (uio_oe),
      .ena    (1'b1),
      .clk    (clk),
      .rst_n  (rst_n)
  );
  uart_host_bridge #(.CLK_HZ(50_000_000), .BAUD(2_000_000)) u_bridge (
      .clk(clk), .rst(!rst_n), .rxd(rxd), .txd(txd),
      .spi_cs_n(cs_n), .spi_sck(sck), .spi_mosi(mosi),
      .spi_miso(uo_out[7]), .irq(uo_out[6])
  );
endmodule
