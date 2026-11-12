/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * pg_sysio: system registers seen by the CPU at 0x8000_0000.
 * Word access only. Reads are combinational in the AHB data phase.
 *
 *   0x00 ID        RO  0x5047_0002 ("PG", version 2)
 *   0x04 NOW       RO  free-running cycle counter
 *   0x10 GPIO_OUT  RW  output value, one bit per pin (see pin numbering)
 *   0x14 GPIO_OE   RW  output enable for bidirectional pins (bits 7:0)
 *   0x18 GPIO_IN   RO  synchronized pin inputs
 *   0x20 HOST_MBOX RW  32-bit mailbox (host access added later)
 *
 * Pin numbering (shared with protocol.h):
 *   0..7   uio[0..7]  bidirectional
 *   8..13  uo[0..5]   outputs
 *   16..20 ui[0..4]   inputs
 *
 * This is the bring-up block. Timed GPIO and the wire engines replace the
 * plain output register in later versions.
 */

`default_nettype none

module pg_sysio (
    input  wire        clk,
    input  wire        rst,

    // Register port (AHB data phase)
    input  wire        valid,
    input  wire        write,
    input  wire [7:0]  addr,
    input  wire [31:0] wdata,
    output reg  [31:0] rdata,
    output wire        ready,

    // Pins
    output wire [7:0]  uio_out,
    output wire [7:0]  uio_oe,
    output wire [5:0]  uo_out,
    input  wire [7:0]  uio_in,
    input  wire [4:0]  ui_in
);

  localparam [31:0] ID = 32'h5047_0002;

  reg [31:0] now;
  reg [13:0] gpio_out;
  reg [7:0]  gpio_oe;
  reg [31:0] mbox;

  assign ready = 1'b1;

  // Two-flop input synchronizers
  reg [12:0] in_s1, in_s2;
  always @(posedge clk) begin
    in_s1 <= {ui_in, uio_in};
    in_s2 <= in_s1;
  end
  wire [31:0] gpio_in = {11'd0, in_s2[12:8], 8'd0, in_s2[7:0]};

  always @(posedge clk) begin
    if (rst) now <= 32'd0;
    else     now <= now + 32'd1;
  end

  always @(posedge clk) begin
    if (rst) begin
      gpio_out <= 14'd0;
      gpio_oe  <= 8'd0;
      mbox     <= 32'd0;
    end else if (valid && write && ready) begin
      case (addr)
        8'h10: gpio_out <= wdata[13:0];
        8'h14: gpio_oe  <= wdata[7:0];
        8'h20: mbox     <= wdata;
        default: ;
      endcase
    end
  end

  always @(*) begin
    case (addr)
      8'h00:   rdata = ID;
      8'h04:   rdata = now;
      8'h10:   rdata = {18'd0, gpio_out};
      8'h14:   rdata = {24'd0, gpio_oe};
      8'h18:   rdata = gpio_in;
      8'h20:   rdata = mbox;
      default: rdata = 32'd0;
    endcase
  end

  assign uio_out = gpio_out[7:0];
  assign uio_oe  = gpio_oe;
  assign uo_out  = gpio_out[13:8];

endmodule
