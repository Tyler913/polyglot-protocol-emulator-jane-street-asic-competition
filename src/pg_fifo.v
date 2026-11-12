/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * pg_fifo: small synchronous FIFO built from flops, 2^AW entries.
 * dout shows the head entry whenever empty is low (first-word fall-through).
 * A push when full and a pop when empty are ignored.
 */

`default_nettype none

module pg_fifo #(
    parameter W  = 32,
    parameter AW = 2
) (
    input  wire          clk,
    input  wire          rst,
    input  wire          push,
    input  wire [W-1:0]  din,
    input  wire          pop,
    output wire [W-1:0]  dout,
    output wire [AW:0]   level,
    output wire          full,
    output wire          empty
);

  reg [W-1:0] mem [0:(1<<AW)-1];
  reg [AW:0]  wp, rp;

  assign level = wp - rp;
  assign full  = level[AW];
  assign empty = (wp == rp);
  assign dout  = mem[rp[AW-1:0]];

  always @(posedge clk) begin
    if (push && !full) mem[wp[AW-1:0]] <= din;
  end

  always @(posedge clk) begin
    if (rst) begin
      wp <= {(AW+1){1'b0}};
      rp <= {(AW+1){1'b0}};
    end else begin
      if (push && !full) wp <= wp + 1'b1;
      if (pop && !empty) rp <= rp + 1'b1;
    end
  end

endmodule
