/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * pg_sram for the FPGA prototype: the same ports and timing as src/pg_sram.v
 * (read data after the clock edge on which en && !we was sampled), as one
 * inferred block RAM. All writes are byte-wide or wider, so the bit mask is
 * always byte-uniform: one bit per byte is used as the byte write enable.
 */

`default_nettype none

module pg_sram #(
    parameter AW = 12   // 16 KB of block RAM: a superset of the chip's 12 KB (no address repeat)
) (
    input  wire        clk,
    input  wire        en,
    input  wire        we,
    input  wire [AW-1:0] addr,
    input  wire [31:0] wdata,
    input  wire [31:0] wmask,
    output reg  [31:0] rdata
);

  (* ram_style = "block" *) reg [31:0] mem [0:(1<<AW)-1];
  integer i;

  always @(posedge clk) begin
    if (en) begin
      if (we) begin
        for (i = 0; i < 4; i = i + 1)
          if (wmask[8 * i]) mem[addr][8 * i +: 8] <= wdata[8 * i +: 8];
      end else begin
        rdata <= mem[addr];
      end
    end
  end

  wire _unused = &{1'b0, wmask};

endmodule
