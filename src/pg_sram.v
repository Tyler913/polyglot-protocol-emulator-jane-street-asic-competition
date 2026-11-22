/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * pg_sram: single-port RAM with per-bit write mask: 2^AW words of 32 bits
 * (AW = 11: 8 KB).
 *
 * Synthesis uses the IHP macro RM_IHPSG13_1P_1024x64_c2_bm_bist (AW = 11;
 * the 4 KB layouts up to 2026-11-21 used the 1024 x 32 macro). RTL
 * simulation uses a behavioural model with the same timing: the read data
 * appears after the clock edge on which en && !we was sampled.
 */

`default_nettype none

module pg_sram #(
    parameter AW = 11
) (
    input  wire        clk,
    input  wire        en,
    input  wire        we,
    input  wire [AW-1:0] addr,
    input  wire [31:0] wdata,
    input  wire [31:0] wmask,   // 1 = write this bit
    output wire [31:0] rdata
);

`ifdef SYNTHESIS
  // The macro's inputs are characterized only up to 0.6 ns transition, and
  // its pins sit along one 784 um edge, far from much of the logic. Each
  // input gets its own buf_4, so a weak logic gate never drives the long wire
  // to the pin by itself. (No dont-touch: hold repair must still be able to
  // add delay in front of these buffers.)
  //
  // 8 KB as one 1024 x 64 macro (784 x 336 um): as short as the 4 KB macro, so
  // the top half of the tile stays free for horizontal wiring, and as fast
  // (the 2048 x 32 macro is 627 um tall and 1.7 ns slower). Word address bit 0
  // picks the half: writes drive both halves with the same data and mask one
  // half; reads take the half selected in the access cycle.
  wire [63:0] bm = addr[0] ? {wmask, 32'd0} : {32'd0, wmask};
  localparam NB = 3 + 10 + 32 + 64;
  wire [NB-1:0] ib_in = {en, en & we, en & ~we, addr[10:1], wdata, bm};
  wire [NB-1:0] ib_out;
  wire [63:0] dout;
  reg         hi_q;
  always @(posedge clk) if (en && !we) hi_q <= addr[0];
  assign rdata = hi_q ? dout[63:32] : dout[31:0];
  genvar i;
  generate
    for (i = 0; i < NB; i = i + 1) begin : g_sram_ibuf
      sg13cmos5l_buf_4 sram_ibuf (.A(ib_in[i]), .X(ib_out[i]));
    end
  endgenerate

  RM_IHPSG13_1P_1024x64_c2_bm_bist u_macro (
      .A_CLK       (clk),
      .A_MEN       (ib_out[NB-1]),
      .A_WEN       (ib_out[NB-2]),
      .A_REN       (ib_out[NB-3]),
      .A_ADDR      (ib_out[NB-4 -: 10]),
      .A_DIN       ({ib_out[95:64], ib_out[95:64]}),
      .A_DLY       (1'b1),
      .A_DOUT      (dout),
      .A_BM        (ib_out[63:0]),
      .A_BIST_CLK  (1'b0),
      .A_BIST_EN   (1'b0),
      .A_BIST_MEN  (1'b0),
      .A_BIST_WEN  (1'b0),
      .A_BIST_REN  (1'b0),
      .A_BIST_ADDR (10'd0),
      .A_BIST_DIN  (64'd0),
      .A_BIST_BM   (64'd0)
  );
`else
  reg [31:0] mem [0:(1<<AW)-1];
  reg [31:0] rdata_q;
  assign rdata = rdata_q;

  always @(posedge clk) begin
    if (en) begin
      if (we) mem[addr] <= (mem[addr] & ~wmask) | (wdata & wmask);
      else    rdata_q   <= mem[addr];
    end
  end
`endif

endmodule
