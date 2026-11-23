/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * pg_sram: single-port RAM with per-bit write mask: 3072 words of 32 bits
 * (12 KB), word address addr[11:0].
 *
 *   words    0..2047  RM_IHPSG13_1P_1024x64_c2_bm_bist (8 KB, used as 2048 x 32)
 *   words 2048..3071  RM_IHPSG13_1P_1024x32_c2_bm_bist (4 KB)
 *   words 3072..4095  repeat words 2048..3071 (addr[10] is not decoded there)
 *
 * The size need not be a power of two: the top address bit picks the macro.
 * (Layouts up to 2026-11-21 had 4 KB in one 1024 x 32 macro, then 8 KB in
 * one 1024 x 64 macro.) RTL simulation uses a behavioural model with the same
 * timing and the same address repeat: the read data appears after the clock
 * edge on which en && !we was sampled.
 */

`default_nettype none

module pg_sram (
    input  wire        clk,
    input  wire        en,
    input  wire        we,
    input  wire [11:0] addr,
    input  wire [31:0] wdata,
    input  wire [31:0] wmask,   // 1 = write this bit
    output wire [31:0] rdata
);

`ifdef SYNTHESIS
  // The macros' inputs are characterized only up to 0.6 ns transition, and
  // their pins sit along long edges, far from much of the logic. Each input
  // pin gets its own buf_4, so a weak logic gate never drives the long wire to
  // the pin by itself. (No dont-touch: the resizer must still be able to
  // buffer the nets in front of them and add hold delay.)
  //
  // Macro A: 8 KB as one 1024 x 64 macro (784 x 336 um): as short as the 4 KB
  // macro, so the top half of the tile stays free for horizontal wiring, and
  // as fast (the 2048 x 32 macro is 627 um tall and 1.7 ns slower). Word
  // address bit 0 picks the half: writes drive both halves with the same data
  // and mask one half; reads take the half selected in the access cycle.
  // Macro B: 4 KB 1024 x 32 (417 x 336 um), left of macro A.
  wire selb = addr[11];
  wire [63:0] bm = addr[0] ? {wmask, 32'd0} : {32'd0, wmask};
  localparam NA = 3 + 10 + 32 + 64;
  wire [NA-1:0] ia_in = {en & ~selb, en & ~selb & we, en & ~selb & ~we, addr[10:1], wdata, bm};
  wire [NA-1:0] ia;
  localparam NB = 3 + 10 + 32 + 32;
  wire [NB-1:0] ib_in = {en & selb, en & selb & we, en & selb & ~we, addr[9:0], wdata, wmask};
  wire [NB-1:0] ib;
  wire [63:0] douta;
  wire [31:0] doutb;
  reg  [1:0]  sel_q;
  always @(posedge clk) if (en && !we) sel_q <= {selb, addr[0]};
  assign rdata = sel_q[1] ? doutb : sel_q[0] ? douta[63:32] : douta[31:0];
  genvar i;
  generate
    for (i = 0; i < NA; i = i + 1) begin : g_ia
      sg13cmos5l_buf_4 sram_ibuf (.A(ia_in[i]), .X(ia[i]));
    end
    for (i = 0; i < NB; i = i + 1) begin : g_ib
      sg13cmos5l_buf_4 sram_ibuf (.A(ib_in[i]), .X(ib[i]));
    end
  endgenerate

  RM_IHPSG13_1P_1024x64_c2_bm_bist u_macro (
      .A_CLK       (clk),
      .A_MEN       (ia[NA-1]),
      .A_WEN       (ia[NA-2]),
      .A_REN       (ia[NA-3]),
      .A_ADDR      (ia[NA-4 -: 10]),
      .A_DIN       ({ia[95:64], ia[95:64]}),
      .A_DLY       (1'b1),
      .A_DOUT      (douta),
      .A_BM        (ia[63:0]),
      .A_BIST_CLK  (1'b0),
      .A_BIST_EN   (1'b0),
      .A_BIST_MEN  (1'b0),
      .A_BIST_WEN  (1'b0),
      .A_BIST_REN  (1'b0),
      .A_BIST_ADDR (10'd0),
      .A_BIST_DIN  (64'd0),
      .A_BIST_BM   (64'd0)
  );

  RM_IHPSG13_1P_1024x32_c2_bm_bist u_macro2 (
      .A_CLK       (clk),
      .A_MEN       (ib[NB-1]),
      .A_WEN       (ib[NB-2]),
      .A_REN       (ib[NB-3]),
      .A_ADDR      (ib[NB-4 -: 10]),
      .A_DIN       (ib[63:32]),
      .A_DLY       (1'b1),
      .A_DOUT      (doutb),
      .A_BM        (ib[31:0]),
      .A_BIST_CLK  (1'b0),
      .A_BIST_EN   (1'b0),
      .A_BIST_MEN  (1'b0),
      .A_BIST_WEN  (1'b0),
      .A_BIST_REN  (1'b0),
      .A_BIST_ADDR (10'd0),
      .A_BIST_DIN  (32'd0),
      .A_BIST_BM   (32'd0)
  );
`else
  reg  [31:0] mem [0:3071];
  reg  [31:0] rdata_q;
  wire [11:0] a = addr[11] ? {2'b10, addr[9:0]} : addr;   // same repeat as the macros
  assign rdata = rdata_q;

  always @(posedge clk) begin
    if (en) begin
      if (we) mem[a] <= (mem[a] & ~wmask) | (wdata & wmask);
      else    rdata_q <= mem[a];
    end
  end
`endif

endmodule
