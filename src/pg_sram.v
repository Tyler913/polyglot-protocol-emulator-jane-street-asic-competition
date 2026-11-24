/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * pg_sram: single-port RAM with per-bit write mask: 3584 words of 32 bits
 * (14 KB), word address addr[11:0].
 *
 *   words    0..2047  RM_IHPSG13_1P_1024x64_c2_bm_bist (8 KB, used as 2048 x 32)
 *   words 2048..3071  RM_IHPSG13_1P_1024x32_c2_bm_bist (4 KB)
 *   words 3072..3583  RM_IHPSG13_1P_256x64_c2_bm_bist  (2 KB, used as 512 x 32)
 *   words 3584..4095  repeat words 3072..3583 (addr[9] is not decoded there)
 *
 * The size need not be a power of two: the top address bits pick the macro.
 * (Layouts up to 2026-11-21 had 4 KB in one 1024 x 32 macro, then 8 KB in
 * one 1024 x 64 macro; 12 KB until 2026-11-23.) RTL simulation uses a behavioural model with the same
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
  // Macro C: 2 KB 256 x 64, above macro A, used like macro A (bit 0 picks the
  // half).
  wire sela = ~addr[11];
  wire selb = addr[11] & ~addr[10];
  wire selc = addr[11] & addr[10];
  wire [63:0] bm = addr[0] ? {wmask, 32'd0} : {32'd0, wmask};
  localparam NA = 3 + 10 + 32 + 64;
  wire [NA-1:0] ia_in = {en & sela, en & sela & we, en & sela & ~we, addr[10:1], wdata, bm};
  wire [NA-1:0] ia;
  localparam NB = 3 + 10 + 32 + 32;
  wire [NB-1:0] ib_in = {en & selb, en & selb & we, en & selb & ~we, addr[9:0], wdata, wmask};
  wire [NB-1:0] ib;
  localparam NC = 3 + 8 + 32 + 64;
  wire [NC-1:0] ic_in = {en & selc, en & selc & we, en & selc & ~we, addr[8:1], wdata, bm};
  wire [NC-1:0] ic;
  wire [63:0] douta;
  wire [31:0] doutb;
  wire [63:0] doutc;
  reg  [2:0]  sel_q;
  always @(posedge clk) if (en && !we) sel_q <= {selc, selb, addr[0]};
  assign rdata = sel_q[2] ? (sel_q[0] ? doutc[63:32] : doutc[31:0])
               : sel_q[1] ? doutb
               : sel_q[0] ? douta[63:32] : douta[31:0];
  genvar i;
  generate
    for (i = 0; i < NA; i = i + 1) begin : g_ia
      sg13cmos5l_buf_4 sram_ibuf (.A(ia_in[i]), .X(ia[i]));
    end
    for (i = 0; i < NB; i = i + 1) begin : g_ib
      sg13cmos5l_buf_4 sram_ibuf (.A(ib_in[i]), .X(ib[i]));
    end
    for (i = 0; i < NC; i = i + 1) begin : g_ic
      sg13cmos5l_buf_4 sram_ibuf (.A(ic_in[i]), .X(ic[i]));
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

  RM_IHPSG13_1P_256x64_c2_bm_bist u_macro3 (
      .A_CLK       (clk),
      .A_MEN       (ic[NC-1]),
      .A_WEN       (ic[NC-2]),
      .A_REN       (ic[NC-3]),
      .A_ADDR      (ic[NC-4 -: 8]),
      .A_DIN       ({ic[95:64], ic[95:64]}),
      .A_DLY       (1'b1),
      .A_DOUT      (doutc),
      .A_BM        (ic[63:0]),
      .A_BIST_CLK  (1'b0),
      .A_BIST_EN   (1'b0),
      .A_BIST_MEN  (1'b0),
      .A_BIST_WEN  (1'b0),
      .A_BIST_REN  (1'b0),
      .A_BIST_ADDR (8'd0),
      .A_BIST_DIN  (64'd0),
      .A_BIST_BM   (64'd0)
  );
`else
  reg  [31:0] mem [0:3583];
  reg  [31:0] rdata_q;
  // same repeat as the macros
  wire [11:0] a = !addr[11] ? addr : !addr[10] ? {2'b10, addr[9:0]} : {3'b110, addr[8:0]};
  assign rdata = rdata_q;

  always @(posedge clk) begin
    if (en) begin
      if (we) mem[a] <= (mem[a] & ~wmask) | (wdata & wmask);
      else    rdata_q <= mem[a];
    end
  end
`endif

endmodule
