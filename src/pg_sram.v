/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * pg_sram: 1024 x 32 single-port RAM (4 KB) with per-bit write mask.
 *
 * Synthesis uses the IHP macro RM_IHPSG13_1P_1024x32_c2_bm_bist. RTL
 * simulation uses a behavioural model with the same timing: the read data
 * appears after the clock edge on which en && !we was sampled.
 */

`default_nettype none

module pg_sram (
    input  wire        clk,
    input  wire        en,
    input  wire        we,
    input  wire [9:0]  addr,
    input  wire [31:0] wdata,
    input  wire [31:0] wmask,   // 1 = write this bit
    output wire [31:0] rdata
);

`ifdef SYNTHESIS
  RM_IHPSG13_1P_1024x32_c2_bm_bist u_macro (
      .A_CLK       (clk),
      .A_MEN       (en),
      .A_WEN       (en & we),
      .A_REN       (en & ~we),
      .A_ADDR      (addr),
      .A_DIN       (wdata),
      .A_DLY       (1'b1),
      .A_DOUT      (rdata),
      .A_BM        (wmask),
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
  reg [31:0] mem [0:1023];
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
