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
  // The macro's inputs are characterized only up to 0.6 ns transition, and
  // its pins sit along one 417 um edge, far from much of the logic. Each
  // input gets its own buf_4, so a weak logic gate never drives the long wire
  // to the pin by itself. (No dont-touch: hold repair must still be able to
  // add delay in front of these buffers.)
  wire [76:0] ib_in = {en, en & we, en & ~we, addr, wdata, wmask};
  wire [76:0] ib_out;
  genvar i;
  generate
    for (i = 0; i < 77; i = i + 1) begin : g_sram_ibuf
      sg13cmos5l_buf_4 sram_ibuf (.A(ib_in[i]), .X(ib_out[i]));
    end
  endgenerate

  RM_IHPSG13_1P_1024x32_c2_bm_bist u_macro (
      .A_CLK       (clk),
      .A_MEN       (ib_out[76]),
      .A_WEN       (ib_out[75]),
      .A_REN       (ib_out[74]),
      .A_ADDR      (ib_out[73:64]),
      .A_DIN       (ib_out[63:32]),
      .A_DLY       (1'b1),
      .A_DOUT      (rdata),
      .A_BM        (ib_out[31:0]),
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
