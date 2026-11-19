/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * pg_rmii: a 100 Mbit/s Ethernet port for an RMII PHY (e.g. LAN8720). RMII
 * moves two bits per 50 MHz REF_CLK cycle in each direction; the chip clock
 * must be that REF_CLK (one 50 MHz source for both). The block does only the
 * bit layer: preamble/SFD, dibit shifting, CRC-32 (two bits per cycle) and
 * the inter-frame gap. Frames (MAC addresses, type, payload, padding) are C.
 *
 * Registers (word offsets in the block's page):
 *   0x00 TX     W  push a word: 4 frame bytes, first byte in bits 7:0 (blocks
 *                  while the TX FIFO is full)
 *   0x04 RX     R  pop the next RX word (blocks while the RX FIFO is empty)
 *   0x08 RXHEAD R  peek at the next RX entry without popping it (blocks while
 *                  empty): [31] it is the end of a frame, then [16] FCS good,
 *                  [15:0] frame length in bytes, FCS included; a data word
 *                  reads [31] = 0. Pop the end entry with a read of RX.
 *   0x0C TXLEN W  start a frame of n bytes (FCS not counted, n >= 1): preamble
 *                  and SFD go out at once (32 cycles), then the bytes from the
 *                  TX FIFO, then the FCS; C must keep the FIFO fed (a word
 *                  every 16 cycles).
 *   0x10 CTRL   RW [0] enable (the block then owns uo[0..2])
 *   0x14 STATUS R  [0] TX busy (frame or gap), [1] TX underrun (sticky),
 *                  [2] RX overflow (sticky), [10:8] RX entries, [17:16] TX
 *                  words free; write 1s to clear the sticky bits
 * Pins: TXD0 uo[0], TXD1 uo[1], TX_EN uo[2]; RXD0 ui[0], RXD1 ui[1],
 * CRS_DV ui[2] (inputs through the chip's synchronizers: a fixed delay).
 * The RX side treats the end of CRS_DV as the end of the frame.
 */

`default_nettype none

module pg_rmii (
    input  wire        clk,
    input  wire        rst,
    input  wire        valid,
    input  wire        write,
    input  wire [7:0]  addr,
    input  wire [31:0] wdata,
    output reg  [31:0] rdata,
    output reg         ready,
    input  wire [1:0]  rxd,
    input  wire        crs_dv,
    output reg  [1:0]  txd,
    output reg         tx_en,
    output wire        enabled,
    output wire        rx_avail
);

  localparam [3:0] R_TX = 4'd0, R_RX = 4'd1, R_RXHEAD = 4'd2, R_TXLEN = 4'd3, R_CTRL = 4'd4, R_STATUS = 4'd5;
  wire [3:0] ra  = addr[5:2];
  wire       sel = valid && (addr[1:0] == 2'd0) && (addr[7:6] == 2'd0);

  reg en;
  assign enabled = en;
  wire srst = rst || !en;

  // Reflected CRC-32 (polynomial 0xEDB88320), one bit per step, LSB first
  function [31:0] crc_bit(input [31:0] c, input b);
    crc_bit = (c[0] ^ b) ? ((c >> 1) ^ 32'hEDB8_8320) : (c >> 1);
  endfunction
  function [31:0] crc_dibit(input [31:0] c, input [1:0] d);
    crc_dibit = crc_bit(crc_bit(c, d[0]), d[1]);
  endfunction

  // ---------------------------------------------------------------------
  // TX FIFO: 2 words
  // ---------------------------------------------------------------------
  reg [31:0] tf_mem [0:1];
  reg        tf_wp, tf_rp;
  reg [1:0]  tf_n;
  wire       tf_full  = tf_n == 2'd2;
  wire       tf_empty = tf_n == 2'd0;
  wire       acc      = sel && ready;
  wire       tf_push  = acc && write && ra == R_TX;
  reg        tf_pop;

  always @(posedge clk) begin
    if (srst) begin
      tf_wp <= 1'b0; tf_rp <= 1'b0; tf_n <= 2'd0;
    end else begin
      if (tf_push) begin tf_mem[tf_wp] <= wdata; tf_wp <= !tf_wp; end
      if (tf_pop) tf_rp <= !tf_rp;
      tf_n <= tf_n + {1'b0, tf_push} - {1'b0, tf_pop};
    end
  end

  // ---------------------------------------------------------------------
  // TX: preamble + SFD (32 dibits), data, FCS (16 dibits), gap (48 cycles)
  // ---------------------------------------------------------------------
  localparam [2:0] T_IDLE = 3'd0, T_PRE = 3'd1, T_DATA = 3'd2, T_FCS = 3'd3, T_GAP = 3'd4;
  reg [2:0]  ts;
  reg [5:0]  tcnt;          // dibits left in the preamble / FCS, gap cycles
  reg [15:0] tleft;         // data dibits left
  reg [31:0] tsh;           // current word
  reg [3:0]  tpos;          // dibit position in the word
  reg        tword;         // tsh holds a word
  reg [31:0] tcrc;
  reg        underrun;
  wire       start = acc && write && ra == R_TXLEN && ts == T_IDLE && wdata[15:0] != 16'd0;

  always @(*) begin
    tf_pop = 1'b0;
    if (!srst && ts == T_DATA && (!tword || tpos == 4'd15) && tleft > (tword ? 16'd1 : 16'd0) && !tf_empty)
      tf_pop = 1'b1;
  end

  always @(posedge clk) begin
    if (srst) begin
      ts <= T_IDLE; tx_en <= 1'b0; txd <= 2'b00; tword <= 1'b0; underrun <= 1'b0;
      tcnt <= 6'd0; tleft <= 16'd0; tpos <= 4'd0; tcrc <= 32'hFFFF_FFFF; tsh <= 32'd0;
    end else begin
      if (acc && write && ra == R_STATUS && wdata[1]) underrun <= 1'b0;
      case (ts)
        T_IDLE: begin
          tx_en <= 1'b0; txd <= 2'b00;
          if (start) begin
            ts <= T_PRE; tcnt <= 6'd31; tleft <= {wdata[13:0], 2'b00};
            tcrc <= 32'hFFFF_FFFF; tword <= 1'b0; tpos <= 4'd0;
          end
        end
        T_PRE: begin
          tx_en <= 1'b1;
          txd   <= (tcnt == 6'd0) ? 2'b11 : 2'b01;   // 0x55 x 7, then 0xD5
          if (tcnt == 6'd0) ts <= T_DATA; else tcnt <= tcnt - 6'd1;
        end
        T_DATA: begin
          // the dibit to send: from the current word, or the word popped now
          if (tword && tpos != 4'd15) begin
            txd  <= tsh[2*tpos +: 2];
            tcrc <= crc_dibit(tcrc, tsh[2*tpos +: 2]);
            tpos <= tpos + 4'd1;
            tleft <= tleft - 16'd1;
            if (tleft == 16'd1) begin ts <= T_FCS; tcnt <= 6'd15; end
          end else if (tword && tpos == 4'd15) begin
            txd  <= tsh[31:30];
            tcrc <= crc_dibit(tcrc, tsh[31:30]);
            tleft <= tleft - 16'd1;
            tword <= 1'b0;
            if (tleft == 16'd1) begin ts <= T_FCS; tcnt <= 6'd15; end
            else if (tf_pop) begin tsh <= tf_mem[tf_rp]; tword <= 1'b1; tpos <= 4'd0; end
            else begin underrun <= 1'b1; tx_en <= 1'b0; txd <= 2'b00; ts <= T_GAP; tcnt <= 6'd48; end
          end else if (tf_pop) begin                // first word of the frame
            txd  <= tf_mem[tf_rp][1:0];
            tcrc <= crc_dibit(tcrc, tf_mem[tf_rp][1:0]);
            tsh  <= tf_mem[tf_rp]; tword <= 1'b1; tpos <= 4'd1;
            tleft <= tleft - 16'd1;
            if (tleft == 16'd1) begin ts <= T_FCS; tcnt <= 6'd15; end
          end else begin
            underrun <= 1'b1; tx_en <= 1'b0; txd <= 2'b00; ts <= T_GAP; tcnt <= 6'd48;
          end
        end
        T_FCS: begin
          txd <= ~tcrc[2*(15 - tcnt[3:0]) +: 2];
          if (tcnt == 6'd0) begin ts <= T_GAP; tcnt <= 6'd48; end else tcnt <= tcnt - 6'd1;
        end
        default: begin                              // T_GAP: 96 bit times
          tx_en <= 1'b0; txd <= 2'b00;
          if (tcnt == 6'd0) ts <= T_IDLE; else tcnt <= tcnt - 6'd1;
        end
      endcase
    end
  end

  // ---------------------------------------------------------------------
  // RX: wait for CRS_DV, hunt the SFD (dibit 11 after 01s), shift dibits
  // ---------------------------------------------------------------------
  reg [32:0] rf_mem [0:3];  // [32] = end entry
  reg [1:0]  rf_wp, rf_rp;
  reg [2:0]  rf_n;
  reg        overflow;
  wire       rf_empty = rf_n == 3'd0;
  wire       rf_pop   = acc && !write && ra == R_RX && !rf_empty;
  reg        rf_push;
  reg [32:0] rf_din;
  assign rx_avail = en && !rf_empty;

  localparam [1:0] R_IDLE = 2'd0, R_PRE = 2'd1, R_DATA = 2'd2;
  reg [1:0]  rs;
  reg [31:0] rsh;
  reg [3:0]  rpos;
  reg [15:0] rbytes;
  reg [31:0] rcrc;

  always @(*) begin
    rf_push = 1'b0;
    rf_din  = 33'd0;
    if (!srst && rs == R_DATA) begin
      if (crs_dv && rpos == 4'd15) begin
        rf_push = 1'b1; rf_din = {1'b0, rxd, rsh[31:2]};
      end else if (!crs_dv) begin
        rf_push = 1'b1;                              // end: partial word first (next cycle), then the end entry
        rf_din  = (rpos != 4'd0) ? {1'b0, rsh >> (6'd32 - {rpos, 1'b0})} :
                                   {1'b1, 15'd0, rcrc == 32'hDEBB_20E3, rbytes};
      end
    end
  end

  always @(posedge clk) begin
    if (srst) begin
      rs <= R_IDLE; rpos <= 4'd0; rbytes <= 16'd0; rcrc <= 32'hFFFF_FFFF; rsh <= 32'd0;
      rf_wp <= 2'd0; rf_rp <= 2'd0; rf_n <= 3'd0; overflow <= 1'b0;
    end else begin
      if (acc && write && ra == R_STATUS && wdata[2]) overflow <= 1'b0;
      if (rf_push && rf_n != 3'd4) begin
        rf_mem[rf_wp] <= rf_din; rf_wp <= rf_wp + 2'd1;
      end else if (rf_push) begin
        overflow <= 1'b1;
      end
      if (rf_pop) rf_rp <= rf_rp + 2'd1;
      rf_n <= rf_n + {2'd0, rf_push && rf_n != 3'd4} - {2'd0, rf_pop};
      case (rs)
        R_IDLE: if (crs_dv) rs <= R_PRE;
        R_PRE: begin
          if (!crs_dv) rs <= R_IDLE;
          else if (rxd == 2'b11) begin
            rs <= R_DATA; rpos <= 4'd0; rbytes <= 16'd0; rcrc <= 32'hFFFF_FFFF;
          end
        end
        default: begin                              // R_DATA
          if (crs_dv) begin
            rsh  <= {rxd, rsh[31:2]};
            rcrc <= crc_dibit(rcrc, rxd);
            rpos <= rpos + 4'd1;
            if (rpos[1:0] == 2'd3) rbytes <= rbytes + 16'd1;
          end else if (rpos != 4'd0) begin
            rpos <= 4'd0;                           // partial word pushed; the end entry follows
          end else begin
            rs <= R_IDLE;
          end
        end
      endcase
    end
  end

  // ---------------------------------------------------------------------
  // Registers
  // ---------------------------------------------------------------------
  always @(*) begin
    ready = 1'b1;
    if (sel) begin
      if (ra == R_TX && write)                       ready = !en || !tf_full;
      if ((ra == R_RX || ra == R_RXHEAD) && !write) ready = !en || !rf_empty;
    end
  end

  wire [32:0] head = rf_mem[rf_rp];
  always @(*) begin
    rdata = 32'd0;
    if (sel && !write) begin
      case (ra)
        R_RX:     rdata = head[31:0];
        R_RXHEAD: rdata = {head[32], head[30:0]};
        R_CTRL:   rdata = {31'd0, en};
        R_STATUS: rdata = {14'd0, 2'd2 - tf_n, 4'd0, 1'b0, rf_n, 5'd0, overflow, underrun, ts != T_IDLE};
        default:  rdata = 32'd0;
      endcase
    end
  end

  always @(posedge clk) begin
    if (rst) en <= 1'b0;
    else if (acc && write && ra == R_CTRL) en <= wdata[0];
  end

endmodule
