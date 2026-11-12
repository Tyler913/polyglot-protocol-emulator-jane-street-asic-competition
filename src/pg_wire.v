/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * pg_wire: Wire Engine v1. A protocol-agnostic serial engine: a TX FIFO and
 * shift register, an RX shift register and FIFO, and a bit clock that is
 * either an internal divider or edges on an external clock pin. It knows
 * nothing about UART or SPI; protocol.h builds those from these parameters.
 *
 * Registers (word access, offsets inside the engine page):
 *   0x00 TX     W   push a word (the low LEN bits are sent). Blocks while the
 *                   TX FIFO is full.
 *   0x04 RX     R   pop a received word (LEN bits, right-aligned). Blocks
 *                   while the RX FIFO is empty.
 *   0x08 STATUS R   [3:0] TX FIFO level, [11:8] RX FIFO level,
 *                   [16] TX active (shifting or FIFO not empty), [17] RX frame
 *                   in progress (async mode), [24] RX overflow (sticky,
 *                   write 1 to clear)
 *   0x0C CTRL   RW  [0] EN (0 clears FIFOs and shift state), [1] TX_EN,
 *                   [2] RX_EN, [3] MSB_FIRST, [4] IDLE level of the TX and
 *                   RX lines, [5] RX_ASYNC (RX frame starts on an edge from
 *                   IDLE and has its own bit timer; a frame whose first
 *                   sample is back at IDLE is dropped as a glitch; otherwise RX
 *                   samples in step with the TX bit clock), [6] CLK_EXT (bit
 *                   clock from edges on the CLK pin), [7] CLK_OUT (drive the
 *                   internal bit clock on the CLK pin), [8] CPOL, [9] CPHA,
 *                   [11:10] OE mode of the TX pin (0 always driven, 1 driven
 *                   only while a word is shifting, 2 open drain, 3 never),
 *                   [20:16] LEN-1 (word length 1..32 bits)
 *   0x10 DIV    RW  [15:0] bit period in clock cycles (>= 2)
 *   0x14 PINS   RW  [4:0] TX pin, [12:8] RX pin, [20:16] CLK pin
 *   0x18 FLUSH  R   blocks until the TX FIFO is empty and the last word has
 *                   been fully sent; reads 0
 *
 * Bit timing, internal clock: each bit lasts DIV cycles. The TX line changes
 * at the start of the bit ("launch"); inputs are sampled after DIV/2 cycles
 * ("sample"). CLK_OUT shows CPOL^CPHA in the first part of the bit and the
 * inverse in the second part, which gives SPI modes 0-3. Words follow each
 * other without a gap while the TX FIFO has data.
 *
 * External clock: the sample edge is rising when CPOL == CPHA, falling
 * otherwise; the other edge is the launch edge. Bit 0 of a word is presented
 * as soon as the word is loaded; later bits advance on launch edges.
 *
 * Pins use the chip-wide numbering (0..7 uio, 8..13 uo, 16..20 ui). pin_in
 * is already synchronized to clk.
 */

`default_nettype none

module pg_wire #(
    parameter FIFO_AW = 2
) (
    input  wire        clk,
    input  wire        rst,

    // Register port (AHB data phase)
    input  wire        valid,
    input  wire        write,
    input  wire [7:0]  addr,
    input  wire [31:0] wdata,
    output reg  [31:0] rdata,
    output reg         ready,

    // Pins
    input  wire [31:0] pin_in,
    output wire [4:0]  tx_pin,
    output wire        tx_on,
    output wire        tx_val,
    output wire        tx_oe,
    output wire [4:0]  clk_pin,
    output wire        clk_on,
    output wire        clk_val
);

  localparam [2:0] R_TX = 3'd0, R_RX = 3'd1, R_STATUS = 3'd2, R_CTRL = 3'd3,
                   R_DIV = 3'd4, R_PINS = 3'd5, R_FLUSH = 3'd6;

  // ---------------------------------------------------------------------
  // Configuration
  // ---------------------------------------------------------------------
  reg [20:0] ctrl;
  reg [15:0] div;
  reg [4:0]  txpin, rxpin, ckpin;

  wire       en        = ctrl[0];
  wire       tx_en     = ctrl[1];
  wire       rx_en     = ctrl[2];
  wire       msb_first = ctrl[3];
  wire       idle      = ctrl[4];
  wire       rx_async  = ctrl[5];
  wire       clk_ext   = ctrl[6];
  wire       clk_out   = ctrl[7];
  wire       cpol      = ctrl[8];
  wire       cpha      = ctrl[9];
  wire [1:0] oe_mode   = ctrl[11:10];
  wire [4:0] len_m1    = ctrl[20:16];

  wire [15:0] half = {1'b0, div[15:1]};   // sample point, counted down from DIV-1

  wire ra_valid = valid && (addr[7:5] == 3'd0) && (addr[1:0] == 2'd0);
  wire [2:0] ra = addr[4:2];

  // Engine state is cleared while disabled
  wire srst = rst || !en;

  // ---------------------------------------------------------------------
  // FIFOs
  // ---------------------------------------------------------------------
  wire [31:0]      txf_dout, rxf_dout;
  wire [FIFO_AW:0] txf_level, rxf_level;
  wire             txf_full, txf_empty, rxf_full, rxf_empty;
  wire             txf_pop;
  reg              rxf_push;
  reg  [31:0]      rxf_din;

  wire acc      = ra_valid && ready;
  wire txf_push = acc && write  && (ra == R_TX) && en;
  wire rxf_pop  = acc && !write && (ra == R_RX) && en;

  pg_fifo #(.W(32), .AW(FIFO_AW)) u_txf (
      .clk(clk), .rst(srst), .push(txf_push), .din(wdata), .pop(txf_pop),
      .dout(txf_dout), .level(txf_level), .full(txf_full), .empty(txf_empty)
  );

  pg_fifo #(.W(32), .AW(FIFO_AW)) u_rxf (
      .clk(clk), .rst(srst), .push(rxf_push), .din(rxf_din), .pop(rxf_pop),
      .dout(rxf_dout), .level(rxf_level), .full(rxf_full), .empty(rxf_empty)
  );

  // ---------------------------------------------------------------------
  // Clock pin edges (external clock)
  // ---------------------------------------------------------------------
  wire ck_in = pin_in[ckpin];
  reg  ck_q;
  always @(posedge clk) ck_q <= ck_in;

  wire ck_rise     = ck_in && !ck_q;
  wire ck_fall     = !ck_in && ck_q;
  wire samp_rising = (cpol == cpha);
  wire ext_sample  = clk_ext && (samp_rising ? ck_rise : ck_fall);
  wire ext_launch  = clk_ext && (samp_rising ? ck_fall : ck_rise);

  // ---------------------------------------------------------------------
  // TX: shift register and internal bit timer
  // ---------------------------------------------------------------------
  reg        tx_busy;
  reg [31:0] tx_sh;
  reg [4:0]  tx_cnt;
  reg [15:0] tmr;
  reg        second;    // in the second part of the bit (after the sample point)
  reg        sampled;   // external clock: a sample edge came since the last launch

  wire [4:0] tx_idx  = msb_first ? (len_m1 - tx_cnt) : tx_cnt;
  wire       tx_bit  = tx_sh[tx_idx];
  wire       tx_last = (tx_cnt == len_m1);

  // Internal timer events (only while a word is shifting)
  wire int_sample = !clk_ext && tx_busy && (tmr == half) && !second;
  wire int_end    = !clk_ext && tx_busy && (tmr == 16'd0);

  wire tx_load = tx_en && !txf_empty &&
                 (!tx_busy || (tx_last && (int_end || ext_sample)));
  assign txf_pop = tx_load;

  always @(posedge clk) begin
    if (srst) begin
      tx_busy <= 1'b0;
      tx_sh   <= 32'd0;
      tx_cnt  <= 5'd0;
      tmr     <= 16'd0;
      second  <= 1'b0;
      sampled <= 1'b0;
    end else if (tx_load) begin
      tx_busy <= 1'b1;
      tx_sh   <= txf_dout;
      tx_cnt  <= 5'd0;
      tmr     <= div - 16'd1;
      second  <= 1'b0;
      sampled <= 1'b0;
    end else if (tx_busy) begin
      if (!clk_ext) begin
        if (int_end) begin
          tmr    <= div - 16'd1;
          second <= 1'b0;
          if (tx_last) tx_busy <= 1'b0;
          else         tx_cnt  <= tx_cnt + 5'd1;
        end else begin
          tmr <= tmr - 16'd1;
          if (int_sample) second <= 1'b1;
        end
      end else begin
        if (ext_sample) begin
          sampled <= 1'b1;
          if (tx_last) tx_busy <= 1'b0;
        end else if (ext_launch && sampled) begin
          sampled <= 1'b0;
          tx_cnt  <= tx_cnt + 5'd1;
        end
      end
    end
  end

  wire line = tx_busy ? tx_bit : idle;

  assign tx_pin = txpin;
  assign tx_on  = en && tx_en;
  assign tx_val = (oe_mode == 2'd2) ? 1'b0 : line;
  assign tx_oe  = (oe_mode == 2'd0) ? 1'b1 :
                  (oe_mode == 2'd1) ? tx_busy :
                  (oe_mode == 2'd2) ? !line : 1'b0;

  assign clk_pin = ckpin;
  assign clk_on  = en && clk_out && !clk_ext;
  assign clk_val = tx_busy ? ((cpol ^ cpha) ^ second) : cpol;

  // ---------------------------------------------------------------------
  // RX
  // ---------------------------------------------------------------------
  wire       rx_in = pin_in[rxpin];
  reg        rx_busy;       // async frame in progress
  reg [31:0] rx_sh;
  reg [4:0]  rx_cnt;
  reg [15:0] rtmr;
  reg        rx_ovf;
  reg        rx_prev;       // RX line one cycle ago (start edge detection)

  wire       async_mode = rx_async && !clk_ext;
  wire [4:0] rx_idx     = msb_first ? (len_m1 - rx_cnt) : rx_cnt;
  wire [31:0] rx_word   = rx_sh | ({31'd0, rx_in} << rx_idx);
  // Async start: an edge from idle to active. A frame whose first sample
  // (middle of the start bit) is back at idle was a glitch and is dropped.
  wire       rx_start   = async_mode && !rx_busy && (rx_in != idle) && (rx_prev == idle);
  wire       rx_glitch  = async_mode && (rx_cnt == 5'd0) && (rx_in == idle);
  wire       rx_sample  = async_mode ? (rx_busy && (rtmr == half))
                                     : (clk_ext ? ext_sample : int_sample);
  wire       rx_last    = (rx_cnt == len_m1);

  always @(*) begin
    rxf_push = rx_en && rx_sample && rx_last && !rx_glitch;
    rxf_din  = rx_word;
  end

  always @(posedge clk) rx_prev <= rx_in;

  always @(posedge clk) begin
    if (srst) begin
      rx_busy <= 1'b0;
      rx_sh   <= 32'd0;
      rx_cnt  <= 5'd0;
      rtmr    <= 16'd0;
    end else begin
      if (rx_start) begin
        rx_busy <= 1'b1;
        rtmr    <= div - 16'd1;
      end else if (rx_busy) begin
        rtmr <= (rtmr == 16'd0) ? div - 16'd1 : rtmr - 16'd1;
      end

      if (rx_sample) begin
        if (rx_last || rx_glitch) begin
          rx_sh   <= 32'd0;
          rx_cnt  <= 5'd0;
          rx_busy <= 1'b0;
        end else begin
          rx_sh   <= rx_word;
          rx_cnt  <= rx_cnt + 5'd1;
        end
      end
    end
  end

  // ---------------------------------------------------------------------
  // Registers
  // ---------------------------------------------------------------------
  wire tx_active = tx_busy || !txf_empty;

  always @(*) begin
    case (ra)
      R_TX:    ready = !en || !txf_full || !write;
      R_RX:    ready = !en || !rxf_empty || write;
      R_FLUSH: ready = !en || !tx_active || write;
      default: ready = 1'b1;
    endcase
    if (!ra_valid) ready = 1'b1;
  end

  wire [3:0] txl = {{(3-FIFO_AW){1'b0}}, txf_level};   // FIFO_AW <= 3
  wire [3:0] rxl = {{(3-FIFO_AW){1'b0}}, rxf_level};

  always @(*) begin
    case (ra)
      R_RX:     rdata = en ? rxf_dout : 32'd0;
      R_STATUS: rdata = {7'd0, rx_ovf, 6'd0, rx_busy, tx_active, 4'd0, rxl, 4'd0, txl};
      R_CTRL:   rdata = {11'd0, ctrl};
      R_DIV:    rdata = {16'd0, div};
      R_PINS:   rdata = {11'd0, ckpin, 3'd0, rxpin, 3'd0, txpin};
      default:  rdata = 32'd0;
    endcase
  end

  always @(posedge clk) begin
    if (rst) begin
      ctrl  <= 21'd0;
      div   <= 16'd2;
      txpin <= 5'd0;
      rxpin <= 5'd0;
      ckpin <= 5'd0;
    end else if (acc && write) begin
      case (ra)
        R_CTRL: ctrl <= wdata[20:0];
        R_DIV:  div  <= (wdata[15:0] < 16'd2) ? 16'd2 : wdata[15:0];
        R_PINS: begin
          txpin <= wdata[4:0];
          rxpin <= wdata[12:8];
          ckpin <= wdata[20:16];
        end
        default: ;
      endcase
    end
  end

  always @(posedge clk) begin
    if (srst)                                    rx_ovf <= 1'b0;
    else if (rxf_push && rxf_full)               rx_ovf <= 1'b1;
    else if (acc && write && ra == R_STATUS && wdata[24]) rx_ovf <= 1'b0;
  end

  wire _unused = &{1'b0, ctrl[15:12]};

endmodule
