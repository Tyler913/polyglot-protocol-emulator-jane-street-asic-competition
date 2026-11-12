/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * pg_wire: Wire Engine v2. A protocol-agnostic serial engine: a TX FIFO and
 * shift register, an RX shift register and FIFO, and a bit clock that is
 * either an internal divider or edges on an external clock pin, plus generic
 * framing and flow-control mechanisms. It knows nothing about UART, SPI or
 * I2C; protocol.h builds those from these parameters.
 *
 * Registers (word access, offsets inside the engine page):
 *   0x00 TX     W   push a word of CTRL.LEN bits. Blocks while the TX FIFO
 *                   is full.
 *   0x04 RX     R   pop a received word (right-aligned). Blocks while the RX
 *                   FIFO is empty. Frame events (CTRL.EVENTS) arrive as
 *                   0x8000_0001 (frame start) and 0x8000_0002 (frame end).
 *   0x08 STATUS R   [3:0] TX FIFO level, [11:8] RX FIFO level,
 *                   [16] TX active (shifting, finishing, or FIFO not empty),
 *                   [17] RX frame in progress (async mode), [18] frame
 *                   active (gate), [19] holding the clock low (stretch),
 *                   [24] RX overflow, [25] arbitration lost (both sticky,
 *                   write 1 to clear)
 *   0x0C CTRL   RW  [0] EN (0 clears FIFOs and shift state), [1] TX_EN,
 *                   [2] RX_EN, [3] MSB_FIRST, [4] IDLE level of the TX and
 *                   RX lines, [5] RX_ASYNC (RX frame starts on an edge from
 *                   IDLE and has its own bit timer; a frame whose first
 *                   sample is back at IDLE is dropped as a glitch; otherwise
 *                   RX samples in step with the bit clock), [6] CLK_EXT (bit
 *                   clock from edges on the CLK pin), [7] CLK_OUT (drive the
 *                   internal bit clock on the CLK pin), [8] CPOL, [9] CPHA,
 *                   [11:10] OE mode of the TX pin (0 always driven, 1 driven
 *                   only while a word is shifting or, with a gate, while the
 *                   frame is active, 2 open drain, 3 never),
 *                   [14:12] RX_DELAY: internal clock, sample k cycles after
 *                   the nominal sample point, [15] TX_LATE: internal clock,
 *                   TX data changes a quarter bit after the bit start,
 *                   [20:16] LEN-1 (word length 1..32 bits), [21] CLK_OD
 *                   (CLK pin open drain), [22] CLK_WAIT (internal clock: the
 *                   second half of a bit waits until the CLK pin reads high,
 *                   and RX samples then), [24:23] GATE: 0 none, 1 frame while
 *                   the GATE pin is low, 2 frame from START (RX line falls
 *                   while CLK is high) to STOP (RX line rises while CLK is
 *                   high), [25] EVENTS (frame start/end words into the RX
 *                   FIFO), [26] STRETCH (external clock: at a word boundary
 *                   with an empty TX FIFO, hold the CLK pin low until a word
 *                   is pushed), [27] ARB_STOP (stop driving after a lost
 *                   arbitration)
 *   0x10 DIV    RW  [15:0] bit period in clock cycles (>= 2)
 *   0x14 PINS   RW  [4:0] TX pin, [12:8] RX pin, [20:16] CLK pin,
 *                   [28:24] GATE pin
 *   0x18 FLUSH  R   blocks until the TX FIFO is empty and the last word has
 *                   been fully sent (internal clock); reads 0
 *   0x1C CTRL2  RW  [7:0] DIV_FRAC: the bit period is DIV + DIV_FRAC/256 cycles
 *                   on average (a bit is one cycle longer whenever an 8-bit
 *                   phase accumulator overflows), [8] RESYNC: async RX
 *                   restarts its bit timer on every edge of the RX line
 *   0x80 + 4*(n-1)  W  push a word of n bits (n = 1..32), whatever CTRL.LEN
 *
 * Bit timing, internal clock: each bit lasts DIV cycles. The TX line changes
 * at the start of the bit ("launch", or a quarter bit later with TX_LATE);
 * inputs are sampled after DIV/2 cycles ("sample"). CLK_OUT shows CPOL^CPHA
 * in the first part of the bit and the inverse in the second part, which
 * gives SPI modes 0-3. Words follow each other without a gap while the TX
 * FIFO has data.
 *
 * External clock: the sample edge is rising when CPOL == CPHA, falling
 * otherwise; the other edge is the launch edge. The first word of a frame is
 * presented as soon as it is loaded; later words load on the launch edge
 * after the previous word's last sample, and the line keeps the last bit
 * until then. RX words take the length of the TX word being shifted (CTRL.LEN
 * when TX is idle). At a frame start the bit count restarts and a TX word
 * that already started shifting is dropped; a frame end drops the current
 * word and flushes the TX FIFO (queue the next frame's reply after the end).
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
    output wire        clk_val,
    output wire        clk_oe
);

  localparam [3:0] R_TX = 4'd0, R_RX = 4'd1, R_STATUS = 4'd2, R_CTRL = 4'd3,
                   R_DIV = 4'd4, R_PINS = 4'd5, R_FLUSH = 4'd6, R_CTRL2 = 4'd7;

  // ---------------------------------------------------------------------
  // Configuration
  // ---------------------------------------------------------------------
  reg [27:0] ctrl;
  reg [8:0]  ctrl2;
  reg [15:0] div;
  reg [4:0]  txpin, rxpin, ckpin, gtpin;

  wire       en         = ctrl[0];
  wire       tx_en      = ctrl[1];
  wire       rx_en      = ctrl[2];
  wire       msb_first  = ctrl[3];
  wire       idle       = ctrl[4];
  wire       rx_async   = ctrl[5];
  wire       clk_ext    = ctrl[6];
  wire       clk_out    = ctrl[7];
  wire       cpol       = ctrl[8];
  wire       cpha       = ctrl[9];
  wire [1:0] oe_mode    = ctrl[11:10];
  wire [2:0] rx_delay   = ctrl[14:12];
  wire       tx_late    = ctrl[15];
  wire [4:0] len_m1     = ctrl[20:16];
  wire       clk_od     = ctrl[21];
  wire       clk_wait   = ctrl[22];
  wire [1:0] gate_mode  = ctrl[24:23];
  wire       events     = ctrl[25];
  wire       stretch_en = ctrl[26];
  wire       arb_stop   = ctrl[27];
  wire [7:0] frac       = ctrl2[7:0];
  wire       resync     = ctrl2[8];

  wire [15:0] half = {1'b0, div[15:1]};                     // sample point (counting down from DIV-1)
  wire [15:0] qpt  = div - 16'd1 - {2'b00, div[15:2]};      // quarter bit after the start

  wire sel     = valid && (addr[1:0] == 2'd0);
  wire tx_win  = sel && addr[7];                            // 0x80..0xFC: push with length
  wire reg_sel = sel && !addr[7] && !addr[6];
  wire [3:0] ra = addr[5:2];

  // Engine state is cleared while disabled
  wire srst = rst || !en;

  // ---------------------------------------------------------------------
  // FIFOs (TX entries carry their length)
  // ---------------------------------------------------------------------
  wire [36:0]      txf_dout;
  wire [31:0]      rxf_dout;
  wire [FIFO_AW:0] txf_level, rxf_level;
  wire             txf_full, txf_empty, rxf_full, rxf_empty;
  wire             txf_pop;
  wire             rxf_push;
  wire [31:0]      rxf_din;
  wire             f_start, f_stop;   // frame boundaries (see below)

  wire acc      = (reg_sel || tx_win) && ready;
  wire tx_write = acc && write && (tx_win || ra == R_TX);
  wire txf_push = tx_write && en;
  wire rxf_pop  = acc && reg_sel && !write && (ra == R_RX) && en;

  pg_fifo #(.W(37), .AW(FIFO_AW)) u_txf (
      .clk(clk), .rst(srst || f_stop), .push(txf_push),
      .din({tx_win ? addr[6:2] : len_m1, wdata}), .pop(txf_pop),
      .dout(txf_dout), .level(txf_level), .full(txf_full), .empty(txf_empty)
  );

  pg_fifo #(.W(32), .AW(FIFO_AW)) u_rxf (
      .clk(clk), .rst(srst), .push(rxf_push), .din(rxf_din), .pop(rxf_pop),
      .dout(rxf_dout), .level(rxf_level), .full(rxf_full), .empty(rxf_empty)
  );

  // ---------------------------------------------------------------------
  // Pin inputs, external clock edges, frame gate
  // ---------------------------------------------------------------------
  wire ck_in = pin_in[ckpin];
  wire rx_in = pin_in[rxpin];
  wire gt_in = pin_in[gtpin];
  reg  ck_q, rx_prev, gt_q;
  always @(posedge clk) begin
    ck_q    <= ck_in;
    rx_prev <= rx_in;
    gt_q    <= gt_in;
  end

  reg  frame_i2c;
  wire ck_high   = ck_in && ck_q;
  wire i2c_start = (gate_mode == 2'd2) && ck_high && rx_prev && !rx_in;
  wire i2c_stop  = (gate_mode == 2'd2) && ck_high && !rx_prev && rx_in;
  assign f_start = i2c_start || ((gate_mode == 2'd1) && gt_q && !gt_in);
  assign f_stop  = i2c_stop  || ((gate_mode == 2'd1) && !gt_q && gt_in);
  wire gated     = (gate_mode != 2'd0);
  wire active    = (gate_mode == 2'd0) ? 1'b1 :
                   (gate_mode == 2'd1) ? !gt_in : frame_i2c;

  always @(posedge clk) begin
    if (srst || gate_mode != 2'd2) frame_i2c <= 1'b0;
    else if (i2c_start)             frame_i2c <= 1'b1;
    else if (i2c_stop)              frame_i2c <= 1'b0;
  end

  wire ck_rise     = ck_in && !ck_q;
  wire ck_fall     = !ck_in && ck_q;
  wire samp_rising = (cpol == cpha);
  wire ext_sample  = clk_ext && active && (samp_rising ? ck_rise : ck_fall);
  wire ext_launch  = clk_ext && active && (samp_rising ? ck_fall : ck_rise);

  // ---------------------------------------------------------------------
  // TX: shift register and internal bit timer
  // ---------------------------------------------------------------------
  reg        tx_busy;
  reg        tail;       // word done, line still shows its last bit
  reg [31:0] tx_sh;
  reg [4:0]  tx_cnt, tx_len;
  reg [15:0] tmr;
  reg        second;     // in the second part of the bit (after the sample point)
  reg        sampled;    // a sample was taken in this bit (ext clock, CLK_WAIT)
  reg        prev_bit;   // line value before the current bit (TX_LATE)
  reg        boundary;   // ext clock: a word ended; the next one waits for a launch edge
  reg        stretching;
  reg  [7:0] tacc;       // fractional bit period accumulator (TX timer)

  // Next bit length: DIV, plus one cycle when the accumulator overflows
  wire [8:0] tacc_sum = {1'b0, tacc} + {1'b0, frac};
  wire [15:0] tbit_m1 = div - 16'd1 + {15'd0, tacc_sum[8]};

  wire [4:0] tx_idx  = msb_first ? (tx_len - tx_cnt) : tx_cnt;
  wire       tx_bit  = tx_sh[tx_idx];
  wire       tx_last = (tx_cnt == tx_len);

  // CLK_WAIT: hold the timer in the second half while the clock pin is low
  wire hold       = clk_wait && second && !ck_in;
  wire int_sample = !clk_ext && tx_busy &&
                    (clk_wait ? (second && ck_in && !sampled) : ((tmr == half) && !second));
  wire int_end    = !clk_ext && tx_busy && (tmr == 16'd0) && !hold;

  wire ext_ready  = !boundary || ext_launch || stretching;
  wire tx_load    = tx_en && !txf_empty &&
                    (clk_ext ? (!tx_busy && ext_ready) : (!tx_busy || (tx_last && int_end)));
  assign txf_pop = tx_load;

  wire late_phase = tx_late && !clk_ext && tx_busy && !second && (tmr > qpt);
  wire line       = (tx_busy && late_phase) ? prev_bit :
                    (tx_busy || tail)       ? tx_bit   : idle;

  always @(posedge clk) begin
    if (srst) begin
      tx_busy  <= 1'b0;
      tail     <= 1'b0;
      tx_sh    <= 32'd0;
      tx_cnt   <= 5'd0;
      tx_len   <= 5'd0;
      tmr      <= 16'd0;
      second   <= 1'b0;
      sampled  <= 1'b0;
      prev_bit <= 1'b1;
      tacc     <= 8'd0;
    end else begin
      if (tx_load) begin
        tx_busy  <= 1'b1;
        tail     <= 1'b0;
        tx_sh    <= txf_dout[31:0];
        tx_len   <= txf_dout[36:32];
        tx_cnt   <= 5'd0;
        tmr      <= tbit_m1;
        tacc     <= tacc_sum[7:0];
        second   <= 1'b0;
        sampled  <= 1'b0;
        prev_bit <= line;
      end else if (tx_busy && !clk_ext) begin
        if (int_end) begin
          tmr      <= tbit_m1;
          tacc     <= tacc_sum[7:0];
          second   <= 1'b0;
          sampled  <= 1'b0;
          prev_bit <= tx_bit;
          if (tx_last) begin
            tx_busy <= 1'b0;
            tail    <= tx_late;
          end else begin
            tx_cnt  <= tx_cnt + 5'd1;
          end
        end else if (!hold) begin
          tmr <= tmr - 16'd1;
          if (tmr == half) second <= 1'b1;
          if (int_sample)  sampled <= 1'b1;
        end else if (int_sample) begin
          sampled <= 1'b1;
        end
      end else if (tx_busy) begin            // external clock
        if (ext_sample) begin
          sampled <= 1'b1;
          if (tx_last) begin
            tx_busy <= 1'b0;
            tail    <= 1'b1;
          end
        end else if (ext_launch && sampled) begin
          sampled <= 1'b0;
          tx_cnt  <= tx_cnt + 5'd1;
        end
      end else if (tail) begin
        if (clk_ext) begin
          if (ext_launch) tail <= 1'b0;
        end else if (tmr <= qpt) begin
          tail <= 1'b0;
        end else begin
          tmr <= tmr - 16'd1;
        end
      end

      // Frame end: drop the current word (the TX FIFO is flushed too). Frame
      // start: drop a word that already started shifting (the controller
      // abandoned it); a word that has not started is kept for the new frame.
      if (f_stop) begin
        tx_busy <= 1'b0;
        tail    <= 1'b0;
        tx_cnt  <= 5'd0;
        sampled <= 1'b0;
      end else if (f_start) begin
        tx_cnt  <= 5'd0;
        sampled <= 1'b0;
        if (!tx_load) begin
          tail <= 1'b0;
          if (tx_cnt != 5'd0 || sampled) tx_busy <= 1'b0;
        end
      end
    end
  end

  // ---------------------------------------------------------------------
  // RX
  // ---------------------------------------------------------------------
  reg        rx_busy;       // async frame in progress
  reg [31:0] rx_sh;
  reg [4:0]  rx_cnt;
  reg [15:0] rtmr;
  reg        rx_ovf, arb_lost;
  reg [6:0]  samp_d;        // delayed internal sample strobes (RX_DELAY)
  reg [7:0]  racc;          // fractional bit period accumulator (async RX timer)
  wire [8:0] racc_sum = {1'b0, racc} + {1'b0, frac};
  wire [15:0] rbit_m1 = div - 16'd1 + {15'd0, racc_sum[8]};

  always @(posedge clk) begin
    if (srst) samp_d <= 7'd0;
    else      samp_d <= {samp_d[5:0], int_sample};
  end

  wire       async_mode = rx_async && !clk_ext;
  wire       int_samp_d = (rx_delay == 3'd0) ? int_sample : samp_d[rx_delay - 3'd1];
  wire [4:0] rx_len     = async_mode ? len_m1 : ((tx_busy || tail) ? tx_len : len_m1);
  wire [4:0] rx_idx     = msb_first ? (rx_len - rx_cnt) : rx_cnt;
  wire [31:0] rx_word   = rx_sh | ({31'd0, rx_in} << rx_idx);
  // Async start: an edge from idle to active. A frame whose first sample
  // (middle of the start bit) is back at idle was a glitch and is dropped.
  wire       rx_start   = async_mode && !rx_busy && (rx_in != idle) && (rx_prev == idle);
  wire       rx_glitch  = async_mode && (rx_cnt == 5'd0) && (rx_in == idle);
  wire       rx_sample  = async_mode ? (rx_busy && (rtmr == half))
                                     : (clk_ext ? ext_sample : int_samp_d);
  wire       rx_last    = (rx_cnt == rx_len);

  wire data_push = rx_en && rx_sample && rx_last && !rx_glitch;
  wire ev_push   = events && (f_start || f_stop) && !data_push;
  assign rxf_push = data_push || ev_push;
  assign rxf_din  = data_push ? rx_word : {30'd0, f_stop, f_start} | 32'h8000_0000;

  always @(posedge clk) begin
    if (srst) begin
      rx_busy <= 1'b0;
      rx_sh   <= 32'd0;
      rx_cnt  <= 5'd0;
      rtmr    <= 16'd0;
      racc    <= 8'd0;
    end else begin
      if (rx_start || (resync && rx_busy && rx_in != rx_prev)) begin
        rx_busy <= 1'b1;             // a start edge, or (RESYNC) any edge: realign the bit timer
        rtmr    <= div - 16'd1;
        if (rx_start) racc <= 8'd0;
      end else if (rx_busy) begin
        if (rtmr == 16'd0) begin
          rtmr <= rbit_m1;
          racc <= racc_sum[7:0];
        end else begin
          rtmr <= rtmr - 16'd1;
        end
      end

      if (f_start || f_stop) begin
        rx_sh  <= 32'd0;
        rx_cnt <= 5'd0;
      end else if (rx_sample) begin
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

  // Word boundary and clock stretching (external clock)
  always @(posedge clk) begin
    if (srst || f_start || f_stop) begin
      boundary   <= 1'b0;
      stretching <= 1'b0;
    end else begin
      if (tx_load) begin
        boundary   <= 1'b0;
        stretching <= 1'b0;
      end else begin
        if (clk_ext && rx_sample && rx_last) boundary <= 1'b1;
        if (stretch_en && clk_ext && boundary && !tx_busy && ext_launch && txf_empty)
          stretching <= 1'b1;
      end
    end
  end

  // ---------------------------------------------------------------------
  // Pin outputs
  // ---------------------------------------------------------------------
  wire od       = (oe_mode == 2'd2);
  wire tx_shown = !gated || active;           // gated frames hide the TX line outside the frame
  wire drv_line = tx_shown ? line : idle;
  wire arb_off  = arb_stop && arb_lost;

  assign tx_pin = txpin;
  assign tx_on  = en && tx_en;
  assign tx_val = od ? 1'b0 : drv_line;
  assign tx_oe  = arb_off ? 1'b0 :
                  (oe_mode == 2'd0) ? 1'b1 :
                  (oe_mode == 2'd1) ? (gated ? active : (tx_busy || tail)) :
                  od                ? !drv_line : 1'b0;

  wire sck = tx_busy ? ((cpol ^ cpha) ^ second) : cpol;

  assign clk_pin = ckpin;
  assign clk_on  = en && ((clk_out && !clk_ext) || (clk_ext && stretching));
  assign clk_val = (clk_ext || clk_od) ? 1'b0 : sck;
  assign clk_oe  = clk_ext ? 1'b1 : (clk_od ? !sck : 1'b1);

  // ---------------------------------------------------------------------
  // Registers
  // ---------------------------------------------------------------------
  wire tx_active = tx_busy || (tail && !clk_ext) || !txf_empty;

  always @(*) begin
    ready = 1'b1;
    if (tx_win)
      ready = !en || !txf_full || !write;
    else if (reg_sel) begin
      case (ra)
        R_TX:    ready = !en || !txf_full || !write;
        R_RX:    ready = !en || !rxf_empty || write;
        R_FLUSH: ready = !en || !tx_active || write;
        default: ready = 1'b1;
      endcase
    end
  end

  wire [3:0] txl = {{(3-FIFO_AW){1'b0}}, txf_level};   // FIFO_AW <= 3
  wire [3:0] rxl = {{(3-FIFO_AW){1'b0}}, rxf_level};

  always @(*) begin
    rdata = 32'd0;
    if (reg_sel) begin
      case (ra)
        R_RX:     rdata = en ? rxf_dout : 32'd0;
        R_STATUS: rdata = {6'd0, arb_lost, rx_ovf, 4'd0, stretching, active, rx_busy, tx_active,
                           4'd0, rxl, 4'd0, txl};
        R_CTRL:   rdata = {4'd0, ctrl};
        R_CTRL2:  rdata = {23'd0, ctrl2};
        R_DIV:    rdata = {16'd0, div};
        R_PINS:   rdata = {3'd0, gtpin, 3'd0, ckpin, 3'd0, rxpin, 3'd0, txpin};
        default:  rdata = 32'd0;
      endcase
    end
  end

  wire cfg_wr = acc && write && reg_sel;

  always @(posedge clk) begin
    if (rst) begin
      ctrl  <= 28'd0;
      ctrl2 <= 9'd0;
      div   <= 16'd2;
      txpin <= 5'd0;
      rxpin <= 5'd0;
      ckpin <= 5'd0;
      gtpin <= 5'd0;
    end else if (cfg_wr) begin
      case (ra)
        R_CTRL: ctrl <= wdata[27:0];
        R_CTRL2: ctrl2 <= wdata[8:0];
        R_DIV:  div  <= (wdata[15:0] < 16'd2) ? 16'd2 : wdata[15:0];
        R_PINS: begin
          txpin <= wdata[4:0];
          rxpin <= wdata[12:8];
          ckpin <= wdata[20:16];
          gtpin <= wdata[28:24];
        end
        default: ;
      endcase
    end
  end

  wire arb_check = od && tx_busy && line && !rx_in && (clk_ext ? ext_sample : int_sample);

  always @(posedge clk) begin
    if (srst) begin
      rx_ovf   <= 1'b0;
      arb_lost <= 1'b0;
    end else begin
      if (rxf_push && rxf_full)                              rx_ovf <= 1'b1;
      else if (cfg_wr && ra == R_STATUS && wdata[24])       rx_ovf <= 1'b0;
      if (arb_check)                                         arb_lost <= 1'b1;
      else if (cfg_wr && ra == R_STATUS && wdata[25])       arb_lost <= 1'b0;
    end
  end

endmodule
