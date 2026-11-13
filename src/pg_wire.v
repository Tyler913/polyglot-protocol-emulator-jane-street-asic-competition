/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * pg_wire: Wire Engine v3. A protocol-agnostic serial engine: a TX FIFO and
 * shift register, an RX shift register and FIFO, and a bit clock that is
 * either an internal divider or edges on an external clock pin, plus generic
 * framing, flow-control and bit-level mechanisms (line code, bit stuffing,
 * CRC, differential output, timed auto-reply). It knows nothing about UART,
 * SPI, I2C or USB; protocol.h builds those from these parameters.
 *
 * Registers (word access, offsets inside the engine page):
 *   0x00 TX     W   push a word of CTRL.LEN bits with the TXATTR attributes.
 *                   Blocks while the TX FIFO is full.
 *   0x04 RX     R   pop a received word (right-aligned). Blocks while the RX
 *                   FIFO is empty. Frame events (CTRL.EVENTS) arrive as
 *                   0x8000_0001 (frame start) and 0x8000_0002 (frame end).
 *                   At the end of an RX_PACKET packet, bits that do not fill
 *                   a word are pushed as a last word (in their word
 *                   positions), and the end marker carries their number in
 *                   bits [12:8].
 *   0x08 STATUS R   [3:0] TX FIFO level, [11:8] RX FIFO level,
 *                   [16] TX active (shifting, finishing, or FIFO not empty),
 *                   [17] RX frame in progress (async mode), [18] frame
 *                   active (gate), [19] holding the clock low (stretch),
 *                   [24] RX overflow, [25] arbitration lost, [26] RX stuff
 *                   error (all three sticky, write 1 to clear)
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
 *                   is pushed), [27] ARB_STOP (after a lost arbitration send
 *                   only the IDLE level; with it the read-back compare also
 *                   works on push-pull lines with a separate RX pin, e.g. CAN)
 *   0x10 DIV    RW  [15:0] bit period in clock cycles (>= 2)
 *   0x14 PINS   RW  [4:0] TX pin, [12:8] RX pin, [20:16] CLK pin,
 *                   [28:24] GATE pin
 *   0x18 FLUSH  R   blocks until the TX FIFO is empty and the last word has
 *                   been fully sent (internal clock); reads 0
 *               W   [0] = 1: discard the TX FIFO and the word being sent (RX
 *                   and configuration are not touched)
 *   0x1C CTRL2  RW  [7:0] DIV_FRAC: the bit period is DIV + DIV_FRAC/256 cycles
 *                   on average, [8] RESYNC: async RX restarts its bit timer
 *                   on every edge of the RX line; bit-level options (internal
 *                   clock TX, async RX): [10:9] CODE (0 NRZ, 1 NRZI: a 0 bit
 *                   toggles the line, 2 Manchester (TX): first half of a bit
 *                   is the complement, second half the bit, IEEE 802.3
 *                   polarity), [13:11] STUFF_N (0 off; after N
 *                   equal bits insert/remove one), [14] STUFF_EQUAL (runs of
 *                   equal bits, CAN; else runs of ones, USB), [15] DIFF (CLK
 *                   pin carries the complement of TX; SE0 on RX = RX and
 *                   GATE pins both low; RX ignores the line while TX
 *                   sends: half duplex), [16] RX_PACKET (async RX keeps
 *                   receiving word after word until SE0 instead of stopping
 *                   after one word), [18:17] TAIL_BITS: after the last word
 *                   the TX line stays driven at the IDLE level for 0..3 more
 *                   bit times (USB end-of-packet J, 10BASE-T idle),
 *                   [21:19] EOF_RUN: with RX_PACKET, a packet also ends after
 *                   this many consecutive idle-level bits (CAN end of frame;
 *                   a run of idle bits is then not a stuff error)
 *   0x20 TXATTR RW  attributes of the words pushed from now on: [0] NOSTUFF,
 *                   [1] NOCRC (not part of the CRC), [2] CRC_INIT (reset the
 *                   TX CRC before this word), [3] CRC_OUT (send the TX CRC,
 *                   MSB first, after this word), [4] SE0 (both TX lines low
 *                   for this word's bits), [5] AFTER_RX (wait until TURN
 *                   cycles after the end of a received packet, then send)
 *   0x24 CRCCFG RW  [4:0] width-1 (1..32 bits), [5] XOROUT (send the CRC
 *                   complemented), [12:8] RX_SKIP (received bits after a
 *                   frame start that are not part of the RX CRC)
 *   0x28 CRCINIT RW initial CRC value (TX and RX)
 *   0x2C RXCRC  R   RX CRC register (after the last data bit: the residual)
 *   0x30 TURN   RW  [15:0] turnaround delay for AFTER_RX words, in cycles
 *   0x34 CRCPOLY RW CRC polynomial without the top bit (e.g. 0x04C11DB7)
 *   0x38 RXMATCH RW auto-reply filter: [7:0] value, [15:8] mask, [19:16]
 *                   word index k. A received packet triggers AFTER_RX words
 *                   only if (word k & mask) == (value & mask); mask 0 = any.
 *
 * AFTER_RX words wait for the end of a packet received after they were
 * pushed (pushing one cancels a pending trigger), then TURN cycles.
 *   0x80 + 4*(n-1)  W  push a word of n bits (n = 1..32), whatever CTRL.LEN
 *
 * Bit timing, internal clock: each bit lasts DIV cycles. The TX line changes
 * at the start of the bit ("launch", or a quarter bit later with TX_LATE);
 * inputs are sampled after DIV/2 cycles ("sample"). CLK_OUT shows CPOL^CPHA
 * in the first part of the bit and the inverse in the second part, which
 * gives SPI modes 0-3. Words follow each other without a gap while the TX
 * FIFO has data. The TX bit stream (data, stuff bits, appended CRC) is
 * continuous across words, so stuffing and NRZI span a whole packet.
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
 *
 * FULL = 0 builds the engine without CTRL2 bit-level options, CRC and
 * auto-reply (registers read 0), for a smaller "lite" engine.
 */

`default_nettype none

module pg_wire #(
    parameter FIFO_AW = 2,
    parameter FULL    = 1
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
    output wire        clk_oe,

    // Event: received data waiting (for EVWAIT in pg_io)
    output wire        rx_avail
);

  localparam [3:0] R_TX = 4'd0, R_RX = 4'd1, R_STATUS = 4'd2, R_CTRL = 4'd3,
                   R_DIV = 4'd4, R_PINS = 4'd5, R_FLUSH = 4'd6, R_CTRL2 = 4'd7,
                   R_TXATTR = 4'd8, R_CRCCFG = 4'd9, R_CRCINIT = 4'd10,
                   R_RXCRC = 4'd11, R_TURN = 4'd12, R_CRCPOLY = 4'd13, R_RXMATCH = 4'd14;

  localparam FW = 43;                     // TX FIFO entry: {attr[5:0], len-1[4:0], data[31:0]}
  wire full = (FULL != 0);

  // ---------------------------------------------------------------------
  // Configuration
  // ---------------------------------------------------------------------
  reg [27:0] ctrl;
  reg [21:0] ctrl2;
  reg [5:0]  txattr;
  reg [12:0] crccfg;
  reg [31:0] crcinit;
  reg [31:0] poly;
  reg [15:0] turn;
  reg [19:0] rxmatch;
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
  wire       nrzi       = full && (ctrl2[10:9] == 2'd1);
  wire       manch      = full && (ctrl2[10:9] == 2'd2);
  wire [2:0] stuff_n    = full ? ctrl2[13:11] : 3'd0;
  wire       stuff_eq   = ctrl2[14];
  wire       diff       = full && ctrl2[15];
  wire       rx_packet  = full && ctrl2[16];
  wire [1:0] tail_bits  = full ? ctrl2[18:17] : 2'd0;
  wire [2:0] eof_run    = full ? ctrl2[21:19] : 3'd0;
  wire [4:0]  crc_lm1   = crccfg[4:0];
  wire        crc_xor   = crccfg[5];
  wire [4:0]  rx_skip   = crccfg[12:8];
  wire [31:0] crc_mask  = 32'hFFFF_FFFF >> (5'd31 - crc_lm1);

  wire [15:0] half = {1'b0, div[15:1]};                     // sample point (counting down from DIV-1)
  wire [15:0] qpt  = div - 16'd1 - {2'b00, div[15:2]};      // quarter bit after the start

  wire sel     = valid && (addr[1:0] == 2'd0);
  wire tx_win  = sel && addr[7];                            // 0x80..0xFC: push with length
  wire reg_sel = sel && !addr[7] && !addr[6];
  wire [3:0] ra = addr[5:2];

  // Engine state is cleared while disabled
  wire srst = rst || !en;

  // One bit-serial CRC step (MSB-out LFSR over the transmitted bit order)
  function [31:0] crc_step;
    input [31:0] c;
    input        b;
    input [4:0]  lm1;
    input [31:0] p;
    input [31:0] m;
    begin
      crc_step = ((c << 1) ^ ((b ^ c[lm1]) ? p : 32'd0)) & m;
    end
  endfunction

  // ---------------------------------------------------------------------
  // FIFOs (TX entries carry their length and attributes)
  // ---------------------------------------------------------------------
  wire [FW-1:0]    txf_dout;
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
  wire tx_discard = acc && reg_sel && write && (ra == R_FLUSH) && wdata[0];

  pg_fifo #(.W(FW), .AW(FIFO_AW)) u_txf (
      .clk(clk), .rst(srst || f_stop || tx_discard), .push(txf_push),
      .din({full ? txattr : 6'd0, tx_win ? addr[6:2] : len_m1, wdata}), .pop(txf_pop),
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
  // Turnaround timer: AFTER_RX words start TURN cycles after an RX packet
  // ends (set below by the RX side).
  // ---------------------------------------------------------------------
  wire        rx_pkt_end;
  wire        rx_start;
  reg         match_ok;      // the packet being received passes RXMATCH
  wire        push_after = txf_push && full && txattr[5];
  reg  [15:0] turn_cnt;
  reg         turn_run, turn_go;

  // ---------------------------------------------------------------------
  // TX: shift register, internal bit timer, stuffing, CRC, line code
  // ---------------------------------------------------------------------
  reg        tx_busy;
  reg        tail;       // word done, line still shows its last bit
  reg [31:0] tx_sh;
  reg [4:0]  tx_cnt, tx_len;
  reg [5:0]  tx_attr;    // attributes of the current word
  reg        tx_msb;     // bit order of the current word (CRC words are MSB first)
  reg        in_crc;     // the current word is the appended CRC
  reg        stuffing;   // the current bit is a stuff bit
  reg [2:0]  srun;       // run length for stuffing
  reg        slast;      // previous transmitted bit (before line coding)
  reg        lvl_prev;   // line level of the previous bit (NRZI)
  reg [31:0] tcrc;
  reg [15:0] tmr;
  reg        second;     // in the second part of the bit (after the sample point)
  reg        sampled;    // a sample was taken in this bit (ext clock, CLK_WAIT)
  reg        prev_bit;   // line value before the current bit (TX_LATE)
  reg        boundary;   // ext clock: a word ended; the next one waits for a launch edge
  reg        stretching;
  reg  [7:0] tacc;       // fractional bit period accumulator (TX timer)
  reg  [1:0] itail;      // TAIL_BITS still to drive at the idle level

  // Next bit length: DIV, plus one cycle when the accumulator overflows
  wire [8:0] tacc_sum = {1'b0, tacc} + {1'b0, frac};
  wire [15:0] tbit_m1 = div - 16'd1 + {15'd0, tacc_sum[8]};

  wire [4:0] tx_idx   = tx_msb ? (tx_len - tx_cnt) : tx_cnt;
  wire       data_bit = tx_sh[tx_idx];
  wire       tx_last  = (tx_cnt == tx_len);

  wire a_nostuff = tx_attr[0];
  wire a_nocrc   = tx_attr[1];
  wire a_crcout  = tx_attr[3];
  wire a_se0     = tx_attr[4];
  wire h_crcinit = full && txf_dout[39];
  wire h_after   = full && txf_dout[42];

  // Bit-level stage (internal clock only)
  wire stuff_on  = !clk_ext && (stuff_n != 3'd0) && !a_nostuff;
  wire stuff_val = stuff_eq ? !slast : 1'b0;
  wire cur_bit   = stuffing ? stuff_val : data_bit;
  wire [2:0] run_n = stuff_eq ? ((cur_bit == slast) ? srun + 3'd1 : 3'd1)
                              : (cur_bit ? srun + 3'd1 : 3'd0);
  wire next_stuff = stuff_on && (run_n == stuff_n);
  wire crc_upd    = full && !stuffing && !a_nocrc && !in_crc;
  wire [31:0] tcrc_n = crc_step(tcrc, cur_bit, crc_lm1, poly, crc_mask);
  wire [31:0] tcrc_o = (crc_upd ? tcrc_n : tcrc) ^ (crc_xor ? crc_mask : 32'd0);
  wire lvl_cur    = a_se0 ? 1'b0 :
                    nrzi  ? (lvl_prev ^ !cur_bit) :
                    manch ? (cur_bit ^ !second) : cur_bit;

  // CLK_WAIT: hold the timer in the second half while the clock pin is low
  wire hold       = clk_wait && second && !ck_in;
  wire int_sample = !clk_ext && tx_busy &&
                    (clk_wait ? (second && ck_in && !sampled) : ((tmr == half) && !second));
  wire int_end    = !clk_ext && tx_busy && (tmr == 16'd0) && !hold;

  wire word_done  = int_end && !next_stuff && tx_last;
  wire crc_follow = word_done && a_crcout && !in_crc;

  wire can_go     = !h_after || turn_go;
  wire ext_ready  = !boundary || ext_launch || stretching;
  wire tx_load    = tx_en && !txf_empty && can_go &&
                    (clk_ext ? (!tx_busy && ext_ready)
                             : (!tx_busy || (word_done && !crc_follow)));
  assign txf_pop = tx_load;

  wire late_phase = tx_late && !clk_ext && tx_busy && !second && (tmr > qpt);
  wire itail_on   = (itail != 2'd0);
  wire line       = (tx_busy && late_phase) ? prev_bit :
                    tx_busy                 ? lvl_cur  :
                    tail                    ? data_bit : idle;   // itail: idle level
  wire se0_now    = tx_busy && a_se0;

  always @(posedge clk) begin
    if (srst) begin
      tx_busy  <= 1'b0;
      tail     <= 1'b0;
      tx_sh    <= 32'd0;
      tx_cnt   <= 5'd0;
      tx_len   <= 5'd0;
      tx_attr  <= 6'd0;
      tx_msb   <= 1'b0;
      in_crc   <= 1'b0;
      stuffing <= 1'b0;
      srun     <= 3'd0;
      slast    <= 1'b1;
      lvl_prev <= 1'b1;
      tcrc     <= 32'd0;
      tmr      <= 16'd0;
      second   <= 1'b0;
      sampled  <= 1'b0;
      prev_bit <= 1'b1;
      tacc     <= 8'd0;
      itail    <= 2'd0;
    end else begin
      if (tx_load) begin
        if (tx_busy) begin                   // continuing a packet: account for the bit just sent
          srun     <= a_nostuff ? 3'd0 : run_n;
          slast    <= cur_bit;
          lvl_prev <= a_se0 ? idle : lvl_cur;
          if (crc_upd) tcrc <= tcrc_n;
        end else begin                       // packet start
          srun     <= 3'd0;
          slast    <= idle;
          lvl_prev <= idle;
        end
        if (h_crcinit) tcrc <= crcinit;
        tx_busy  <= 1'b1;
        tail     <= 1'b0;
        itail    <= 2'd0;
        tx_sh    <= txf_dout[31:0];
        tx_len   <= txf_dout[36:32];
        tx_attr  <= txf_dout[42:37];
        tx_msb   <= msb_first;
        in_crc   <= 1'b0;
        stuffing <= 1'b0;
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
          prev_bit <= line;
          srun     <= a_nostuff ? 3'd0 : run_n;
          slast    <= cur_bit;
          lvl_prev <= a_se0 ? idle : lvl_cur;
          if (crc_upd) tcrc <= tcrc_n;
          if (next_stuff) begin
            stuffing <= 1'b1;                // a stuff bit, then carry on with the same data bit position
          end else begin
            stuffing <= 1'b0;
            if (crc_follow) begin            // append the CRC as an MSB-first word
              tx_sh   <= tcrc_o;
              tx_len  <= crc_lm1;
              tx_msb  <= 1'b1;
              in_crc  <= 1'b1;
              tx_cnt  <= 5'd0;
            end else if (tx_last) begin
              tx_busy <= 1'b0;
              tail    <= tx_late;
              itail   <= tail_bits;
            end else begin
              tx_cnt  <= tx_cnt + 5'd1;
            end
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
      end else if (itail_on && !clk_ext) begin   // driven idle bits after the last word
        if (tmr == 16'd0) begin
          tmr   <= tbit_m1;
          tacc  <= tacc_sum[7:0];
          itail <= itail - 2'd1;
        end else begin
          tmr   <= tmr - 16'd1;
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
      if (f_stop || tx_discard) begin
        tx_busy <= 1'b0;
        tail    <= 1'b0;
        itail   <= 2'd0;
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

  always @(posedge clk) begin
    if (srst || !full) begin
      turn_cnt <= 16'd0;
      turn_run <= 1'b0;
      turn_go  <= 1'b0;
    end else begin
      if (rx_pkt_end && match_ok) begin
        turn_run <= 1'b1;
        turn_cnt <= 16'd0;
        turn_go  <= 1'b0;
      end else if (turn_run) begin
        if (turn_cnt >= turn) begin
          turn_run <= 1'b0;
          turn_go  <= 1'b1;
        end else begin
          turn_cnt <= turn_cnt + 16'd1;
        end
      end
      if (rx_start || (tx_load && h_after) || push_after || tx_discard) turn_go <= 1'b0;
      if (push_after || tx_discard) turn_run <= 1'b0;
    end
  end

  // ---------------------------------------------------------------------
  // RX
  // ---------------------------------------------------------------------
  reg        rx_busy;       // async frame in progress
  reg [31:0] rx_sh;
  reg [4:0]  rx_cnt;
  reg [15:0] rtmr;
  reg        rx_ovf, arb_lost, stuff_err;
  reg [6:0]  samp_d;        // delayed internal sample strobes (RX_DELAY)
  reg [7:0]  racc;          // fractional bit period accumulator (async RX timer)
  reg        rlvl;          // previous sampled level (NRZI decode)
  reg [2:0]  rrun;          // run length for destuffing
  reg        rlast;         // previous decoded bit
  reg [4:0]  rbits;         // decoded bits since the frame start (saturating), for RX_SKIP
  reg [3:0]  rwidx;         // words completed in this packet (saturating), for RXMATCH
  reg [2:0]  irun;          // consecutive idle-level samples (EOF_RUN)
  reg [31:0] rcrc;
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

  // Bit-level decode (async RX)
  wire       dbit       = nrzi ? (rx_in == rlvl) : rx_in;
  wire       rstuff_on  = async_mode && (stuff_n != 3'd0);
  wire       rx_is_stuf = rstuff_on && (rrun == stuff_n);
  wire       rstuf_bad  = stuff_eq ? (dbit == rlast) : dbit;
  wire [2:0] rrun_n     = stuff_eq ? ((dbit == rlast) ? rrun + 3'd1 : 3'd1)
                                   : (dbit ? rrun + 3'd1 : 3'd0);
  wire       se0_in     = diff && !rx_in && !gt_in;

  wire [31:0] rx_word   = rx_sh | ({31'd0, dbit} << rx_idx);
  // Async start: an edge from idle to active. A frame whose first sample
  // (middle of the start bit) is back at idle was a glitch and is dropped.
  // DIFF links are half duplex: RX ignores the line while TX drives it.
  wire       rx_mute    = diff && tx_busy;
  assign     rx_start   = async_mode && !rx_busy && !rx_mute && (rx_in != idle) && (rx_prev == idle);
  wire       rx_glitch  = async_mode && (rbits == 5'd0) && (rx_in == idle) && !se0_in;
  wire       rx_sample  = async_mode ? (rx_busy && (rtmr == half))
                                     : (clk_ext ? ext_sample : int_samp_d);
  wire       rx_eof     = rx_packet && (eof_run != 3'd0) && (rx_in == idle) &&
                          ({1'b0, irun} + 4'd1 >= {1'b0, eof_run});
  wire       rx_eop     = rx_sample && rx_packet && (se0_in || rx_eof);
  wire       rx_data    = rx_sample && !rx_eop && !rx_glitch && !rx_is_stuf;
  wire       rx_last    = (rx_cnt == rx_len);
  assign     rx_pkt_end = rx_eop;

  reg        end_pend;      // end marker still to push after a partial last word
  reg  [4:0] end_bits;
  wire data_push = rx_en && rx_data && rx_last;
  wire part_push = rx_en && rx_eop && (rx_cnt != 5'd0);
  wire ev_end    = (rx_eop && !part_push) || end_pend;
  wire ev_push   = events && (f_start || f_stop || ev_end) && !data_push && !part_push;
  wire [4:0] ev_bits = end_pend ? end_bits : 5'd0;
  assign rxf_push = data_push || part_push || ev_push;
  assign rxf_din  = data_push ? rx_word :
                    part_push ? rx_sh :
                    {19'd0, ev_bits, 6'd0, f_stop || ev_end, f_start} | 32'h8000_0000;

  always @(posedge clk) begin
    if (srst) begin
      end_pend <= 1'b0;
      end_bits <= 5'd0;
    end else if (part_push) begin
      end_pend <= 1'b1;
      end_bits <= rx_cnt;
    end else if (ev_push || !events) begin
      end_pend <= 1'b0;
    end
  end

  always @(posedge clk) begin
    if (srst) begin
      rx_busy <= 1'b0;
      rx_sh   <= 32'd0;
      rx_cnt  <= 5'd0;
      rtmr    <= 16'd0;
      racc    <= 8'd0;
      rlvl    <= 1'b1;
      rrun    <= 3'd0;
      rlast   <= 1'b1;
      rbits   <= 5'd0;
      rcrc    <= 32'd0;
      rwidx   <= 4'd0;
      irun    <= 3'd0;
      match_ok <= 1'b1;
    end else begin
      if (rx_start || (resync && rx_busy && rx_in != rx_prev)) begin
        rx_busy <= 1'b1;             // a start edge, or (RESYNC) any edge: realign the bit timer
        rtmr    <= div - 16'd1;
        if (rx_start) begin
          racc  <= 8'd0;
          rlvl  <= idle;
          rrun  <= 3'd0;
          rlast <= idle;
          rbits <= 5'd0;
          rwidx <= 4'd0;
          irun  <= 3'd0;
          match_ok <= (rxmatch[15:8] == 8'd0);
          rcrc  <= crcinit;
          rx_sh <= 32'd0;
          rx_cnt <= 5'd0;
        end
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
      end else if (rx_eop) begin       // end of packet (SE0)
        rx_sh   <= 32'd0;
        rx_cnt  <= 5'd0;
        rx_busy <= 1'b0;
      end else if (rx_sample) begin
        rlvl <= rx_in;
        irun <= (rx_in == idle) ? ((irun == 3'd7) ? irun : irun + 3'd1) : 3'd0;
        if (rx_glitch) begin
          rx_sh   <= 32'd0;
          rx_cnt  <= 5'd0;
          rx_busy <= 1'b0;
        end else if (rx_is_stuf) begin
          rrun  <= stuff_eq ? 3'd1 : 3'd0;
          rlast <= dbit;
        end else begin
          rrun  <= rrun_n;
          rlast <= dbit;
          if (rbits != 5'd31) rbits <= rbits + 5'd1;
          if (full && rbits >= rx_skip) rcrc <= crc_step(rcrc, dbit, crc_lm1, poly, crc_mask);
          if (rx_last) begin
            rx_sh   <= 32'd0;
            rx_cnt  <= 5'd0;
            if (!rx_packet) rx_busy <= 1'b0;
            if (rwidx != 4'd15) rwidx <= rwidx + 4'd1;
            if (rwidx == rxmatch[19:16] && rxmatch[15:8] != 8'd0)
              match_ok <= ((rx_word[7:0] ^ rxmatch[7:0]) & rxmatch[15:8]) == 8'd0;
          end else begin
            rx_sh   <= rx_word;
            rx_cnt  <= rx_cnt + 5'd1;
          end
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
  wire arb_off  = arb_stop && arb_lost;   // lost: send only the idle (recessive) level
  wire drv_line = (tx_shown && !arb_off) ? line : idle;

  assign tx_pin = txpin;
  assign tx_on  = en && tx_en;
  assign tx_val = od ? 1'b0 : drv_line;
  assign tx_oe  = (oe_mode == 2'd0) ? 1'b1 :
                  (oe_mode == 2'd1) ? (gated ? active : (tx_busy || tail || itail_on)) :
                  od                ? !drv_line : 1'b0;

  wire sck = tx_busy ? ((cpol ^ cpha) ^ second) : cpol;

  // CLK pin: bit clock, stretch (external clock), or the differential pair's second line
  assign clk_pin = ckpin;
  assign clk_on  = en && ((diff && tx_en) || (clk_out && !clk_ext) || (clk_ext && stretching));
  assign clk_val = diff ? (se0_now ? 1'b0 : !drv_line) :
                   (clk_ext || clk_od) ? 1'b0 : sck;
  assign clk_oe  = diff ? tx_oe : clk_ext ? 1'b1 : (clk_od ? !sck : 1'b1);

  // ---------------------------------------------------------------------
  // Registers
  // ---------------------------------------------------------------------
  wire tx_active = tx_busy || (tail && !clk_ext) || itail_on || !txf_empty;
  assign rx_avail = en && !rxf_empty;

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
        R_RX:      rdata = en ? rxf_dout : 32'd0;
        R_STATUS:  rdata = {5'd0, stuff_err, arb_lost, rx_ovf, 4'd0, stretching, active, rx_busy, tx_active,
                            4'd0, rxl, 4'd0, txl};
        R_CTRL:    rdata = {4'd0, ctrl};
        R_DIV:     rdata = {16'd0, div};
        R_PINS:    rdata = {3'd0, gtpin, 3'd0, ckpin, 3'd0, rxpin, 3'd0, txpin};
        R_CTRL2:   rdata = {10'd0, ctrl2};
        R_TXATTR:  rdata = full ? {26'd0, txattr} : 32'd0;
        R_CRCCFG:  rdata = full ? {19'd0, crccfg} : 32'd0;
        R_CRCINIT: rdata = full ? crcinit : 32'd0;
        R_RXCRC:   rdata = full ? rcrc : 32'd0;
        R_TURN:    rdata = full ? {16'd0, turn} : 32'd0;
        R_CRCPOLY: rdata = full ? poly : 32'd0;
        R_RXMATCH: rdata = full ? {12'd0, rxmatch} : 32'd0;
        default:   rdata = 32'd0;
      endcase
    end
  end

  wire cfg_wr = acc && write && reg_sel;

  always @(posedge clk) begin
    if (rst) begin
      ctrl    <= 28'd0;
      ctrl2   <= 22'd0;
      txattr  <= 6'd0;
      crccfg  <= 13'd0;
      crcinit <= 32'd0;
      poly    <= 32'd0;
      rxmatch <= 20'd0;
      turn    <= 16'd0;
      div     <= 16'd2;
      txpin   <= 5'd0;
      rxpin   <= 5'd0;
      ckpin   <= 5'd0;
      gtpin   <= 5'd0;
    end else if (cfg_wr) begin
      case (ra)
        R_CTRL:    ctrl    <= wdata[27:0];
        R_CTRL2:   ctrl2   <= wdata[21:0];
        R_TXATTR:  txattr  <= wdata[5:0];
        R_CRCCFG:  crccfg  <= wdata[12:0];
        R_CRCINIT: crcinit <= wdata;
        R_CRCPOLY: poly    <= wdata;
        R_RXMATCH: rxmatch <= wdata[19:0];
        R_TURN:    turn    <= wdata[15:0];
        R_DIV:     div     <= (wdata[15:0] < 16'd2) ? 16'd2 : wdata[15:0];
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

  // Read-back compare on words that are not NOSTUFF (CAN: the arbitration and
  // data fields; the ACK slot is in a NOSTUFF word, where a dominant bit is an
  // acknowledgement, not a lost arbitration)
  wire arb_check = (od || arb_stop) && tx_busy && !a_nostuff && line && !rx_in &&
                   (clk_ext ? ext_sample : int_sample);

  wire _unused = &{1'b0, tx_attr[5], tx_attr[2]};   // CRC_INIT and AFTER_RX act at load time

  always @(posedge clk) begin
    if (srst) begin
      rx_ovf    <= 1'b0;
      arb_lost  <= 1'b0;
      stuff_err <= 1'b0;
    end else begin
      if (rxf_push && rxf_full)                              rx_ovf <= 1'b1;
      else if (cfg_wr && ra == R_STATUS && wdata[24])       rx_ovf <= 1'b0;
      if (arb_check)                                         arb_lost <= 1'b1;
      else if (cfg_wr && ra == R_STATUS && wdata[25])       arb_lost <= 1'b0;
      if (rx_sample && rx_is_stuf && rstuf_bad && !rx_eop &&
          !(eof_run != 3'd0 && rx_in == idle))               stuff_err <= 1'b1;
      else if (cfg_wr && ra == R_STATUS && wdata[26])       stuff_err <= 1'b0;
    end
  end

endmodule
