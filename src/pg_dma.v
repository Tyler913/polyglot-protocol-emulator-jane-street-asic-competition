/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * pg_dma: two DMA channels between a ring buffer in SRAM and one IO register
 * (any address behind the register port: a Wire Engine's RX or TX register,
 * GPIO_IN, GPIO_OUT, TG_PUSH, ...). The DMA is a second master on the IO bus
 * (it takes a cycle from the CPU, whose access simply waits one cycle) and a
 * requester on the SRAM port (after CPU writes and the host). Each word moves
 * when its pace allows: a timer tick (every PERIOD cycles), "the engine has
 * received data", "the engine's TX FIFO has room", or always.
 *
 * Registers (0x8000_0500 + 0x20 * channel, word access):
 *   0x00 CTRL   RW  [0] EN, [1] TO_IO (0: IO register -> SRAM, 1: SRAM -> IO
 *                   register), [3:2] PACE (0 timer, 1 engine RX data, 2
 *                   engine TX room, 3 free), [5:4] engine for PACE 1/2,
 *                   [6] STREAM (timer pace): the ring is a continuous stream,
 *                   so a tick that finds an IO -> SRAM ring full or an
 *                   SRAM -> IO ring empty is missed (a lost or late sample)
 *                   instead of the normal end of a one-shot run
 *   0x04 REG    RW  [11:0] IO register offset (from 0x8000_0000)
 *   0x08 START  RW  ring start, byte address in SRAM (word aligned)
 *   0x0C END    RW  ring end (exclusive)
 *   0x10 HEAD   RW  DMA position (next word it reads or writes)
 *   0x14 TAIL   RW  CPU position. IO -> SRAM: C has read up to TAIL; the DMA
 *                   stops while the ring is full (HEAD one word behind TAIL),
 *                   so START = TAIL captures END - START - 4 bytes once.
 *                   SRAM -> IO: C has written up to TAIL; the DMA sends while
 *                   HEAD != TAIL.
 *   0x18 PERIOD RW  [15:0] timer pace period in cycles (>= 1)
 *   0x1C STATUS R   [0] missed (sticky, write 1 to clear): a timer tick
 *                   found the channel not ready (the previous word not yet
 *                   stored, or the next one not yet fetched); an empty
 *                   SRAM -> IO ring or a full IO -> SRAM ring is the normal
 *                   end, not a miss, unless CTRL.STREAM is set.
 *                   [1] busy, [2] ring empty, [3] ring full
 *
 * Timer pace is exact: the IO access happens in the tick's cycle (SRAM ->
 * IO words are fetched ahead), unless the other channel uses the IO bus in
 * that cycle (channel 0 first), which delays it by one cycle.
 */

`default_nettype none

module pg_dma #(
    parameter NUM_WIRES = 4
) (
    input  wire        clk,
    input  wire        rst,

    // CPU register port (its page)
    input  wire        valid,
    input  wire        write,
    input  wire [7:0]  addr,
    input  wire [31:0] wdata,
    output reg  [31:0] rdata,

    // IO bus master: the access completes in a cycle with bus_req && bus_ready
    output wire        bus_req,
    output wire        bus_write,
    output wire [11:0] bus_addr,
    output wire [31:0] bus_wdata,
    input  wire [31:0] bus_rdata,
    input  wire        bus_ready,

    // Pace
    input  wire [NUM_WIRES-1:0] rx_avail,
    input  wire [NUM_WIRES-1:0] tx_room,

    // SRAM requester: the access happens in a cycle with sram_req && sram_gnt;
    // read data comes the cycle after
    output wire        sram_req,
    output wire        sram_we,
    output wire [11:0] sram_addr,
    output wire [31:0] sram_wdata,
    input  wire        sram_gnt,
    input  wire [31:0] sram_rdata,

    // Events: channel k (IO -> SRAM) has words for C
    output wire [1:0]  ev
);

  reg  [6:0]  ctrl   [0:1];
  reg  [11:0] ioreg  [0:1];
  reg  [11:0] start  [0:1];   // SRAM word addresses (14 KB)
  reg  [11:0] rend   [0:1];
  reg  [11:0] head   [0:1];
  reg  [11:0] tail   [0:1];
  reg  [15:0] period [0:1];
  reg  [15:0] tmr    [0:1];
  reg  [31:0] buf_d  [0:1];
  reg  [1:0]  st     [0:1];   // see below
  reg         pend   [0:1];   // timer tick waiting for the IO bus
  reg         missed [0:1];

  wire [1:0] want_bus, want_sram, bus_done, sram_done, tick, idle_end, lost;
  wire [11:0] nxt [0:1];

  genvar k;
  generate
    for (k = 0; k < 2; k = k + 1) begin : g_ch
      wire        en    = ctrl[k][0];
      wire        to_io = ctrl[k][1];
      wire [1:0]  pace  = ctrl[k][3:2];
      wire [1:0]  eng   = ctrl[k][5:4];
      wire        strm  = ctrl[k][6];
      wire        ptick = (tmr[k] == 16'd0);
      wire        epace = (pace == 2'd1) ? rx_avail[eng] : (pace == 2'd2) ? tx_room[eng] : 1'b1;
      assign nxt[k]  = (head[k] + 12'd1 == rend[k]) ? start[k] : head[k] + 12'd1;
      wire        full  = (nxt[k] == tail[k]);
      wire        empty = (head[k] == tail[k]);
      assign tick[k] = en && pace == 2'd0 && ptick;
      // nothing left to do: an empty SRAM -> IO ring, a full IO -> SRAM ring
      assign idle_end[k] = to_io ? (st[k] == 2'd0 && empty) : full;
      assign lost[k]     = strm && idle_end[k];         // a stream ran dry / over
      wire        go    = (pace == 2'd0) ? (ptick || pend[k]) : epace;
      // IO -> SRAM: st 0 wait, IO read -> st 1, SRAM write -> st 0
      // SRAM -> IO: st 0 SRAM read -> st 1, data -> st 2, IO write -> st 0
      assign want_bus[k]  = en && (to_io ? (st[k] == 2'd2 && go) : (st[k] == 2'd0 && go && !full));
      assign want_sram[k] = en && (to_io ? (st[k] == 2'd0 && !empty) : (st[k] == 2'd1));
    end
  endgenerate

  // Channel 0 first on both ports
  wire use0_bus  = want_bus[0];
  wire use0_sram = want_sram[0];
  assign bus_req   = want_bus[0] || want_bus[1];
  assign bus_write = use0_bus ? ctrl[0][1] : ctrl[1][1];
  assign bus_addr  = use0_bus ? ioreg[0] : ioreg[1];
  assign bus_wdata = use0_bus ? buf_d[0] : buf_d[1];
  assign bus_done  = {want_bus[1] && !use0_bus && bus_ready, use0_bus && bus_ready};
  assign sram_req   = want_sram[0] || want_sram[1];
  assign sram_we    = use0_sram ? !ctrl[0][1] : !ctrl[1][1];
  assign sram_addr  = use0_sram ? head[0] : head[1];
  assign sram_wdata = use0_sram ? buf_d[0] : buf_d[1];
  assign sram_done  = {want_sram[1] && !use0_sram && sram_gnt, use0_sram && sram_gnt};

  assign ev = {ctrl[1][0] && !ctrl[1][1] && head[1] != tail[1],
               ctrl[0][0] && !ctrl[0][1] && head[0] != tail[0]};

  // CPU register writes
  wire       cfg_wr = valid && write;
  wire       ch     = addr[5];
  wire [2:0] ra     = addr[4:2];

  integer c;
  always @(posedge clk) begin
    for (c = 0; c < 2; c = c + 1) begin
      if (rst) begin
        ctrl[c]   <= 7'd0;
        ioreg[c]  <= 12'd0;
        start[c]  <= 12'd0;
        rend[c]   <= 12'd0;
        head[c]   <= 12'd0;
        tail[c]   <= 12'd0;
        period[c] <= 16'd1;
        tmr[c]    <= 16'd0;
        buf_d[c]  <= 32'd0;
        st[c]     <= 2'd0;
        pend[c]   <= 1'b0;
        missed[c] <= 1'b0;
      end else begin
        // timer: ticks every PERIOD cycles while the channel is on
        if (!ctrl[c][0] || tmr[c] == 16'd0) tmr[c] <= period[c] - 16'd1;
        else                                tmr[c] <= tmr[c] - 16'd1;
        if (ctrl[c][1]) begin                           // SRAM -> IO
          if (sram_done[c])                 st[c] <= 2'd1;
          else if (st[c] == 2'd1) begin
            buf_d[c] <= sram_rdata;
            st[c]    <= 2'd2;
          end else if (bus_done[c]) begin
            head[c]  <= nxt[c];
            st[c]    <= 2'd0;
          end
        end else begin                                  // IO -> SRAM
          if (bus_done[c]) begin
            buf_d[c] <= bus_rdata;
            st[c]    <= 2'd1;
          end else if (sram_done[c]) begin
            head[c]  <= nxt[c];
            st[c]    <= 2'd0;
          end
        end
        // a timer tick that cannot use the IO bus at once waits for it; it is
        // "missed" (late) if the channel was not ready or a tick was pending
        if (bus_done[c])                    pend[c] <= 1'b0;
        else if (tick[c] && !idle_end[c])   pend[c] <= 1'b1;
        if (tick[c] && (lost[c] || (!idle_end[c] && (pend[c] || !want_bus[c])))) missed[c] <= 1'b1;
        if (!ctrl[c][0]) begin
          st[c]   <= 2'd0;
          pend[c] <= 1'b0;
        end
        if (cfg_wr && ch == c[0]) begin
          case (ra)
            3'd0: ctrl[c]   <= wdata[6:0];
            3'd1: ioreg[c]  <= wdata[11:0];
            3'd2: start[c]  <= wdata[13:2];
            3'd3: rend[c]   <= wdata[13:2];
            3'd4: head[c]   <= wdata[13:2];
            3'd5: tail[c]   <= wdata[13:2];
            3'd6: period[c] <= (wdata[15:0] == 16'd0) ? 16'd1 : wdata[15:0];
            3'd7: if (wdata[0]) missed[c] <= 1'b0;
            default: ;
          endcase
        end
      end
    end
  end

  always @(*) begin
    case (ra)
      3'd0:    rdata = {25'd0, ctrl[ch]};
      3'd1:    rdata = {20'd0, ioreg[ch]};
      3'd2:    rdata = {18'd0, start[ch], 2'b00};
      3'd3:    rdata = {18'd0, rend[ch], 2'b00};
      3'd4:    rdata = {18'd0, head[ch], 2'b00};
      3'd5:    rdata = {18'd0, tail[ch], 2'b00};
      3'd6:    rdata = {16'd0, period[ch]};
      default: rdata = {28'd0, nxt[ch] == tail[ch], head[ch] == tail[ch], st[ch] != 2'd0, missed[ch]};
    endcase
  end

  wire _unused = &{1'b0, addr[7:6], addr[1:0], wdata[31:16]};

endmodule
