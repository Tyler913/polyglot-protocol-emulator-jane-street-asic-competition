/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * pg_sysio: system registers seen by the CPU at 0x8000_0000: timebase,
 * GPIO, timed GPIO and input edge capture. Word access only. Reads are
 * combinational in the AHB data phase; some reads block (ready = 0).
 *
 *   0x00 ID        RO  0x5047_0003 ("PG", version 3)
 *   0x04 NOW       RO  free-running cycle counter
 *   0x10 GPIO_OUT  RW  output value, one bit per pin (pins 0..13)
 *   0x14 GPIO_OE   RW  output enable for bidirectional pins (bits 7:0)
 *   0x18 GPIO_IN   RO  synchronized pin inputs (pins 0..7, 16..20; 8..13 read
 *                      back the uo outputs through the same synchronizer)
 *   0x20 HOST_MBOX RW  32-bit mailbox (host access added later)
 *   0x24 HOST_IRQ  RW  [0] drives the HOST_IRQ pin (uo[6])
 *   0x48 PARK      R   blocks forever (or until BLK_TIMEOUT). Firmware parks
 *                      here when done so the CPU stops using the bus and the
 *                      host can read the SRAM at full speed.
 *
 *   Timed GPIO: a queue of events, each "at cycle T, set the pins in MASK
 *   to VALUE" in GPIO_OUT (or GPIO_OE). The event takes effect exactly in
 *   the cycle in which NOW == T. An event whose time has already passed
 *   takes effect at once and sets LATE. Events must be pushed in time order.
 *   0x30 TG_TIME   RW  time T for the next pushed event
 *   0x34 TG_PUSH   W   [13:0] VALUE, [29:16] MASK, [31] 1 = target GPIO_OE;
 *                      pushes {TG_TIME, ...}; blocks while the queue is full
 *   0x38 TG_STATUS R   [3:0] queue level, [8] LATE (sticky, write 1 to
 *                      clear), [16] capture valid
 *   0x3C TG_WAIT   R   blocks until the queue is empty; reads NOW
 *
 *   Edge capture: one channel. Writing CAP_CFG selects the pin and edge and
 *   arms it. The first matching edge stores the cycle in which the pin
 *   changed. Reading CAP_TIME blocks until an edge was captured, returns
 *   its time and re-arms.
 *   0x40 CAP_CFG   RW  [4:0] pin, [9:8] edge (1 rising, 2 falling, 3 both)
 *   0x44 CAP_TIME  R   blocking, see above
 *
 * Pin numbering (shared with protocol.h):
 *   0..7   uio[0..7]  bidirectional
 *   8..13  uo[0..5]   outputs
 *   16..20 ui[0..4]   inputs
 */

`default_nettype none

module pg_sysio #(
    parameter TG_AW = 2
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
    input  wire [31:0] pin_in,     // synchronized inputs
    output reg  [13:0] gpio_out,
    output reg  [7:0]  gpio_oe,
    output reg         host_irq
);

  localparam [31:0] ID = 32'h5047_0003;

  wire acc = valid && ready;
  wire wr  = acc && write;

  // ---------------------------------------------------------------------
  // Timebase
  // ---------------------------------------------------------------------
  reg  [31:0] now;
  wire [31:0] now_nx = now + 32'd1;
  always @(posedge clk) begin
    if (rst) now <= 32'd0;
    else     now <= now_nx;
  end

  // ---------------------------------------------------------------------
  // Timed GPIO queue: {oe, mask, value, time}
  // ---------------------------------------------------------------------
  reg  [31:0]    tg_time;
  wire [60:0]    tg_head;
  wire [TG_AW:0] tg_level;
  wire           tg_full, tg_empty;
  reg            tg_late;

  wire        tg_push  = wr && (addr == 8'h34);
  wire [31:0] h_time   = tg_head[31:0];
  wire [13:0] h_value  = tg_head[45:32];
  wire [13:0] h_mask   = tg_head[59:46];
  wire        h_oe     = tg_head[60];

  // Fire at the clock edge where NOW becomes h_time (or at once if passed)
  wire [31:0] tg_diff  = now_nx - h_time;
  wire        tg_fire  = !tg_empty && !tg_diff[31];
  wire        tg_miss  = tg_fire && (tg_diff != 32'd0);

  pg_fifo #(.W(61), .AW(TG_AW)) u_tgq (
      .clk(clk), .rst(rst), .push(tg_push),
      .din({wdata[31], wdata[29:16], wdata[13:0], tg_time}),
      .pop(tg_fire), .dout(tg_head), .level(tg_level),
      .full(tg_full), .empty(tg_empty)
  );

  // ---------------------------------------------------------------------
  // GPIO registers (CPU writes first, a firing event wins)
  // ---------------------------------------------------------------------
  reg [31:0] mbox;

  always @(posedge clk) begin
    if (rst) begin
      gpio_out <= 14'd0;
      gpio_oe  <= 8'd0;
      mbox     <= 32'd0;
      tg_time  <= 32'd0;
      host_irq <= 1'b0;
    end else begin
      if (wr) begin
        case (addr)
          8'h10: gpio_out <= wdata[13:0];
          8'h14: gpio_oe  <= wdata[7:0];
          8'h20: mbox     <= wdata;
          8'h24: host_irq <= wdata[0];
          8'h30: tg_time  <= wdata;
          default: ;
        endcase
      end
      if (tg_fire) begin
        if (h_oe) gpio_oe  <= (gpio_oe & ~h_mask[7:0]) | (h_value[7:0] & h_mask[7:0]);
        else      gpio_out <= (gpio_out & ~h_mask) | (h_value & h_mask);
      end
    end
  end

  always @(posedge clk) begin
    if (rst)                               tg_late <= 1'b0;
    else if (tg_miss)                      tg_late <= 1'b1;
    else if (wr && addr == 8'h38 && wdata[8]) tg_late <= 1'b0;
  end

  // ---------------------------------------------------------------------
  // Edge capture. pin_in is two flops behind the pin, cap_q one more, so
  // the pin changed in cycle NOW - 2 when the edge is seen here.
  // ---------------------------------------------------------------------
  reg  [4:0]  cap_pin;
  reg  [1:0]  cap_edge;
  reg         cap_skip, cap_valid, cap_q;
  reg  [31:0] cap_time;

  wire cap_in   = pin_in[cap_pin];
  wire cap_hit  = !cap_skip && !cap_valid &&
                  ((cap_edge[0] && cap_in && !cap_q) || (cap_edge[1] && !cap_in && cap_q));
  wire cap_read = acc && !write && (addr == 8'h44);

  always @(posedge clk) begin
    cap_q <= cap_in;
    if (rst) begin
      cap_pin   <= 5'd0;
      cap_edge  <= 2'd0;
      cap_skip  <= 1'b1;
      cap_valid <= 1'b0;
      cap_time  <= 32'd0;
    end else if (wr && addr == 8'h40) begin
      cap_pin   <= wdata[4:0];
      cap_edge  <= wdata[9:8];
      cap_skip  <= 1'b1;        // cap_q still holds the old pin
      cap_valid <= 1'b0;
    end else begin
      cap_skip <= 1'b0;
      if (cap_hit) begin
        cap_valid <= 1'b1;
        cap_time  <= now - 32'd2;
      end else if (cap_read) begin
        cap_valid <= 1'b0;
      end
    end
  end

  // ---------------------------------------------------------------------
  // Register reads and blocking
  // ---------------------------------------------------------------------
  wire [3:0] tgl = {{(3-TG_AW){1'b0}}, tg_level};   // TG_AW <= 3

  always @(*) begin
    ready = 1'b1;
    if (valid) begin
      case (addr)
        8'h34:   ready = !write || !tg_full;
        8'h3C:   ready = write || tg_empty;
        8'h44:   ready = write || cap_valid;
        8'h48:   ready = write;
        default: ready = 1'b1;
      endcase
    end
  end

  always @(*) begin
    case (addr)
      8'h00:   rdata = ID;
      8'h04:   rdata = now;
      8'h10:   rdata = {18'd0, gpio_out};
      8'h14:   rdata = {24'd0, gpio_oe};
      8'h18:   rdata = pin_in;
      8'h20:   rdata = mbox;
      8'h24:   rdata = {31'd0, host_irq};
      8'h30:   rdata = tg_time;
      8'h38:   rdata = {15'd0, cap_valid, 7'd0, tg_late, 4'd0, tgl};
      8'h3C:   rdata = now;
      8'h40:   rdata = {22'd0, cap_edge, 3'd0, cap_pin};
      8'h44:   rdata = cap_time;
      default: rdata = 32'd0;
    endcase
  end

  wire _unused = &{1'b0, wdata[30], wdata[15:14], h_mask[13:8], h_value[13:8]};

endmodule
