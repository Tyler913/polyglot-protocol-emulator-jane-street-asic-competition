/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * pg_io: everything behind the CPU register port (0x8000_0000).
 *
 *   0x8000_0000  system registers, timed GPIO, capture (pg_sysio)
 *   0x8000_00F0  BLK_TIMEOUT RW  maximum wait of a blocking access in
 *                                cycles; 0 = wait forever (reset value)
 *   0x8000_00F4  BLK_FLAGS   RW  [0] a blocking access timed out (sticky,
 *                                write 1 to clear)
 *   0x8000_00F8  EVWAIT      RW  write: event mask; read: blocks until an
 *                                event in the mask is pending, returns the
 *                                pending events. Events: [n] Wire Engine n
 *                                has received data, [8] an edge is captured.
 *                                Lets C sleep on several sources at once.
 *   0x8000_0100 + 0x100*n  Wire Engine n (pg_wire), n = 0..NUM_WIRES-1
 *
 * A blocking access (full TX FIFO, empty RX FIFO, ...) holds HREADY low.
 * If it waits longer than BLK_TIMEOUT cycles it completes without effect:
 * writes are dropped, reads return 0xFFFF_FFFF, and BLK_FLAGS[0] is set.
 *
 * Pinmux: an enabled engine owns its TX pin and its CLK pin while it drives
 * it (CLK_OUT, or holding an external clock low); engine 0 has the highest
 * priority. Pins not owned by an engine follow
 * GPIO_OUT / GPIO_OE.
 */

`default_nettype none

module pg_io #(
    parameter NUM_WIRES = 2
) (
    input  wire        clk,
    input  wire        rst,

    // Register port (AHB data phase)
    input  wire        valid,
    input  wire        write,
    input  wire [11:0] addr,
    input  wire [31:0] wdata,
    output reg  [31:0] rdata,
    output wire        ready,

    // Pins
    output reg  [7:0]  uio_out,
    output reg  [7:0]  uio_oe,
    output reg  [5:0]  uo_out,
    output wire        host_irq,
    input  wire [7:0]  uio_in,
    input  wire [4:0]  ui_in
);

  // ---------------------------------------------------------------------
  // Two-flop input synchronizers. uo outputs are read back through the
  // same path so a loopback sees the same latency as a real input.
  // ---------------------------------------------------------------------
  reg [18:0] in_s1, in_s2;
  always @(posedge clk) begin
    in_s1 <= {ui_in, uo_out, uio_in};
    in_s2 <= in_s1;
  end
  wire [31:0] pin_in = {11'd0, in_s2[18:14], 2'd0, in_s2[13:0]};

  // ---------------------------------------------------------------------
  // Decode
  // ---------------------------------------------------------------------
  wire [3:0] page    = addr[11:8];
  wire       sel_sys = valid && (page == 4'd0) && (addr[7:4] != 4'hF);
  wire       sel_blk = valid && (page == 4'd0) && (addr[7:4] == 4'hF);

  wire [31:0] sys_rdata;
  wire        sys_ready;
  wire [13:0] gpio_out;
  wire [7:0]  gpio_oe;
  wire        cap_ready;

  pg_sysio u_sys (
      .clk     (clk),
      .rst     (rst),
      .valid   (sel_sys),
      .write   (write),
      .addr    (addr[7:0]),
      .wdata   (wdata),
      .rdata   (sys_rdata),
      .ready   (sys_ready),
      .pin_in  (pin_in),
      .gpio_out(gpio_out),
      .gpio_oe (gpio_oe),
      .host_irq(host_irq),
      .cap_ready(cap_ready)
  );

  wire [32*NUM_WIRES-1:0] w_rdata;
  wire [NUM_WIRES-1:0]    w_ready, w_sel;
  wire [5*NUM_WIRES-1:0]  w_tx_pin, w_clk_pin;
  wire [NUM_WIRES-1:0]    w_tx_on, w_tx_val, w_tx_oe, w_clk_on, w_clk_val, w_clk_oe, w_rx_avail;

  genvar g;
  generate
    for (g = 0; g < NUM_WIRES; g = g + 1) begin : g_wire
      assign w_sel[g] = valid && (page == g + 1);
      pg_wire u_wire (
          .clk    (clk),
          .rst    (rst),
          .valid  (w_sel[g]),
          .write  (write),
          .addr   (addr[7:0]),
          .wdata  (wdata),
          .rdata  (w_rdata[32*g +: 32]),
          .ready  (w_ready[g]),
          .pin_in (pin_in),
          .tx_pin (w_tx_pin[5*g +: 5]),
          .tx_on  (w_tx_on[g]),
          .tx_val (w_tx_val[g]),
          .tx_oe  (w_tx_oe[g]),
          .clk_pin(w_clk_pin[5*g +: 5]),
          .clk_on (w_clk_on[g]),
          .clk_val(w_clk_val[g]),
          .clk_oe (w_clk_oe[g]),
          .rx_avail(w_rx_avail[g])
      );
    end
  endgenerate

  // ---------------------------------------------------------------------
  // Blocking timeout
  // ---------------------------------------------------------------------
  reg  [31:0] blk_limit, blk_cnt;
  reg         blk_flag;
  reg  [8:0]  ev_mask;
  wire [8:0]  ev_pend   = {cap_ready, {(8-NUM_WIRES){1'b0}}, w_rx_avail} & ev_mask;
  wire        ev_rd     = sel_blk && !write && addr[3:0] == 4'h8;
  wire        ev_ready  = !ev_rd || (ev_pend != 9'd0);
  wire        slave_ready = sys_ready && (&w_ready) && ev_ready;
  wire        timed_out   = valid && !slave_ready && (blk_limit != 32'd0) && (blk_cnt >= blk_limit);

  assign ready = slave_ready || timed_out;

  always @(posedge clk) begin
    if (rst) begin
      blk_cnt   <= 32'd0;
      blk_limit <= 32'd0;
      blk_flag  <= 1'b0;
      ev_mask   <= 9'd0;
    end else begin
      blk_cnt <= (valid && !ready) ? blk_cnt + 32'd1 : 32'd0;
      if (timed_out) blk_flag <= 1'b1;
      if (sel_blk && write) begin
        if (addr[3:0] == 4'h0) blk_limit <= wdata;
        if (addr[3:0] == 4'h4 && wdata[0]) blk_flag <= 1'b0;
        if (addr[3:0] == 4'h8) ev_mask <= wdata[8:0];
      end
    end
  end

  // ---------------------------------------------------------------------
  // Read data
  // ---------------------------------------------------------------------
  integer i;
  always @(*) begin
    rdata = 32'd0;
    if (page == 4'd0) begin
      if (addr[7:4] != 4'hF)       rdata = sys_rdata;
      else if (addr[3:0] == 4'h0)  rdata = blk_limit;
      else if (addr[3:0] == 4'h4)  rdata = {31'd0, blk_flag};
      else if (addr[3:0] == 4'h8)  rdata = {23'd0, ev_pend};
    end
    for (i = 0; i < NUM_WIRES; i = i + 1)
      if (page == i[3:0] + 4'd1) rdata = w_rdata[32*i +: 32];
    if (!slave_ready) rdata = 32'hFFFF_FFFF;
  end

  // ---------------------------------------------------------------------
  // Pinmux (pins 0..7 = uio, 8..13 = uo[0..5]); engine 0 wins
  // ---------------------------------------------------------------------
  reg [13:0] p_out;
  reg [13:0] p_oe;
  integer e, p;
  always @(*) begin
    p_out = gpio_out;
    p_oe  = {6'h3F, gpio_oe};
    for (e = NUM_WIRES - 1; e >= 0; e = e - 1) begin
      for (p = 0; p < 14; p = p + 1) begin
        if (w_clk_on[e] && w_clk_pin[5*e +: 5] == p[4:0]) begin
          p_out[p] = w_clk_val[e];
          p_oe[p]  = w_clk_oe[e];
        end
        if (w_tx_on[e] && w_tx_pin[5*e +: 5] == p[4:0]) begin
          p_out[p] = w_tx_val[e];
          p_oe[p]  = w_tx_oe[e];
        end
      end
    end
    uio_out = p_out[7:0];
    uio_oe  = p_oe[7:0];
    uo_out  = p_out[13:8];
  end

  wire _unused = &{1'b0, p_oe[13:8]};

endmodule
