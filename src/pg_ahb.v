/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * pg_ahb: AHB-Lite slave fabric between the Hazard3 bus and
 *   - the SRAM (haddr[31] == 0), zero wait states for reads,
 *   - the register port (haddr[31] == 1, 4 KB window), word access only, in
 *     the data phase; the register slave may insert wait states with reg_ready = 0
 *     (this is how blocking FIFO reads stall the CPU). The register read data
 *     and the ready flag are registered here: every register access takes one
 *     extra cycle, and the slave's decode/ready logic never reaches the CPU
 *     in the same cycle (it was the slow-corner critical path at 50 MHz),
 *   - the host SPI loader, which owns the SRAM while the CPU is halted. While
 *     the CPU runs, a host byte access has priority over a CPU SRAM read
 *     (the CPU waits one cycle; only a pending CPU write goes first), so every
 *     host byte is served within two cycles. Earlier the host only used idle
 *     SRAM cycles, and a busy CPU could hold it off for longer than one SPI
 *     byte: the host port then dropped the byte (found by the job-queue test).
 *
 * SRAM writes happen in the AHB data phase. If a read address phase arrives
 * in the same cycle, the fabric stalls one cycle (the single SRAM port can
 * only do one access per cycle).
 */

`default_nettype none

module pg_ahb (
    input  wire        clk,
    input  wire        rst,
    input  wire        cpu_halted,

    // AHB-Lite slave side (from the CPU)
    input  wire [31:0] haddr,
    input  wire        hwrite,
    input  wire [1:0]  htrans,
    input  wire [2:0]  hsize,
    input  wire [31:0] hwdata,
    output wire        hready,
    output wire [31:0] hrdata,

    // SRAM port
    output reg         sram_en,
    output reg         sram_we,
    output reg  [9:0]  sram_addr,
    output reg  [31:0] sram_wdata,
    output reg  [31:0] sram_wmask,
    input  wire [31:0] sram_rdata,

    // Register port (data phase)
    output wire        reg_valid,
    output wire        reg_write,
    output wire [11:0] reg_addr,
    output wire [31:0] reg_wdata,
    input  wire [31:0] reg_rdata,
    input  wire        reg_ready,

    // Host byte port
    input  wire        host_req,
    input  wire        host_we,
    input  wire [15:0] host_addr,
    input  wire [7:0]  host_wdata,
    output wire        host_ack,
    output reg  [7:0]  host_rdata
);

  // ---------------------------------------------------------------------
  // Address phase decode
  // ---------------------------------------------------------------------
  wire ap_valid = htrans[1];
  wire ap_sram  = ap_valid && !haddr[31];
  wire ap_io    = ap_valid &&  haddr[31];

  // Data phase state, captured when an address phase is accepted
  reg        dp_sram, dp_io, dp_write;
  reg [31:0] dp_addr;
  reg [2:0]  dp_size;
  reg        wr_done;  // the data-phase SRAM write was already performed
  reg        io_done;  // the register slave accepted the access last cycle
  reg [31:0] alt_d;    // register read data, or a held SRAM word (see below)

  wire dp_sram_write = dp_sram && dp_write;
  wire dp_sram_read  = dp_sram && !dp_write;
  reg  hold_v;        // alt_d holds an SRAM word (see the read data mux below)
  wire wr_now        = dp_sram_write && !wr_done;
  wire ap_sram_read  = ap_sram && !hwrite;

  // Host byte access: anytime the CPU is halted; while it runs, unless the
  // data-phase CPU write needs the port this cycle.
  reg  host_busy;     // host access issued last cycle; ack now
  wire host_go = host_req && !host_busy && (cpu_halted || !wr_now);

  // Stall when the pending write and a new SRAM read compete for the port,
  // when the host takes the port from a new SRAM read, or when the register
  // slave is not ready.
  wire stall_sram = wr_now && ap_sram_read;
  wire stall_host = !cpu_halted && host_go && ap_sram_read;
  wire stall_io   = dp_io && !io_done;
  assign hready   = !stall_sram && !stall_host && !stall_io;

  always @(posedge clk) begin
    if (rst) begin
      dp_sram  <= 1'b0;
      dp_io    <= 1'b0;
      dp_write <= 1'b0;
      dp_addr  <= 32'd0;
      dp_size  <= 3'd0;
      wr_done  <= 1'b0;
      io_done  <= 1'b0;
      alt_d    <= 32'd0;
      hold_v   <= 1'b0;
    end else begin
      if (reg_valid && reg_ready) begin
        io_done  <= 1'b1;
        alt_d    <= reg_rdata;
      end else if (hready) begin
        io_done  <= 1'b0;
      end
      // Hold the SRAM word when a host access stretches a CPU read data phase
      if (hready) begin
        hold_v   <= 1'b0;
      end else if (dp_sram_read && !hold_v) begin
        hold_v   <= 1'b1;
        alt_d    <= sram_rdata;
      end
      if (hready) begin
        dp_sram  <= ap_sram;
        dp_io    <= ap_io;
        dp_write <= hwrite;
        dp_addr  <= haddr;
        dp_size  <= hsize;
        wr_done  <= 1'b0;
      end else if (wr_now) begin
        wr_done  <= 1'b1;
      end
    end
  end

  // Byte mask for a write of size dp_size at dp_addr
  reg [3:0] dp_strb;
  always @(*) begin
    case (dp_size[1:0])
      2'd0:    dp_strb = 4'b0001 << dp_addr[1:0];
      2'd1:    dp_strb = dp_addr[1] ? 4'b1100 : 4'b0011;
      default: dp_strb = 4'b1111;
    endcase
  end

  // ---------------------------------------------------------------------
  // SRAM port
  // ---------------------------------------------------------------------
  reg  [1:0] host_lane;

  always @(*) begin
    sram_en    = 1'b0;
    sram_we    = 1'b0;
    sram_addr  = 10'd0;
    sram_wdata = 32'd0;
    sram_wmask = 32'd0;
    if (!cpu_halted && wr_now) begin
      sram_en    = 1'b1;
      sram_we    = 1'b1;
      sram_addr  = dp_addr[11:2];
      sram_wdata = hwdata;
      sram_wmask = {{8{dp_strb[3]}}, {8{dp_strb[2]}}, {8{dp_strb[1]}}, {8{dp_strb[0]}}};
    end else if (host_go) begin
      sram_en    = 1'b1;
      sram_we    = host_we;
      sram_addr  = host_addr[11:2];
      sram_wdata = {4{host_wdata}};
      sram_wmask = {{8{host_addr[1:0] == 2'd3}}, {8{host_addr[1:0] == 2'd2}},
                    {8{host_addr[1:0] == 2'd1}}, {8{host_addr[1:0] == 2'd0}}};
    end else if (!cpu_halted && ap_sram_read && hready) begin
      sram_en    = 1'b1;
      sram_addr  = haddr[11:2];
    end
  end

  always @(posedge clk) begin
    if (rst) begin
      host_busy <= 1'b0;
      host_lane <= 2'd0;
    end else begin
      host_busy <= host_go;
      if (host_go) host_lane <= host_addr[1:0];
    end
  end

  assign host_ack = host_busy;
  always @(*) begin
    case (host_lane)
      2'd0:    host_rdata = sram_rdata[7:0];
      2'd1:    host_rdata = sram_rdata[15:8];
      2'd2:    host_rdata = sram_rdata[23:16];
      default: host_rdata = sram_rdata[31:24];
    endcase
  end

  // ---------------------------------------------------------------------
  // Register port and read data
  // ---------------------------------------------------------------------
  assign reg_valid = dp_io && !io_done;
  assign reg_write = dp_write;
  assign reg_addr  = dp_addr[11:0];
  assign reg_wdata = hwdata;

  // A host access can stretch the data phase of a CPU SRAM read by one cycle
  // (stall_host); the SRAM output changes meanwhile, so the read word is held.
  // It is held in alt_d, the register that otherwise carries register-port read
  // data: the two never overlap (a hold happens only in an SRAM read data phase).
  // So the SRAM output reaches the CPU through one 2-input mux with a registered
  // select, as before; this path is the slow-corner critical path.
  wire alt_sel = dp_io || hold_v;   // both registered
  assign hrdata = alt_sel ? alt_d : sram_rdata;

  wire _unused = &{1'b0, dp_addr[31:12], htrans[0], dp_size[2], host_addr[15:12]};

endmodule
