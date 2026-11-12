/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * tt_um_tyler913_polyglot: top level.
 *
 * A Hazard3 core (RV32EC, by Luke Wren, Apache-2.0; the core used in the
 * RP2350) runs C programs from on-chip SRAM. The host MCU loads the SRAM and
 * starts the CPU over an SPI target port.
 *
 * Pins
 *   ui[4:0]   protocol inputs (pins 16..20)
 *   ui[5]     HOST_SCK
 *   ui[6]     HOST_MOSI
 *   ui[7]     HOST_CS_N
 *   uo[5:0]   protocol outputs (pins 8..13)
 *   uo[6]     HOST_IRQ (reserved, 0)
 *   uo[7]     HOST_MISO
 *   uio[7:0]  bidirectional protocol pins (pins 0..7)
 *
 * CPU memory map
 *   0x0000_0000 - 0x0000_0FFF  SRAM, 4 KB (code + data)
 *   0x8000_0000 - 0x8000_00FF  system registers (pg_sysio)
 *
 * Host CTRL register (SPI command 0x03)
 *   bit 0  RUN: 1 releases the CPU from reset
 * Host STATUS register (SPI command 0x04)
 *   bit 0  RUN
 */

`default_nettype none

module tt_um_tyler913_polyglot (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);

  // ---------------------------------------------------------------------
  // Reset: asynchronous assert, synchronous release
  // ---------------------------------------------------------------------
  reg [1:0] rst_sync;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) rst_sync <= 2'b00;
    else        rst_sync <= {rst_sync[0], 1'b1};
  end
  wire rst = ~rst_sync[1];

  // ---------------------------------------------------------------------
  // Host SPI port
  // ---------------------------------------------------------------------
  wire        host_req, host_we, host_ack;
  wire [15:0] host_addr;
  wire [7:0]  host_wdata, host_rdata;
  wire [7:0]  host_ctrl;
  wire        host_miso;

  pg_host_spi u_host (
      .clk      (clk),
      .rst      (rst),
      .spi_sck  (ui_in[5]),
      .spi_mosi (ui_in[6]),
      .spi_cs_n (ui_in[7]),
      .spi_miso (host_miso),
      .mem_req  (host_req),
      .mem_we   (host_we),
      .mem_addr (host_addr),
      .mem_wdata(host_wdata),
      .mem_ack  (host_ack),
      .mem_rdata(host_rdata),
      .ctrl     (host_ctrl),
      .status   ({7'd0, host_ctrl[0]})
  );

  // CPU reset: registered so the CPU sees a clean synchronous signal
  reg cpu_run;
  always @(posedge clk) begin
    if (rst) cpu_run <= 1'b0;
    else     cpu_run <= host_ctrl[0];
  end

  // ---------------------------------------------------------------------
  // CPU
  // ---------------------------------------------------------------------
  wire [31:0] haddr, hwdata, hrdata;
  wire        hwrite, hready;
  wire [1:0]  htrans;
  wire [2:0]  hsize;
  wire        pwrup_req, unblock_out;

  hazard3_cpu_1port #(
      .RESET_VECTOR       (32'h0000_0000),
      .MTVEC_INIT         (32'h0000_0000),
      .EXTENSION_A        (0),
      .EXTENSION_C        (1),
      .EXTENSION_E        (1),
      .EXTENSION_M        (0),
      .CSR_M_MANDATORY    (1),
      .CSR_M_TRAP         (1),
      .CSR_COUNTER        (0),
      .DEBUG_SUPPORT      (0),
      .PMP_REGIONS        (0),
      .BREAKPOINT_TRIGGERS(0),
      .NUM_IRQS           (1)
  ) u_cpu (
      .clk                       (clk),
      .clk_always_on             (clk),
      .rst_n                     (cpu_run),
      .pwrup_req                 (pwrup_req),
      .pwrup_ack                 (pwrup_req),     // tied back: no power control
      .clk_en                    (),
      .unblock_out               (unblock_out),
      .unblock_in                (unblock_out),   // tied back: single core
      .haddr                     (haddr),
      .hwrite                    (hwrite),
      .htrans                    (htrans),
      .hsize                     (hsize),
      .hburst                    (),
      .hprot                     (),
      .hmastlock                 (),
      .hmaster                   (),
      .hexcl                     (),
      .hready                    (hready),
      .hresp                     (1'b0),
      .hexokay                   (1'b0),
      .hwdata                    (hwdata),
      .hrdata                    (hrdata),
      .fence_i_vld               (),
      .fence_d_vld               (),
      .fence_rdy                 (1'b1),
      .dbg_req_halt              (1'b0),
      .dbg_req_halt_on_reset     (1'b0),
      .dbg_req_resume            (1'b0),
      .dbg_halted                (),
      .dbg_running               (),
      .dbg_data0_rdata           (32'd0),
      .dbg_data0_wdata           (),
      .dbg_data0_wen             (),
      .dbg_instr_data            (32'd0),
      .dbg_instr_data_vld        (1'b0),
      .dbg_instr_data_rdy        (),
      .dbg_instr_caught_exception(),
      .dbg_instr_caught_ebreak   (),
      .dbg_sbus_addr             (32'd0),
      .dbg_sbus_write            (1'b0),
      .dbg_sbus_size             (2'd0),
      .dbg_sbus_vld              (1'b0),
      .dbg_sbus_rdy              (),
      .dbg_sbus_err              (),
      .dbg_sbus_wdata            (32'd0),
      .dbg_sbus_rdata            (),
      .mhartid_val               (32'd0),
      .eco_version               (4'd0),
      .irq                       (1'b0),
      .soft_irq                  (1'b0),
      .timer_irq                 (1'b0)
  );

  // ---------------------------------------------------------------------
  // Bus fabric, SRAM, system registers
  // ---------------------------------------------------------------------
  wire        sram_en, sram_we;
  wire [9:0]  sram_addr;
  wire [31:0] sram_wdata, sram_wmask, sram_rdata;
  wire        reg_valid, reg_write, reg_ready;
  wire [7:0]  reg_addr;
  wire [31:0] reg_wdata, reg_rdata;

  pg_ahb u_ahb (
      .clk       (clk),
      .rst       (rst),
      .cpu_halted(~cpu_run),
      .haddr     (haddr),
      .hwrite    (hwrite),
      .htrans    (htrans),
      .hsize     (hsize),
      .hwdata    (hwdata),
      .hready    (hready),
      .hrdata    (hrdata),
      .sram_en   (sram_en),
      .sram_we   (sram_we),
      .sram_addr (sram_addr),
      .sram_wdata(sram_wdata),
      .sram_wmask(sram_wmask),
      .sram_rdata(sram_rdata),
      .reg_valid (reg_valid),
      .reg_write (reg_write),
      .reg_addr  (reg_addr),
      .reg_wdata (reg_wdata),
      .reg_rdata (reg_rdata),
      .reg_ready (reg_ready),
      .host_req  (host_req),
      .host_we   (host_we),
      .host_addr (host_addr),
      .host_wdata(host_wdata),
      .host_ack  (host_ack),
      .host_rdata(host_rdata)
  );

  pg_sram u_sram (
      .clk  (clk),
      .en   (sram_en),
      .we   (sram_we),
      .addr (sram_addr),
      .wdata(sram_wdata),
      .wmask(sram_wmask),
      .rdata(sram_rdata)
  );

  wire [5:0] proto_uo;

  pg_sysio u_sysio (
      .clk    (clk),
      .rst    (~cpu_run),
      .valid  (reg_valid),
      .write  (reg_write),
      .addr   (reg_addr),
      .wdata  (reg_wdata),
      .rdata  (reg_rdata),
      .ready  (reg_ready),
      .uio_out(uio_out),
      .uio_oe (uio_oe),
      .uo_out (proto_uo),
      .uio_in (uio_in),
      .ui_in  (ui_in[4:0])
  );

  assign uo_out = {host_miso, 1'b0, proto_uo};

  wire _unused = &{1'b0, ena};

endmodule
