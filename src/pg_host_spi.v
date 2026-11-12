/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * pg_host_spi: SPI target (mode 0, MSB first) used by the host MCU to load
 * programs, read memory back and control the CPU.
 *
 * The SPI pins are oversampled by the system clock, so SCK must be at most
 * clk/8. Every transaction starts with CS_N falling and is one command:
 *
 *   0x01 WRITE   : [0x01][addr_hi][addr_lo][d0][d1]...   bytes written to addr, addr+1, ...
 *   0x02 READ    : [0x02][addr_hi][addr_lo][dummy][..]   MISO returns d0, d1, ... after the dummy byte
 *   0x03 CTRL    : [0x03][value]                          value is latched into ctrl
 *   0x04 STATUS  : [0x04][xx]                             MISO returns status during the 2nd byte
 *
 * Addresses are 16-bit byte addresses into CPU memory. Memory accesses are
 * one byte each, through a request/acknowledge port.
 */

`default_nettype none

module pg_host_spi (
    input  wire        clk,
    input  wire        rst,

    // SPI pins (asynchronous to clk)
    input  wire        spi_sck,
    input  wire        spi_cs_n,
    input  wire        spi_mosi,
    output wire        spi_miso,

    // Memory port (byte wide). req stays high until ack.
    output reg         mem_req,
    output reg         mem_we,
    output reg  [15:0] mem_addr,
    output reg  [7:0]  mem_wdata,
    input  wire        mem_ack,
    input  wire [7:0]  mem_rdata,

    // Control and status
    output reg  [7:0]  ctrl,
    input  wire [7:0]  status
);

  localparam CMD_WRITE  = 8'h01;
  localparam CMD_READ   = 8'h02;
  localparam CMD_CTRL   = 8'h03;
  localparam CMD_STATUS = 8'h04;

  // Two-flop synchronizers plus one extra stage for edge detection
  reg [2:0] sck_s;
  reg [1:0] cs_s;
  reg [1:0] mosi_s;
  always @(posedge clk) begin
    if (rst) begin
      sck_s  <= 3'b000;
      cs_s   <= 2'b11;
      mosi_s <= 2'b00;
    end else begin
      sck_s  <= {sck_s[1:0], spi_sck};
      cs_s   <= {cs_s[0], spi_cs_n};
      mosi_s <= {mosi_s[0], spi_mosi};
    end
  end

  wire cs_active = ~cs_s[1];
  wire sck_rise  = (sck_s[2:1] == 2'b01);
  wire sck_fall  = (sck_s[2:1] == 2'b10);
  wire mosi      = mosi_s[1];

  reg [2:0] bit_cnt;
  reg [6:0] rx_shift;
  reg [7:0] tx_shift;
  reg       skip_fall;
  reg [2:0] byte_idx;      // saturating byte counter within a transaction
  reg [7:0] cmd;
  reg [7:0] rd_buf;        // prefetched read byte
  reg       rd_pending;    // a read request has been issued

  assign spi_miso = tx_shift[7];

  wire [7:0] rx_byte = {rx_shift, mosi};
  wire       byte_done = cs_active && sck_rise && (bit_cnt == 3'd7);

  always @(posedge clk) begin
    if (rst) begin
      bit_cnt    <= 3'd0;
      rx_shift   <= 7'd0;
      tx_shift   <= 8'd0;
      skip_fall  <= 1'b0;
      byte_idx   <= 3'd0;
      cmd        <= 8'd0;
      rd_buf     <= 8'd0;
      rd_pending <= 1'b0;
      mem_req    <= 1'b0;
      mem_we     <= 1'b0;
      mem_addr   <= 16'd0;
      mem_wdata  <= 8'd0;
      ctrl       <= 8'd0;
    end else begin
      // Memory handshake
      if (mem_req && mem_ack) begin
        mem_req <= 1'b0;
        if (!mem_we) begin
          rd_buf     <= mem_rdata;
          rd_pending <= 1'b0;
        end
      end

      if (!cs_active) begin
        bit_cnt   <= 3'd0;
        byte_idx  <= 3'd0;
        skip_fall <= 1'b0;
        tx_shift  <= 8'd0;
      end else begin
        if (sck_rise) begin
          rx_shift <= {rx_shift[5:0], mosi};
          bit_cnt  <= bit_cnt + 3'd1;
        end
        if (sck_fall) begin
          if (skip_fall) skip_fall <= 1'b0;
          else           tx_shift  <= {tx_shift[6:0], 1'b0};
        end

        if (byte_done) begin
          skip_fall <= 1'b1;
          tx_shift  <= 8'd0;
          if (byte_idx != 3'd7) byte_idx <= byte_idx + 3'd1;

          case (byte_idx)
            3'd0: begin
              cmd <= rx_byte;
              if (rx_byte == CMD_STATUS) tx_shift <= status;
            end
            3'd1: begin
              mem_addr[15:8] <= rx_byte;
              if (cmd == CMD_CTRL) ctrl <= rx_byte;
            end
            3'd2: begin
              mem_addr[7:0] <= rx_byte;
              if (cmd == CMD_READ) begin
                mem_req    <= 1'b1;
                mem_we     <= 1'b0;
                rd_pending <= 1'b1;
              end
            end
            default: begin
              if (cmd == CMD_WRITE) begin
                // byte 3 onwards is data; the address was set by bytes 1-2
                mem_req   <= 1'b1;
                mem_we    <= 1'b1;
                mem_wdata <= rx_byte;
                if (byte_idx != 3'd3) mem_addr <= mem_addr + 16'd1;
              end else if (cmd == CMD_READ) begin
                // byte 3 is the dummy byte; from here on, shift out the
                // prefetched byte and fetch the next one
                tx_shift   <= rd_buf;
                mem_addr   <= mem_addr + 16'd1;
                mem_req    <= 1'b1;
                mem_we     <= 1'b0;
                rd_pending <= 1'b1;
              end
            end
          endcase
        end
      end
    end
  end

  wire _unused = &{1'b0, rd_pending};

endmodule
