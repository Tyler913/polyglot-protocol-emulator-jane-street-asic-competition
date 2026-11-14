/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * uart_host_bridge: FPGA-only helper (not part of the chip). Lets a PC drive
 * the chip's host SPI port through the board's USB UART, so no host MCU is
 * needed for the prototype.
 *
 * UART 8N1. A request is a 16-bit length (high byte first) and then that many
 * bytes. The bridge pulls CS_N low, sends each byte as it arrives (SPI mode 0,
 * MSB first, SCK = clk / (2 * HALF)), returns the byte read on MISO for each,
 * and raises CS_N after the last. Length 0 returns one status byte instead:
 * bit 0 = HOST_IRQ. The reply to a byte leaves while the next one arrives, so
 * the PC should send long transactions in chunks and read each chunk's reply
 * (tools/polyglot_host.py UartBridge does this).
 */

`default_nettype none

module uart_host_bridge #(
    parameter CLK_HZ = 50_000_000,
    parameter BAUD   = 115_200,
    parameter HALF   = 8            // SCK half period in clk cycles (chip needs SCK <= clk/8)
) (
    input  wire clk,
    input  wire rst,
    input  wire rxd,
    output reg  txd,
    output reg  spi_cs_n,
    output reg  spi_sck,
    output reg  spi_mosi,
    input  wire spi_miso,
    input  wire irq
);

  localparam DIV = (CLK_HZ + BAUD / 2) / BAUD;

  // ---- UART receiver ------------------------------------------------------
  reg [1:0]  rx_s;
  reg [15:0] rx_cnt;
  reg [3:0]  rx_bit;
  reg [7:0]  rx_sh;
  reg        rx_busy, rx_valid;
  reg [7:0]  rx_data;
  always @(posedge clk) begin
    rx_s     <= {rx_s[0], rxd};
    rx_valid <= 1'b0;
    if (rst) begin
      rx_s    <= 2'b11;
      rx_busy <= 1'b0;
    end else if (!rx_busy) begin
      if (!rx_s[1]) begin                 // start bit edge
        rx_busy <= 1'b1;
        rx_cnt  <= DIV / 2;
        rx_bit  <= 4'd0;
      end
    end else if (rx_cnt != 0) begin
      rx_cnt <= rx_cnt - 16'd1;
    end else begin
      rx_cnt <= DIV - 1;
      rx_bit <= rx_bit + 4'd1;
      if (rx_bit == 4'd0) begin
        if (rx_s[1]) rx_busy <= 1'b0;     // glitch, not a start bit
      end else if (rx_bit <= 4'd8) begin
        rx_sh <= {rx_s[1], rx_sh[7:1]};
      end else begin                       // stop bit
        rx_busy <= 1'b0;
        if (rx_s[1]) begin
          rx_valid <= 1'b1;
          rx_data  <= rx_sh;
        end
      end
    end
  end

  // ---- UART transmitter: one holding register plus the shift register ----
  reg        hold_v;
  reg [7:0]  hold;
  reg [8:0]  tx_sh;
  reg [3:0]  tx_left;
  reg [15:0] tx_cnt;
  reg        put;
  reg [7:0]  put_data;
  always @(posedge clk) begin
    if (rst) begin
      txd     <= 1'b1;
      hold_v  <= 1'b0;
      tx_left <= 4'd0;
      tx_cnt  <= 16'd0;
    end else begin
      if (put) begin
        hold   <= put_data;
        hold_v <= 1'b1;
      end
      if (tx_cnt != 0) begin
        tx_cnt <= tx_cnt - 16'd1;
      end else if (tx_left != 0) begin    // data bits, then the stop bit
        txd     <= tx_sh[0];
        tx_sh   <= {1'b1, tx_sh[8:1]};
        tx_left <= tx_left - 4'd1;
        tx_cnt  <= DIV - 1;
      end else if (hold_v && !put) begin  // start bit
        txd     <= 1'b0;
        tx_sh   <= {1'b1, hold};
        tx_left <= 4'd9;
        tx_cnt  <= DIV - 1;
        hold_v  <= 1'b0;
      end
    end
  end

  // ---- Request state machine and SPI master --------------------------------
  localparam S_LEN_HI = 3'd0, S_LEN_LO = 3'd1, S_DATA = 3'd2, S_SHIFT = 3'd3,
             S_END = 3'd4, S_GAP = 3'd5;
  reg [2:0]  st;
  reg [15:0] left;
  reg [7:0]  sh_out, sh_in;
  reg [3:0]  nbit;
  reg [7:0]  tick;
  reg        phase;   // 0: SCK low half, 1: SCK high half

  always @(posedge clk) begin
    put <= 1'b0;
    if (rst) begin
      st       <= S_LEN_HI;
      spi_cs_n <= 1'b1;
      spi_sck  <= 1'b0;
      spi_mosi <= 1'b0;
    end else begin
      case (st)
        S_LEN_HI: if (rx_valid) begin
          left[15:8] <= rx_data;
          st <= S_LEN_LO;
        end
        S_LEN_LO: if (rx_valid) begin
          if (left[15:8] == 8'd0 && rx_data == 8'd0) begin
            put      <= 1'b1;
            put_data <= {7'd0, irq};
            st       <= S_LEN_HI;
          end else begin
            left[7:0] <= rx_data;
            spi_cs_n  <= 1'b0;
            st        <= S_DATA;
          end
        end
        S_DATA: if (rx_valid) begin
          sh_out <= rx_data;
          nbit   <= 4'd0;
          tick   <= HALF - 1;
          phase  <= 1'b0;
          spi_mosi <= rx_data[7];
          st     <= S_SHIFT;
        end
        S_SHIFT: if (tick != 0) begin
          tick <= tick - 8'd1;
        end else begin
          tick <= HALF - 1;
          if (!phase) begin                 // rising edge: sample MISO
            spi_sck <= 1'b1;
            sh_in   <= {sh_in[6:0], spi_miso};
            phase   <= 1'b1;
          end else begin                    // falling edge: next bit
            spi_sck <= 1'b0;
            phase   <= 1'b0;
            sh_out  <= {sh_out[6:0], 1'b0};
            spi_mosi <= sh_out[6];
            nbit    <= nbit + 4'd1;
            if (nbit == 4'd7) begin
              put      <= 1'b1;
              put_data <= sh_in;
              left     <= left - 16'd1;
              st       <= (left == 16'd1) ? S_END : S_DATA;
            end
          end
        end
        S_END: if (tick != 0) begin
          tick <= tick - 8'd1;
        end else begin
          spi_cs_n <= 1'b1;
          tick     <= 2 * HALF - 1;
          st       <= S_GAP;
        end
        S_GAP: if (tick != 0) tick <= tick - 8'd1; else st <= S_LEN_HI;
        default: st <= S_LEN_HI;
      endcase
    end
  end

endmodule
