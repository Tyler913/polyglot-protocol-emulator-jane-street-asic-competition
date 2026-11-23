# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Four protocols at once under sustained load (sw/tests/stress4): CAN frames
back to back on engine 0, SPI RAM write/read rounds on engine 1, a long UART
echo stream on engine 2 and I2C EEPROM rounds on engine 3. Each bus is judged
by its own independent model. Rates and amounts come from environment
variables, so a script can sweep them (not in the default regression):

  STRESS_UART=baud,bytes  STRESS_SPI=hz,rounds  STRESS_I2C=hz,rounds
  STRESS_CAN=bit/s,frames  (a count of 0 leaves that bus idle)
  STRESS_DMA=1             UART through two DMA channels instead of the CPU

The test logs one line "STRESS {...}" with the measured results, then checks
that every transfer was correct."""

import json
import os

import cocotb
from cocotb.triggers import ClockCycles

from lib.can import CanBus, CanNode
from lib.i2c import I2cEeprom, OpenDrainBus
from lib.soc import F_CPU, load, read_shared, start, wait_irq
from lib.spi import SpiRam
from lib.uart import UartDriver, UartMonitor, bit_cycles


def env(name, default):
    v = os.environ.get(name, default)
    a, b = v.split(",")
    return int(float(a)), int(b)


def text(n):
    return bytes((0x20 + (i * 37) % 0x5E) for i in range(n))


@cocotb.test()
async def test_stress_four_protocols(dut):
    baud, n_uart = env("STRESS_UART", "1000000,200")
    spi_hz, n_spi = env("STRESS_SPI", "5000000,4")
    i2c_hz, n_i2c = env("STRESS_I2C", "400000,2")
    can_bps, n_can = env("STRESS_CAN", "500000,4")
    ubit = bit_cycles(baud)
    host = await start(dut)

    can = CanBus(dut, txd_uio=0, rxd_ui=2, bit_cycles=F_CPU / (can_bps or 500_000))
    node = can.add(CanNode(can, "node"))
    ram = SpiRam(dut, 0, sck=2, mosi=3, cs=4, miso_ui=1)
    i2c = OpenDrainBus(dut, pins={1, 2})
    ee = i2c.add(I2cEeprom(i2c, sda=1, scl=2, write_cycles=30 * F_CPU // (i2c_hz or 400_000)))
    mon = UartMonitor(dut, lambda: (int(dut.uo_out.value) >> 1) & 1, ubit)
    drv = UartDriver(dut, 0, ubit)
    data = text(n_uart)

    dma = os.environ.get("STRESS_DMA", "0") == "1"
    await load(host, "stress4", [baud, n_uart | (dma << 31), spi_hz, n_spi, i2c_hz, n_i2c, can_bps, n_can])
    for m in (can, ram, i2c, mon):
        m.start()
    await host.ctrl(0x01)
    await ClockCycles(dut.clk, 20000)  # every task is configured by now (runtime divisions)
    t_send = cocotb.start_soon(drv.send(data))
    budget = 200_000 + 12 * n_uart * ubit + (n_spi * 140 * F_CPU / spi_hz if n_spi else 0) + \
        (n_i2c * 120 * F_CPU / i2c_hz if n_i2c else 0) + (n_can * 140 * F_CPU / can_bps if n_can else 0)
    timeout = False
    try:
        await wait_irq(dut, 1, timeout=int(budget))
    except AssertionError:
        timeout = True
    await t_send
    await ClockCycles(dut.clk, int(12 * ubit))
    for m in (can, ram, i2c):
        m.stop()

    res = await read_shared(host, 0, 31)
    times = [(res[16 + 2 * k], res[17 + 2 * k]) for k in range(4)]
    exp_uart = bytes((c + 1) & 0xFF for c in data)
    got = bytes(mon.bytes)
    uart_ok = got == exp_uart and not mon.errors and res[10] == 0
    first_bad = next((i for i, (a, b) in enumerate(zip(got, exp_uart)) if a != b), None)
    out = dict(uart=[baud, n_uart], dma=dma, uart_first_bad=first_bad, spi=[spi_hz, n_spi], i2c=[i2c_hz, n_i2c], can=[can_bps, n_can],
               timeout=timeout, done=res[8] == 0x600D0004,
               uart_echoed=res[9], uart_frame_err=res[10], uart_ovf=res[11],
               uart_ok=uart_ok, uart_rx_bytes=len(got),
               spi_bad=res[12], spi_err=len(ram.errors), i2c_bad=res[13],
               i2c_err=len(i2c.errors) + len(ee.errors), can_good=res[14],
               can_err=len(node.errors), can_bus_frames=len(node.frames), can_ovf=res[24], can_words=hex(res[25]), loops=res[15], worst_pass=res[26], worst_step=dict(can=res[27], spi=res[28], uart=res[29], i2c=res[30]),
               dur={k: (e - s) for k, (s, e) in zip(("can", "spi", "uart", "i2c"), times)})
    # effective throughput of each bus (payload bits per second while its task ran)
    tp = {}
    if n_spi and times[1][1] > times[1][0]:
        tp["spi_kbps"] = round(n_spi * 64 * 8 / ((times[1][1] - times[1][0]) / F_CPU) / 1000)
    if n_i2c and times[3][1] > times[3][0]:
        tp["i2c_kbps"] = round(n_i2c * 16 * 8 / ((times[3][1] - times[3][0]) / F_CPU) / 1000)
    if n_can and times[0][1] > times[0][0]:
        tp["can_frames_per_s"] = round(n_can / ((times[0][1] - times[0][0]) / F_CPU))
    out["throughput"] = tp
    dut._log.info("STRESS " + json.dumps(out))

    assert not timeout, f"timeout: {out}"
    assert uart_ok, f"UART: {out}; first diff at " \
        f"{next((i for i, (a, b) in enumerate(zip(got, exp_uart)) if a != b), min(len(got), len(exp_uart)))}"
    assert res[12] == 0 and not ram.errors, f"SPI: {out} {ram.errors[:3]}"
    assert res[13] == 0 and not i2c.errors and not ee.errors, f"I2C: {out} {(i2c.errors + ee.errors)[:3]}"
    assert res[14] == n_can and not node.errors, f"CAN: {out} {node.errors[:3]}"
