# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Four different protocols at once (sw/tests/mixed4): CAN on engine 0, SPI on
engine 1, UART on engine 2 and I2C on engine 3, run as four cooperative tasks
in one program. Each bus is judged by its own independent model, all running
in the same simulation, and the four tasks must overlap in time."""

import cocotb
from cocotb.triggers import ClockCycles

from lib.can import CanBus, CanNode
from lib.i2c import I2cEeprom, OpenDrainBus
from lib.soc import F_CPU, load, read_shared, start, wait_irq
from lib.spi import SpiRam
from lib.uart import UartDriver, UartMonitor, bit_cycles

SPI_PATTERN = bytes((0x3C + 17 * i) & 0xFF for i in range(16))
I2C_DATA = bytes((0xA0 + 7 * i) & 0xFF for i in range(8))
TEXT = b"four protocols, one CPU"


@cocotb.test()
async def test_mixed_four_protocols(dut):
    ubit = bit_cycles(1_000_000)
    host = await start(dut)

    can = CanBus(dut, txd_uio=0, rxd_ui=2, bit_cycles=F_CPU / 500_000)
    node = can.add(CanNode(can, "node"))
    ram = SpiRam(dut, 0, sck=2, mosi=3, cs=4, miso_ui=1)
    i2c = OpenDrainBus(dut, pins={1, 2})
    ee = i2c.add(I2cEeprom(i2c, sda=1, scl=2, write_cycles=30 * F_CPU // 400_000))
    mon = UartMonitor(dut, lambda: (int(dut.uo_out.value) >> 1) & 1, ubit)
    drv = UartDriver(dut, 0, ubit)

    await load(host, "mixed4", [len(TEXT)])
    can.start()
    ram.start()
    i2c.start()
    mon.start()
    await host.ctrl(0x01)
    await ClockCycles(dut.clk, 3000)  # the UART task is configured by now
    await drv.send(TEXT)
    try:
        await wait_irq(dut, 1, timeout=400_000)
    except AssertionError:
        await ClockCycles(dut.clk, 1)
        res = await read_shared(host, 0, 24)
        dut._log.error(f"timeout; shared {[hex(x) for x in res]}; UART got {bytes(mon.bytes)!r}; "
                       f"CAN frames {node.frames}; SPI frames {len(ram.frames)}; I2C log {ee.log[:6]}")
        raise
    await ClockCycles(dut.clk, int(12 * ubit))
    for m in (can, ram, i2c):
        m.stop()

    res = await read_shared(host, 0, 24)
    times = [(res[16 + 2 * k], res[17 + 2 * k]) for k in range(4)]
    dut._log.info(f"task (start, end) cycles CAN/SPI/UART/I2C: {times}; I2C polls {res[6]}")

    assert res[1] == 0x600D0004, f"done marker {res[1]:#x}"
    # CAN: frame on the bus, acknowledged by the node; the chip saw the ACK and a good CRC
    assert not node.errors, node.errors[:5]
    assert node.frames == [dict(id=0x123, data=[0xDE, 0xAD, 0xBE, 0xEF], crc_ok=True, acked=True, by="other")], node.frames
    assert res[7] == 1, "chip did not see its CAN frame acknowledged with a good CRC"
    # SPI: write then read frames, RAM content, firmware read-back
    assert not ram.errors, ram.errors
    assert len(ram.frames) == 2 and ram.frames[0] == bytes([0x02, 0x40]) + SPI_PATTERN, ram.frames
    assert bytes(ram.mem[0x40:0x50]) == SPI_PATTERN
    assert b"".join(w.to_bytes(4, "little") for w in res[8:12]) == SPI_PATTERN
    assert res[4] == 0x600D0000, f"SPI result {res[4]:#x}"
    # UART: every byte echoed + 1, no framing errors
    assert not mon.errors, mon.errors
    assert bytes(mon.bytes) == bytes(c + 1 for c in TEXT), f"UART received {bytes(mon.bytes)!r}"
    assert (res[2], res[3]) == (len(TEXT), 0), f"echoed {res[2]}, framing errors {res[3]}"
    # I2C: EEPROM written, ACK polling seen, read back
    assert not i2c.errors, i2c.errors[:5]
    assert not ee.errors, ee.errors[:5]
    assert bytes(ee.mem[0x10:0x18]) == I2C_DATA, ee.mem[0x10:0x18].hex()
    assert res[6] >= 2, f"expected ACK polling, got {res[6]} poll(s)"
    assert b"".join(w.to_bytes(4, "little") for w in res[12:14]) == I2C_DATA
    assert res[5] == 0x600D0000, f"I2C result {res[5]:#x}"
    # Concurrency: every task started before any task finished
    assert max(s for s, _ in times) < min(e for _, e in times), f"tasks did not overlap: {times}"
