# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""A queue of eight protocol jobs, four at a time (sw/tests/queue + sw/jobs).

The resident kernel runs up to four jobs, one per Wire Engine. The host model
loads each job image into its slot over the host SPI port and starts it; when
a slot finishes, the host reads its results and loads the next job for that
slot while the other slots keep running. Every bus is judged by its own
independent model; all of them run in the same simulation.

  wave 1: I2C EEPROM 100 kHz (slot 3), CAN 500 kbit/s (0), SPI RAM 5 MHz (1), UART 115200 (2)
  wave 2: CAN 250 kbit/s (0), WS2812 (1), UART 1 Mbaud (2), I2C EEPROM 400 kHz (3)

Wave-2 jobs use other pins than wave 1, so a job can start while the jobs of
the first wave are still running."""

import cocotb
from cocotb.triggers import ClockCycles, ReadOnly, RisingEdge

from lib.can import CanBus, CanNode
from lib.i2c import I2cEeprom, OpenDrainBus
from lib.soc import CLK_NS, F_CPU, SHARED, build_firmware, host_irq, load, read_shared, start, words_to_bytes
from lib.pins import bit
from lib.spi import SpiRam
from lib.uart import UartDriver, UartMonitor, bit_cycles
from lib.ws2812 import Ws2812Model

SLOT_ADDR = [0x100, 0x440, 0x740, 0x940]
RUN, DONE = 1, 3


def uio(n):
    return n


def uo(n):
    return 8 + n


def ui(n):
    return 16 + n


def fix8(rate):
    return (F_CPU * 256 + rate // 2) // rate


SPI_PATTERN = bytes((0x3C + 17 * i) & 0xFF for i in range(16))
I2C_DATA = bytes((0xA0 + 7 * i) & 0xFF for i in range(8))
TEXT1 = b"first wave, 115200 baud: a long line"
TEXT2 = b"second wave at 1 Mbaud"
LEDS = [(0x00, 0xFF, 0x00), (0x12, 0x34, 0x56), (0x80, 0x00, 0x01)]  # G, R, B


def job_list():
    """(name, job, slot, params {mailbox index: value})"""
    return [
        ("i2c-100k", "i2c_eeprom", 3, {1: uio(1), 2: uio(2), 3: F_CPU // 100_000, 4: 0x10}),
        ("can-500k", "can_send", 0, {1: uio(0), 2: ui(2), 3: fix8(500_000), 4: 0x123, 5: 0xDEADBEEF}),
        ("spi-5M", "spi_ram", 1, {1: uo(2), 2: uo(3), 3: uo(4), 4: ui(1), 5: 5_000_000, 6: 0x40}),
        ("uart-115k", "uart_echo", 2, {1: uo(1), 2: ui(0), 3: fix8(115_200), 4: len(TEXT1)}),
        ("can-250k", "can_send", 0, {1: uio(3), 2: ui(3), 3: fix8(250_000), 4: 0x0AB, 5: 0x01020304}),
        ("ws2812", "ws2812", 1, {1: uo(5), 2: len(LEDS),
                                 **{3 + i: (g << 16) | (r << 8) | b for i, (g, r, b) in enumerate(LEDS)}}),
        ("uart-1M", "uart_echo", 2, {1: uo(0), 2: ui(4), 3: fix8(1_000_000), 4: len(TEXT2)}),
        ("i2c-400k", "i2c_eeprom", 3, {1: uio(4), 2: uio(5), 3: F_CPU // 400_000, 4: 0x20}),
    ]


@cocotb.test()
async def test_queue_eight_jobs(dut):
    host = await start(dut)

    can1 = CanBus(dut, txd_uio=0, rxd_ui=2, bit_cycles=F_CPU / 500_000)
    node1 = can1.add(CanNode(can1, "node1"))
    can2 = CanBus(dut, txd_uio=3, rxd_ui=3, bit_cycles=F_CPU / 250_000)
    node2 = can2.add(CanNode(can2, "node2"))
    ram = SpiRam(dut, 0, sck=2, mosi=3, cs=4, miso_ui=1)
    i2c = OpenDrainBus(dut, pins={1, 2, 4, 5})
    ee1 = i2c.add(I2cEeprom(i2c, sda=1, scl=2, write_cycles=30 * F_CPU // 100_000))
    ee2 = i2c.add(I2cEeprom(i2c, sda=4, scl=5, write_cycles=30 * F_CPU // 400_000))
    u1bit, u2bit = bit_cycles(115_200), bit_cycles(1_000_000)
    mon1 = UartMonitor(dut, lambda: bit(dut.uo_out, 1), u1bit)
    mon2 = UartMonitor(dut, lambda: bit(dut.uo_out, 0), u2bit)
    drv1, drv2 = UartDriver(dut, 0, u1bit), UartDriver(dut, 4, u2bit)
    leds = Ws2812Model(dut, uo_bit=5, clk_ns=CLK_NS)
    for m in (can1, can2, ram, i2c, leds):
        m.start()  # UART monitors start with their jobs (the pins are low until then)

    cycle = [0]

    async def count():
        while True:
            await RisingEdge(dut.clk)
            cycle[0] += 1

    cocotb.start_soon(count())


    await load(host, "queue")
    await host.ctrl(0x01)
    await ClockCycles(dut.clk, 2000)  # the kernel clears the mailbox at boot

    queue = job_list()
    running = {}  # slot -> (name, job, params, start cycle)
    results = {}
    log = []
    senders = []
    while queue or running:
        # start every queued job whose slot is free (the first one for each slot)
        for item in list(queue):
            name, job, slot, params = item
            if slot in running:
                continue
            queue.remove(item)
            t0 = cycle[0]
            others = sorted(n for n, *_ in running.values())
            await host.write(SLOT_ADDR[slot], build_firmware(f"job_{job}_{slot}"))
            words = [params.get(i, 0) for i in range(1, 16)]
            await host.write(SHARED + 64 * slot + 4, words_to_bytes(words))
            await host.write(SHARED + 64 * slot, words_to_bytes([RUN]))
            running[slot] = (name, job, params, t0)
            log.append(f"{t0:7d}: load {name} into slot {slot} ({cycle[0] - t0} cycles) while running {others}")
            dut._log.info(log[-1])
            if name == "uart-115k":
                senders.append(cocotb.start_soon(_send_later(dut, mon1, drv1, TEXT1)))
            if name == "uart-1M":
                senders.append(cocotb.start_soon(_send_later(dut, mon2, drv2, TEXT2)))
        # wait for a slot to finish
        for _ in range(600_000):
            await RisingEdge(dut.clk)
            await ReadOnly()
            if host_irq(dut):
                break
        else:
            await RisingEdge(dut.clk)
            states = await read_shared(host, 0, 64)
            raise AssertionError(f"no job finished; running {running}; mailbox {[hex(x) for x in states]}")
        await RisingEdge(dut.clk)
        for slot in list(running):
            mb = await read_shared(host, 16 * slot, 16)
            if mb[0] == DONE:
                name = running.pop(slot)[0]
                results[name] = mb
                log.append(f"{cycle[0]:7d}: {name} done (chip time {mb[14]}..{mb[15]})")
                dut._log.info(log[-1])
                await host.write(SHARED + 64 * slot, words_to_bytes([0]))  # slot idle

    for s in senders:
        await s
    await ClockCycles(dut.clk, int(12 * u1bit))
    for m in (can1, can2, ram, i2c, leds):
        m.stop()
    for line in log:
        dut._log.info(line)

    # CAN
    for node, rid, data in ((node1, 0x123, [0xDE, 0xAD, 0xBE, 0xEF]), (node2, 0x0AB, [1, 2, 3, 4])):
        assert not node.errors, node.errors[:5]
        assert node.frames == [dict(id=rid, data=data, crc_ok=True, acked=True, by="other")], node.frames
    assert results["can-500k"][8] == 1 and results["can-250k"][8] == 1
    # SPI
    assert not ram.errors, ram.errors
    assert bytes(ram.mem[0x40:0x50]) == SPI_PATTERN
    assert b"".join(w.to_bytes(4, "little") for w in results["spi-5M"][9:13]) == SPI_PATTERN
    assert results["spi-5M"][8] == 0x600D0000, hex(results["spi-5M"][8])
    # UART
    for mon, text, name in ((mon1, TEXT1, "uart-115k"), (mon2, TEXT2, "uart-1M")):
        assert not mon.errors, mon.errors
        assert bytes(mon.bytes) == bytes(c + 1 for c in text), f"{name}: {bytes(mon.bytes)!r}"
        assert results[name][8:10] == [len(text), 0], results[name][8:10]
    # I2C
    assert not i2c.errors, i2c.errors[:5]
    for ee, addr, name in ((ee1, 0x10, "i2c-100k"), (ee2, 0x20, "i2c-400k")):
        assert not ee.errors, ee.errors[:5]
        assert bytes(ee.mem[addr:addr + 8]) == I2C_DATA, f"{name}: {ee.mem[addr:addr + 8].hex()}"
        assert b"".join(w.to_bytes(4, "little") for w in results[name][10:12]) == I2C_DATA
        assert results[name][9] >= 2, f"{name}: {results[name][9]} poll(s)"
        assert results[name][8] == 0x600D0000, f"{name}: {results[name][8]:#x}"
    # WS2812
    assert not leds.errors, leds.errors[:5]
    assert len(leds.frames) == 1 and leds.leds(leds.frames[0]) == LEDS, leds.frames
    # Jobs of the second wave were loaded and ran while the first wave's UART job was still running
    uart_end = results["uart-115k"][15]
    overlap = [n for n in ("can-250k", "ws2812", "i2c-400k") if results[n][14] < uart_end]
    assert overlap, "no second-wave job started before the first-wave UART job ended"
    dut._log.info(f"second-wave jobs started while uart-115k was running: {overlap}")


async def _send_later(dut, mon, drv, text):
    await ClockCycles(dut.clk, 3000)  # the job has configured its engine by then
    mon.start()
    await drv.send(text)
