# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Shared driver for ui_in. Several models (host SPI port, UART, SPI peers)
drive different bits of the same input bus, so they all write through one
shadow value instead of read-modify-writing the signal."""

_drivers = {}


class UiPins:
    def __init__(self, dut):
        self.dut = dut
        self.value = 0

    def set(self, bit, v):
        if v:
            self.value |= 1 << bit
        else:
            self.value &= ~(1 << bit)
        self.dut.ui_in.value = self.value

    def reset(self, value):
        self.value = value
        self.dut.ui_in.value = value


def ui_pins(dut):
    """The one UiPins object for this testbench."""
    key = id(dut)
    if key not in _drivers:
        _drivers[key] = UiPins(dut)
    return _drivers[key]


def bit(signal, i):
    """Bit i of a bus as 0/1. Gate-level sims can have X on bits the test
    does not use, so read single bits instead of converting the whole bus."""
    v = str(signal.value)
    c = v[len(v) - 1 - i]
    if c not in "01":
        raise ValueError(f"{signal._name}[{i}] is {c}")
    return int(c)
