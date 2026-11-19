# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""PDM microphone recording (sw/examples/pdm_mic): an SPI engine clocks an
independent sigma-delta microphone model (lib/pdm.py) playing a 1 kHz sine
at half scale; the chip's C code decimates by 64 with a CIC filter. A sine
fitted to the 16.28 kHz output must have the right amplitude (CIC droop
included) and leave a small residual."""

import math

import cocotb

from lib.pdm import PdmMic
from lib.soc import load, read_shared, start, wait_irq

N = 112
F_CLK = 50e6 / 48
FS = F_CLK / 64


@cocotb.test()
async def test_pdm_mic(dut):
    host = await start(dut)
    mic = PdmMic(dut, clk_uo=2, data_ui=4, freq=1000.0, amp=0.5)
    mic.start()
    await load(host, "pdm_mic", [N])
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=10_000)
    await wait_irq(dut, 0, timeout=600_000)
    mic.stop()
    gaps = {b - a for a, b in zip(mic.edges[:-1], mic.edges[1:])}
    assert gaps == {48}, f"PDM clock periods {gaps} (want a steady 48 cycles)"
    res = await read_shared(host, 0, 8 + N // 2)
    assert res[2] == 0x600D0011, hex(res[2])
    s = []
    for w in res[8:8 + N // 2]:
        for v in (w & 0xFFFF, w >> 16):
            s.append(v - 65536 if v & 0x8000 else v)
    s = s[8:]                                      # CIC start-up
    w = 2 * math.pi * 1000.0 / FS
    # least-squares fit a*sin + b*cos + c
    import numpy as np
    k = np.arange(len(s))
    A = np.stack([np.sin(w * k), np.cos(w * k), np.ones(len(s))], axis=1)
    coef, *_ = np.linalg.lstsq(A, np.array(s, dtype=float), rcond=None)
    amp = math.hypot(coef[0], coef[1])
    resid = np.array(s) - A @ coef
    rms = float(np.sqrt(np.mean(resid ** 2)))
    droop = (math.sin(math.pi * 1000 / FS) / (64 * math.sin(math.pi * 1000 / FS / 64))) ** 3
    want = 0.5 * 32768 * droop
    dut._log.info("amplitude %.0f (want %.0f), offset %.0f, residual rms %.1f (%.1f dB below full scale)",
                  amp, want, coef[2], rms, 20 * math.log10(32768 / rms))
    assert abs(amp - want) < 0.03 * want, f"amplitude {amp:.0f}, want {want:.0f}"
    assert abs(coef[2]) < 300, f"offset {coef[2]:.0f}"
    assert rms < 100, f"residual rms {rms:.1f}"
