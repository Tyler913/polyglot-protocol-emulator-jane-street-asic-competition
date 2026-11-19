#!/usr/bin/env python3
# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Mutation check: do the models fail when they should? For each entry, build
a deliberately broken copy of a program (in a scratch copy of sw/), put its
image in test/firmware/, run its test with FIRMWARE_PREBUILT=1 (Verilator),
expect a FAIL, and restore the image from git.
Usage (repo root, RISC-V GCC and Verilator on PATH): python3 test/mutation.py [names...]"""
import os, shutil, subprocess, sys, tempfile, re
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MUT = {  # name: (program, file, old, new, test module, testcase or None, why)
    "sbus":    ("sbus", "examples/sbus/main.c", "SER_FMT(8, SER_EVEN, 2, 1)", "SER_FMT(8, SER_ODD, 2, 1)", "test_sbus", None, "odd instead of even parity"),
    "crsf":    ("crsf", "examples/crsf/main.c", "0xD5u", "0xD7u", "test_crsf", None, "CRC-8 polynomial 0xD7"),
    "mbs":     ("modbus_server", "examples/modbus_server/main.c", "if (t - last > CHAR + T15) ok = 0;", "", "test_modbus_server", None, "no 1.5-character gap check"),
    "lins":    ("lin_slave", "examples/lin_slave/main.c", "lin_respond(d, 2, (uint32_t)p);", "lin_respond(d, 2, 0);", "test_lin_slave", None, "classic checksum on ID 0x21"),
    "iol":     ("iolink_master", "examples/iolink_master/main.c", "uint32_t c = 0x52u;", "uint32_t c = 0x53u;", "test_iolink", None, "checksum seed 0x53"),
    "biss":    ("biss", "examples/biss/main.c", "(crc ^ 0x3Fu) == crc6", "crc == crc6", "test_biss", "test_biss_short_cable", "CRC taken as not inverted"),
    "dht":     ("dht22", "examples/dht22/main.c", "(t - prev) > 100u * US", "(t - prev) > 130u * US", "test_dht22", None, "bit threshold 130 us"),
    "qspi":    ("qspi_flash", "examples/qspi_flash/main.c", "wire_putn(W0, 24, addr & 0xFFFFFFu);\n    wire_flush(W0);", "wire_putn(W0, 24, addr & 0xFFFFFFu);\n    wire_putn(W0, 8, 0);\n    wire_flush(W0);", "test_qspi_flash", None, "IO0 driven during the dummy clocks"),
    "m1553":   ("mil1553", "examples/mil1553/main.c", "uint32_t par = 1u;", "uint32_t par = 0u;", "test_mil1553", None, "even parity"),
    "a429":    ("arinc429", "examples/arinc429/main.c", "w | ((ser_par(w) ^ 1u) << 31)", "w | (ser_par(w) << 31)", "test_arinc429", None, "even parity"),
    "iso":     ("iso7816", "examples/iso7816/main.c", "WIRE_TX_LEVEL(CK->STATUS) < 3u", "WIRE_TX_LEVEL(CK->STATUS) < 1u", "test_iso7816", None, "card clock FIFO refilled only when empty"),
    "iso2":    ("iso7816", "examples/iso7816/main.c", "wait_ticks(16u * ETU);", "delay_ticks(16u * ETU);", "test_iso7816", None, "clock not fed during the 16-etu wait"),
    "canb":    ("can_b", "../include/protocol.h", "WIRE_ACK_AT(hdr + 16u + 8u * dlc)", "WIRE_ACK_AT(hdr + 17u + 8u * dlc)", "test_can", "test_can_b_receive", "ACK one bit late"),
    "pdm":     ("pdm_mic", "examples/pdm_mic/main.c", "            i3 += i2;\n", "", "test_pdm_mic", None, "second-order CIC"),
    "hdlcrx":  ("hdlc_rx", "examples/hdlc_rx/main.c", "if (ones == 5u) { ones = 0; continue; }", "if (ones == 5u) { ones = 0; }", "test_framing", "test_hdlc_rx", "stuffed zeros kept"),
    "usbfs":   ("usb_host_fs", "examples/usb_host/main.c", "last_keepalive += PG_US_TO_TICKS(1000);", "last_keepalive += PG_US_TO_TICKS(1001);", "test_usb_host", "test_usb_fs_host", "SOF every 1.001 ms"),
    "sdi12":   ("sdi12", "examples/sdi12/main.c", "wire_putn(SD, 25, 0x7FFFu);", "wire_putn(SD, 25, 0x1FFFu);", "test_sdi12", None, "10.8 ms BREAK"),
    "kline":   ("kline", "examples/kline/main.c", "t += 6u * MS;", "t += 4u * MS;", "test_kline", None, "P4 = 4 ms"),
}
names = sys.argv[1:] or list(MUT)
env = dict(os.environ)
env["PATH"] = ROOT + "/.venv/bin:" + env["PATH"]
for n in names:
    prog, f, old, new, mod, case, why = MUT[n]
    tmp = tempfile.mkdtemp(prefix="mut_")
    shutil.copytree(ROOT + "/sw", tmp + "/sw", ignore=shutil.ignore_patterns("build"))
    path = os.path.normpath(os.path.join(tmp, "sw", f if not f.startswith("..") else f[3:]))
    src = open(path).read()
    assert src.count(old) == 1, (n, "pattern not found once")
    open(path, "w").write(src.replace(old, new))
    subprocess.run(["make", "-s", "-C", tmp + "/sw", f"build/{prog}.bin"], check=True, env=env, stdout=subprocess.DEVNULL)
    fw = f"{ROOT}/test/firmware/{prog}.bin"
    shutil.copy(f"{tmp}/sw/build/{prog}.bin", fw)
    e = dict(env, FIRMWARE_PREBUILT="1")
    cmd = ["make", "-B", "SIM=verilator", "SIM_BUILD=sim_build/v_mut", f"COCOTB_RESULTS_FILE=results_mut_{n}.xml", f"COCOTB_TEST_MODULES={mod}"]
    if case:
        cmd.append(f"COCOTB_TEST_FILTER={case}")
    r = subprocess.run(cmd, cwd=ROOT + "/test", env=e, capture_output=True, text=True, timeout=3600)
    subprocess.run(["git", "checkout", "--", f"test/firmware/{prog}.bin"], cwd=ROOT, check=True)
    shutil.rmtree(tmp)
    out = r.stdout + r.stderr
    fails = re.findall(r"\*\* (\S+)\s+FAIL", out)
    passes = re.findall(r"\*\* (\S+)\s+PASS", out)
    first = next((l.strip()[:160] for l in out.splitlines() if "AssertionError" in l), "")
    print(f"{n:7s} {why:40s} FAIL={len(fails)} PASS={len(passes)}  {'REJECTED' if fails else 'NOT CAUGHT'}  {first}", flush=True)
