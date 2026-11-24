# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Merge tie cells before placement (P&R experiment).

Synthesis gives every constant input its own tie cell (4,497 tiehi on this
design, almost all on the RESET_B pins of flip-flops without a reset: the
library has no flip-flop without one). This keeps one tiehi and one tielo and
moves every load onto their nets; repair_design later buffers those nets for
fanout (DESIGN_REPAIR_TIE_FANOUT must be off, or repair_tie_fanout splits them
again). Run by librelane_plugin_polyglot.py (Polyglot.MergeTies).
"""

import odb  # noqa: F401
from reader import click, click_odb


@click.command()
@click.option("--tiehi", default="sg13cmos5l_tiehi")
@click.option("--tielo", default="sg13cmos5l_tielo")
@click.option("--group", default=0, type=int, help="loads per kept tie (0: one tie for all)")
@click_odb
def main(reader, tiehi, tielo, group):
    block = reader.block
    for master in (tiehi, tielo):
        ties = [i for i in block.getInsts() if i.getMaster().getName() == master]

        def first_load(t):
            out = [it for it in t.getITerms() if it.isOutputSignal()]
            net = out[0].getNet() if out else None
            names = [it.getInst().getName() for it in net.getITerms() if it.getInst().getName() != t.getName()] if net else []
            return min(names) if names else ""
        # sorted by load name, so each group holds flip-flops of the same module (placed together)
        ties.sort(key=first_load)
        keep_net, moved, removed, kept_loads = None, 0, 0, 0
        for t in ties:
            out = [it for it in t.getITerms() if it.isOutputSignal()]
            net = out[0].getNet() if out else None
            if net is None or net.getBTerms():           # unconnected, or drives a port: leave it
                continue
            if keep_net is None or (group and kept_loads >= group):
                keep_net = net
                kept_loads = len(net.getITerms()) - 1
                continue
            loads = [it for it in net.getITerms() if it.getInst().getName() != t.getName()]
            for it in loads:
                it.disconnect()
                it.connect(keep_net)
                moved += 1
                kept_loads += 1
            odb.dbInst_destroy(t)
            odb.dbNet_destroy(net)
            removed += 1
        print(f"[INFO] {master}: {len(ties)} cells, removed {removed}, moved {moved} loads (group {group or 'all'})")


if __name__ == "__main__":
    main()
