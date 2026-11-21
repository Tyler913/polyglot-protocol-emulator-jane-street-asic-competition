# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Power the IHP SRAM macros on the CMOS5L tile (Metal4-only PDN).

Tiny Tapeout requires every VPWR/VGND stripe of the tile to run (nearly) the
full die height, but a stripe may only cross an SRAM macro on one of the
macro's own Metal4 supply columns of the same net (VDD!/VDDARRAY! for VPWR,
VSS! for VGND), and the columns are not on one uniform grid (the 1024x32
macro's right bit-cell array is 0.665 um off the left one). So pdngen is
kept out of the macro's x-range (a Metal4 PDN obstruction in config.json),
and this script, run right after pdngen:

  1. trunks: wherever the tile grid (pitch/offset/spacing of the PDN config)
     would have put a VPWR or VGND stripe inside --region, draws a
     full-height stripe on the nearest same-net supply column instead, and
     connects it to the Metal1 rails of that net outside the macro with the
     same via stack pdngen uses (copied from an existing pdngen via stack);
  2. column stripes: every other supply column gets a stripe over the macro
     height, so all columns are fed.

Shapes of one LEF pin are connected inside the macro, so LVS sees one net.
Idea after Ken Pettit's IHP SRAM walkthrough (ExtendPowerStripes); this code
is our own. Run by librelane_plugin_polyglot.py (Polyglot.SramStripes).
"""

import odb  # noqa: F401
from reader import click, click_odb

POWER = {"VDD!": "VPWR", "VDDARRAY!": "VPWR"}
GROUND = {"VSS!": "VGND"}


def layer_name(w):
    tl = w.getTechLayer()
    return tl.getName() if tl is not None else None


@click.command()
@click.option("--master-prefix", required=True)
@click.option("--layer", required=True)
@click.option("--width", type=float, required=True)
@click.option("--pitch", type=float, required=True, help="tile PDN stripe pitch (FP_PDN_VPITCH)")
@click.option("--offset", type=float, required=True, help="first VPWR stripe centre from the core edge (FP_PDN_VOFFSET)")
@click.option("--spacing", type=float, required=True, help="VPWR-VGND stripe spacing (FP_PDN_VSPACING)")
@click.option("--region", type=(float, float), required=True, help="x range kept free of pdngen stripes")
@click.option("--halo", type=float, default=10.0, help="no rail vias this close to a macro")
@click_odb
def main(reader, master_prefix, layer, width, pitch, offset, spacing, region, halo):
    block = reader.block
    dbu = reader.tech.getDbUnitsPerMicron()
    um = lambda v: int(round(v * dbu))  # noqa: E731
    tech_layer = reader.tech.findLayer(layer)
    half_w = um(width) // 2
    core = block.getCoreArea()
    rx0, rx1 = um(region[0]), um(region[1])

    nets = {n: block.findNet(n) for n in ("VPWR", "VGND")}
    if None in nets.values():
        raise SystemExit("[ERROR] VPWR/VGND not found")

    # Full-height extent and via stack template from pdngen's stripes outside the region
    ylo, yhi, stack = None, None, {}
    for name, net in nets.items():
        for sw in net.getSWires():
            for w in sw.getWires():
                if w.isVia():
                    bv = w.getBlockVia()
                    if bv is not None and not rx0 <= (w.xMin() + w.xMax()) // 2 <= rx1:
                        cut = bv.getName().split("_")[0] + "_" + bv.getName().split("_")[1]   # e.g. via1_2
                        per = stack.setdefault(name, {}).setdefault(cut, {})
                        n, _ = per.get(bv.getName(), (0, bv))
                        per[bv.getName()] = (n + 1, bv)
                elif layer_name(w) == layer and w.yMax() - w.yMin() > w.xMax() - w.xMin() and (w.xMin() > rx1 or w.xMax() < rx0):
                    ylo = w.yMin() if ylo is None else min(ylo, w.yMin())
                    yhi = w.yMax() if yhi is None else max(yhi, w.yMax())
    if ylo is None or not stack.get("VPWR") or not stack.get("VGND"):
        raise SystemExit("[ERROR] no pdngen stripes/vias found outside the region")
    # the most common via of each layer pair (via1_2, via2_3, via3_4), bottom up
    vias = {n: [max(stack[n][cut].values(), key=lambda t: t[0])[1] for cut in sorted(stack[n])] for n in stack}

    # Supply columns of every SRAM instance (absolute x centres), and macro boxes
    cols = {"VPWR": {}, "VGND": {}}  # x -> instance box
    boxes = []
    for inst in block.getInsts():
        master = inst.getMaster()
        if not master.getName().startswith(master_prefix):
            continue
        box = inst.getBBox()
        boxes.append(box)
        orient = inst.getOrient()
        if orient not in ("R0", "MX", "MY", "R180"):
            raise SystemExit(f"[ERROR] {inst.getName()}: orientation {orient} not supported")
        flip_x = orient in ("MY", "R180")
        for mterm in master.getMTerms():
            net = POWER.get(mterm.getName()) or GROUND.get(mterm.getName())
            if net is None:
                continue
            for mpin in mterm.getMPins():
                for g in mpin.getGeometry():
                    if g.getTechLayer() is None or g.getTechLayer().getName() != layer:
                        continue
                    xc = (g.xMin() + g.xMax()) // 2
                    xc = box.xMin() + (master.getWidth() - xc if flip_x else xc)
                    cols[net][xc] = box
    if not boxes:
        raise SystemExit("[ERROR] no SRAM instances")

    # 1. Trunks: tile-grid positions inside the region move to the nearest same-net column
    trunks = {"VPWR": [], "VGND": []}
    k = 0
    while True:
        xp = core.xMin() + um(offset) + k * um(pitch)
        if xp > rx1:
            break
        for net, target in (("VPWR", xp), ("VGND", xp + um(width) + um(spacing))):
            if not rx0 <= target <= rx1:
                continue
            free = [x for x in cols[net] if x not in trunks[net]]
            if not free:
                continue
            x = min(free, key=lambda c: abs(c - target))
            if abs(x - target) > um(pitch) / 2:
                continue
            trunks[net].append(x)
        k += 1

    added = vias_added = 0
    for net_name, net in nets.items():
        swire = odb.dbSWire_create(net, "ROUTED")
        rails = []
        for sw in net.getSWires():
            for w in sw.getWires():
                if not w.isVia() and layer_name(w) == "Metal1" and w.xMax() - w.xMin() > w.yMax() - w.yMin():
                    rails.append(w)
        for x in trunks[net_name]:
            odb.dbSBox_create(swire, tech_layer, x - half_w, ylo, x + half_w, yhi, "STRIPE")
            added += 1
            for r in rails:
                yc = (r.yMin() + r.yMax()) // 2
                if not r.xMin() + half_w <= x <= r.xMax() - half_w:
                    continue
                if any(b.xMin() - um(halo) <= x <= b.xMax() + um(halo) and b.yMin() - um(halo) <= yc <= b.yMax() + um(halo)
                       for b in boxes):
                    continue
                for v in vias[net_name]:
                    odb.dbSBox_create(swire, v, x, yc, "STRIPE")
                    vias_added += 1
        # 2. Column stripes over the macro for every other column
        for x, box in cols[net_name].items():
            if x in trunks[net_name]:
                continue
            odb.dbSBox_create(swire, tech_layer, x - half_w, box.yMin(), x + half_w, box.yMax(), "STRIPE")
            added += 1

    print(f"[INFO] trunks VPWR at {[x / dbu for x in trunks['VPWR']]}")
    print(f"[INFO] trunks VGND at {[x / dbu for x in trunks['VGND']]}")
    print(f"[INFO] added {added} stripes, {vias_added} vias; full height y {ylo / dbu:.2f}..{yhi / dbu:.2f}")


if __name__ == "__main__":
    main()
