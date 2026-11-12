# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Add Metal4 VPWR/VGND stripes on the supply columns of IHP SRAM macros.

For every instance whose master starts with --master-prefix, each Metal4
shape of a power pin (VDD!, VDDARRAY!) or ground pin (VSS!) becomes part of a
column; columns are grouped by x. A stripe of --width is centred on each
column (or every n-th) and runs over the full macro height. Where a tile
stripe of the same net (from pdngen, which stops at the macro halo) lies on
the column (centres within --join-tolerance), the new stripe continues
--extend um past the macro edge facing the die centre, so the two overlap.
It stays centred on the column, except when the centres differ by less than
0.1 um, where it takes the tile stripe's exact x (no tiny jog). The 1024x32
macro's right bit-cell array is 0.665 um off the left array's column grid,
hence the tolerance. The other columns end at the macro edge: shapes of one
LEF pin are connected inside the macro. Run by
librelane_plugin_polyglot.py (Polyglot.SramStripes).
"""

import odb  # noqa: F401 (dbSWire_create, dbSBox_create)
from reader import click, click_odb

POWER = {"VDD!": "VPWR", "VDDARRAY!": "VPWR"}
GROUND = {"VSS!": "VGND"}


@click.command()
@click.option("--master-prefix", required=True)
@click.option("--layer", required=True)
@click.option("--width", type=float, required=True)
@click.option("--every", type=int, default=1)
@click.option("--extend", type=float, default=0.0)
@click.option("--join-tolerance", type=float, default=0.7)
@click_odb
def main(reader, master_prefix, layer, width, every, extend, join_tolerance):
    block = reader.block
    tech_layer = reader.tech.findLayer(layer)
    dbu = reader.tech.getDbUnitsPerMicron()
    half_w = int(round(width * dbu / 2))
    ext = int(round(extend * dbu))
    die = block.getDieArea()

    added = 0
    for inst in block.getInsts():
        master = inst.getMaster()
        if not master.getName().startswith(master_prefix):
            continue
        box = inst.getBBox()
        orient = inst.getOrient()
        if orient not in ("R0", "MX", "MY", "R180"):
            raise SystemExit(f"[ERROR] {inst.getName()}: orientation {orient} not supported")
        lx, ly = box.xMin(), box.yMin()
        mw, mh = master.getWidth(), master.getHeight()
        flip_x = orient in ("MY", "R180")
        flip_y = orient in ("MX", "R180")

        def tx(x):
            return lx + (mw - x if flip_x else x)

        def ty(y):
            return ly + (mh - y if flip_y else y)

        pin_y = []  # signal pin centres, for the log only
        cols = {}  # x centre -> net name
        for mterm in master.getMTerms():
            name = mterm.getName()
            net = POWER.get(name) or GROUND.get(name)
            for mpin in mterm.getMPins():
                for g in mpin.getGeometry():
                    if g.getTechLayer() is None or g.getTechLayer().getName() != layer:
                        if net is None:
                            pin_y.append(ty((g.yMin() + g.yMax()) // 2))
                        continue
                    if net is None:
                        continue
                    xc = tx((g.xMin() + g.xMax()) // 2)
                    prev = cols.get(xc)
                    if prev is not None and prev != net:
                        raise SystemExit(f"[ERROR] {inst.getName()}: column at x={xc / dbu} has both {prev} and {net}")
                    cols[xc] = net

        pins_at_top = bool(pin_y) and (sum(pin_y) / len(pin_y)) > (box.yMin() + box.yMax()) / 2
        toward_up = (box.yMin() + box.yMax()) < (die.yMin() + die.yMax())
        y0 = box.yMin() if toward_up else box.yMin() - ext
        y1 = box.yMax() + ext if toward_up else box.yMax()
        y0, y1 = max(y0, die.yMin()), min(y1, die.yMax())

        by_net = {"VPWR": [], "VGND": []}
        for xc in sorted(cols):
            by_net[cols[xc]].append(xc)
        snap = int(0.1 * dbu)
        tol = int(join_tolerance * dbu)
        joined = 0
        for net_name, xs in by_net.items():
            net = block.findNet(net_name)
            if net is None:
                raise SystemExit(f"[ERROR] net {net_name} not found")
            # Existing tile stripes of this net on the layer, by centre x
            tile = []
            for sw in net.getSWires():
                for w in sw.getWires():
                    if not w.isVia() and w.getTechLayer() is not None and w.getTechLayer().getName() == layer \
                            and w.yMax() - w.yMin() > w.xMax() - w.xMin():
                        tile.append(((w.xMin() + w.xMax()) // 2, w.xMin(), w.xMax()))
            swire = odb.dbSWire_create(net, "ROUTED")
            for i, xc in enumerate(xs):
                if i % every:
                    continue
                match = [t for t in tile if abs(t[0] - xc) <= tol]
                if match:
                    if abs(match[0][0] - xc) <= snap:
                        x0, x1 = match[0][1], match[0][2]
                    else:
                        x0, x1 = xc - half_w, xc + half_w
                    odb.dbSBox_create(swire, tech_layer, x0, y0, x1, y1, "STRIPE")
                    joined += 1
                else:
                    odb.dbSBox_create(swire, tech_layer, xc - half_w, box.yMin(), xc + half_w, box.yMax(), "STRIPE")
                added += 1
        print(
            f"[INFO] {inst.getName()} ({master.getName()}): {len(by_net['VPWR'])} VPWR / {len(by_net['VGND'])} VGND columns,"
            f" pins {'top' if pins_at_top else 'bottom'}, {joined} stripes joined to the tile grid (y {y0 / dbu:.2f}..{y1 / dbu:.2f})"
        )
        if joined == 0:
            raise SystemExit(f"[ERROR] {inst.getName()}: no column lines up with a tile stripe; check FP_PDN_VPITCH/VOFFSET")
    print(f"[INFO] added {added} stripes")


if __name__ == "__main__":
    main()
