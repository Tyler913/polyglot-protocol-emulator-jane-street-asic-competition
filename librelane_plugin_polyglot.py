# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""LibreLane plugin for the Polyglot tile.

LibreLane imports modules named ``librelane_plugin_*`` from the Python path;
``python -m librelane`` runs from the repository root, so this file is found
both locally and by the Tiny Tapeout GDS action.

Polyglot.SramStripes: the CMOS5L tile has a single Metal4 stripe PDN, and the
IHP SRAM macros have their supplies on Metal4 columns too. pdngen cannot draw
an instance grid on a layer the macro uses, so this step adds Metal4 VPWR/VGND
stripes on the macro's supply columns after pdngen (idea from Ken Pettit's IHP
SRAM walkthrough, https://kdp1965.github.io/ihp-um-janestreet-prism/).
"""

import os
from decimal import Decimal
from typing import List

from librelane.config import Variable
from librelane.steps import Step
from librelane.steps.odb import OdbpyStep


@Step.factory.register()
class SramStripes(OdbpyStep):
    id = "Polyglot.SramStripes"
    name = "Metal4 stripes on SRAM supply columns"

    config_vars = [
        Variable("SRAM_STRIPES_MASTER_PREFIX", str, "Master name prefix of the SRAM macros.", default="RM_IHPSG13_"),
        Variable("SRAM_STRIPES_LAYER", str, "Layer of the tile stripes and the macro supply columns.", default="Metal4"),
        Variable("SRAM_STRIPES_WIDTH", Decimal, "Width of each added stripe.", units="µm", default=Decimal("2.1")),
        Variable("SRAM_STRIPES_EVERY", int, "Feed every n-th column (1 = all columns).", default=1),
        Variable(
            "SRAM_STRIPES_EXTEND",
            Decimal,
            "How far a stripe continues past the macro edge facing the die centre, to reach the tile grid (must exceed PDN_VERTICAL_HALO).",
            units="µm",
            default=Decimal("12"),
        ),
        Variable(
            "SRAM_STRIPES_JOIN_TOLERANCE",
            Decimal,
            "A column joins a tile stripe of the same net if their centres differ by at most this.",
            units="µm",
            default=Decimal("0.7"),
        ),
    ]

    def get_script_path(self):
        return os.path.join(os.path.dirname(os.path.abspath(__file__)), "flow", "odb_sram_stripes.py")

    def get_command(self) -> List[str]:
        return super().get_command() + [
            "--master-prefix", self.config["SRAM_STRIPES_MASTER_PREFIX"],
            "--layer", self.config["SRAM_STRIPES_LAYER"],
            "--width", str(self.config["SRAM_STRIPES_WIDTH"]),
            "--every", str(self.config["SRAM_STRIPES_EVERY"]),
            "--extend", str(self.config["SRAM_STRIPES_EXTEND"]),
            "--join-tolerance", str(self.config["SRAM_STRIPES_JOIN_TOLERANCE"]),
        ]
