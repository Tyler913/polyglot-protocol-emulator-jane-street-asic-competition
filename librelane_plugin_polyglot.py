# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""LibreLane plugin for the Polyglot tile.

LibreLane imports modules named ``librelane_plugin_*`` from the Python path;
``python -m librelane`` runs from the repository root, so this file is found
both locally and by the Tiny Tapeout GDS action.

Polyglot.SramStripes: the CMOS5L tile has a single Metal4 stripe PDN, and the
IHP SRAM macros have their supplies on Metal4 columns too. Tiny Tapeout needs
every tile stripe to run the full die height, and a stripe can only cross the
macro on a same-net supply column. pdngen is kept out of the macro's x-range;
this step then draws full-height trunks on supply columns (with rail via
stacks) and stripes on the remaining columns. See flow/odb_sram_stripes.py.
Idea after Ken Pettit's IHP SRAM walkthrough
(https://kdp1965.github.io/ihp-um-janestreet-prism/); the code is our own.
"""

import os
from decimal import Decimal
from typing import List, Tuple

from librelane.config import Variable
from librelane.steps import Step
from librelane.steps.common_variables import pdn_variables
from librelane.steps.odb import OdbpyStep


@Step.factory.register()
class SramStripes(OdbpyStep):
    id = "Polyglot.SramStripes"
    name = "Metal4 stripes on SRAM supply columns"

    config_vars = pdn_variables + [
        Variable("SRAM_STRIPES_MASTER_PREFIX", str, "Master name prefix of the SRAM macros.", default="RM_IHPSG13_"),
        Variable("SRAM_STRIPES_LAYER", str, "Layer of the tile stripes and the macro supply columns.", default="Metal4"),
        Variable(
            "SRAM_STRIPES_REGION",
            Tuple[Decimal, Decimal],
            "x range (um) kept free of pdngen stripes by PDN_OBSTRUCTIONS; the tile grid's stripes there become trunks on SRAM supply columns.",
        ),
        Variable("SRAM_STRIPES_HALO", Decimal, "No rail vias this close to a macro.", units="µm", default=Decimal("10")),
    ]

    def get_script_path(self):
        return os.path.join(os.path.dirname(os.path.abspath(__file__)), "flow", "odb_sram_stripes.py")

    def get_command(self) -> List[str]:
        x0, x1 = self.config["SRAM_STRIPES_REGION"]
        return super().get_command() + [
            "--master-prefix", self.config["SRAM_STRIPES_MASTER_PREFIX"],
            "--layer", self.config["SRAM_STRIPES_LAYER"],
            "--width", str(self.config["PDN_VWIDTH"]),
            "--pitch", str(self.config["PDN_VPITCH"]),
            "--offset", str(self.config["PDN_VOFFSET"]),
            "--spacing", str(self.config["PDN_VSPACING"]),
            "--region", str(x0), str(x1),
            "--halo", str(self.config["SRAM_STRIPES_HALO"]),
        ]
