#!/usr/bin/env python3

# Copyright 2024 Jacob Hartzer
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import numpy as np

from bokeh.models import TabPanel, Tabs
from bokeh.plotting import figure, show

from bokeh.layouts import column, row, layout
from bokeh.plotting import figure, show

def tab_body(body_state_dfs, body_truth_dfs):
    p1 = figure(width=600, height=200)
    p2 = figure(width=600, height=200)
    p3 = figure(width=600, height=200)
    p4 = figure(width=600, height=200)

    p1.circle([1, 2, 3, 4, 5], [6, 7, 2, 4, 5], size=20, color="navy", alpha=0.5)
    p2.circle([1, 2, 3, 4, 5], [6, 7, 2, 4, 5], size=20, color="navy", alpha=0.5)
    p3.circle([1, 2, 3, 4, 5], [6, 7, 2, 4, 5], size=20, color="navy", alpha=0.5)
    p4.circle([1, 2, 3, 4, 5], [6, 7, 2, 4, 5], size=20, color="navy", alpha=0.5)

    tab_layout = layout([[p1],[p2],[p3],[p4]], sizing_mode="stretch_width")
    tab = TabPanel(child=tab_layout, title=f"Body")

    return tab