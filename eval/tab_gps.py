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

from bokeh.layouts import layout
from bokeh.models import Paragraph, TabPanel
from bokeh.plotting import figure
import numpy as np

from utilities import calculate_alpha, plot_update_timing


def plot_gps_measurements(gps_dfs):
    """Plot camera GPS measurements."""
    fig = figure(width=800, height=300, x_axis_label='time [s]',
                 y_axis_label='Position [m]', title='GPS Measurements')
    a = calculate_alpha(len(gps_dfs))
    for gps_df in gps_dfs:
        t_cam = gps_df['time'].to_list()
        fig.line(t_cam, gps_df['x'].to_list(), alpha=a, color='cyan')
        fig.line(t_cam, gps_df['y'].to_list(), alpha=a, color='yellow')
        fig.line(t_cam, gps_df['z'].to_list(), alpha=a, color='magenta')
    return fig


def plot_gps_residuals(gps_dfs):
    """Plot camera GPS residuals."""
    fig = figure(width=800, height=300, x_axis_label='time [s]',
                 y_axis_label='Position Residual [m]', title='GPS Residuals')
    a = calculate_alpha(len(gps_dfs))
    for gps_df in gps_dfs:
        t_cam = gps_df['time'].to_list()
        fig.line(t_cam, gps_df['residual_0'].to_list(), alpha=a, color='cyan')
        fig.line(t_cam, gps_df['residual_1'].to_list(), alpha=a, color='yellow')
        fig.line(t_cam, gps_df['residual_2'].to_list(), alpha=a, color='magenta')
    return fig


def print_gps_init_pos_error(gps_dfs):
    pos_err_list = []
    for gps_df in gps_dfs:
        ref_lat = gps_df['ref_lat']
        ref_lon = gps_df['ref_lon']
        ref_alt = gps_df['ref_alt']
        pos_err_list.append(np.sqrt(ref_lat*ref_lat + ref_lon*ref_lon + ref_alt*ref_alt))

    pos_err = np.mean(pos_err_list)

    return Paragraph(text='Position Error: {:.3f} m'.format(pos_err))


def print_gps_init_hdg_error(gps_dfs):
    hdg_err_list = []
    for gps_df in gps_dfs:
        hdg_err_list.append(gps_df['ref_heading'])

    hdg_err = np.mean(hdg_err_list)

    return Paragraph(text='Heading Error: {:.3f} rad'.format(hdg_err))


def tab_gps(gps_dfs):

    layout_plots = [
        [plot_gps_measurements(gps_dfs)],
        [plot_gps_residuals(gps_dfs)],
        [plot_update_timing(gps_dfs)],
        [print_gps_init_pos_error(gps_dfs)],
        [print_gps_init_hdg_error(gps_dfs)]
    ]

    tab_layout = layout(layout_plots, sizing_mode='stretch_width')
    tab = TabPanel(child=tab_layout, title=f"GPS {gps_dfs[0].attrs['id']}")

    return tab
