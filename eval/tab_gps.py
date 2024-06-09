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
from bokeh.models import Paragraph, Spacer, TabPanel
from bokeh.plotting import figure
import numpy as np

from utilities import calculate_alpha, colors, interpolate_error, plot_update_timing


def plot_gps_measurements(gps_dfs):
    """Plot camera GPS measurements."""
    fig = figure(width=400, height=300, x_axis_label='Time [s]',
                 y_axis_label='Position [m]', title='GPS Measurements')
    a = calculate_alpha(len(gps_dfs))
    for gps_df in gps_dfs:
        t_gps = gps_df['time']
        fig.line(t_gps, gps_df['x'], alpha=a, color=colors[0], legend_label='x')
        fig.line(t_gps, gps_df['y'], alpha=a, color=colors[1], legend_label='y')
        fig.line(t_gps, gps_df['z'], alpha=a, color=colors[2], legend_label='z')
    return fig


def plot_gps_residuals(gps_dfs):
    """Plot camera GPS residuals."""
    fig = figure(width=400, height=300, x_axis_label='Time [s]',
                 y_axis_label='Position Residual [m]', title='GPS Residuals')
    a = calculate_alpha(len(gps_dfs))
    for gps_df in gps_dfs:
        t_gps = gps_df['time']
        fig.line(t_gps, gps_df['residual_0'], alpha=a, color=colors[0], legend_label='x')
        fig.line(t_gps, gps_df['residual_1'], alpha=a, color=colors[1], legend_label='y')
        fig.line(t_gps, gps_df['residual_2'], alpha=a, color=colors[2], legend_label='z')
    return fig


def plot_ant_pos_error(gps_dfs, body_truth_dfs):
    """Plot camera GPS residuals."""
    fig = figure(width=400, height=300, x_axis_label='Time [s]',
                 y_axis_label='Antenna Position Error [mm]', title='Antenna Position Error')
    a = calculate_alpha(len(gps_dfs))
    for gps_df, body_truth in zip(gps_dfs, body_truth_dfs):
        true_t = body_truth['time']
        true_p0 = body_truth[f"gps_pos_{gps_df.attrs['id']}_0"]
        true_p1 = body_truth[f"gps_pos_{gps_df.attrs['id']}_1"]
        true_p2 = body_truth[f"gps_pos_{gps_df.attrs['id']}_2"]

        gps_t = gps_df['time']
        est_p0 = gps_df['antenna_0']
        est_p1 = gps_df['antenna_1']
        est_p2 = gps_df['antenna_2']

        err_pos_0 = np.array(interpolate_error(true_t, true_p0, gps_t, est_p0))*1e3
        err_pos_1 = np.array(interpolate_error(true_t, true_p1, gps_t, est_p1))*1e3
        err_pos_2 = np.array(interpolate_error(true_t, true_p2, gps_t, est_p2))*1e3

        fig.line(gps_t, err_pos_0, alpha=a, color=colors[0], legend_label='x')
        fig.line(gps_t, err_pos_1, alpha=a, color=colors[1], legend_label='y')
        fig.line(gps_t, err_pos_2, alpha=a, color=colors[2], legend_label='z')
    return fig
    return fig


def plot_gps_cov(gps_dfs):
    """Plot GPS antenna position covariance."""
    fig = figure(width=800, height=300, x_axis_label='Time [s]',
                 y_axis_label='Position Covariance [mm]', title='GPS Antenna Position Covariance')
    a = calculate_alpha(len(gps_dfs))
    for gps_df in gps_dfs:
        t_gps = gps_df['time']
        gps_int_cov_3 = np.array(gps_df['gps_cov_0'])*1e3
        gps_int_cov_4 = np.array(gps_df['gps_cov_1'])*1e3
        gps_int_cov_5 = np.array(gps_df['gps_cov_2'])*1e3
        fig.line(t_gps, gps_int_cov_3, alpha=a, color=colors[0], legend_label='x')
        fig.line(t_gps, gps_int_cov_4, alpha=a, color=colors[1], legend_label='y')
        fig.line(t_gps, gps_int_cov_5, alpha=a, color=colors[2], legend_label='z')
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


def tab_gps(gps_dfs, body_truth_dfs):

    layout_plots = [[plot_gps_measurements(gps_dfs), plot_gps_residuals(gps_dfs)]]

    if ('gps_cov_0' in gps_dfs[0].keys()):
        layout_plots.append([plot_ant_pos_error(gps_dfs, body_truth_dfs), plot_gps_cov(gps_dfs)])

    layout_plots.append([plot_update_timing(gps_dfs), Spacer()])
    layout_plots.append([print_gps_init_pos_error(gps_dfs)])
    layout_plots.append([print_gps_init_hdg_error(gps_dfs)])

    tab_layout = layout(layout_plots, sizing_mode='stretch_width')
    tab = TabPanel(child=tab_layout, title=f"GPS {gps_dfs[0].attrs['id']}")

    return tab
