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
from bokeh.models import Spacer, TabPanel
from bokeh.plotting import figure
import numpy as np

from utilities import calculate_alpha, get_colors, interpolate_error, plot_update_timing


class tab_gps:

    def __init__(self, gps_dfs, body_truth_dfs, args):
        self.gps_dfs = gps_dfs
        self.body_truth_dfs = body_truth_dfs

        self.alpha = calculate_alpha(len(self.gps_dfs))
        self.colors = get_colors(args)

    def plot_gps_measurements(self):
        """Plot camera GPS measurements."""
        fig = figure(
            width=400,
            height=300,
            x_axis_label='Time [s]',
            y_axis_label='Position [m]',
            title='GPS Measurements')
        for gps_df in self.gps_dfs:
            t_gps = gps_df['time']
            fig.line(
                t_gps,
                gps_df['x'],
                alpha=self.alpha,
                color=self.colors[0],
                legend_label='X')
            fig.line(
                t_gps,
                gps_df['y'],
                alpha=self.alpha,
                color=self.colors[1],
                legend_label='Y')
            fig.line(
                t_gps,
                gps_df['z'],
                alpha=self.alpha,
                color=self.colors[2],
                legend_label='Z')
        return fig

    def plot_gps_residuals(self):
        """Plot camera GPS residuals."""
        fig = figure(
            width=400,
            height=300,
            x_axis_label='Time [s]',
            y_axis_label='Position Residual [m]',
            title='GPS Residuals')
        for gps_df in self.gps_dfs:
            t_gps = gps_df['time']
            fig.line(
                t_gps,
                gps_df['residual_0'],
                alpha=self.alpha,
                color=self.colors[0],
                legend_label='X')
            fig.line(
                t_gps,
                gps_df['residual_1'],
                alpha=self.alpha,
                color=self.colors[1],
                legend_label='Y')
            fig.line(
                t_gps,
                gps_df['residual_2'],
                alpha=self.alpha,
                color=self.colors[2],
                legend_label='Z')
        return fig

    def plot_ant_pos_error(self):
        """Plot camera GPS residuals."""
        fig = figure(
            width=400,
            height=300,
            x_axis_label='Time [s]',
            y_axis_label='Position Error [mm]',
            title='GPS Antenna Position Error')
        for gps_df, body_truth in zip(self.gps_dfs, self.body_truth_dfs):
            true_t = body_truth['time']
            true_p0 = body_truth[f"gps_pos_{gps_df.attrs['id']}_0"]
            true_p1 = body_truth[f"gps_pos_{gps_df.attrs['id']}_1"]
            true_p2 = body_truth[f"gps_pos_{gps_df.attrs['id']}_2"]

            t_gps = gps_df['time']
            est_p0 = gps_df['ant_pos_0']
            est_p1 = gps_df['ant_pos_1']
            est_p2 = gps_df['ant_pos_2']

            err_pos_0 = np.array(interpolate_error(true_t, true_p0, t_gps, est_p0)) * 1e3
            err_pos_1 = np.array(interpolate_error(true_t, true_p1, t_gps, est_p1)) * 1e3
            err_pos_2 = np.array(interpolate_error(true_t, true_p2, t_gps, est_p2)) * 1e3

            fig.line(
                t_gps,
                err_pos_0,
                alpha=self.alpha,
                color=self.colors[0],
                legend_label='X')
            fig.line(
                t_gps,
                err_pos_1,
                alpha=self.alpha,
                color=self.colors[1],
                legend_label='Y')
            fig.line(
                t_gps,
                err_pos_2,
                alpha=self.alpha,
                color=self.colors[2],
                legend_label='Z')
        return fig

    def plot_gps_cov(self):
        """Plot GPS antenna position covariance."""
        fig = figure(
            width=800,
            height=300,
            x_axis_label='Time [s]',
            y_axis_label='Position Covariance [mm]',
            title='GPS Antenna Position Covariance')
        for gps_df in self.gps_dfs:
            t_gps = gps_df['time']
            gps_int_cov_3 = np.array(gps_df['gps_cov_0'])*1e3
            gps_int_cov_4 = np.array(gps_df['gps_cov_1'])*1e3
            gps_int_cov_5 = np.array(gps_df['gps_cov_2'])*1e3
            fig.line(
                t_gps,
                gps_int_cov_3,
                alpha=self.alpha,
                color=self.colors[0],
                legend_label='X')
            fig.line(
                t_gps,
                gps_int_cov_4,
                alpha=self.alpha,
                color=self.colors[1],
                legend_label='Y')
            fig.line(
                t_gps,
                gps_int_cov_5,
                alpha=self.alpha,
                color=self.colors[2],
                legend_label='Z')
        return fig

    def get_tab(self):

        layout_plots = [[self.plot_gps_measurements(), self.plot_gps_residuals()]]

        if ('gps_cov_0' in self.gps_dfs[0].keys()):
            layout_plots.append([self.plot_ant_pos_error(), self.plot_gps_cov()])

        layout_plots.append([plot_update_timing(self.gps_dfs), Spacer()])

        tab_layout = layout(layout_plots, sizing_mode='stretch_width')
        tab = TabPanel(child=tab_layout,
                       title=f"GPS {self.gps_dfs[0].attrs['id']}")

        return tab
