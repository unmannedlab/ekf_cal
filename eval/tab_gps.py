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

    def plot_ant_pos_error(self, gps_dfs, body_truth_dfs):
        """Plot camera GPS residuals."""
        fig = figure(
            width=400,
            height=300,
            x_axis_label='Time [s]',
            y_axis_label='Antenna Position Error [mm]',
            title='Antenna Position Error')
        for gps_df, body_truth in zip(gps_dfs, body_truth_dfs):
            true_t = body_truth['time']
            true_p0 = body_truth[f"gps_pos_{gps_df.attrs['id']}_0"]
            true_p1 = body_truth[f"gps_pos_{gps_df.attrs['id']}_1"]
            true_p2 = body_truth[f"gps_pos_{gps_df.attrs['id']}_2"]

            t_gps = gps_df['time']
            est_p0 = gps_df['antenna_0']
            est_p1 = gps_df['antenna_1']
            est_p2 = gps_df['antenna_2']

            err_pos_0 = np.array(interpolate_error(true_t, true_p0, t_gps, est_p0))*1e3
            err_pos_1 = np.array(interpolate_error(true_t, true_p1, t_gps, est_p1))*1e3
            err_pos_2 = np.array(interpolate_error(true_t, true_p2, t_gps, est_p2))*1e3

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

    def print_gps_init_pos_error(self):
        pos_err_list = []
        for gps_df in self.gps_dfs:
            ref_lat = gps_df['ref_lat']
            ref_lon = gps_df['ref_lon']
            ref_alt = gps_df['ref_alt']
            pos_err_list.append(np.sqrt(ref_lat*ref_lat + ref_lon*ref_lon + ref_alt*ref_alt))

        pos_err = np.mean(pos_err_list)

        return Paragraph(text='Position Error: {:.3f} m'.format(pos_err))

    def print_gps_init_hdg_error(self):
        hdg_err_list = []
        for gps_df in self.gps_dfs:
            hdg_err_list.append(gps_df['ref_heading'])

        hdg_err = np.mean(hdg_err_list)

        return Paragraph(text='Heading Error: {:.3f} rad'.format(hdg_err))

    def get_tab(self):

        layout_plots = [[self.plot_gps_measurements(), self.plot_gps_residuals()]]

        if ('gps_cov_0' in self.gps_dfs[0].keys()):
            layout_plots.append([self.plot_ant_pos_error(), self.plot_gps_cov()])

        layout_plots.append([plot_update_timing(self.gps_dfs), Spacer()])
        layout_plots.append([self.print_gps_init_pos_error()])
        layout_plots.append([self.print_gps_init_hdg_error()])

        tab_layout = layout(layout_plots, sizing_mode='stretch_width')
        tab = TabPanel(child=tab_layout,
                       title=f"GPS {self.gps_dfs[0].attrs['id']}")

        return tab
