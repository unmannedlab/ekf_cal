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

import collections

from bokeh.layouts import layout
from bokeh.models import Band, Spacer, TabPanel
from bokeh.plotting import ColumnDataSource, figure

import numpy as np

from utilities import calculate_alpha, get_colors, interpolate_error, interpolate_quat_error, \
    plot_update_timing


class tab_msckf:

    def __init__(self, msckf_dfs, tri_dfs, feat_dfs, body_truth_dfs, args):
        self.msckf_dfs = msckf_dfs
        self.tri_dfs = tri_dfs
        self.feat_dfs = feat_dfs
        self.body_truth_dfs = body_truth_dfs

        self.alpha = calculate_alpha(len(self.msckf_dfs))
        self.colors = get_colors(args)

    def plot_cam_pos(self):
        """Plot camera position offsets."""
        fig = figure(
            width=800,
            height=300,
            x_axis_label='Time [s]',
            y_axis_label='Position [m]',
            title='Camera Position')
        for msckf_df in self.msckf_dfs:
            t_cam = msckf_df['time']
            fig.line(
                t_cam,
                msckf_df['cam_pos_0'],
                alpha=self.alpha,
                color=self.colors[0],
                legend_label='X')
            fig.line(
                t_cam,
                msckf_df['cam_pos_1'],
                alpha=self.alpha,
                color=self.colors[1],
                legend_label='Y')
            fig.line(
                t_cam,
                msckf_df['cam_pos_2'],
                alpha=self.alpha,
                color=self.colors[2],
                legend_label='Z')
        return fig

    def plot_cam_ang(self):
        """Plot camera angular offsets."""
        fig = figure(
            width=800,
            height=300,
            x_axis_label='Time [s]',
            y_axis_label='Orientation',
            title='Camera Orientation')
        for msckf_df in self.msckf_dfs:
            t_cam = msckf_df['time']
            fig.line(
                t_cam,
                msckf_df['cam_ang_pos_0'],
                alpha=self.alpha,
                color=self.colors[0],
                legend_label='X')
            fig.line(
                t_cam,
                msckf_df['cam_ang_pos_1'],
                alpha=self.alpha,
                color=self.colors[1],
                legend_label='Y')
            fig.line(
                t_cam,
                msckf_df['cam_ang_pos_2'],
                alpha=self.alpha,
                color=self.colors[2],
                legend_label='Z')
        return fig

    def plot_cam_pos_err(self):
        """Plot camera extrinsic position errors."""
        fig = figure(
            width=400,
            height=300,
            x_axis_label='Time [s]',
            y_axis_label='Position Error [mm]',
            title='Camera Extrinsic Position Error')
        for msckf_df, body_truth in zip(self.msckf_dfs, self.body_truth_dfs):
            true_t = body_truth['time']
            true_p0 = body_truth[f"cam_pos_{msckf_df.attrs['id']}_0"]
            true_p1 = body_truth[f"cam_pos_{msckf_df.attrs['id']}_1"]
            true_p2 = body_truth[f"cam_pos_{msckf_df.attrs['id']}_2"]

            t_gps = msckf_df['time']
            est_p0 = msckf_df['cam_pos_0']
            est_p1 = msckf_df['cam_pos_1']
            est_p2 = msckf_df['cam_pos_2']

            err_pos_0 = np.array(interpolate_error(true_t, true_p0, t_gps, est_p0))
            err_pos_1 = np.array(interpolate_error(true_t, true_p1, t_gps, est_p1))
            err_pos_2 = np.array(interpolate_error(true_t, true_p2, t_gps, est_p2))

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

    def plot_cam_ang_err(self):
        """Plot the camera extrinsic angular error."""
        fig = figure(
            width=800,
            height=300,
            x_axis_label='Time [s]',
            y_axis_label='Angle Error [m]',
            title='Camera Extrinsic Angle Error')
        for msckf_df, body_truth in zip(self.msckf_dfs, self.body_truth_dfs):
            est_t = msckf_df['time']
            est_w = msckf_df['cam_ang_pos_0']
            est_x = msckf_df['cam_ang_pos_1']
            est_y = msckf_df['cam_ang_pos_2']
            est_z = msckf_df['cam_ang_pos_3']
            true_t = body_truth['time']
            true_w = body_truth[f"cam_ang_pos_{msckf_df.attrs['id']}_0"]
            true_x = body_truth[f"cam_ang_pos_{msckf_df.attrs['id']}_1"]
            true_y = body_truth[f"cam_ang_pos_{msckf_df.attrs['id']}_2"]
            true_z = body_truth[f"cam_ang_pos_{msckf_df.attrs['id']}_3"]

            eul_err_x, eul_err_y, eul_err_z = interpolate_quat_error(
                true_t, true_w, true_x, true_y, true_z,
                est_t, est_w, est_x, est_y, est_z)

            fig.line(
                est_t,
                eul_err_x,
                alpha=self.alpha,
                color=self.colors[0],
                legend_label='x')
            fig.line(
                est_t,
                eul_err_y,
                alpha=self.alpha,
                color=self.colors[1],
                legend_label='y')
            fig.line(
                est_t,
                eul_err_z,
                alpha=self.alpha,
                color=self.colors[2],
                legend_label='z')
        return fig

    def plot_cam_pos_cov(self):
        """Plot extrinsic position covariance."""
        fig = figure(
            width=800,
            height=300,
            x_axis_label='Time [s]',
            y_axis_label='Position Covariance [m]',
            title='Position Covariance')
        for msckf_df in self.msckf_dfs:
            t_cam = msckf_df['time']
            cam_cov_0 = msckf_df['cam_cov_0']
            cam_cov_1 = msckf_df['cam_cov_1']
            cam_cov_2 = msckf_df['cam_cov_2']
            fig.line(
                t_cam,
                cam_cov_0,
                alpha=self.alpha,
                color=self.colors[0],
                legend_label='X')
            fig.line(
                t_cam,
                cam_cov_1,
                alpha=self.alpha,
                color=self.colors[1],
                legend_label='Y')
            fig.line(
                t_cam,
                cam_cov_2,
                alpha=self.alpha,
                color=self.colors[2],
                legend_label='Z')
        return fig

    def plot_cam_ang_cov(self):
        """Plot extrinsic angle covariance."""
        fig = figure(
            width=800,
            height=300,
            x_axis_label='Time [s]',
            y_axis_label='Angle Covariance [m]',
            title='Angle Covariance')
        for msckf_df in self.msckf_dfs:
            t_cam = msckf_df['time']
            cam_cov_3 = msckf_df['cam_cov_3']
            cam_cov_4 = msckf_df['cam_cov_4']
            cam_cov_5 = msckf_df['cam_cov_5']
            fig.line(
                t_cam,
                cam_cov_3,
                alpha=self.alpha,
                color=self.colors[0],
                legend_label='X')
            fig.line(
                t_cam,
                cam_cov_4,
                alpha=self.alpha,
                color=self.colors[1],
                legend_label='Y')
            fig.line(
                t_cam,
                cam_cov_5,
                alpha=self.alpha,
                color=self.colors[2],
                legend_label='Z')
        return fig

    def plot_triangulation_error(self):
        """Plot MSCKF feature point triangulation error."""
        fig = figure(
            width=800,
            height=300,
            x_axis_label='Time [s]',
            y_axis_label='Triangulation Error [m]',
            title='MSCKF Triangulation Error')

        err_x = collections.defaultdict(list)
        err_y = collections.defaultdict(list)
        err_z = collections.defaultdict(list)

        for tri_df, feat_df in zip(self.tri_dfs, self.feat_dfs):
            time = tri_df['time']
            feature = tri_df['feature']
            feat_x = tri_df['x']
            feat_y = tri_df['y']
            feat_z = tri_df['z']

            true_x = feat_df['x']
            true_y = feat_df['y']
            true_z = feat_df['z']

            for (t, f, x, y, z) in zip(time, feature, feat_x, feat_y, feat_z):
                err_x[t].append(x - true_x[int(f)])
                err_y[t].append(y - true_y[int(f)])
                err_z[t].append(z - true_z[int(f)])
        times = []
        mean_x = []
        mean_y = []
        mean_z = []
        std_x = []
        std_y = []
        std_z = []

        for time in err_x:
            times.append(time)
            mean_x.append(np.mean(err_x[time]))
            mean_y.append(np.mean(err_y[time]))
            mean_z.append(np.mean(err_z[time]))
            std_x.append(np.std(err_x[time]))
            std_y.append(np.std(err_y[time]))
            std_z.append(np.std(err_z[time]))

        times = np.array(times)
        mean_x = np.array(mean_x)
        mean_y = np.array(mean_y)
        mean_z = np.array(mean_z)
        std_x = np.array(std_x)
        std_y = np.array(std_y)
        std_z = np.array(std_z)

        t_indices = times.argsort()
        times = times[t_indices]
        mean_x = mean_x[t_indices]
        mean_y = mean_y[t_indices]
        mean_z = mean_z[t_indices]
        std_x = std_x[t_indices]
        std_y = std_y[t_indices]
        std_z = std_z[t_indices]

        fig.line(
            times,
            mean_x,
            color=self.colors[0],
            legend_label='X')
        fig.line(
            times,
            mean_y,
            color=self.colors[1],
            legend_label='Y')
        fig.line(
            times,
            mean_z,
            color=self.colors[2],
            legend_label='Z')

        cds_x = ColumnDataSource({'base': times, 'lower': mean_x - std_x, 'upper': mean_x + std_x})
        cds_y = ColumnDataSource({'base': times, 'lower': mean_y - std_y, 'upper': mean_y + std_y})
        cds_z = ColumnDataSource({'base': times, 'lower': mean_z - std_z, 'upper': mean_z + std_z})

        fig.add_layout(
            Band(
                base='base',
                lower='lower',
                upper='upper',
                fill_alpha=0.3,
                source=cds_x,
                fill_color=self.colors[0],
                line_color=self.colors[0]))
        fig.add_layout(
            Band(
                base='base',
                lower='lower',
                upper='upper',
                fill_alpha=0.3,
                source=cds_y,
                fill_color=self.colors[1],
                line_color=self.colors[1]))
        fig.add_layout(
            Band(
                base='base',
                lower='lower',
                upper='upper',
                fill_alpha=0.3,
                source=cds_z,
                fill_color=self.colors[2],
                line_color=self.colors[2]))

        return fig

    def plot_track_count(self):
        """Plot number of tracks."""
        fig = figure(
            width=800,
            height=300,
            x_axis_label='Time [s]',
            y_axis_label='Number of Tracks',
            title='Track Counts')
        for msckf_df in self.msckf_dfs:
            t_cam = msckf_df['time']
            track_count = msckf_df['FeatureTracks']
            fig.line(
                t_cam,
                track_count,
                alpha=self.alpha,
                color=self.colors[0],
                legend_label='Track Count')
        return fig

    def get_tab(self):
        layout_plots = [[plot_update_timing(self.msckf_dfs), self.plot_triangulation_error()]]

        if ('cam_cov_0' in self.msckf_dfs[0].keys()):
            layout_plots.append([self.plot_cam_pos(), self.plot_cam_ang()])
            layout_plots.append([self.plot_cam_pos_err(), self.plot_cam_ang_err()])
            layout_plots.append([self.plot_cam_pos_cov(), self.plot_cam_ang_cov()])

        layout_plots.append([self.plot_track_count(), Spacer()])

        tab_layout = layout(layout_plots, sizing_mode='stretch_width')
        tab = TabPanel(child=tab_layout, title=f"MSCKF {self.msckf_dfs[0].attrs['id']}")

        return tab
