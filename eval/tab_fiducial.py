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
from utilities import calculate_alpha, get_colors, plot_update_timing


class tab_fiducial:

    def __init__(self, fiducial_dfs, tri_dfs, board_dfs, args):
        self.fiducial_dfs = fiducial_dfs
        self.tri_dfs = tri_dfs
        self.board_dfs = board_dfs

        self.alpha = calculate_alpha(len(self.fiducial_dfs))
        self.colors = get_colors(args)

    def plot_camera_pos(self):
        """Plot camera position offsets."""
        fig = figure(
            width=800,
            height=300,
            x_axis_label='Time [s]',
            y_axis_label='Position [m]',
            title='Camera Position')
        for mskcf_df in self.fiducial_dfs:
            t_cam = mskcf_df['time']
            fig.line(
                t_cam,
                mskcf_df['cam_pos_0'],
                alpha=self.alpha,
                color=self.colors[0])
            fig.line(
                t_cam,
                mskcf_df['cam_pos_1'],
                alpha=self.alpha,
                color=self.colors[1])
            fig.line(
                t_cam,
                mskcf_df['cam_pos_2'],
                alpha=self.alpha,
                color=self.colors[2])
        return fig

    def plot_camera_ang(self):
        """Plot camera angular offsets."""
        fig = figure(
            width=800,
            height=300,
            x_axis_label='Time [s]',
            y_axis_label='Orientation',
            title='Camera Orientation')
        for mskcf_df in self.fiducial_dfs:
            t_cam = mskcf_df['time']
            fig.line(
                t_cam,
                mskcf_df['cam_ang_pos_0'],
                alpha=self.alpha,
                color=self.colors[0])
            fig.line(
                t_cam,
                mskcf_df['cam_ang_pos_1'],
                alpha=self.alpha,
                color=self.colors[1])
            fig.line(
                t_cam,
                mskcf_df['cam_ang_pos_2'],
                alpha=self.alpha,
                color=self.colors[2])
        return fig

    def plot_cam_pos_cov(self):
        """Plot extrinsic position covariance."""
        fig = figure(
            width=800,
            height=300,
            x_axis_label='Time [s]',
            y_axis_label='Position Covariance [m]',
            title='Camera Position Covariance')
        for mskcf_df in self.fiducial_dfs:
            t_cam = mskcf_df['time']
            cam_cov_0 = mskcf_df['cam_cov_0']
            cam_cov_1 = mskcf_df['cam_cov_1']
            cam_cov_2 = mskcf_df['cam_cov_2']
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
            title='Camera Angle Covariance')
        for mskcf_df in self.fiducial_dfs:
            t_cam = mskcf_df['time']
            cam_cov_3 = mskcf_df['cam_cov_3']
            cam_cov_4 = mskcf_df['cam_cov_4']
            cam_cov_5 = mskcf_df['cam_cov_5']
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

    def plot_fiducial_error_pos(self):
        """Plot fiducial position error."""
        fig = figure(
            width=800,
            height=300,
            x_axis_label='Time [s]',
            y_axis_label='Position Error [m]',
            title='Fiducial Position Error')

        err_px = collections.defaultdict(list)
        err_py = collections.defaultdict(list)
        err_pz = collections.defaultdict(list)

        for tri_df, board_df in zip(self.tri_dfs, self.board_dfs):
            time = tri_df['time']
            board = tri_df['board']
            board_px = tri_df['pos_x']
            board_py = tri_df['pos_y']
            board_pz = tri_df['pos_z']

            true_px = board_df['pos_x']
            true_py = board_df['pos_y']
            true_pz = board_df['pos_z']

            for (t, b, px, py, pz) in zip(time, board, board_px, board_py, board_pz):
                err_px[t].append(px - true_px[int(b)])
                err_py[t].append(py - true_py[int(b)])
                err_pz[t].append(pz - true_pz[int(b)])
        times = []
        mean_px = []
        mean_py = []
        mean_pz = []
        std_px = []
        std_py = []
        std_pz = []

        for time in err_px:
            times.append(time)
            mean_px.append(np.mean(err_px[time]))
            mean_py.append(np.mean(err_py[time]))
            mean_pz.append(np.mean(err_pz[time]))
            std_px.append(np.std(err_px[time]))
            std_py.append(np.std(err_py[time]))
            std_pz.append(np.std(err_pz[time]))

        times = np.array(times)
        mean_px = np.array(mean_px)
        mean_py = np.array(mean_py)
        mean_pz = np.array(mean_pz)
        std_px = np.array(std_px)
        std_py = np.array(std_py)
        std_pz = np.array(std_pz)

        t_indices = times.argsort()
        times = times[t_indices]
        mean_px = mean_px[t_indices]
        mean_py = mean_py[t_indices]
        mean_pz = mean_pz[t_indices]
        std_px = std_px[t_indices]
        std_py = std_py[t_indices]
        std_pz = std_pz[t_indices]

        fig.line(times, mean_px, color=self.colors[0])
        fig.line(times, mean_py, color=self.colors[1])
        fig.line(times, mean_pz, color=self.colors[2])

        cds_x = ColumnDataSource({'base': times, 'lower': mean_px -
                                 std_px, 'upper': mean_px + std_px})
        cds_y = ColumnDataSource({'base': times, 'lower': mean_py -
                                 std_py, 'upper': mean_py + std_py})
        cds_z = ColumnDataSource({'base': times, 'lower': mean_pz -
                                 std_pz, 'upper': mean_pz + std_pz})

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

    def plot_fiducial_error_ang(self):
        """Plot fiducial angular error."""
        fig = figure(
            width=800,
            height=300,
            x_axis_label='Time [s]',
            y_axis_label='Angular Error',
            title='Fiducial Angular Error')

        err_qw = collections.defaultdict(list)
        err_qx = collections.defaultdict(list)
        err_qy = collections.defaultdict(list)
        err_qz = collections.defaultdict(list)

        for tri_df, board_df in zip(self.tri_dfs, self.board_dfs):
            time = tri_df['time']
            board = tri_df['board']
            board_qw = tri_df['quat_w']
            board_qx = tri_df['quat_x']
            board_qy = tri_df['quat_y']
            board_qz = tri_df['quat_z']

            true_qw = board_df['quat_w']
            true_qx = board_df['quat_x']
            true_qy = board_df['quat_y']
            true_qz = board_df['quat_z']

            for (t, b, qw, qx, qy, qz) in zip(time, board, board_qw, board_qx, board_qy, board_qz):
                err_qw[t].append(qw - true_qw[int(b)])
                err_qx[t].append(qx - true_qx[int(b)])
                err_qy[t].append(qy - true_qy[int(b)])
                err_qz[t].append(qz - true_qz[int(b)])
        times = []
        mean_qw = []
        mean_qx = []
        mean_qy = []
        mean_qz = []
        std_qw = []
        std_qx = []
        std_qy = []
        std_qz = []

        for time in err_qw:
            times.append(time)
            mean_qw.append(np.mean(err_qw[time]))
            mean_qx.append(np.mean(err_qx[time]))
            mean_qy.append(np.mean(err_qy[time]))
            mean_qz.append(np.mean(err_qz[time]))
            std_qw.append(np.std(err_qw[time]))
            std_qx.append(np.std(err_qx[time]))
            std_qy.append(np.std(err_qy[time]))
            std_qz.append(np.std(err_qz[time]))

        times = np.array(times)
        mean_qw = np.array(mean_qw)
        mean_qx = np.array(mean_qx)
        mean_qy = np.array(mean_qy)
        mean_qz = np.array(mean_qz)
        std_qw = np.array(std_qw)
        std_qx = np.array(std_qx)
        std_qy = np.array(std_qy)
        std_qz = np.array(std_qz)

        t_indices = times.argsort()
        times = times[t_indices]
        mean_qw = mean_qw[t_indices]
        mean_qx = mean_qx[t_indices]
        mean_qy = mean_qy[t_indices]
        mean_qz = mean_qz[t_indices]
        std_qw = std_qw[t_indices]
        std_qx = std_qx[t_indices]
        std_qy = std_qy[t_indices]
        std_qz = std_qz[t_indices]

        fig.line(times, mean_qw, color=self.colors[0])
        fig.line(times, mean_qx, color=self.colors[1])
        fig.line(times, mean_qy, color=self.colors[2])
        fig.line(times, mean_qz, color='purple')

        cds_w = ColumnDataSource(
            {'base': times,
             'lower': mean_qw - std_qw,
             'upper': mean_qw + std_qw})
        cds_x = ColumnDataSource(
            {'base': times,
             'lower': mean_qx - std_qx,
             'upper': mean_qx + std_qx})
        cds_y = ColumnDataSource(
            {'base': times,
             'lower': mean_qy - std_qy,
             'upper': mean_qy + std_qy})
        cds_z = ColumnDataSource(
            {'base': times,
             'lower': mean_qz - std_qz,
             'upper': mean_qz + std_qz})

        fig.add_layout(
            Band(
                base='base',
                lower='lower',
                upper='upper',
                fill_alpha=0.3,
                source=cds_w,
                fill_color=self.colors[0],
                line_color=self.colors[0]))
        fig.add_layout(
            Band(
                base='base',
                lower='lower',
                upper='upper',
                fill_alpha=0.3,
                source=cds_x,
                fill_color=self.colors[1],
                line_color=self.colors[1]))
        fig.add_layout(
            Band(
                base='base',
                lower='lower',
                upper='upper',
                fill_alpha=0.3,
                source=cds_y,
                fill_color=self.colors[2],
                line_color=self.colors[2]))
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

    def get_tab(self):
        layout_plots = [
            [
                self.plot_camera_pos(),
                self.plot_camera_ang()
            ],
            [
                self.plot_cam_pos_cov(),
                self.plot_cam_ang_cov()
            ],
            [
                self.plot_fiducial_error_pos(),
                self.plot_fiducial_error_ang()
            ],
            [
                plot_update_timing(self.fiducial_dfs),
                Spacer()
            ]
        ]

        tab_layout = layout(layout_plots, sizing_mode='stretch_width')
        tab = TabPanel(child=tab_layout,
                       title=f"Fiducial {self.fiducial_dfs[0].attrs['id']}")

        return tab
