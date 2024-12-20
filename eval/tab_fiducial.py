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
from scipy.spatial.transform import Rotation
from utilities import calculate_alpha, get_colors, plot_update_timing


class tab_fiducial:

    def __init__(self, fiducial_dfs, board_dfs, board_truth_dfs, args):
        self.fiducial_dfs = fiducial_dfs
        self.board_dfs = board_dfs
        self.board_truth_dfs = board_truth_dfs

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
                mskcf_df['cam_ang_0'],
                alpha=self.alpha,
                color=self.colors[0])
            fig.line(
                t_cam,
                mskcf_df['cam_ang_1'],
                alpha=self.alpha,
                color=self.colors[1])
            fig.line(
                t_cam,
                mskcf_df['cam_ang_2'],
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

        for tri_df, board_df in zip(self.board_dfs, self.board_truth_dfs):
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

        cds_x = ColumnDataSource({'base': times,
                                  'lower': mean_px - std_px,
                                  'upper': mean_px + std_px})
        cds_y = ColumnDataSource({'base': times,
                                  'lower': mean_py - std_py,
                                  'upper': mean_py + std_py})
        cds_z = ColumnDataSource({'base': times,
                                  'lower': mean_pz - std_pz,
                                  'upper': mean_pz + std_pz})

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

        err_x = collections.defaultdict(list)
        err_y = collections.defaultdict(list)
        err_z = collections.defaultdict(list)

        for tri_df, board_df in zip(self.board_dfs, self.board_truth_dfs):
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

            # TODO(jhartzer): Use common euler function
            for (t, b, qw, qx, qy, qz) in zip(time, board, board_qw, board_qx, board_qy, board_qz):
                board_q = Rotation.from_quat([qw, qx, qy, qz], scalar_first=True)
                true_q = Rotation.from_quat([
                    true_qw[int(b)],
                    true_qx[int(b)],
                    true_qy[int(b)],
                    true_qz[int(b)]], scalar_first=True)
                error_q = board_q.inv() * true_q
                error_eul = error_q.as_euler('XYZ')
                err_x[t].append(error_eul[0])
                err_y[t].append(error_eul[1])
                err_z[t].append(error_eul[2])

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

        fig.line(times, mean_x, color=self.colors[0])
        fig.line(times, mean_y, color=self.colors[1])
        fig.line(times, mean_z, color=self.colors[2])

        cds_x = ColumnDataSource(
            {'base': times,
             'lower': mean_x - std_x,
             'upper': mean_x + std_x})
        cds_y = ColumnDataSource(
            {'base': times,
             'lower': mean_y - std_y,
             'upper': mean_y + std_y})
        cds_z = ColumnDataSource(
            {'base': times,
             'lower': mean_z - std_z,
             'upper': mean_z + std_z})

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

    def get_tab(self):
        layout_plots = [[self.plot_fiducial_error_pos(), self.plot_fiducial_error_ang()]]

        if ('cam_cov_0' in self.fiducial_dfs[0].keys()):
            layout_plots.append([self.plot_camera_pos(), self.plot_camera_ang()])
            layout_plots.append([self.plot_cam_pos_cov(), self.plot_cam_ang_cov()])

        layout_plots.append([plot_update_timing(self.fiducial_dfs), Spacer()])

        tab_layout = layout(layout_plots, sizing_mode='stretch_width')
        tab = TabPanel(child=tab_layout, title=f"Fiducial {self.fiducial_dfs[0].attrs['id']}")

        return tab
