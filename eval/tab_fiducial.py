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
from utilities import calculate_alpha, get_colors, interpolate_error, interpolate_quat_error, \
    plot_update_timing


class tab_fiducial:

    def __init__(self, fiducial_dfs, board_truth_dfs, body_truth_dfs, args):
        self.fiducial_dfs = fiducial_dfs
        self.board_truth_dfs = board_truth_dfs
        self.body_truth_dfs = body_truth_dfs

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
        for fiducial_df in self.fiducial_dfs:
            t_cam = fiducial_df['time']
            fig.line(
                t_cam,
                fiducial_df['cam_pos_0'],
                alpha=self.alpha,
                color=self.colors[0])
            fig.line(
                t_cam,
                fiducial_df['cam_pos_1'],
                alpha=self.alpha,
                color=self.colors[1])
            fig.line(
                t_cam,
                fiducial_df['cam_pos_2'],
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
        for fiducial_df in self.fiducial_dfs:
            t_cam = fiducial_df['time']
            fig.line(
                t_cam,
                fiducial_df['cam_ang_pos_0'],
                alpha=self.alpha,
                color=self.colors[0])
            fig.line(
                t_cam,
                fiducial_df['cam_ang_pos_1'],
                alpha=self.alpha,
                color=self.colors[1])
            fig.line(
                t_cam,
                fiducial_df['cam_ang_pos_2'],
                alpha=self.alpha,
                color=self.colors[2])
        return fig

    def plot_cam_pos_err(self):
        """Plot camera extrinsic position errors."""
        fig = figure(
            width=400,
            height=300,
            x_axis_label='Time [s]',
            y_axis_label='Position Error [mm]',
            title='Camera Extrinsic Position Error')
        for fiducial_df, body_truth in zip(self.fiducial_dfs, self.body_truth_dfs):
            true_t = body_truth['time']
            true_p0 = body_truth[f"cam_pos_{fiducial_df.attrs['id']}_0"]
            true_p1 = body_truth[f"cam_pos_{fiducial_df.attrs['id']}_1"]
            true_p2 = body_truth[f"cam_pos_{fiducial_df.attrs['id']}_2"]

            t_gps = fiducial_df['time']
            est_p0 = fiducial_df['cam_pos_0']
            est_p1 = fiducial_df['cam_pos_1']
            est_p2 = fiducial_df['cam_pos_2']

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

    def plot_cam_ang_err(self):
        """Plot the camera extrinsic angular error."""
        fig = figure(
            width=800,
            height=300,
            x_axis_label='Time [s]',
            y_axis_label='Angle Error [mrad]',
            title='Camera Extrinsic Angle Error')
        for fiducial_df, body_truth in zip(self.fiducial_dfs, self.body_truth_dfs):
            est_t = fiducial_df['time']
            est_w = fiducial_df['cam_ang_pos_0']
            est_x = fiducial_df['cam_ang_pos_1']
            est_y = fiducial_df['cam_ang_pos_2']
            est_z = fiducial_df['cam_ang_pos_3']
            true_t = body_truth['time']
            true_w = body_truth[f"cam_ang_pos_{fiducial_df.attrs['id']}_0"]
            true_x = body_truth[f"cam_ang_pos_{fiducial_df.attrs['id']}_1"]
            true_y = body_truth[f"cam_ang_pos_{fiducial_df.attrs['id']}_2"]
            true_z = body_truth[f"cam_ang_pos_{fiducial_df.attrs['id']}_3"]

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
            title='Camera Position Covariance')
        for fiducial_df in self.fiducial_dfs:
            t_cam = fiducial_df['time']
            cam_cov_0 = fiducial_df['cam_cov_0']
            cam_cov_1 = fiducial_df['cam_cov_1']
            cam_cov_2 = fiducial_df['cam_cov_2']
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
        for fiducial_df in self.fiducial_dfs:
            t_cam = fiducial_df['time']
            cam_cov_3 = fiducial_df['cam_cov_3']
            cam_cov_4 = fiducial_df['cam_cov_4']
            cam_cov_5 = fiducial_df['cam_cov_5']
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
        """Plot fiducial position in Camera Frame."""
        fig = figure(
            width=800,
            height=300,
            x_axis_label='Time [s]',
            y_axis_label='Position [m]',
            title='Fiducial Position in Camera Frame')

        err_px = collections.defaultdict(list)
        err_py = collections.defaultdict(list)
        err_pz = collections.defaultdict(list)

        for fiducial_df, true_df in zip(self.fiducial_dfs, self.board_truth_dfs):
            time = fiducial_df['time']
            board = fiducial_df['board']
            board_px = fiducial_df['board_pos_0']
            board_py = fiducial_df['board_pos_1']
            board_pz = fiducial_df['board_pos_2']

            true_px = true_df['pos_x']
            true_py = true_df['pos_y']
            true_pz = true_df['pos_z']

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
        """Plot fiducial angle in camera frame."""
        fig = figure(
            width=800,
            height=300,
            x_axis_label='Time [s]',
            y_axis_label='Angular',
            title='Fiducial Angle in Camera Frame')

        err_x = collections.defaultdict(list)
        err_y = collections.defaultdict(list)
        err_z = collections.defaultdict(list)

        for fiducial_df, true_df in zip(self.fiducial_dfs, self.board_truth_dfs):
            time = fiducial_df['time']
            board = fiducial_df['board']
            board_qw = fiducial_df['board_ang_0']
            board_qx = fiducial_df['board_ang_1']
            board_qy = fiducial_df['board_ang_2']
            board_qz = fiducial_df['board_ang_3']

            true_qw = true_df['quat_w']
            true_qx = true_df['quat_x']
            true_qy = true_df['quat_y']
            true_qz = true_df['quat_z']

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
            layout_plots.append([self.plot_cam_pos_err(), self.plot_cam_ang_err()])
            layout_plots.append([self.plot_cam_pos_cov(), self.plot_cam_ang_cov()])

        layout_plots.append([plot_update_timing(self.fiducial_dfs), Spacer()])

        tab_layout = layout(layout_plots, sizing_mode='stretch_width')
        tab = TabPanel(child=tab_layout, title=f"Fiducial {self.fiducial_dfs[0].attrs['id']}")

        return tab
