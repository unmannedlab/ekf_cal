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
from bokeh.models import Range1d, Spacer, TabPanel
from bokeh.plotting import figure
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.stats.distributions import chi2
from utilities import calculate_alpha, get_colors, interpolate_error, interpolate_quat_error, \
    plot_update_timing


class tab_fiducial:
    """Class for plotting fiducial data."""

    def __init__(self, fiducial_dfs, board_truth_dfs, body_truth_dfs, args):
        """Initializes the tab_fiducial class for plotting fiducial information."""
        self.fiducial_dfs = fiducial_dfs
        self.board_truth_dfs = board_truth_dfs
        self.body_truth_dfs = body_truth_dfs
        self.is_fid_extrinsic = 'fid_cov_0' in self.fiducial_dfs[0].keys()
        self.is_cam_extrinsic = 'cam_cov_0' in self.fiducial_dfs[0].keys()

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
            time = fiducial_df['time']
            pos_x = fiducial_df['cam_pos_0']
            pos_y = fiducial_df['cam_pos_1']
            pos_z = fiducial_df['cam_pos_2']
            fig.line(time, pos_x, alpha=self.alpha, color=self.colors[0], legend_label='X')
            fig.line(time, pos_y, alpha=self.alpha, color=self.colors[1], legend_label='Y')
            fig.line(time, pos_z, alpha=self.alpha, color=self.colors[2], legend_label='Z')
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
            time = fiducial_df['time']
            ang_x = fiducial_df['cam_ang_pos_0']
            ang_y = fiducial_df['cam_ang_pos_1']
            ang_z = fiducial_df['cam_ang_pos_2']
            fig.line(time, ang_x, alpha=self.alpha, color=self.colors[0], legend_label='X')
            fig.line(time, ang_y, alpha=self.alpha, color=self.colors[1], legend_label='Y')
            fig.line(time, ang_z, alpha=self.alpha, color=self.colors[2], legend_label='Z')
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

            time = fiducial_df['time']
            est_p0 = fiducial_df['cam_pos_0']
            est_p1 = fiducial_df['cam_pos_1']
            est_p2 = fiducial_df['cam_pos_2']

            err_pos_0 = np.array(interpolate_error(true_t, true_p0, time, est_p0)) * 1e3
            err_pos_1 = np.array(interpolate_error(true_t, true_p1, time, est_p1)) * 1e3
            err_pos_2 = np.array(interpolate_error(true_t, true_p2, time, est_p2)) * 1e3

            fig.line(time, err_pos_0, alpha=self.alpha, color=self.colors[0], legend_label='X')
            fig.line(time, err_pos_1, alpha=self.alpha, color=self.colors[1], legend_label='Y')
            fig.line(time, err_pos_2, alpha=self.alpha, color=self.colors[2], legend_label='Z')
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
            time = fiducial_df['time']
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
                time, est_w, est_x, est_y, est_z)

            fig.line(time, eul_err_x, alpha=self.alpha, color=self.colors[0], legend_label='X')
            fig.line(time, eul_err_y, alpha=self.alpha, color=self.colors[1], legend_label='Y')
            fig.line(time, eul_err_z, alpha=self.alpha, color=self.colors[2], legend_label='Z')
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
            time = fiducial_df['time']
            cam_cov_0 = fiducial_df['cam_cov_0']
            cam_cov_1 = fiducial_df['cam_cov_1']
            cam_cov_2 = fiducial_df['cam_cov_2']
            fig.line(time, cam_cov_0, alpha=self.alpha, color=self.colors[0], legend_label='X')
            fig.line(time, cam_cov_1, alpha=self.alpha, color=self.colors[1], legend_label='Y')
            fig.line(time, cam_cov_2, alpha=self.alpha, color=self.colors[2], legend_label='Z')
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
            time = fiducial_df['time']
            cam_cov_3 = fiducial_df['cam_cov_3']
            cam_cov_4 = fiducial_df['cam_cov_4']
            cam_cov_5 = fiducial_df['cam_cov_5']
            fig.line(time, cam_cov_3, alpha=self.alpha, color=self.colors[0], legend_label='X')
            fig.line(time, cam_cov_4, alpha=self.alpha, color=self.colors[1], legend_label='Y')
            fig.line(time, cam_cov_5, alpha=self.alpha, color=self.colors[2], legend_label='Z')
        return fig

    def plot_fiducial_error_pos(self):
        """Plot fiducial position in Camera Frame."""
        fig = figure(
            width=800,
            height=300,
            x_axis_label='Time [s]',
            y_axis_label='Position [m]',
            title='Fiducial Position in Camera Frame')

        for fiducial_df in self.fiducial_dfs:
            time = fiducial_df['time']
            board_px = fiducial_df['board_meas_pos_0']
            board_py = fiducial_df['board_meas_pos_1']
            board_pz = fiducial_df['board_meas_pos_2']
            fig.line(time, board_px, alpha=self.alpha, color=self.colors[0], legend_label='X')
            fig.line(time, board_py, alpha=self.alpha, color=self.colors[1], legend_label='Y')
            fig.line(time, board_pz, alpha=self.alpha, color=self.colors[2], legend_label='Z')

        return fig

    def plot_fiducial_error_ang(self):
        """Plot fiducial angle in camera frame."""
        fig = figure(
            width=800,
            height=300,
            x_axis_label='Time [s]',
            y_axis_label='Angular',
            title='Fiducial Angle in Camera Frame')

        for fiducial_df in self.fiducial_dfs:
            time = fiducial_df['time']
            board_qw = fiducial_df['board_meas_ang_0']
            board_qx = fiducial_df['board_meas_ang_1']
            board_qy = fiducial_df['board_meas_ang_2']
            board_qz = fiducial_df['board_meas_ang_3']

            board_a = []
            board_b = []
            board_g = []

            # TODO(jhartzer): Use common euler function
            for (w, x, y, z) in zip(board_qw, board_qx, board_qy, board_qz):
                board_rot = Rotation.from_quat([w, x, y, z], scalar_first=True)
                board_eul = board_rot.as_euler('XYZ')
                board_a.append(board_eul[0])
                board_b.append(board_eul[1])
                board_g.append(board_eul[2])

            time = fiducial_df['time']
            fig.line(time, board_a, alpha=self.alpha, color=self.colors[0], legend_label='X')
            fig.line(time, board_b, alpha=self.alpha, color=self.colors[1], legend_label='Y')
            fig.line(time, board_g, alpha=self.alpha, color=self.colors[2], legend_label='Z')
        return fig

    def plot_fid_nees(self):
        """Plot fiducial normalized estimation error squared."""
        fig = figure(width=800, height=300, x_axis_label='Time [s]',
                     y_axis_label='NEES', title='Normalized Estimation Error Squared')
        for fiducial_df, board_truth in zip(self.fiducial_dfs, self.board_truth_dfs):
            t00 = np.array(board_truth['pos_x'])[0]
            t01 = np.array(board_truth['pos_y'])[0]
            t02 = np.array(board_truth['pos_z'])[0]
            tw = np.array(board_truth['quat_w'])[0]
            tx = np.array(board_truth['quat_x'])[0]
            ty = np.array(board_truth['quat_y'])[0]
            tz = np.array(board_truth['quat_z'])[0]

            xt = fiducial_df['time']
            e00 = fiducial_df['fid_pos_0'] - t00
            e01 = fiducial_df['fid_pos_1'] - t01
            e02 = fiducial_df['fid_pos_2'] - t02

            e03 = []
            e04 = []
            e05 = []
            for i in range(len(fiducial_df['fid_ang_0'])):
                ew = fiducial_df['fid_ang_0'][i]
                ex = fiducial_df['fid_ang_1'][i]
                ey = fiducial_df['fid_ang_2'][i]
                ez = fiducial_df['fid_ang_3'][i]
                qt = Rotation.from_quat([tw, tx, ty, tz], scalar_first=True)
                qe = Rotation.from_quat([ew, ex, ey, ez], scalar_first=True)
                q_err = qt * qe.inv()
                error_euler = q_err.as_euler('XYZ')
                e03.append(error_euler[0] * 1e3)
                e04.append(error_euler[1] * 1e3)
                e05.append(error_euler[2] * 1e3)
            e03 = np.array(e03)
            e04 = np.array(e04)
            e05 = np.array(e05)

            c00 = fiducial_df['fid_cov_0']
            c01 = fiducial_df['fid_cov_1']
            c02 = fiducial_df['fid_cov_2']
            c03 = fiducial_df['fid_cov_3']
            c04 = fiducial_df['fid_cov_4']
            c05 = fiducial_df['fid_cov_5']

            nees = \
                e00 * e00 / c00 / c00 + \
                e01 * e01 / c01 / c01 + \
                e02 * e02 / c02 / c02 + \
                e03 * e03 / c03 / c03 + \
                e04 * e04 / c04 / c04 + \
                e05 * e05 / c05 / c05

            fig.line(xt, nees, alpha=self.alpha, color=self.colors[0])

        fig.hspan(y=chi2.ppf(0.025, df=6), line_color='red')
        fig.hspan(y=chi2.ppf(0.975, df=6), line_color='red')
        fig.y_range = Range1d(0, 40)

        return fig

    def get_tab(self):
        """Generates the Bokeh TabPanel containing all fiducial plots."""
        layout_plots = [[self.plot_fiducial_error_pos(), self.plot_fiducial_error_ang()]]

        if self.is_cam_extrinsic:
            layout_plots.append([self.plot_camera_pos(), self.plot_camera_ang()])
            layout_plots.append([self.plot_cam_pos_err(), self.plot_cam_ang_err()])
            layout_plots.append([self.plot_cam_pos_cov(), self.plot_cam_ang_cov()])

        layout_plots.append([plot_update_timing(self.fiducial_dfs), Spacer()])

        if self.is_fid_extrinsic:
            layout_plots.append([self.plot_fid_nees(), Spacer()])

        tab_layout = layout(layout_plots, sizing_mode='stretch_width')
        tab = TabPanel(child=tab_layout, title=f"Fiducial {self.fiducial_dfs[0].attrs['id']}")

        return tab
