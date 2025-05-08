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
from utilities import calculate_alpha, calculate_rotation_errors, get_colors, interpolate_error, \
    interpolate_quat_error, lists_to_rot, plot_update_timing


class tab_body:

    def __init__(self, body_state_dfs, aug_state_dfs, body_truth_dfs, args):
        self.body_state_dfs = body_state_dfs
        self.aug_state_dfs = aug_state_dfs
        self.body_truth_dfs = body_truth_dfs
        self.alpha = calculate_alpha(len(self.body_state_dfs))
        self.colors = get_colors(args)

    def plot_body_pos(self):
        """Plot body position."""
        fig = figure(width=800, height=300, x_axis_label='Time [s]',
                     y_axis_label='Position [m]', title='Body Position')
        for body_df in self.body_state_dfs:
            time = body_df['time']
            pos_x = body_df['body_pos_0']
            pos_y = body_df['body_pos_1']
            pos_z = body_df['body_pos_2']
            fig.line(time, pos_x, alpha=self.alpha, color=self.colors[0], legend_label='X')
            fig.line(time, pos_y, alpha=self.alpha, color=self.colors[1], legend_label='Y')
            fig.line(time, pos_z, alpha=self.alpha, color=self.colors[2], legend_label='Z')
        return fig

    def plot_body_vel(self):
        """Plot body velocity."""
        fig = figure(width=800, height=300, x_axis_label='Time [s]',
                     y_axis_label='Velocity [m/s]', title='Body Velocity')
        for body_df in self.body_state_dfs:
            time = body_df['time']
            vel_x = body_df['body_vel_0']
            vel_y = body_df['body_vel_1']
            vel_z = body_df['body_vel_2']
            fig.line(time, vel_x, alpha=self.alpha, color=self.colors[0], legend_label='X')
            fig.line(time, vel_y, alpha=self.alpha, color=self.colors[1], legend_label='Y')
            fig.line(time, vel_z, alpha=self.alpha, color=self.colors[2], legend_label='Z')
        return fig

    def plot_body_acc(self):
        """Plot body acceleration."""
        fig = figure(width=800, height=300, x_axis_label='Time [s]',
                     y_axis_label='Acceleration [m/s/s]', title='Body Acceleration')
        for body_df in self.body_state_dfs:
            time = body_df['time']
            acc_x = body_df['body_acc_0']
            acc_y = body_df['body_acc_1']
            acc_z = body_df['body_acc_2']
            fig.line(time, acc_x, alpha=self.alpha, color=self.colors[0], legend_label='X')
            fig.line(time, acc_y, alpha=self.alpha, color=self.colors[1], legend_label='Y')
            fig.line(time, acc_z, alpha=self.alpha, color=self.colors[2], legend_label='Z')
        return fig

    def plot_body_ang(self):
        """Plot body angular position."""
        fig = figure(width=800, height=300, x_axis_label='Time [s]',
                     y_axis_label='Angle [rad]', title='Body Angle')
        for body_df in self.body_state_dfs:

            body_w = body_df['body_ang_pos_0']
            body_x = body_df['body_ang_pos_1']
            body_y = body_df['body_ang_pos_2']
            body_z = body_df['body_ang_pos_3']

            body_a = []
            body_b = []
            body_g = []

            # TODO(jhartzer): Use common euler function
            for (w, x, y, z) in zip(body_w, body_x, body_y, body_z):
                body_rot = Rotation.from_quat([w, x, y, z], scalar_first=True)
                body_eul = body_rot.as_euler('XYZ')
                body_a.append(body_eul[0])
                body_b.append(body_eul[1])
                body_g.append(body_eul[2])

            time = body_df['time']
            fig.line(time, body_a, alpha=self.alpha, color=self.colors[0], legend_label='X')
            fig.line(time, body_b, alpha=self.alpha, color=self.colors[1], legend_label='Y')
            fig.line(time, body_g, alpha=self.alpha, color=self.colors[2], legend_label='Z')
        return fig

    def plot_body_ang_vel(self):
        """Plot body angular velocity."""
        fig = figure(width=800, height=300, x_axis_label='Time [s]',
                     y_axis_label='Angular Velocity', title='Body Angular Velocity')
        for body_df in self.body_state_dfs:
            time = body_df['time']
            ang_vel_x = body_df['body_ang_vel_0']
            ang_vel_y = body_df['body_ang_vel_1']
            ang_vel_z = body_df['body_ang_vel_2']
            fig.line(time, ang_vel_x, alpha=self.alpha, color=self.colors[0], legend_label='X')
            fig.line(time, ang_vel_y, alpha=self.alpha, color=self.colors[1], legend_label='Y')
            fig.line(time, ang_vel_z, alpha=self.alpha, color=self.colors[2], legend_label='Z')
        return fig

    def plot_body_ang_acc(self):
        """Plot body angular acceleration."""
        fig = figure(width=800, height=300, x_axis_label='Time [s]',
                     y_axis_label='Angular Acceleration', title='Body Angular Acceleration')
        for body_df in self.body_state_dfs:
            time = body_df['time']
            ang_acc_x = body_df['body_ang_acc_0']
            ang_acc_y = body_df['body_ang_acc_1']
            ang_acc_z = body_df['body_ang_acc_2']
            fig.line(time, ang_acc_x, alpha=self.alpha, color=self.colors[0], legend_label='X')
            fig.line(time, ang_acc_y, alpha=self.alpha, color=self.colors[1], legend_label='Y')
            fig.line(time, ang_acc_z, alpha=self.alpha, color=self.colors[2], legend_label='Z')
        return fig

    def plot_body_err_pos(self):
        """Plot the body state position error."""
        fig = figure(width=800, height=300, x_axis_label='Time [s]',
                     y_axis_label='Position Error [m]', title='Body Position Error')
        for body_state, body_truth in zip(self.body_state_dfs, self.body_truth_dfs):
            true_time = body_truth['time']
            true_pos_0 = body_truth['body_pos_0']
            true_pos_1 = body_truth['body_pos_1']
            true_pos_2 = body_truth['body_pos_2']

            time = body_state['time']
            est_pos_0 = body_state['body_pos_0']
            est_pos_1 = body_state['body_pos_1']
            est_pos_2 = body_state['body_pos_2']

            err_pos_0 = interpolate_error(true_time, true_pos_0, time, est_pos_0)
            err_pos_1 = interpolate_error(true_time, true_pos_1, time, est_pos_1)
            err_pos_2 = interpolate_error(true_time, true_pos_2, time, est_pos_2)

            fig.line(time, err_pos_0, alpha=self.alpha, color=self.colors[0], legend_label='X')
            fig.line(time, err_pos_1, alpha=self.alpha, color=self.colors[1], legend_label='Y')
            fig.line(time, err_pos_2, alpha=self.alpha, color=self.colors[2], legend_label='Z')

        return fig

    def plot_body_err_vel(self):
        """Plot the body state velocity error."""
        fig = figure(width=800, height=300, x_axis_label='Time [s]',
                     y_axis_label='Velocity Error [m/s]', title='Body Velocity Error')
        for body_state, body_truth in zip(self.body_state_dfs, self.body_truth_dfs):
            true_time = body_truth['time']
            true_vel_0 = body_truth['body_vel_0']
            true_vel_1 = body_truth['body_vel_1']
            true_vel_2 = body_truth['body_vel_2']

            time = body_state['time']
            est_vel_0 = body_state['body_vel_0']
            est_vel_1 = body_state['body_vel_1']
            est_vel_2 = body_state['body_vel_2']

            err_vel_0 = interpolate_error(true_time, true_vel_0, time, est_vel_0)
            err_vel_1 = interpolate_error(true_time, true_vel_1, time, est_vel_1)
            err_vel_2 = interpolate_error(true_time, true_vel_2, time, est_vel_2)

            fig.line(time, err_vel_0, alpha=self.alpha, color=self.colors[0], legend_label='X')
            fig.line(time, err_vel_1, alpha=self.alpha, color=self.colors[1], legend_label='Y')
            fig.line(time, err_vel_2, alpha=self.alpha, color=self.colors[2], legend_label='Z')

        return fig

    def plot_body_err_acc(self):
        """Plot the body state acceleration error."""
        fig = figure(width=800, height=300, x_axis_label='Time [s]',
                     y_axis_label='Acceleration Error [m/s/s]', title='Body Acceleration Error')
        for body_state, body_truth in zip(self.body_state_dfs, self.body_truth_dfs):
            true_time = body_truth['time']
            true_acc_0 = body_truth['body_acc_0']
            true_acc_1 = body_truth['body_acc_1']
            true_acc_2 = body_truth['body_acc_2']

            time = body_state['time']
            est_acc_0 = body_state['body_acc_0']
            est_acc_1 = body_state['body_acc_1']
            est_acc_2 = body_state['body_acc_2']

            err_acc_0 = interpolate_error(true_time, true_acc_0, time, est_acc_0)
            err_acc_1 = interpolate_error(true_time, true_acc_1, time, est_acc_1)
            err_acc_2 = interpolate_error(true_time, true_acc_2, time, est_acc_2)

            fig.line(time, err_acc_0, alpha=self.alpha, color=self.colors[0], legend_label='X')
            fig.line(time, err_acc_1, alpha=self.alpha, color=self.colors[1], legend_label='Y')
            fig.line(time, err_acc_2, alpha=self.alpha, color=self.colors[2], legend_label='Z')
        return fig

    def plot_body_err_ang(self):
        """Plot the body state angular error."""
        fig = figure(width=800, height=300, x_axis_label='Time [s]',
                     y_axis_label='Angular Error', title='Body Angular Error')
        for body_state, body_truth in zip(self.body_state_dfs, self.body_truth_dfs):
            true_time = body_truth['time']
            true_ang_pos_w = body_truth['body_ang_pos_0']
            true_ang_pos_x = body_truth['body_ang_pos_1']
            true_ang_pos_y = body_truth['body_ang_pos_2']
            true_ang_pos_z = body_truth['body_ang_pos_3']

            time = body_state['time']
            est_ang_pos_w = body_state['body_ang_pos_0']
            est_ang_pos_x = body_state['body_ang_pos_1']
            est_ang_pos_y = body_state['body_ang_pos_2']
            est_ang_pos_z = body_state['body_ang_pos_3']
            est_ang_pos_r = lists_to_rot(
                est_ang_pos_w,
                est_ang_pos_x,
                est_ang_pos_y,
                est_ang_pos_z)

            interp_w = np.interp(time, true_time, true_ang_pos_w)
            interp_x = np.interp(time, true_time, true_ang_pos_x)
            interp_y = np.interp(time, true_time, true_ang_pos_y)
            interp_z = np.interp(time, true_time, true_ang_pos_z)
            interp_r = lists_to_rot(interp_w, interp_x, interp_y, interp_z)

            pos_err_x, pos_err_y, pos_err_z = \
                calculate_rotation_errors(est_ang_pos_r, interp_r)

            fig.line(time, pos_err_x, alpha=self.alpha, color=self.colors[0], legend_label='X')
            fig.line(time, pos_err_y, alpha=self.alpha, color=self.colors[1], legend_label='Y')
            fig.line(time, pos_err_z, alpha=self.alpha, color=self.colors[2], legend_label='Z')
        return fig

    def plot_body_err_ang_vel(self):
        """Plot the body state angular velocity error."""
        fig = figure(width=800, height=300, x_axis_label='Time [s]',
                     y_axis_label='Angular Velocity Error [rad/s]',
                     title='Body Angular Velocity Error')
        for body_state, body_truth in zip(self.body_state_dfs, self.body_truth_dfs):
            true_time = body_truth['time']
            true_ang_vel_0 = body_truth['body_ang_vel_0']
            true_ang_vel_1 = body_truth['body_ang_vel_1']
            true_ang_vel_2 = body_truth['body_ang_vel_2']

            time = body_state['time']
            est_ang_vel_0 = body_state['body_ang_vel_0']
            est_ang_vel_1 = body_state['body_ang_vel_1']
            est_ang_vel_2 = body_state['body_ang_vel_2']

            vel_err_0 = interpolate_error(true_time, true_ang_vel_0, time, est_ang_vel_0)
            vel_err_1 = interpolate_error(true_time, true_ang_vel_1, time, est_ang_vel_1)
            vel_err_2 = interpolate_error(true_time, true_ang_vel_2, time, est_ang_vel_2)

            fig.line(time, vel_err_0, alpha=self.alpha, color=self.colors[0], legend_label='X')
            fig.line(time, vel_err_1, alpha=self.alpha, color=self.colors[1], legend_label='Y')
            fig.line(time, vel_err_2, alpha=self.alpha, color=self.colors[2], legend_label='Z')
        return fig

    def plot_body_err_ang_acc(self):
        """Plot the body state angular acceleration error."""
        fig = figure(width=800, height=300, x_axis_label='Time [s]',
                     y_axis_label='Angular Acceleration Error [rad/s/s]',
                     title='Body Angular Acceleration Error')
        for body_state, body_truth in zip(self.body_state_dfs, self.body_truth_dfs):
            true_time = body_truth['time']
            true_ang_acc_0 = body_truth['body_ang_acc_0']
            true_ang_acc_1 = body_truth['body_ang_acc_1']
            true_ang_acc_2 = body_truth['body_ang_acc_2']

            time = body_state['time']
            est_ang_acc_0 = body_state['body_ang_acc_0']
            est_ang_acc_1 = body_state['body_ang_acc_1']
            est_ang_acc_2 = body_state['body_ang_acc_2']

            acc_err_0 = interpolate_error(true_time, true_ang_acc_0, time, est_ang_acc_0)
            acc_err_1 = interpolate_error(true_time, true_ang_acc_1, time, est_ang_acc_1)
            acc_err_2 = interpolate_error(true_time, true_ang_acc_2, time, est_ang_acc_2)

            fig.line(time, acc_err_0, alpha=self.alpha, color=self.colors[0], legend_label='X')
            fig.line(time, acc_err_1, alpha=self.alpha, color=self.colors[1], legend_label='Y')
            fig.line(time, acc_err_2, alpha=self.alpha, color=self.colors[2], legend_label='Z')
        return fig

    def plot_body_pos_cov(self):
        """Plot body covariances for body position."""
        fig = figure(width=800, height=300, x_axis_label='Time [s]',
                     y_axis_label='Position [m]', title='Body Position Covariance')
        for body_df in self.body_state_dfs:
            time = body_df['time']
            cov_x = body_df['body_cov_0']
            cov_y = body_df['body_cov_1']
            cov_z = body_df['body_cov_2']
            fig.line(time, cov_x, alpha=self.alpha, color=self.colors[0], legend_label='X')
            fig.line(time, cov_y, alpha=self.alpha, color=self.colors[1], legend_label='Y')
            fig.line(time, cov_z, alpha=self.alpha, color=self.colors[2], legend_label='Z')
        return fig

    def plot_body_vel_cov(self):
        """Plot body covariances for body velocity."""
        fig = figure(width=800, height=300, x_axis_label='Time [s]',
                     y_axis_label='Velocity [m/s]', title='Body Velocity Covariance')
        for body_df in self.body_state_dfs:
            time = body_df['time']
            cov_x = body_df['body_cov_3']
            cov_y = body_df['body_cov_4']
            cov_z = body_df['body_cov_5']
            fig.line(time, cov_x, alpha=self.alpha, color=self.colors[0], legend_label='X')
            fig.line(time, cov_y, alpha=self.alpha, color=self.colors[1], legend_label='Y')
            fig.line(time, cov_z, alpha=self.alpha, color=self.colors[2], legend_label='Z')
        return fig

    def plot_body_acc_cov(self):
        """Plot body covariances for body acceleration."""
        fig = figure(width=800, height=300, x_axis_label='Time [s]',
                     y_axis_label='Acceleration [m/s/s]', title='Body Acceleration Covariance')
        for body_df in self.body_state_dfs:
            time = body_df['time']
            cov_x = body_df['body_cov_6']
            cov_y = body_df['body_cov_7']
            cov_z = body_df['body_cov_8']
            fig.line(time, cov_x, alpha=self.alpha, color=self.colors[0], legend_label='X')
            fig.line(time, cov_y, alpha=self.alpha, color=self.colors[1], legend_label='Y')
            fig.line(time, cov_z, alpha=self.alpha, color=self.colors[2], legend_label='Z')
        return fig

    def plot_body_ang_cov(self):
        """Plot body covariances for body angles."""
        fig = figure(width=800, height=300, x_axis_label='Time [s]',
                     y_axis_label='Angle [rad]', title='Body Angular Covariance')
        for body_df in self.body_state_dfs:
            time = body_df['time']
            cov_x = body_df['body_cov_9']
            cov_y = body_df['body_cov_10']
            cov_z = body_df['body_cov_11']
            fig.line(time, cov_x, alpha=self.alpha, color=self.colors[0], legend_label='X')
            fig.line(time, cov_y, alpha=self.alpha, color=self.colors[1], legend_label='Y')
            fig.line(time, cov_z, alpha=self.alpha, color=self.colors[2], legend_label='Z')
        return fig

    def plot_body_ang_vel_cov(self):
        """Plot body covariances for body angular rate."""
        fig = figure(width=800, height=300, x_axis_label='Time [s]',
                     y_axis_label='Angular Rate [rad/s]', title='Body Angular Rate Covariance')
        for body_df in self.body_state_dfs:
            time = body_df['time']
            cov_x = body_df['body_cov_12']
            cov_y = body_df['body_cov_13']
            cov_z = body_df['body_cov_14']
            fig.line(time, cov_x, alpha=self.alpha, color=self.colors[0], legend_label='X')
            fig.line(time, cov_y, alpha=self.alpha, color=self.colors[1], legend_label='Y')
            fig.line(time, cov_z, alpha=self.alpha, color=self.colors[2], legend_label='Z')
        return fig

    def plot_body_ang_acc_cov(self):
        """Plot body covariances for body angular acceleration."""
        fig = figure(width=800, height=300, x_axis_label='Time [s]',
                     y_axis_label='Angular Acceleration [rad/s/s]',
                     title='Body Angular Acceleration Covariance')
        for body_df in self.body_state_dfs:
            time = body_df['time']
            cov_x = body_df['body_cov_15']
            cov_y = body_df['body_cov_16']
            cov_z = body_df['body_cov_17']
            fig.line(time, cov_x, alpha=self.alpha, color=self.colors[0], legend_label='X')
            fig.line(time, cov_y, alpha=self.alpha, color=self.colors[1], legend_label='Y')
            fig.line(time, cov_z, alpha=self.alpha, color=self.colors[2], legend_label='Z')
        return fig

    def plot_body_nees(self):
        """Plot body normalized estimation error squared."""
        fig = figure(width=800, height=300, x_axis_label='Time [s]',
                     y_axis_label='NEES', title='Normalized Estimation Error Squared')
        for body_state, body_truth in zip(self.body_state_dfs, self.body_truth_dfs):
            xt  = body_state['time']
            x00 = body_state['body_pos_0']
            x01 = body_state['body_pos_1']
            x02 = body_state['body_pos_2']
            x03 = body_state['body_vel_0']
            x04 = body_state['body_vel_1']
            x05 = body_state['body_vel_2']
            x06 = body_state['body_acc_0']
            x07 = body_state['body_acc_1']
            x08 = body_state['body_acc_2']
            xw  = body_state['body_ang_pos_0']
            xx  = body_state['body_ang_pos_1']
            xy  = body_state['body_ang_pos_2']
            xz  = body_state['body_ang_pos_3']
            x12 = body_state['body_ang_vel_0']
            x13 = body_state['body_ang_vel_1']
            x14 = body_state['body_ang_vel_2']
            x15 = body_state['body_ang_acc_0']
            x16 = body_state['body_ang_acc_1']
            x17 = body_state['body_ang_acc_2']

            c00 = body_state['body_cov_0']
            c01 = body_state['body_cov_1']
            c02 = body_state['body_cov_2']
            c03 = body_state['body_cov_3']
            c04 = body_state['body_cov_4']
            c05 = body_state['body_cov_5']
            c06 = body_state['body_cov_6']
            c07 = body_state['body_cov_7']
            c08 = body_state['body_cov_8']
            c09 = body_state['body_cov_9']
            c10 = body_state['body_cov_10']
            c11 = body_state['body_cov_11']
            c12 = body_state['body_cov_12']
            c13 = body_state['body_cov_13']
            c14 = body_state['body_cov_14']
            c15 = body_state['body_cov_15']
            c16 = body_state['body_cov_16']
            c17 = body_state['body_cov_17']

            tt  = body_truth['time']
            t00 = body_truth['body_pos_0']
            t01 = body_truth['body_pos_1']
            t02 = body_truth['body_pos_2']
            t03 = body_truth['body_vel_0']
            t04 = body_truth['body_vel_1']
            t05 = body_truth['body_vel_2']
            t06 = body_truth['body_acc_0']
            t07 = body_truth['body_acc_1']
            t08 = body_truth['body_acc_2']
            tw  = body_truth['body_ang_pos_0']
            tx  = body_truth['body_ang_pos_1']
            ty  = body_truth['body_ang_pos_2']
            tz  = body_truth['body_ang_pos_3']
            t12 = body_truth['body_ang_vel_0']
            t13 = body_truth['body_ang_vel_1']
            t14 = body_truth['body_ang_vel_2']
            t15 = body_truth['body_ang_acc_0']
            t16 = body_truth['body_ang_acc_1']
            t17 = body_truth['body_ang_acc_2']

            e00 = interpolate_error(tt, t00, xt, x00)
            e01 = interpolate_error(tt, t01, xt, x01)
            e02 = interpolate_error(tt, t02, xt, x02)
            e03 = interpolate_error(tt, t03, xt, x03)
            e04 = interpolate_error(tt, t04, xt, x04)
            e05 = interpolate_error(tt, t05, xt, x05)
            e06 = interpolate_error(tt, t06, xt, x06)
            e07 = interpolate_error(tt, t07, xt, x07)
            e08 = interpolate_error(tt, t08, xt, x08)
            e09, e10, e11  = interpolate_quat_error(tt, tw, tx, ty, tz, xt, xw, xx, xy, xz)
            e12 = interpolate_error(tt, t12, xt, x12)
            e13 = interpolate_error(tt, t13, xt, x13)
            e14 = interpolate_error(tt, t14, xt, x14)
            e15 = interpolate_error(tt, t15, xt, x15)
            e16 = interpolate_error(tt, t16, xt, x16)
            e17 = interpolate_error(tt, t17, xt, x17)

            nees = \
                e00 * e00 / c00 / c00 + \
                e01 * e01 / c01 / c01 + \
                e02 * e02 / c02 / c02 + \
                e03 * e03 / c03 / c03 + \
                e04 * e04 / c04 / c04 + \
                e05 * e05 / c05 / c05 + \
                e06 * e06 / c06 / c06 + \
                e07 * e07 / c07 / c07 + \
                e08 * e08 / c08 / c08 + \
                e09 * e09 / c09 / c09 + \
                e10 * e10 / c10 / c10 + \
                e11 * e11 / c11 / c11 + \
                e12 * e12 / c12 / c12 + \
                e13 * e13 / c13 / c13 + \
                e14 * e14 / c14 / c14 + \
                e15 * e15 / c15 / c15 + \
                e16 * e16 / c16 / c16 + \
                e17 * e17 / c17 / c17

            fig.line(xt, nees, alpha=self.alpha, color=self.colors[0])

        fig.hspan(y=chi2.ppf(0.025, df=18), line_color='red')
        fig.hspan(y=chi2.ppf(0.975, df=18), line_color='red')
        fig.y_range = Range1d(0, 40)

        return fig

    def plot_aug_pos(self):
        """Plot augmented state position."""
        fig = figure(width=800, height=300, x_axis_label='Time [s]',
                     y_axis_label='Augmented Position [m]',
                     title='Augmented State Position')
        for aug_df in self.aug_state_dfs:
            time = aug_df['time']
            fig.scatter(
                time,
                aug_df['aug_pos_0'],
                alpha=self.alpha,
                color=self.colors[0],
                legend_label='X',
                size=1)
            fig.scatter(
                time,
                aug_df['aug_pos_1'],
                alpha=self.alpha,
                color=self.colors[1],
                legend_label='Y',
                size=1)
            fig.scatter(
                time,
                aug_df['aug_pos_2'],
                alpha=self.alpha,
                color=self.colors[2],
                legend_label='Z',
                size=1)

        return fig

    def plot_aug_ang(self):
        """Plot augmented state orientation."""
        fig = figure(width=800, height=300, x_axis_label='Time [s]',
                     y_axis_label='Augmented Orientation [rad]',
                     title='Augmented State Orientation')

        for aug_df in self.aug_state_dfs:
            aug_w = aug_df['aug_ang_0']
            aug_x = aug_df['aug_ang_1']
            aug_y = aug_df['aug_ang_2']
            aug_z = aug_df['aug_ang_3']

            aug_a = []
            aug_b = []
            aug_g = []

            # TODO(jhartzer): Use common euler function
            for (w, x, y, z) in zip(aug_w, aug_x, aug_y, aug_z):
                aug_rot = Rotation.from_quat([w, x, y, z], scalar_first=True)
                aug_eul = aug_rot.as_euler('XYZ')
                aug_a.append(aug_eul[0])
                aug_b.append(aug_eul[1])
                aug_g.append(aug_eul[2])

            time = aug_df['time']
            fig.scatter(
                time,
                aug_a,
                alpha=self.alpha,
                color=self.colors[0],
                legend_label='X',
                size=1)
            fig.scatter(
                time,
                aug_b,
                alpha=self.alpha,
                color=self.colors[1],
                legend_label='Y',
                size=1)
            fig.scatter(
                time,
                aug_g,
                alpha=self.alpha,
                color=self.colors[2],
                legend_label='Z',
                size=1)

        return fig

    def plot_state_size(self):
        """Plot total state size."""
        fig = figure(width=800, height=300, x_axis_label='Time [s]',
                     y_axis_label='State Size',
                     title='State Size')
        for body_df in self.body_state_dfs:

            fig.line(
                body_df['time'],
                body_df['state_size'],
                alpha=self.alpha,
                color=self.colors[0])

        return fig

    def get_tab(self):
        layout_plots = [
            [
                self.plot_body_pos(),
                self.plot_body_ang()
            ],
            [
                self.plot_body_err_pos(),
                self.plot_body_err_ang()
            ],
            [
                self.plot_body_pos_cov(),
                self.plot_body_ang_cov()
            ],
            [
                self.plot_body_vel(),
                self.plot_body_ang_vel()
            ],
            [
                self.plot_body_err_vel(),
                self.plot_body_err_ang_vel()
            ],
            [
                self.plot_body_vel_cov(),
                self.plot_body_ang_vel_cov()
            ],
            [
                self.plot_body_acc(),
                self.plot_body_ang_acc()
            ],
            [
                self.plot_body_err_acc(),
                self.plot_body_err_ang_acc()
            ],
            [
                self.plot_body_acc_cov(),
                self.plot_body_ang_acc_cov()
            ]
        ]

        if self.aug_state_dfs:
            layout_plots.append([self.plot_aug_pos(), self.plot_aug_ang()])

        layout_plots.append([plot_update_timing(self.body_state_dfs), self.plot_state_size()])
        layout_plots.append([self.plot_body_nees(), Spacer()])

        tab_layout = layout(layout_plots, sizing_mode='stretch_width')
        tab = TabPanel(child=tab_layout, title='Body')

        return tab
