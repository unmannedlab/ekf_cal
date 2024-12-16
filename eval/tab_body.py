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
from scipy.spatial.transform import Rotation
from utilities import calculate_alpha, calculate_rotation_errors, get_colors, interpolate_error, \
    lists_to_rot, plot_update_timing


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
            fig.line(
                time,
                body_df['body_pos_0'],
                alpha=self.alpha,
                color=self.colors[0],
                legend_label='X')
            fig.line(
                time,
                body_df['body_pos_1'],
                alpha=self.alpha,
                color=self.colors[1],
                legend_label='Y')
            fig.line(
                time,
                body_df['body_pos_2'],
                alpha=self.alpha,
                color=self.colors[2],
                legend_label='Z')
        return fig

    def plot_body_vel(self):
        """Plot body velocity."""
        fig = figure(width=800, height=300, x_axis_label='Time [s]',
                     y_axis_label='Velocity [m/s]', title='Body Velocity')
        for body_df in self.body_state_dfs:
            time = body_df['time']
            fig.line(
                time,
                body_df['body_vel_0'],
                alpha=self.alpha,
                color=self.colors[0],
                legend_label='X')
            fig.line(
                time,
                body_df['body_vel_1'],
                alpha=self.alpha,
                color=self.colors[1],
                legend_label='Y')
            fig.line(
                time,
                body_df['body_vel_2'],
                alpha=self.alpha,
                color=self.colors[2],
                legend_label='Z')
        return fig

    def plot_body_acc(self):
        """Plot body acceleration."""
        fig = figure(width=800, height=300, x_axis_label='Time [s]',
                     y_axis_label='Acceleration [m/s/s]', title='Body Acceleration')
        for body_df in self.body_state_dfs:
            time = body_df['time']
            fig.line(
                time,
                body_df['body_acc_0'],
                alpha=self.alpha,
                color=self.colors[0],
                legend_label='X')
            fig.line(
                time,
                body_df['body_acc_1'],
                alpha=self.alpha,
                color=self.colors[1],
                legend_label='Y')
            fig.line(
                time,
                body_df['body_acc_2'],
                alpha=self.alpha,
                color=self.colors[2],
                legend_label='Z')
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
            fig.line(
                time,
                body_a,
                alpha=self.alpha,
                color=self.colors[0],
                legend_label='X')
            fig.line(
                time,
                body_b,
                alpha=self.alpha,
                color=self.colors[1],
                legend_label='Y')
            fig.line(
                time,
                body_g,
                alpha=self.alpha,
                color=self.colors[2],
                legend_label='Z')
        return fig

    def plot_body_ang_vel(self):
        """Plot body angular velocity."""
        fig = figure(width=800, height=300, x_axis_label='Time [s]',
                     y_axis_label='Angular Velocity', title='Body Angular Velocity')
        for body_df in self.body_state_dfs:
            time = body_df['time']
            fig.line(
                time,
                body_df['body_ang_vel_0'],
                alpha=self.alpha,
                color=self.colors[0],
                legend_label='X')
            fig.line(
                time,
                body_df['body_ang_vel_1'],
                alpha=self.alpha,
                color=self.colors[1],
                legend_label='Y')
            fig.line(
                time,
                body_df['body_ang_vel_2'],
                alpha=self.alpha,
                color=self.colors[2],
                legend_label='Z')
        return fig

    def plot_body_ang_acc(self):
        """Plot body angular acceleration."""
        fig = figure(width=800, height=300, x_axis_label='Time [s]',
                     y_axis_label='Angular Acceleration', title='Body Angular Acceleration')
        for body_df in self.body_state_dfs:
            time = body_df['time']
            fig.line(
                time,
                body_df['body_ang_acc_0'],
                alpha=self.alpha,
                color=self.colors[0],
                legend_label='X')
            fig.line(
                time,
                body_df['body_ang_acc_1'],
                alpha=self.alpha,
                color=self.colors[1],
                legend_label='Y')
            fig.line(
                time,
                body_df['body_ang_acc_2'],
                alpha=self.alpha,
                color=self.colors[2],
                legend_label='Z')
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

            est_time = body_state['time']
            est_pos_0 = body_state['body_pos_0']
            est_pos_1 = body_state['body_pos_1']
            est_pos_2 = body_state['body_pos_2']

            err_pos_0 = interpolate_error(true_time, true_pos_0, est_time, est_pos_0)
            err_pos_1 = interpolate_error(true_time, true_pos_1, est_time, est_pos_1)
            err_pos_2 = interpolate_error(true_time, true_pos_2, est_time, est_pos_2)

            fig.line(
                est_time,
                err_pos_0,
                alpha=self.alpha,
                color=self.colors[0],
                legend_label='X')
            fig.line(
                est_time,
                err_pos_1,
                alpha=self.alpha,
                color=self.colors[1],
                legend_label='Y')
            fig.line(
                est_time,
                err_pos_2,
                alpha=self.alpha,
                color=self.colors[2],
                legend_label='Z')

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

            est_time = body_state['time']
            est_vel_0 = body_state['body_vel_0']
            est_vel_1 = body_state['body_vel_1']
            est_vel_2 = body_state['body_vel_2']

            err_vel_0 = interpolate_error(true_time, true_vel_0, est_time, est_vel_0)
            err_vel_1 = interpolate_error(true_time, true_vel_1, est_time, est_vel_1)
            err_vel_2 = interpolate_error(true_time, true_vel_2, est_time, est_vel_2)

            fig.line(
                est_time,
                err_vel_0,
                alpha=self.alpha,
                color=self.colors[0],
                legend_label='X')
            fig.line(
                est_time,
                err_vel_1,
                alpha=self.alpha,
                color=self.colors[1],
                legend_label='Y')
            fig.line(
                est_time,
                err_vel_2,
                alpha=self.alpha,
                color=self.colors[2],
                legend_label='Z')

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

            est_time = body_state['time']
            est_acc_0 = body_state['body_acc_0']
            est_acc_1 = body_state['body_acc_1']
            est_acc_2 = body_state['body_acc_2']

            err_acc_0 = interpolate_error(true_time, true_acc_0, est_time, est_acc_0)
            err_acc_1 = interpolate_error(true_time, true_acc_1, est_time, est_acc_1)
            err_acc_2 = interpolate_error(true_time, true_acc_2, est_time, est_acc_2)

            fig.line(
                est_time,
                err_acc_0,
                alpha=self.alpha,
                color=self.colors[0],
                legend_label='X')
            fig.line(
                est_time,
                err_acc_1,
                alpha=self.alpha,
                color=self.colors[1],
                legend_label='Y')
            fig.line(
                est_time,
                err_acc_2,
                alpha=self.alpha,
                color=self.colors[2],
                legend_label='Z')
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

            est_time = body_state['time']
            est_ang_pos_w = body_state['body_ang_pos_0']
            est_ang_pos_x = body_state['body_ang_pos_1']
            est_ang_pos_y = body_state['body_ang_pos_2']
            est_ang_pos_z = body_state['body_ang_pos_3']
            est_ang_pos_r = lists_to_rot(
                est_ang_pos_w,
                est_ang_pos_x,
                est_ang_pos_y,
                est_ang_pos_z)

            interp_w = np.interp(est_time, true_time, true_ang_pos_w)
            interp_x = np.interp(est_time, true_time, true_ang_pos_x)
            interp_y = np.interp(est_time, true_time, true_ang_pos_y)
            interp_z = np.interp(est_time, true_time, true_ang_pos_z)
            interp_r = lists_to_rot(interp_w, interp_x, interp_y, interp_z)

            err_ang_pos_x, err_ang_pos_y, err_ang_pos_z = \
                calculate_rotation_errors(est_ang_pos_r, interp_r)

            fig.line(
                est_time,
                err_ang_pos_x,
                alpha=self.alpha,
                color=self.colors[0],
                legend_label='X')
            fig.line(
                est_time,
                err_ang_pos_y,
                alpha=self.alpha,
                color=self.colors[1],
                legend_label='Y')
            fig.line(
                est_time,
                err_ang_pos_z,
                alpha=self.alpha,
                color=self.colors[2],
                legend_label='Z')
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

            est_time = body_state['time']
            est_ang_vel_0 = body_state['body_ang_vel_0']
            est_ang_vel_1 = body_state['body_ang_vel_1']
            est_ang_vel_2 = body_state['body_ang_vel_2']

            err_ang_vel_0 = interpolate_error(true_time, true_ang_vel_0, est_time, est_ang_vel_0)
            err_ang_vel_1 = interpolate_error(true_time, true_ang_vel_1, est_time, est_ang_vel_1)
            err_ang_vel_2 = interpolate_error(true_time, true_ang_vel_2, est_time, est_ang_vel_2)

            fig.line(
                est_time,
                err_ang_vel_0,
                alpha=self.alpha,
                color=self.colors[0],
                legend_label='X')
            fig.line(
                est_time,
                err_ang_vel_1,
                alpha=self.alpha,
                color=self.colors[1],
                legend_label='Y')
            fig.line(
                est_time,
                err_ang_vel_2,
                alpha=self.alpha,
                color=self.colors[2],
                legend_label='Z')
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

            est_time = body_state['time']
            est_ang_acc_0 = body_state['body_ang_acc_0']
            est_ang_acc_1 = body_state['body_ang_acc_1']
            est_ang_acc_2 = body_state['body_ang_acc_2']

            err_ang_acc_0 = interpolate_error(true_time, true_ang_acc_0, est_time, est_ang_acc_0)
            err_ang_acc_1 = interpolate_error(true_time, true_ang_acc_1, est_time, est_ang_acc_1)
            err_ang_acc_2 = interpolate_error(true_time, true_ang_acc_2, est_time, est_ang_acc_2)

            fig.line(
                est_time,
                err_ang_acc_0,
                alpha=self.alpha,
                color=self.colors[0],
                legend_label='X')
            fig.line(
                est_time,
                err_ang_acc_1,
                alpha=self.alpha,
                color=self.colors[1],
                legend_label='Y')
            fig.line(
                est_time,
                err_ang_acc_2,
                alpha=self.alpha,
                color=self.colors[2],
                legend_label='Z')
        return fig

    def plot_body_pos_cov(self):
        """Plot body covariances for body position."""
        fig = figure(width=800, height=300, x_axis_label='Time [s]',
                     y_axis_label='Position [m]', title='Body Position Covariance')
        for body_df in self.body_state_dfs:
            time = body_df['time']
            fig.line(
                time,
                body_df['body_cov_0'],
                alpha=self.alpha,
                color=self.colors[0],
                legend_label='X')
            fig.line(
                time,
                body_df['body_cov_1'],
                alpha=self.alpha,
                color=self.colors[1],
                legend_label='Y')
            fig.line(
                time,
                body_df['body_cov_2'],
                alpha=self.alpha,
                color=self.colors[2],
                legend_label='Z')
        return fig

    def plot_body_vel_cov(self):
        """Plot body covariances for body velocity."""
        fig = figure(width=800, height=300, x_axis_label='Time [s]',
                     y_axis_label='Velocity [m/s]', title='Body Velocity Covariance')
        for body_df in self.body_state_dfs:
            time = body_df['time']
            fig.line(
                time,
                body_df['body_cov_3'],
                alpha=self.alpha,
                color=self.colors[0],
                legend_label='X')
            fig.line(
                time,
                body_df['body_cov_4'],
                alpha=self.alpha,
                color=self.colors[1],
                legend_label='Y')
            fig.line(
                time,
                body_df['body_cov_5'],
                alpha=self.alpha,
                color=self.colors[2],
                legend_label='Z')
        return fig

    def plot_body_acc_cov(self):
        """Plot body covariances for body acceleration."""
        fig = figure(width=800, height=300, x_axis_label='Time [s]',
                     y_axis_label='Acceleration [m/s/s]', title='Body Acceleration Covariance')
        for body_df in self.body_state_dfs:
            time = body_df['time']
            fig.line(
                time,
                body_df['body_cov_6'],
                alpha=self.alpha,
                color=self.colors[0],
                legend_label='X')
            fig.line(
                time,
                body_df['body_cov_7'],
                alpha=self.alpha,
                color=self.colors[1],
                legend_label='Y')
            fig.line(
                time,
                body_df['body_cov_8'],
                alpha=self.alpha,
                color=self.colors[2],
                legend_label='Z')
        return fig

    def plot_body_ang_cov(self):
        """Plot body covariances for body angles."""
        fig = figure(width=800, height=300, x_axis_label='Time [s]',
                     y_axis_label='Angle [rad]', title='Body Angular Covariance')
        for body_df in self.body_state_dfs:
            time = body_df['time']
            fig.line(
                time,
                body_df['body_cov_6'],
                alpha=self.alpha,
                color=self.colors[0],
                legend_label='X')
            fig.line(
                time,
                body_df['body_cov_7'],
                alpha=self.alpha,
                color=self.colors[1],
                legend_label='Y')
            fig.line(
                time,
                body_df['body_cov_8'],
                alpha=self.alpha,
                color=self.colors[2],
                legend_label='Z')
        return fig

    def plot_body_ang_vel_cov(self):
        """Plot body covariances for body angular rate."""
        fig = figure(width=800, height=300, x_axis_label='Time [s]',
                     y_axis_label='Angular Rate [rad/s]', title='Body Angular Rate Covariance')
        for body_df in self.body_state_dfs:
            time = body_df['time']
            fig.line(
                time,
                body_df['body_cov_12'],
                alpha=self.alpha,
                color=self.colors[0],
                legend_label='X')
            fig.line(
                time,
                body_df['body_cov_13'],
                alpha=self.alpha,
                color=self.colors[1],
                legend_label='Y')
            fig.line(
                time,
                body_df['body_cov_14'],
                alpha=self.alpha,
                color=self.colors[2],
                legend_label='Z')
        return fig

    def plot_body_ang_acc_cov(self):
        """Plot body covariances for body angular acceleration."""
        fig = figure(width=800, height=300, x_axis_label='Time [s]',
                     y_axis_label='Angular Acceleration [rad/s/s]',
                     title='Body Angular Acceleration Covariance')
        for body_df in self.body_state_dfs:
            time = body_df['time']
            fig.line(
                time,
                body_df['body_cov_15'],
                alpha=self.alpha,
                color=self.colors[0],
                legend_label='X')
            fig.line(
                time,
                body_df['body_cov_16'],
                alpha=self.alpha,
                color=self.colors[1],
                legend_label='Y')
            fig.line(
                time,
                body_df['body_cov_17'],
                alpha=self.alpha,
                color=self.colors[2],
                legend_label='Z')
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
                self.plot_aug_pos() if self.aug_state_dfs else Spacer()
            ],
            [
                self.plot_body_err_vel(),
                self.plot_aug_ang() if self.aug_state_dfs else Spacer()
            ],
            [
                self.plot_body_vel_cov(),
                Spacer()
            ],
            [
                plot_update_timing(self.body_state_dfs),
                Spacer()
            ]
        ]

        tab_layout = layout(layout_plots, sizing_mode='stretch_width')
        tab = TabPanel(child=tab_layout, title='Body')

        return tab
