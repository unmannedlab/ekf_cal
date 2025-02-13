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
from bokeh.models import Range1d, TabPanel
from bokeh.plotting import figure
import numpy as np

from utilities import calculate_alpha, get_colors, interpolate_quat_error, interpolate_error, \
    plot_update_timing


class tab_imu:

    def __init__(self, imu_dfs, body_truth_dfs, args):
        self.imu_dfs = imu_dfs
        self.body_truth_dfs = body_truth_dfs

        self.alpha = calculate_alpha(len(self.imu_dfs))
        self.colors = get_colors(args)

    def plot_acc_measurements(self):
        """Plot acceleration measurements."""
        fig = figure(
            width=800,
            height=300,
            x_axis_label='Time [s]',
            y_axis_label='Acceleration [m/s/s]',
            title='Acceleration Measurements')
        for imu_df in self.imu_dfs:
            t_imu = imu_df['time']
            fig.line(
                t_imu,
                imu_df['acc_0'],
                alpha=self.alpha,
                color=self.colors[0],
                legend_label='X')
            fig.line(
                t_imu,
                imu_df['acc_1'],
                alpha=self.alpha,
                color=self.colors[1],
                legend_label='Y')
            fig.line(
                t_imu,
                imu_df['acc_2'],
                alpha=self.alpha,
                color=self.colors[2],
                legend_label='Z')
        return fig

    def plot_omg_measurements(self):
        """Plot angular rate measurements."""
        fig = figure(
            width=800,
            height=300,
            x_axis_label='Time [s]',
            y_axis_label='Angular Rate [rad/s]',
            title='Angular Rate Measurements')
        for imu_df in self.imu_dfs:
            t_imu = imu_df['time']
            fig.line(
                t_imu,
                imu_df['omg_0'],
                alpha=self.alpha,
                color=self.colors[0],
                legend_label='X')
            fig.line(
                t_imu,
                imu_df['omg_1'],
                alpha=self.alpha,
                color=self.colors[1],
                legend_label='Y')
            fig.line(
                t_imu,
                imu_df['omg_2'],
                alpha=self.alpha,
                color=self.colors[2],
                legend_label='Z')
        return fig

    def plot_acc_residuals(self):
        """Plot acceleration residuals."""
        fig = figure(
            width=800,
            height=300,
            x_axis_label='Time [s]',
            y_axis_label='Acceleration [m/s/s]',
            title='Acceleration Residuals')
        for imu_df in self.imu_dfs:
            t_imu = imu_df['time']
            fig.line(
                t_imu,
                imu_df['residual_0'],
                alpha=self.alpha,
                color=self.colors[0],
                legend_label='X')
            fig.line(
                t_imu,
                imu_df['residual_1'],
                alpha=self.alpha,
                color=self.colors[1],
                legend_label='Y')
            fig.line(
                t_imu,
                imu_df['residual_2'],
                alpha=self.alpha,
                color=self.colors[2],
                legend_label='Z')
        return fig

    def plot_omg_residuals(self):
        """Plot angular rate residuals."""
        fig = figure(
            width=800,
            height=300,
            x_axis_label='Time [s]',
            y_axis_label='Angular Rate [rad/s]',
            title='Angular Rate Residuals')
        for imu_df in self.imu_dfs:
            time = imu_df['time']
            fig.line(
                time,
                imu_df['residual_3'],
                alpha=self.alpha,
                color=self.colors[0],
                legend_label='X')
            fig.line(
                time,
                imu_df['residual_4'],
                alpha=self.alpha,
                color=self.colors[1],
                legend_label='Y')
            fig.line(
                time,
                imu_df['residual_5'],
                alpha=self.alpha,
                color=self.colors[2],
                legend_label='Z')
        return fig

    def plot_ext_pos_err(self):
        """Plot the extrinsic position error."""
        fig = figure(
            width=800,
            height=300,
            x_axis_label='Time [s]',
            y_axis_label='Position Error [mm]',
            title='Extrinsic Position Error')

        for i, (imu_df, body_truth) in enumerate(zip(self.imu_dfs, self.body_truth_dfs)):
            time = self.imu_dfs[i]['time']
            est_x = np.array(self.imu_dfs[i]['imu_pos_0'])
            est_y = np.array(self.imu_dfs[i]['imu_pos_1'])
            est_z = np.array(self.imu_dfs[i]['imu_pos_2'])

            true_t = body_truth['time']
            true_x = body_truth[f"imu_pos_{imu_df.attrs['id']}_0"]
            true_y = body_truth[f"imu_pos_{imu_df.attrs['id']}_1"]
            true_z = body_truth[f"imu_pos_{imu_df.attrs['id']}_2"]

            pos_x = np.array(interpolate_error(true_t, true_x, time, est_x))
            pos_y = np.array(interpolate_error(true_t, true_y, time, est_y))
            pos_z = np.array(interpolate_error(true_t, true_z, time, est_z))

            fig.line(time, pos_x*1e3, alpha=self.alpha, color=self.colors[0], legend_label='X')
            fig.line(time, pos_y*1e3, alpha=self.alpha, color=self.colors[1], legend_label='Y')
            fig.line(time, pos_z*1e3, alpha=self.alpha, color=self.colors[2], legend_label='Z')

        return fig

    def plot_ext_ang_err(self):
        """Plot the extrinsic angular error."""
        fig = figure(
            width=800,
            height=300,
            x_axis_label='Time [s]',
            y_axis_label='Angle Error [mrad]',
            title='Extrinsic Angle Error')
        for imu_df, body_truth in zip(self.imu_dfs, self.body_truth_dfs):
            est_t = imu_df['time']
            est_w = imu_df['imu_ang_pos_0']
            est_x = imu_df['imu_ang_pos_1']
            est_y = imu_df['imu_ang_pos_2']
            est_z = imu_df['imu_ang_pos_3']
            true_t = body_truth['time']
            true_w = body_truth[f"imu_ang_pos_{imu_df.attrs['id']}_0"]
            true_x = body_truth[f"imu_ang_pos_{imu_df.attrs['id']}_1"]
            true_y = body_truth[f"imu_ang_pos_{imu_df.attrs['id']}_2"]
            true_z = body_truth[f"imu_ang_pos_{imu_df.attrs['id']}_3"]

            eul_err_x, eul_err_y, eul_err_z = interpolate_quat_error(
                true_t, true_w, true_x, true_y, true_z, est_t, est_w, est_x, est_y, est_z)

            fig.line(est_t, eul_err_x, alpha=self.alpha, color=self.colors[0], legend_label='X')
            fig.line(est_t, eul_err_y, alpha=self.alpha, color=self.colors[1], legend_label='Y')
            fig.line(est_t, eul_err_z, alpha=self.alpha, color=self.colors[2], legend_label='Z')
        return fig

    def plot_acc_bias_err(self):
        """Plot the intrinsic accelerometer bias error."""
        fig = figure(
            width=800,
            height=300,
            x_axis_label='Time [s]',
            y_axis_label='Bias Error [m]',
            title='Accelerometer Bias Error')
        n = np.max([len(imu_df) for imu_df in self.imu_dfs])
        a_bias_err_x = np.zeros([len(self.imu_dfs), n])
        a_bias_err_y = np.zeros([len(self.imu_dfs), n])
        a_bias_err_z = np.zeros([len(self.imu_dfs), n])

        for i, (imu_df, body_truth) in enumerate(zip(self.imu_dfs, self.body_truth_dfs)):
            m = len(self.imu_dfs[i]['time'])
            time = self.imu_dfs[i]['time']
            est_x = np.array(self.imu_dfs[i]['imu_acc_bias_0'])
            est_y = np.array(self.imu_dfs[i]['imu_acc_bias_1'])
            est_z = np.array(self.imu_dfs[i]['imu_acc_bias_2'])

            true_t = body_truth['time']
            true_x = body_truth[f"imu_acc_bias_{imu_df.attrs['id']}_0"]
            true_y = body_truth[f"imu_acc_bias_{imu_df.attrs['id']}_1"]
            true_z = body_truth[f"imu_acc_bias_{imu_df.attrs['id']}_2"]

            a_bias_err_x[i, 0:m] = interpolate_error(true_t, true_x, time, est_x)
            a_bias_err_y[i, 0:m] = interpolate_error(true_t, true_y, time, est_y)
            a_bias_err_z[i, 0:m] = interpolate_error(true_t, true_z, time, est_z)

            fig.line(time, a_bias_err_x[i, :], color=self.colors[0], alpha=self.alpha)
            fig.line(time, a_bias_err_y[i, :], color=self.colors[1], alpha=self.alpha)
            fig.line(time, a_bias_err_z[i, :], color=self.colors[2], alpha=self.alpha)

        return fig

    def plot_omg_bias_err(self):
        """Plot the intrinsic gyroscope bias error."""
        fig = figure(
            width=800,
            height=300,
            x_axis_label='Time [s]',
            y_axis_label='Bias Error [m]',
            title='Gyroscope Bias Error')
        n = np.max([len(imu_df) for imu_df in self.imu_dfs])
        w_bias_t = np.zeros([len(self.imu_dfs), n])
        w_bias_err_x = np.zeros([len(self.imu_dfs), n])
        w_bias_err_y = np.zeros([len(self.imu_dfs), n])
        w_bias_err_z = np.zeros([len(self.imu_dfs), n])

        for i, (imu_df, body_truth) in enumerate(zip(self.imu_dfs, self.body_truth_dfs)):
            m = len(self.imu_dfs[i]['time'])
            w_bias_t[i, 0:m] = self.imu_dfs[i]['time']
            w_bias_x = np.array(self.imu_dfs[i]['imu_gyr_bias_0'])
            w_bias_y = np.array(self.imu_dfs[i]['imu_gyr_bias_1'])
            w_bias_z = np.array(self.imu_dfs[i]['imu_gyr_bias_2'])

            true_t = body_truth['time']
            true_x = body_truth[f"imu_gyr_bias_{imu_df.attrs['id']}_0"]
            true_y = body_truth[f"imu_gyr_bias_{imu_df.attrs['id']}_1"]
            true_z = body_truth[f"imu_gyr_bias_{imu_df.attrs['id']}_2"]

            w_bias_err_x[i, 0:m] = interpolate_error(true_t, true_x, w_bias_err_x[i, :], w_bias_x)
            w_bias_err_y[i, 0:m] = interpolate_error(true_t, true_y, w_bias_err_y[i, :], w_bias_y)
            w_bias_err_z[i, 0:m] = interpolate_error(true_t, true_z, w_bias_err_z[i, :], w_bias_z)

            fig.line(w_bias_t[i, :], w_bias_err_x[i, :], color=self.colors[0], alpha=self.alpha)
            fig.line(w_bias_t[i, :], w_bias_err_y[i, :], color=self.colors[1], alpha=self.alpha)
            fig.line(w_bias_t[i, :], w_bias_err_z[i, :], color=self.colors[2], alpha=self.alpha)

        return fig

    def plot_imu_ext_pos_cov(self):
        """Plot extrinsic position covariance."""
        fig = figure(
            width=800,
            height=300,
            x_axis_label='Time [s]',
            y_axis_label='Position Covariance [m]',
            title='Position Covariance')
        for imu_df in self.imu_dfs:
            time = imu_df['time']
            imu_ext_cov_0 = imu_df['imu_ext_cov_0']
            imu_ext_cov_1 = imu_df['imu_ext_cov_1']
            imu_ext_cov_2 = imu_df['imu_ext_cov_2']
            fig.line(time, imu_ext_cov_0, alpha=self.alpha, color=self.colors[0], legend_label='X')
            fig.line(time, imu_ext_cov_1, alpha=self.alpha, color=self.colors[1], legend_label='Y')
            fig.line(time, imu_ext_cov_2, alpha=self.alpha, color=self.colors[2], legend_label='Z')
        return fig

    def plot_imu_ext_ang_cov(self):
        """Plot extrinsic angle covariance."""
        fig = figure(
            width=800,
            height=300,
            x_axis_label='Time [s]',
            y_axis_label='Angle Covariance [m]',
            title='Angle Covariance')
        for imu_df in self.imu_dfs:
            time = imu_df['time']
            imu_ext_cov_3 = imu_df['imu_ext_cov_3']
            imu_ext_cov_4 = imu_df['imu_ext_cov_4']
            imu_ext_cov_5 = imu_df['imu_ext_cov_5']
            fig.line(time, imu_ext_cov_3, alpha=self.alpha, color=self.colors[0], legend_label='X')
            fig.line(time, imu_ext_cov_4, alpha=self.alpha, color=self.colors[1], legend_label='Y')
            fig.line(time, imu_ext_cov_5, alpha=self.alpha, color=self.colors[2], legend_label='Z')
        return fig

    def plot_imu_int_pos_cov(self):
        """Plot intrinsic accelerometer bias covariance."""
        fig = figure(
            width=800,
            height=300,
            x_axis_label='Time [s]',
            y_axis_label='Accelerometer Bias Covariance [m/s/s]',
            title='Accelerometer Bias Covariance')
        for imu_df in self.imu_dfs:
            time = imu_df['time']
            imu_int_cov_0 = imu_df['imu_int_cov_0']
            imu_int_cov_1 = imu_df['imu_int_cov_1']
            imu_int_cov_2 = imu_df['imu_int_cov_2']
            fig.line(time, imu_int_cov_0, alpha=self.alpha, color=self.colors[0], legend_label='X')
            fig.line(time, imu_int_cov_1, alpha=self.alpha, color=self.colors[1], legend_label='Y')
            fig.line(time, imu_int_cov_2, alpha=self.alpha, color=self.colors[2], legend_label='Z')
        return fig

    def plot_imu_int_ang_cov(self):
        """Plot intrinsic gyroscope bias covariance."""
        fig = figure(
            width=800,
            height=300,
            x_axis_label='Time [s]',
            y_axis_label='Gyroscope Bias Covariance [rad/s]',
            title='Gyroscope Bias Covariance')
        for imu_df in self.imu_dfs:
            time = imu_df['time']
            imu_int_cov_3 = imu_df['imu_int_cov_3']
            imu_int_cov_4 = imu_df['imu_int_cov_4']
            imu_int_cov_5 = imu_df['imu_int_cov_5']
            fig.line(time, imu_int_cov_3, alpha=self.alpha, color=self.colors[0], legend_label='X')
            fig.line(time, imu_int_cov_4, alpha=self.alpha, color=self.colors[1], legend_label='Y')
            fig.line(time, imu_int_cov_5, alpha=self.alpha, color=self.colors[2], legend_label='Z')
        return fig

    def plot_stationary(self):
        """Plot is stationary update being performed."""
        fig = figure(
            width=800,
            height=300,
            x_axis_label='Time [s]',
            y_axis_label='Is Stationary',
            title='Is Stationary')
        for imu_df in self.imu_dfs:
            t_imu = imu_df['time']
            is_stationary = imu_df['stationary']
            score = imu_df['score']
            fig.line(
                t_imu,
                is_stationary,
                alpha=self.alpha,
                color=self.colors[0],
                legend_label='Is Stationary')
            fig.line(
                t_imu,
                score,
                alpha=self.alpha,
                color=self.colors[1],
                legend_label='Chi^2 Score')
        fig.y_range = Range1d(0, 1)
        return fig

    def get_tab(self):
        layout_plots = [
            [self.plot_acc_measurements(), self.plot_omg_measurements()],
            [self.plot_acc_residuals(), self.plot_omg_residuals()],
        ]

        if ('imu_ext_cov_0' in self.imu_dfs[0].keys()):
            layout_plots.append([self.plot_imu_ext_pos_cov(), self.plot_imu_ext_ang_cov()])
            layout_plots.append([self.plot_ext_pos_err(), self.plot_ext_ang_err()])

        if ('imu_int_cov_0' in self.imu_dfs[0].keys()):
            layout_plots.append([self.plot_imu_int_pos_cov(), self.plot_imu_int_ang_cov()])
            layout_plots.append([self.plot_acc_bias_err(), self.plot_omg_bias_err()])

        layout_plots.append([plot_update_timing(self.imu_dfs), self.plot_stationary()])
        tab_layout = layout(layout_plots, sizing_mode='stretch_width')
        tab = TabPanel(child=tab_layout,
                       title=f"IMU {self.imu_dfs[0].attrs['id']}")

        return tab
