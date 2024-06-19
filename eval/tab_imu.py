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
from utilities import calculate_alpha, get_colors, plot_update_timing


class tab_imu:

    def __init__(self, imu_dfs, args):
        self.imu_dfs = imu_dfs

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
                legend_label='x')
            fig.line(
                t_imu,
                imu_df['acc_1'],
                alpha=self.alpha,
                color=self.colors[1],
                legend_label='y')
            fig.line(
                t_imu,
                imu_df['acc_2'],
                alpha=self.alpha,
                color=self.colors[2],
                legend_label='z')
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
                legend_label='x')
            fig.line(
                t_imu,
                imu_df['omg_1'],
                alpha=self.alpha,
                color=self.colors[1],
                legend_label='y')
            fig.line(
                t_imu,
                imu_df['omg_2'],
                alpha=self.alpha,
                color=self.colors[2],
                legend_label='z')
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
                legend_label='x')
            fig.line(
                t_imu,
                imu_df['residual_1'],
                alpha=self.alpha,
                color=self.colors[1],
                legend_label='y')
            fig.line(
                t_imu,
                imu_df['residual_2'],
                alpha=self.alpha,
                color=self.colors[2],
                legend_label='z')
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
            t_imu = imu_df['time']
            fig.line(
                t_imu,
                imu_df['residual_3'],
                alpha=self.alpha,
                color=self.colors[0],
                legend_label='x')
            fig.line(
                t_imu,
                imu_df['residual_4'],
                alpha=self.alpha,
                color=self.colors[1],
                legend_label='y')
            fig.line(
                t_imu,
                imu_df['residual_5'],
                alpha=self.alpha,
                color=self.colors[2],
                legend_label='z')
        return fig

    def plot_ext_pos_err(self):
        """Plot the extrinsic position error."""
        fig = figure(
            width=800,
            height=300,
            x_axis_label='Time [s]',
            y_axis_label='Position Error [m]',
            title='Extrinsic Position Error')
        t_imu = np.zeros([len(self.imu_dfs), len(self.imu_dfs[0]['time'])])
        pos_0 = np.zeros([len(self.imu_dfs), len(self.imu_dfs[0]['time'])])
        pos_1 = np.zeros([len(self.imu_dfs), len(self.imu_dfs[0]['time'])])
        pos_2 = np.zeros([len(self.imu_dfs), len(self.imu_dfs[0]['time'])])

        for i in range(len(self.imu_dfs)):
            t_imu[i, :] = self.imu_dfs[i]['time']
            pos_0[i, :] = np.array(self.imu_dfs[i]['imu_pos_0'])
            pos_1[i, :] = np.array(self.imu_dfs[i]['imu_pos_1'])
            pos_2[i, :] = np.array(self.imu_dfs[i]['imu_pos_2'])
            fig.line(
                t_imu[i, :], pos_0[i, :],
                alpha=self.alpha,
                color=self.colors[0])
            fig.line(
                t_imu[i, :], pos_1[i, :],
                alpha=self.alpha,
                color=self.colors[1])
            fig.line(
                t_imu[i, :], pos_2[i, :],
                alpha=self.alpha,
                color=self.colors[2])

        pos_0_std = np.std(pos_0, axis=0)
        pos_1_std = np.std(pos_1, axis=0)
        pos_2_std = np.std(pos_2, axis=0)
        fig.line(
            t_imu[0, :], +3.0 * pos_0_std,
            line_dash='dashed',
            color='red',
            alpha=self.alpha)
        fig.line(
            t_imu[0, :], +3.0 * pos_1_std,
            line_dash='dashed',
            color='red',
            alpha=self.alpha)
        fig.line(
            t_imu[0, :], +3.0 * pos_2_std,
            line_dash='dashed',
            color='red',
            alpha=self.alpha)
        fig.line(
            t_imu[0, :], -3.0 * pos_0_std,
            line_dash='dashed',
            color='red',
            alpha=self.alpha)
        fig.line(
            t_imu[0, :], -3.0 * pos_1_std,
            line_dash='dashed',
            color='red',
            alpha=self.alpha)
        fig.line(
            t_imu[0, :], -3.0 * pos_2_std,
            line_dash='dashed',
            color='red',
            alpha=self.alpha)
        return fig

    def plot_ext_ang_err(self):
        """Plot the extrinsic angular error."""
        fig = figure(
            width=800,
            height=300,
            x_axis_label='Time [s]',
            y_axis_label='Angle Error [m]',
            title='Extrinsic Angle Error')
        for i in range(len(self.imu_dfs)):
            time = self.imu_dfs[i]['time']
            err_img_ang_w = self.imu_dfs[i]['imu_ang_pos_0']
            err_img_ang_x = self.imu_dfs[i]['imu_ang_pos_1']
            err_img_ang_y = self.imu_dfs[i]['imu_ang_pos_2']
            err_img_ang_z = self.imu_dfs[i]['imu_ang_pos_3']

            fig.line(
                time,
                err_img_ang_w,
                alpha=self.alpha,
                color=self.colors[0])
            fig.line(
                time,
                err_img_ang_x,
                alpha=self.alpha,
                color=self.colors[1])
            fig.line(
                time,
                err_img_ang_y,
                alpha=self.alpha,
                color=self.colors[2])
            fig.line(
                time,
                err_img_ang_z,
                alpha=self.alpha,
                color=self.colors[3])
        return fig

    def plot_acc_bias_err(self):
        """Plot the intrinsic accelerometer bias error."""
        fig = figure(
            width=800,
            height=300,
            x_axis_label='Time [s]',
            y_axis_label='Bias Error [m]',
            title='Accelerometer Bias Error')
        t_imu = np.zeros([len(self.imu_dfs), len(self.imu_dfs[0]['time'])])
        acc_bias_0 = np.zeros([len(self.imu_dfs), len(self.imu_dfs[0]['time'])])
        acc_bias_1 = np.zeros([len(self.imu_dfs), len(self.imu_dfs[0]['time'])])
        acc_bias_2 = np.zeros([len(self.imu_dfs), len(self.imu_dfs[0]['time'])])

        for i in range(len(self.imu_dfs)):
            t_imu[i, :] = self.imu_dfs[i]['time']
            acc_bias_0[i, :] = np.array(self.imu_dfs[i]['imu_acc_bias_0'])
            acc_bias_1[i, :] = np.array(self.imu_dfs[i]['imu_acc_bias_1'])
            acc_bias_2[i, :] = np.array(self.imu_dfs[i]['imu_acc_bias_2'])
            fig.line(
                t_imu[i, :],
                acc_bias_0[i, :],
                color=self.colors[0])
            fig.line(
                t_imu[i, :],
                acc_bias_1[i, :],
                color=self.colors[1])
            fig.line(
                t_imu[i, :],
                acc_bias_2[i, :],
                color=self.colors[2])

        a_bias_0_std = np.std(acc_bias_0, axis=0)
        a_bias_1_std = np.std(acc_bias_1, axis=0)
        a_bias_2_std = np.std(acc_bias_2, axis=0)
        fig.line(
            t_imu[0, :], +3.0 * a_bias_0_std,
            line_dash='dashed',
            color='red',
            alpha=self.alpha)
        fig.line(
            t_imu[0, :], +3.0 * a_bias_1_std,
            line_dash='dashed',
            color='red',
            alpha=self.alpha)
        fig.line(
            t_imu[0, :], +3.0 * a_bias_2_std,
            line_dash='dashed',
            color='red',
            alpha=self.alpha)
        fig.line(
            t_imu[0, :], -3.0 * a_bias_0_std,
            line_dash='dashed',
            color='red',
            alpha=self.alpha)
        fig.line(
            t_imu[0, :], -3.0 * a_bias_1_std,
            line_dash='dashed',
            color='red',
            alpha=self.alpha)
        fig.line(
            t_imu[0, :], -3.0 * a_bias_2_std,
            line_dash='dashed',
            color='red',
            alpha=self.alpha)
        return fig

    def plot_omg_bias_err(self):
        """Plot the intrinsic gyroscope bias error."""
        fig = figure(
            width=800,
            height=300,
            x_axis_label='Time [s]',
            y_axis_label='Bias Error [m]',
            title='Gyroscope Bias Error')
        t_imu = np.zeros([len(self.imu_dfs), len(self.imu_dfs[0]['time'])])
        w_bias_0 = np.zeros([len(self.imu_dfs), len(self.imu_dfs[0]['time'])])
        w_bias_1 = np.zeros([len(self.imu_dfs), len(self.imu_dfs[0]['time'])])
        w_bias_2 = np.zeros([len(self.imu_dfs), len(self.imu_dfs[0]['time'])])

        for i in range(len(self.imu_dfs)):
            t_imu[i, :] = self.imu_dfs[i]['time']
            w_bias_0[i, :] = np.array(self.imu_dfs[i]['imu_gyr_bias_0'])
            w_bias_1[i, :] = np.array(self.imu_dfs[i]['imu_gyr_bias_1'])
            w_bias_2[i, :] = np.array(self.imu_dfs[i]['imu_gyr_bias_2'])
            fig.line(
                t_imu[i, :],
                w_bias_0[i, :],
                color=self.colors[0])
            fig.line(
                t_imu[i, :],
                w_bias_1[i, :],
                color=self.colors[1])
            fig.line(
                t_imu[i, :],
                w_bias_2[i, :],
                color=self.colors[2])

        w_bias_0_std = np.std(w_bias_0, axis=0)
        w_bias_1_std = np.std(w_bias_1, axis=0)
        w_bias_2_std = np.std(w_bias_2, axis=0)
        fig.line(
            t_imu[0, :], +3.0 * w_bias_0_std,
            line_dash='dashed',
            color='red',
            alpha=self.alpha)
        fig.line(
            t_imu[0, :], +3.0 * w_bias_1_std,
            line_dash='dashed',
            color='red',
            alpha=self.alpha)
        fig.line(
            t_imu[0, :], +3.0 * w_bias_2_std,
            line_dash='dashed',
            color='red',
            alpha=self.alpha)
        fig.line(
            t_imu[0, :], -3.0 * w_bias_0_std,
            line_dash='dashed',
            color='red',
            alpha=self.alpha)
        fig.line(
            t_imu[0, :], -3.0 * w_bias_1_std,
            line_dash='dashed',
            color='red',
            alpha=self.alpha)
        fig.line(
            t_imu[0, :], -3.0 * w_bias_2_std,
            line_dash='dashed',
            color='red',
            alpha=self.alpha)
        return fig

    def plot_imu_ext_pos_update(self):
        """Plot extrinsic position update."""
        fig = figure(
            width=800,
            height=300,
            x_axis_label='Time [s]',
            y_axis_label='Extrinsic Position Update [m]',
            title='Extrinsic Position Update')
        for imu_df in self.imu_dfs:
            t_imu = imu_df['time']
            imu_ext_update_0 = imu_df['imu_ext_update_0']
            imu_ext_update_1 = imu_df['imu_ext_update_1']
            imu_ext_update_2 = imu_df['imu_ext_update_2']
            fig.line(
                t_imu,
                imu_ext_update_0,
                alpha=self.alpha,
                color=self.colors[0],
                legend_label='p_x')
            fig.line(
                t_imu,
                imu_ext_update_1,
                alpha=self.alpha,
                color=self.colors[1],
                legend_label='p_y')
            fig.line(
                t_imu,
                imu_ext_update_2,
                alpha=self.alpha,
                color=self.colors[2],
                legend_label='p_z')
        return fig

    def plot_imu_ext_ang_update(self):
        """Plot extrinsic angle update."""
        fig = figure(
            width=800,
            height=300,
            x_axis_label='Time [s]',
            y_axis_label='Extrinsic Angle Update [m]',
            title='Extrinsic Angle Update')
        for imu_df in self.imu_dfs:
            t_imu = imu_df['time']
            imu_ext_update_3 = imu_df['imu_ext_update_3']
            imu_ext_update_4 = imu_df['imu_ext_update_4']
            imu_ext_update_5 = imu_df['imu_ext_update_5']
            fig.line(
                t_imu,
                imu_ext_update_3,
                alpha=self.alpha,
                color=self.colors[0],
                legend_label='\theta_x')
            fig.line(
                t_imu,
                imu_ext_update_4,
                alpha=self.alpha,
                color=self.colors[1],
                legend_label='\theta_y')
            fig.line(
                t_imu,
                imu_ext_update_5,
                alpha=self.alpha,
                color=self.colors[2],
                legend_label='\theta_z')
        return fig

    def plot_imu_int_pos_update(self):
        """Plot accelerometer bias updates."""
        fig = figure(
            width=800,
            height=300,
            x_axis_label='Time [s]',
            y_axis_label='Accelerometer Bias Updates [m/s/s]',
            title='Accelerometer Bias Updates')
        for imu_df in self.imu_dfs:
            t_imu = imu_df['time']
            imu_int_update_0 = imu_df['imu_int_update_0']
            imu_int_update_1 = imu_df['imu_int_update_1']
            imu_int_update_2 = imu_df['imu_int_update_2']
            fig.line(
                t_imu,
                imu_int_update_0,
                alpha=self.alpha,
                color=self.colors[0],
                legend_label='b_a_x')
            fig.line(
                t_imu,
                imu_int_update_1,
                alpha=self.alpha,
                color=self.colors[1],
                legend_label='b_a_y')
            fig.line(
                t_imu,
                imu_int_update_2,
                alpha=self.alpha,
                color=self.colors[2],
                legend_label='b_a_z')
        return fig

    def plot_imu_int_ang_update(self):
        """Plot intrinsic gyroscope bias updates."""
        fig = figure(
            width=800,
            height=300,
            x_axis_label='Time [s]',
            y_axis_label='Gyroscope Bias Updates [rad/s]',
            title='Gyroscope Bias Updates')
        for imu_df in self.imu_dfs:
            t_imu = imu_df['time']
            imu_int_update_3 = imu_df['imu_int_update_3']
            imu_int_update_4 = imu_df['imu_int_update_4']
            imu_int_update_5 = imu_df['imu_int_update_5']
            fig.line(
                t_imu,
                imu_int_update_3,
                alpha=self.alpha,
                color=self.colors[0],
                legend_label='b_w_x')
            fig.line(
                t_imu,
                imu_int_update_4,
                alpha=self.alpha,
                color=self.colors[1],
                legend_label='b_w_y')
            fig.line(
                t_imu,
                imu_int_update_5,
                alpha=self.alpha,
                color=self.colors[2],
                legend_label='b_w_z')
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
            t_imu = imu_df['time']
            imu_ext_cov_0 = imu_df['imu_ext_cov_0']
            imu_ext_cov_1 = imu_df['imu_ext_cov_1']
            imu_ext_cov_2 = imu_df['imu_ext_cov_2']
            fig.line(
                t_imu,
                imu_ext_cov_0,
                alpha=self.alpha,
                color=self.colors[0],
                legend_label='p_x')
            fig.line(
                t_imu,
                imu_ext_cov_1,
                alpha=self.alpha,
                color=self.colors[1],
                legend_label='p_y')
            fig.line(
                t_imu,
                imu_ext_cov_2,
                alpha=self.alpha,
                color=self.colors[2],
                legend_label='p_z')
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
            t_imu = imu_df['time']
            imu_ext_cov_3 = imu_df['imu_ext_cov_3']
            imu_ext_cov_4 = imu_df['imu_ext_cov_4']
            imu_ext_cov_5 = imu_df['imu_ext_cov_5']
            fig.line(
                t_imu,
                imu_ext_cov_3,
                alpha=self.alpha,
                color=self.colors[0],
                legend_label='\theta_x')
            fig.line(
                t_imu,
                imu_ext_cov_4,
                alpha=self.alpha,
                color=self.colors[1],
                legend_label='\theta_y')
            fig.line(
                t_imu,
                imu_ext_cov_5,
                alpha=self.alpha,
                color=self.colors[2],
                legend_label='\theta_z')
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
            t_imu = imu_df['time']
            imu_int_cov_0 = imu_df['imu_int_cov_0']
            imu_int_cov_1 = imu_df['imu_int_cov_1']
            imu_int_cov_2 = imu_df['imu_int_cov_2']
            fig.line(
                t_imu,
                imu_int_cov_0,
                alpha=self.alpha,
                color=self.colors[0],
                legend_label='b_a_x')
            fig.line(
                t_imu,
                imu_int_cov_1,
                alpha=self.alpha,
                color=self.colors[1],
                legend_label='b_a_y')
            fig.line(
                t_imu,
                imu_int_cov_2,
                alpha=self.alpha,
                color=self.colors[2],
                legend_label='b_a_z')
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
            t_imu = imu_df['time']
            imu_int_cov_3 = imu_df['imu_int_cov_3']
            imu_int_cov_4 = imu_df['imu_int_cov_4']
            imu_int_cov_5 = imu_df['imu_int_cov_5']
            fig.line(
                t_imu,
                imu_int_cov_3,
                alpha=self.alpha,
                color=self.colors[0],
                legend_label='b_w_x')
            fig.line(
                t_imu,
                imu_int_cov_4,
                alpha=self.alpha,
                color=self.colors[1],
                legend_label='b_w_y')
            fig.line(
                t_imu,
                imu_int_cov_5,
                alpha=self.alpha,
                color=self.colors[2],
                legend_label='b_w_z')
        return fig

    def get_tab(self):
        layout_plots = [
            [self.plot_acc_measurements(), self.plot_omg_measurements()],
            [self.plot_acc_residuals(), self.plot_omg_residuals()],
        ]

        if ('imu_ext_cov_0' in self.imu_dfs[0].keys()):
            layout_plots.append([self.plot_imu_ext_pos_update(), self.plot_imu_ext_ang_update()])
            layout_plots.append([self.plot_imu_ext_pos_cov(), self.plot_imu_ext_ang_cov()])
            layout_plots.append([self.plot_ext_pos_err(), self.plot_ext_ang_err()])

        if ('imu_int_cov_0' in self.imu_dfs[0].keys()):
            layout_plots.append([self.plot_imu_int_pos_update(), self.plot_imu_int_ang_update()])
            layout_plots.append([self.plot_imu_int_pos_cov(), self.plot_imu_int_ang_cov()])
            layout_plots.append([self.plot_acc_bias_err(), self.plot_omg_bias_err()])

        layout_plots.append([plot_update_timing(self.imu_dfs), Spacer()])
        tab_layout = layout(layout_plots, sizing_mode='stretch_width')
        tab = TabPanel(child=tab_layout,
                       title=f"IMU {self.imu_dfs[0].attrs['id']}")

        return tab
