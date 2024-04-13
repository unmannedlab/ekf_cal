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
from utilities import calculate_alpha, plot_update_timing


def plot_acc_measurements(imu_dfs):
    """Plot acceleration measurements."""
    fig = figure(width=800, height=300, x_axis_label='time [s]',
                 y_axis_label='Acceleration [m/s/s]', title='Acceleration Measurements')
    a = calculate_alpha(len(imu_dfs))
    for imu_df in imu_dfs:
        t_imu = imu_df['time'].to_list()
        fig.line(t_imu, imu_df['acc_0'].to_list(), alpha=a, color='cyan', legend_label='x')
        fig.line(t_imu, imu_df['acc_1'].to_list(), alpha=a, color='yellow', legend_label='y')
        fig.line(t_imu, imu_df['acc_2'].to_list(), alpha=a, color='magenta', legend_label='z')
    return fig


def plot_omg_measurements(imu_dfs):
    """Plot angular rate measurements."""
    fig = figure(width=800, height=300, x_axis_label='time [s]',
                 y_axis_label='Angular Rate [rad/s]', title='Angular Rate Measurements')
    a = calculate_alpha(len(imu_dfs))
    for imu_df in imu_dfs:
        t_imu = imu_df['time'].to_list()
        fig.line(t_imu, imu_df['omg_0'].to_list(), alpha=a, color='cyan', legend_label='x')
        fig.line(t_imu, imu_df['omg_1'].to_list(), alpha=a, color='yellow', legend_label='y')
        fig.line(t_imu, imu_df['omg_2'].to_list(), alpha=a, color='magenta', legend_label='z')
    return fig


def plot_acc_residuals(imu_dfs):
    """Plot acceleration residuals."""
    fig = figure(width=800, height=300, x_axis_label='time [s]',
                 y_axis_label='Acceleration [m/s/s]', title='Acceleration Residuals')
    a = calculate_alpha(len(imu_dfs))
    for imu_df in imu_dfs:
        t_imu = imu_df['time'].to_list()
        fig.line(t_imu, imu_df['residual_0'].to_list(), alpha=a, color='cyan', legend_label='x')
        fig.line(t_imu, imu_df['residual_1'].to_list(), alpha=a, color='yellow', legend_label='y')
        fig.line(t_imu, imu_df['residual_2'].to_list(), alpha=a, color='magenta', legend_label='z')
    return fig


def plot_omg_residuals(imu_dfs):
    """Plot angular rate residuals."""
    fig = figure(width=800, height=300, x_axis_label='time [s]',
                 y_axis_label='Angular Rate [rad/s]', title='Angular Rate Residuals')
    a = calculate_alpha(len(imu_dfs))
    for imu_df in imu_dfs:
        t_imu = imu_df['time'].to_list()
        fig.line(t_imu, imu_df['residual_3'].to_list(), alpha=a, color='cyan', legend_label='x')
        fig.line(t_imu, imu_df['residual_4'].to_list(), alpha=a, color='yellow', legend_label='y')
        fig.line(t_imu, imu_df['residual_5'].to_list(), alpha=a, color='magenta', legend_label='z')
    return fig


def plot_ext_pos_err(imu_dfs):
    """Plot the extrinsic position error."""
    fig = figure(width=800, height=300, x_axis_label='time [s]',
                 y_axis_label='Position Error [m]', title='Extrinsic Position Error')
    a = calculate_alpha(len(imu_dfs))
    t_imu = np.zeros([len(imu_dfs), len(imu_dfs[0]['time'])])
    pos_0 = np.zeros([len(imu_dfs), len(imu_dfs[0]['time'])])
    pos_1 = np.zeros([len(imu_dfs), len(imu_dfs[0]['time'])])
    pos_2 = np.zeros([len(imu_dfs), len(imu_dfs[0]['time'])])

    for i in range(len(imu_dfs)):
        t_imu[i, :] = imu_dfs[i]['time'].to_list()
        pos_0[i, :] = np.array(imu_dfs[i]['imu_pos_0'].to_list())
        pos_1[i, :] = np.array(imu_dfs[i]['imu_pos_1'].to_list())
        pos_2[i, :] = np.array(imu_dfs[i]['imu_pos_2'].to_list())
        fig.line(t_imu[i, :], pos_0[i, :], alpha=a, color='cyan')
        fig.line(t_imu[i, :], pos_1[i, :], alpha=a, color='yellow')
        fig.line(t_imu[i, :], pos_2[i, :], alpha=a, color='magenta')

    pos_0_std = np.std(pos_0, axis=0)
    pos_1_std = np.std(pos_1, axis=0)
    pos_2_std = np.std(pos_2, axis=0)
    fig.line(t_imu[0, :], +3.0 * pos_0_std, line_dash='dashed', color='red', alpha=a)
    fig.line(t_imu[0, :], +3.0 * pos_1_std, line_dash='dashed', color='red', alpha=a)
    fig.line(t_imu[0, :], +3.0 * pos_2_std, line_dash='dashed', color='red', alpha=a)
    fig.line(t_imu[0, :], -3.0 * pos_0_std, line_dash='dashed', color='red', alpha=a)
    fig.line(t_imu[0, :], -3.0 * pos_1_std, line_dash='dashed', color='red', alpha=a)
    fig.line(t_imu[0, :], -3.0 * pos_2_std, line_dash='dashed', color='red', alpha=a)
    return fig


def plot_ext_ang_err(imu_dfs):
    """Plot the extrinsic angular error."""
    fig = figure(width=800, height=300, x_axis_label='time [s]',
                 y_axis_label='Angle Error [m]', title='Extrinsic Angle Error')
    a = calculate_alpha(len(imu_dfs))
    for i in range(len(imu_dfs)):
        time = imu_dfs[i]['time'].to_list()
        err_img_ang_w = imu_dfs[i]['imu_ang_pos_0'].to_list()
        err_img_ang_x = imu_dfs[i]['imu_ang_pos_1'].to_list()
        err_img_ang_y = imu_dfs[i]['imu_ang_pos_2'].to_list()
        err_img_ang_z = imu_dfs[i]['imu_ang_pos_3'].to_list()

        fig.line(time, err_img_ang_w, alpha=a, color='cyan')
        fig.line(time, err_img_ang_x, alpha=a, color='yellow')
        fig.line(time, err_img_ang_y, alpha=a, color='magenta')
        fig.line(time, err_img_ang_z, alpha=a, color='red')
    return fig


def plot_acc_bias_err(imu_dfs):
    """Plot the intrinsic accelerometer bias error."""
    fig = figure(width=800, height=300, x_axis_label='time [s]',
                 y_axis_label='Bias Error [m]', title='Accelerometer Bias Error')
    a = calculate_alpha(len(imu_dfs))
    t_imu = np.zeros([len(imu_dfs), len(imu_dfs[0]['time'])])
    acc_bias_0 = np.zeros([len(imu_dfs), len(imu_dfs[0]['time'])])
    acc_bias_1 = np.zeros([len(imu_dfs), len(imu_dfs[0]['time'])])
    acc_bias_2 = np.zeros([len(imu_dfs), len(imu_dfs[0]['time'])])

    for i in range(len(imu_dfs)):
        t_imu[i, :] = imu_dfs[i]['time'].to_list()
        acc_bias_0[i, :] = np.array(imu_dfs[i]['imu_acc_bias_0'].to_list())
        acc_bias_1[i, :] = np.array(imu_dfs[i]['imu_acc_bias_1'].to_list())
        acc_bias_2[i, :] = np.array(imu_dfs[i]['imu_acc_bias_2'].to_list())
        fig.line(t_imu[i, :], acc_bias_0[i, :], color='cyan')
        fig.line(t_imu[i, :], acc_bias_1[i, :], color='yellow')
        fig.line(t_imu[i, :], acc_bias_2[i, :], color='magenta')

    a_bias_0_std = np.std(acc_bias_0, axis=0)
    a_bias_1_std = np.std(acc_bias_1, axis=0)
    a_bias_2_std = np.std(acc_bias_2, axis=0)
    fig.line(t_imu[0, :], +3.0 * a_bias_0_std, line_dash='dashed', color='red', alpha=a)
    fig.line(t_imu[0, :], +3.0 * a_bias_1_std, line_dash='dashed', color='red', alpha=a)
    fig.line(t_imu[0, :], +3.0 * a_bias_2_std, line_dash='dashed', color='red', alpha=a)
    fig.line(t_imu[0, :], -3.0 * a_bias_0_std, line_dash='dashed', color='red', alpha=a)
    fig.line(t_imu[0, :], -3.0 * a_bias_1_std, line_dash='dashed', color='red', alpha=a)
    fig.line(t_imu[0, :], -3.0 * a_bias_2_std, line_dash='dashed', color='red', alpha=a)
    return fig


def plot_omg_bias_err(imu_dfs):
    """Plot the intrinsic gyroscope bias error."""
    fig = figure(width=800, height=300, x_axis_label='time [s]',
                 y_axis_label='Bias Error [m]', title='Gyroscope Bias Error')
    a = calculate_alpha(len(imu_dfs))
    t_imu = np.zeros([len(imu_dfs), len(imu_dfs[0]['time'])])
    w_bias_0 = np.zeros([len(imu_dfs), len(imu_dfs[0]['time'])])
    w_bias_1 = np.zeros([len(imu_dfs), len(imu_dfs[0]['time'])])
    w_bias_2 = np.zeros([len(imu_dfs), len(imu_dfs[0]['time'])])

    for i in range(len(imu_dfs)):
        t_imu[i, :] = imu_dfs[i]['time'].to_list()
        w_bias_0[i, :] = np.array(imu_dfs[i]['imu_gyr_bias_0'].to_list())
        w_bias_1[i, :] = np.array(imu_dfs[i]['imu_gyr_bias_1'].to_list())
        w_bias_2[i, :] = np.array(imu_dfs[i]['imu_gyr_bias_2'].to_list())
        fig.line(t_imu[i, :], w_bias_0[i, :], color='cyan')
        fig.line(t_imu[i, :], w_bias_1[i, :], color='yellow')
        fig.line(t_imu[i, :], w_bias_2[i, :], color='magenta')

    w_bias_0_std = np.std(w_bias_0, axis=0)
    w_bias_1_std = np.std(w_bias_1, axis=0)
    w_bias_2_std = np.std(w_bias_2, axis=0)
    fig.line(t_imu[0, :], +3.0 * w_bias_0_std, line_dash='dashed', color='red', alpha=a)
    fig.line(t_imu[0, :], +3.0 * w_bias_1_std, line_dash='dashed', color='red', alpha=a)
    fig.line(t_imu[0, :], +3.0 * w_bias_2_std, line_dash='dashed', color='red', alpha=a)
    fig.line(t_imu[0, :], -3.0 * w_bias_0_std, line_dash='dashed', color='red', alpha=a)
    fig.line(t_imu[0, :], -3.0 * w_bias_1_std, line_dash='dashed', color='red', alpha=a)
    fig.line(t_imu[0, :], -3.0 * w_bias_2_std, line_dash='dashed', color='red', alpha=a)
    return fig


def plot_imu_ext_pos_update(imu_dfs):
    """Plot extrinsic position update."""
    fig = figure(width=800, height=300, x_axis_label='time [s]',
                 y_axis_label='Extrinsic Position Update [m]', title='Extrinsic Position Update')
    a = calculate_alpha(len(imu_dfs))
    for imu_df in imu_dfs:
        t_imu = imu_df['time'].to_list()
        imu_ext_update_0 = imu_df['imu_ext_update_0'].to_list()
        imu_ext_update_1 = imu_df['imu_ext_update_1'].to_list()
        imu_ext_update_2 = imu_df['imu_ext_update_2'].to_list()
        fig.line(t_imu, imu_ext_update_0, alpha=a, color='cyan', legend_label='p_x')
        fig.line(t_imu, imu_ext_update_1, alpha=a, color='yellow', legend_label='p_y')
        fig.line(t_imu, imu_ext_update_2, alpha=a, color='magenta', legend_label='p_z')
    return fig


def plot_imu_ext_ang_update(imu_dfs):
    """Plot extrinsic angle update."""
    fig = figure(width=800, height=300, x_axis_label='time [s]',
                 y_axis_label='Extrinsic Angle Update [m]', title='Extrinsic Angle Update')
    a = calculate_alpha(len(imu_dfs))
    for imu_df in imu_dfs:
        t_imu = imu_df['time'].to_list()
        imu_ext_update_3 = imu_df['imu_ext_update_3'].to_list()
        imu_ext_update_4 = imu_df['imu_ext_update_4'].to_list()
        imu_ext_update_5 = imu_df['imu_ext_update_5'].to_list()
        fig.line(t_imu, imu_ext_update_3, alpha=a, color='cyan', legend_label='\theta_x')
        fig.line(t_imu, imu_ext_update_4, alpha=a, color='yellow', legend_label='\theta_y')
        fig.line(t_imu, imu_ext_update_5, alpha=a, color='magenta', legend_label='\theta_z')
    return fig


def plot_imu_int_pos_update(imu_dfs):
    """Plot accelerometer bias updates."""
    fig = figure(width=800, height=300, x_axis_label='time [s]',
                 y_axis_label='Accelerometer Bias Updates [m/s/s]',
                 title='Accelerometer Bias Updates')
    a = calculate_alpha(len(imu_dfs))
    for imu_df in imu_dfs:
        t_imu = imu_df['time'].to_list()
        imu_int_update_0 = imu_df['imu_int_update_0'].to_list()
        imu_int_update_1 = imu_df['imu_int_update_1'].to_list()
        imu_int_update_2 = imu_df['imu_int_update_2'].to_list()
        fig.line(t_imu, imu_int_update_0, alpha=a, color='cyan', legend_label='b_a_x')
        fig.line(t_imu, imu_int_update_1, alpha=a, color='yellow', legend_label='b_a_y')
        fig.line(t_imu, imu_int_update_2, alpha=a, color='magenta', legend_label='b_a_z')
    return fig


def plot_imu_int_ang_update(imu_dfs):
    """Plot intrinsic gyroscope bias updates."""
    fig = figure(width=800, height=300, x_axis_label='time [s]',
                 y_axis_label='Gyroscope Bias Updates [rad/s]', title='Gyroscope Bias Updates')
    a = calculate_alpha(len(imu_dfs))
    for imu_df in imu_dfs:
        t_imu = imu_df['time'].to_list()
        imu_int_update_3 = imu_df['imu_int_update_3'].to_list()
        imu_int_update_4 = imu_df['imu_int_update_4'].to_list()
        imu_int_update_5 = imu_df['imu_int_update_5'].to_list()
        fig.line(t_imu, imu_int_update_3, alpha=a, color='cyan', legend_label='b_w_x')
        fig.line(t_imu, imu_int_update_4, alpha=a, color='yellow', legend_label='b_w_y')
        fig.line(t_imu, imu_int_update_5, alpha=a, color='magenta', legend_label='b_w_z')
    return fig


def plot_imu_ext_pos_cov(imu_dfs):
    """Plot extrinsic position covariance."""
    fig = figure(width=800, height=300, x_axis_label='time [s]',
                 y_axis_label='Position Covariance [m]', title='Position Covariance')
    a = calculate_alpha(len(imu_dfs))
    for imu_df in imu_dfs:
        t_imu = imu_df['time'].to_list()
        imu_ext_cov_0 = imu_df['imu_ext_cov_0'].to_list()
        imu_ext_cov_1 = imu_df['imu_ext_cov_1'].to_list()
        imu_ext_cov_2 = imu_df['imu_ext_cov_2'].to_list()
        fig.line(t_imu, imu_ext_cov_0, alpha=a, color='cyan', legend_label='p_x')
        fig.line(t_imu, imu_ext_cov_1, alpha=a, color='yellow', legend_label='p_y')
        fig.line(t_imu, imu_ext_cov_2, alpha=a, color='magenta', legend_label='p_z')
    return fig


def plot_imu_ext_ang_cov(imu_dfs):
    """Plot extrinsic angle covariance."""
    fig = figure(width=800, height=300, x_axis_label='time [s]',
                 y_axis_label='Angle Covariance [m]', title='Angle Covariance')
    a = calculate_alpha(len(imu_dfs))
    for imu_df in imu_dfs:
        t_imu = imu_df['time'].to_list()
        imu_ext_cov_3 = imu_df['imu_ext_cov_3'].to_list()
        imu_ext_cov_4 = imu_df['imu_ext_cov_4'].to_list()
        imu_ext_cov_5 = imu_df['imu_ext_cov_5'].to_list()
        fig.line(t_imu, imu_ext_cov_3, alpha=a, color='cyan', legend_label='\theta_x')
        fig.line(t_imu, imu_ext_cov_4, alpha=a, color='yellow', legend_label='\theta_y')
        fig.line(t_imu, imu_ext_cov_5, alpha=a, color='magenta', legend_label='\theta_z')
    return fig


def plot_imu_int_pos_cov(imu_dfs):
    """Plot intrinsic accelerometer bias."""
    fig = figure(width=800, height=300, x_axis_label='time [s]',
                 y_axis_label='Accelerometer Bias [m/s/s]', title='Accelerometer Bias')
    a = calculate_alpha(len(imu_dfs))
    for imu_df in imu_dfs:
        t_imu = imu_df['time'].to_list()
        imu_int_cov_0 = imu_df['imu_int_cov_0'].to_list()
        imu_int_cov_1 = imu_df['imu_int_cov_1'].to_list()
        imu_int_cov_2 = imu_df['imu_int_cov_2'].to_list()
        fig.line(t_imu, imu_int_cov_0, alpha=a, color='cyan', legend_label='b_a_x')
        fig.line(t_imu, imu_int_cov_1, alpha=a, color='yellow', legend_label='b_a_y')
        fig.line(t_imu, imu_int_cov_2, alpha=a, color='magenta', legend_label='b_a_z')
    return fig


def plot_imu_int_ang_cov(imu_dfs):
    """Plot intrinsic gyroscope bias."""
    fig = figure(width=800, height=300, x_axis_label='time [s]',
                 y_axis_label='Gyroscope Bias [rad/s]', title='Gyroscope Bias')
    a = calculate_alpha(len(imu_dfs))
    for imu_df in imu_dfs:
        t_imu = imu_df['time'].to_list()
        imu_int_cov_3 = imu_df['imu_int_cov_3'].to_list()
        imu_int_cov_4 = imu_df['imu_int_cov_4'].to_list()
        imu_int_cov_5 = imu_df['imu_int_cov_5'].to_list()
        fig.line(t_imu, imu_int_cov_3, alpha=a, color='cyan', legend_label='b_w_x')
        fig.line(t_imu, imu_int_cov_4, alpha=a, color='yellow', legend_label='b_w_y')
        fig.line(t_imu, imu_int_cov_5, alpha=a, color='magenta', legend_label='b_w_z')
    return fig


def tab_imu(imu_dfs):
    layout_plots = [
        [plot_acc_measurements(imu_dfs), plot_omg_measurements(imu_dfs)],
        [plot_acc_residuals(imu_dfs), plot_omg_residuals(imu_dfs)],
    ]

    if ('imu_ext_cov_0' in imu_dfs[0].keys()):
        layout_plots.append([plot_imu_ext_pos_update(imu_dfs), plot_imu_ext_ang_update(imu_dfs)])
        layout_plots.append([plot_imu_ext_pos_cov(imu_dfs), plot_imu_ext_ang_cov(imu_dfs)])
        layout_plots.append([plot_ext_pos_err(imu_dfs), plot_ext_ang_err(imu_dfs)])

    if ('imu_int_cov_0' in imu_dfs[0].keys()):
        layout_plots.append([plot_imu_int_pos_update(imu_dfs), plot_imu_int_ang_update(imu_dfs)])
        layout_plots.append([plot_imu_int_pos_cov(imu_dfs), plot_imu_int_ang_cov(imu_dfs)])
        layout_plots.append([plot_acc_bias_err(imu_dfs), plot_omg_bias_err(imu_dfs)])

    layout_plots.append([plot_update_timing(imu_dfs), Spacer()])
    tab_layout = layout(layout_plots, sizing_mode='stretch_width')
    tab = TabPanel(child=tab_layout, title=f"IMU {imu_dfs[0].attrs['id']}")

    return tab
