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
from utilities import calculate_alpha, calculate_rotation_errors, colors, interpolate_error, \
    lists_to_rot, plot_update_timing


def plot_body_pos(body_state_dfs):
    """Plot body position."""
    fig = figure(width=800, height=300, x_axis_label='Time [s]',
                 y_axis_label='Position [m]', title='Body Position')
    a = calculate_alpha(len(body_state_dfs))
    for body_df in body_state_dfs:
        time = body_df['time'].to_list()
        fig.line(time, body_df['body_pos_0'].to_list(), alpha=a, color=colors[0])
        fig.line(time, body_df['body_pos_1'].to_list(), alpha=a, color=colors[1])
        fig.line(time, body_df['body_pos_2'].to_list(), alpha=a, color=colors[2])
    return fig


def plot_body_vel(body_state_dfs):
    """Plot body velocity."""
    fig = figure(width=800, height=300, x_axis_label='Time [s]',
                 y_axis_label='Velocity [m/s]', title='Body Velocity')
    a = calculate_alpha(len(body_state_dfs))
    for body_df in body_state_dfs:
        time = body_df['time'].to_list()
        fig.line(time, body_df['body_vel_0'].to_list(), alpha=a, color=colors[0])
        fig.line(time, body_df['body_vel_1'].to_list(), alpha=a, color=colors[1])
        fig.line(time, body_df['body_vel_2'].to_list(), alpha=a, color=colors[2])
    return fig


def plot_body_acc(body_state_dfs):
    """Plot body acceleration."""
    fig = figure(width=800, height=300, x_axis_label='Time [s]',
                 y_axis_label='Acceleration [m/s/s]', title='Body Acceleration')
    a = calculate_alpha(len(body_state_dfs))
    for body_df in body_state_dfs:
        time = body_df['time'].to_list()
        fig.line(time, body_df['body_acc_0'].to_list(), alpha=a, color=colors[0])
        fig.line(time, body_df['body_acc_1'].to_list(), alpha=a, color=colors[1])
        fig.line(time, body_df['body_acc_2'].to_list(), alpha=a, color=colors[2])
    return fig


def plot_body_ang(body_state_dfs):
    """Plot body angular position."""
    fig = figure(width=800, height=300, x_axis_label='Time [s]',
                 y_axis_label='Angle', title='Body Angle')
    a = calculate_alpha(len(body_state_dfs))
    for body_df in body_state_dfs:
        time = body_df['time'].to_list()
        fig.line(time, body_df['body_ang_pos_0'].to_list(), alpha=a, color=colors[0])
        fig.line(time, body_df['body_ang_pos_1'].to_list(), alpha=a, color=colors[1])
        fig.line(time, body_df['body_ang_pos_2'].to_list(), alpha=a, color=colors[2])
        fig.line(time, body_df['body_ang_pos_3'].to_list(), alpha=a, color='red')
    return fig


def plot_body_ang_vel(body_state_dfs):
    """Plot body angular velocity."""
    fig = figure(width=800, height=300, x_axis_label='Time [s]',
                 y_axis_label='Angular Velocity', title='Body Angular Velocity')
    a = calculate_alpha(len(body_state_dfs))
    for body_df in body_state_dfs:
        time = body_df['time'].to_list()
        fig.line(time, body_df['body_ang_vel_0'].to_list(), alpha=a, color=colors[0])
        fig.line(time, body_df['body_ang_vel_1'].to_list(), alpha=a, color=colors[1])
        fig.line(time, body_df['body_ang_vel_2'].to_list(), alpha=a, color=colors[2])
    return fig


def plot_body_ang_acc(body_state_dfs):
    """Plot body angular acceleration."""
    fig = figure(width=800, height=300, x_axis_label='Time [s]',
                 y_axis_label='Angular Acceleration', title='Body Angular Acceleration')
    a = calculate_alpha(len(body_state_dfs))
    for body_df in body_state_dfs:
        time = body_df['time'].to_list()
        fig.line(time, body_df['body_ang_acc_0'].to_list(), alpha=a, color=colors[0])
        fig.line(time, body_df['body_ang_acc_1'].to_list(), alpha=a, color=colors[1])
        fig.line(time, body_df['body_ang_acc_2'].to_list(), alpha=a, color=colors[2])
    return fig


def plot_body_err_pos(body_state_dfs, body_truth_dfs):
    """Plot the body state position error."""
    fig = figure(width=800, height=300, x_axis_label='Time [s]',
                 y_axis_label='Position Error [m]', title='Body Position Error')
    a = calculate_alpha(len(body_state_dfs))
    for body_state, body_truth in zip(body_state_dfs, body_truth_dfs):
        true_time = body_truth['time'].to_list()
        true_pos_0 = body_truth['body_pos_0'].to_list()
        true_pos_1 = body_truth['body_pos_1'].to_list()
        true_pos_2 = body_truth['body_pos_2'].to_list()

        est_time = body_state['time'].to_list()
        est_pos_0 = body_state['body_pos_0'].to_list()
        est_pos_1 = body_state['body_pos_1'].to_list()
        est_pos_2 = body_state['body_pos_2'].to_list()

        err_pos_0 = interpolate_error(true_time, true_pos_0, est_time, est_pos_0)
        err_pos_1 = interpolate_error(true_time, true_pos_1, est_time, est_pos_1)
        err_pos_2 = interpolate_error(true_time, true_pos_2, est_time, est_pos_2)

        fig.line(est_time, err_pos_0, alpha=a, color=colors[0])
        fig.line(est_time, err_pos_1, alpha=a, color=colors[1])
        fig.line(est_time, err_pos_2, alpha=a, color=colors[2])

    return fig


def plot_body_err_vel(body_state_dfs, body_truth_dfs):
    """Plot the body state velocity error."""
    fig = figure(width=800, height=300, x_axis_label='Time [s]',
                 y_axis_label='Velocity Error [m/s]', title='Body Velocity Error')
    a = calculate_alpha(len(body_state_dfs))
    for body_state, body_truth in zip(body_state_dfs, body_truth_dfs):
        true_time = body_truth['time'].to_list()
        true_vel_0 = body_truth['body_vel_0'].to_list()
        true_vel_1 = body_truth['body_vel_1'].to_list()
        true_vel_2 = body_truth['body_vel_2'].to_list()

        est_time = body_state['time'].to_list()
        est_vel_0 = body_state['body_vel_0'].to_list()
        est_vel_1 = body_state['body_vel_1'].to_list()
        est_vel_2 = body_state['body_vel_2'].to_list()

        err_vel_0 = interpolate_error(true_time, true_vel_0, est_time, est_vel_0)
        err_vel_1 = interpolate_error(true_time, true_vel_1, est_time, est_vel_1)
        err_vel_2 = interpolate_error(true_time, true_vel_2, est_time, est_vel_2)

        fig.line(est_time, err_vel_0, alpha=a, color=colors[0])
        fig.line(est_time, err_vel_1, alpha=a, color=colors[1])
        fig.line(est_time, err_vel_2, alpha=a, color=colors[2])

    return fig


def plot_body_err_acc(body_state_dfs, body_truth_dfs):
    """Plot the body state acceleration error."""
    fig = figure(width=800, height=300, x_axis_label='Time [s]',
                 y_axis_label='Acceleration Error [m/s/s]', title='Body Acceleration Error')
    a = calculate_alpha(len(body_state_dfs))
    for body_state, body_truth in zip(body_state_dfs, body_truth_dfs):
        true_time = body_truth['time'].to_list()
        true_acc_0 = body_truth['body_acc_0'].to_list()
        true_acc_1 = body_truth['body_acc_1'].to_list()
        true_acc_2 = body_truth['body_acc_2'].to_list()

        est_time = body_state['time'].to_list()
        est_acc_0 = body_state['body_acc_0'].to_list()
        est_acc_1 = body_state['body_acc_1'].to_list()
        est_acc_2 = body_state['body_acc_2'].to_list()

        err_acc_0 = interpolate_error(true_time, true_acc_0, est_time, est_acc_0)
        err_acc_1 = interpolate_error(true_time, true_acc_1, est_time, est_acc_1)
        err_acc_2 = interpolate_error(true_time, true_acc_2, est_time, est_acc_2)

        fig.line(est_time, err_acc_0, alpha=a, color=colors[0])
        fig.line(est_time, err_acc_1, alpha=a, color=colors[1])
        fig.line(est_time, err_acc_2, alpha=a, color=colors[2])
    return fig


def plot_body_err_ang(body_state_dfs, body_truth_dfs):
    """Plot the body state angular error."""
    fig = figure(width=800, height=300, x_axis_label='Time [s]',
                 y_axis_label='Angular Error', title='Body Angular Error')
    a = calculate_alpha(len(body_state_dfs))
    for body_state, body_truth in zip(body_state_dfs, body_truth_dfs):
        true_time = body_truth['time'].to_list()
        true_ang_pos_w = body_truth['body_ang_pos_0'].to_list()
        true_ang_pos_x = body_truth['body_ang_pos_1'].to_list()
        true_ang_pos_y = body_truth['body_ang_pos_2'].to_list()
        true_ang_pos_z = body_truth['body_ang_pos_3'].to_list()

        est_time = body_state['time'].to_list()
        est_ang_pos_w = body_state['body_ang_pos_0'].to_list()
        est_ang_pos_x = body_state['body_ang_pos_1'].to_list()
        est_ang_pos_y = body_state['body_ang_pos_2'].to_list()
        est_ang_pos_z = body_state['body_ang_pos_3'].to_list()
        est_ang_pos_r = lists_to_rot(est_ang_pos_w, est_ang_pos_x, est_ang_pos_y, est_ang_pos_z)

        interp_w = np.interp(est_time, true_time, true_ang_pos_w)
        interp_x = np.interp(est_time, true_time, true_ang_pos_x)
        interp_y = np.interp(est_time, true_time, true_ang_pos_y)
        interp_z = np.interp(est_time, true_time, true_ang_pos_z)
        interp_r = lists_to_rot(interp_w, interp_x, interp_y, interp_z)

        err_ang_pos_w, err_ang_pos_x, err_ang_pos_y, err_ang_pos_z = \
            calculate_rotation_errors(est_ang_pos_r, interp_r)

        fig.line(est_time, err_ang_pos_w, alpha=a, color=colors[0])
        fig.line(est_time, err_ang_pos_x, alpha=a, color=colors[1])
        fig.line(est_time, err_ang_pos_y, alpha=a, color=colors[2])
        fig.line(est_time, err_ang_pos_z, alpha=a, color='red')
    return fig


def plot_body_err_ang_vel(body_state_dfs, body_truth_dfs):
    """Plot the body state angular velocity error."""
    fig = figure(width=800, height=300, x_axis_label='Time [s]',
                 y_axis_label='Angular Velocity Error [rad/s]',
                 title='Body Angular Velocity Error')
    a = calculate_alpha(len(body_state_dfs))
    for body_state, body_truth in zip(body_state_dfs, body_truth_dfs):
        true_time = body_truth['time'].to_list()
        true_ang_vel_0 = body_truth['body_ang_vel_0'].to_list()
        true_ang_vel_1 = body_truth['body_ang_vel_1'].to_list()
        true_ang_vel_2 = body_truth['body_ang_vel_2'].to_list()

        est_time = body_state['time'].to_list()
        est_ang_vel_0 = body_state['body_ang_vel_0'].to_list()
        est_ang_vel_1 = body_state['body_ang_vel_1'].to_list()
        est_ang_vel_2 = body_state['body_ang_vel_2'].to_list()

        err_ang_vel_0 = interpolate_error(true_time, true_ang_vel_0, est_time, est_ang_vel_0)
        err_ang_vel_1 = interpolate_error(true_time, true_ang_vel_1, est_time, est_ang_vel_1)
        err_ang_vel_2 = interpolate_error(true_time, true_ang_vel_2, est_time, est_ang_vel_2)

        fig.line(est_time, err_ang_vel_0, alpha=a, color=colors[0])
        fig.line(est_time, err_ang_vel_1, alpha=a, color=colors[1])
        fig.line(est_time, err_ang_vel_2, alpha=a, color=colors[2])
    return fig


def plot_body_err_ang_acc(body_state_dfs, body_truth_dfs):
    """Plot the body state angular acceleration error."""
    fig = figure(width=800, height=300, x_axis_label='Time [s]',
                 y_axis_label='Angular Acceleration Error [rad/s/s]',
                 title='Body Angular Acceleration Error')
    a = calculate_alpha(len(body_state_dfs))
    for body_state, body_truth in zip(body_state_dfs, body_truth_dfs):
        true_time = body_truth['time'].to_list()
        true_ang_acc_0 = body_truth['body_ang_acc_0'].to_list()
        true_ang_acc_1 = body_truth['body_ang_acc_1'].to_list()
        true_ang_acc_2 = body_truth['body_ang_acc_2'].to_list()

        est_time = body_state['time'].to_list()
        est_ang_acc_0 = body_state['body_ang_acc_0'].to_list()
        est_ang_acc_1 = body_state['body_ang_acc_1'].to_list()
        est_ang_acc_2 = body_state['body_ang_acc_2'].to_list()

        err_ang_acc_0 = interpolate_error(true_time, true_ang_acc_0, est_time, est_ang_acc_0)
        err_ang_acc_1 = interpolate_error(true_time, true_ang_acc_1, est_time, est_ang_acc_1)
        err_ang_acc_2 = interpolate_error(true_time, true_ang_acc_2, est_time, est_ang_acc_2)

        fig.line(est_time, err_ang_acc_0, alpha=a, color=colors[0])
        fig.line(est_time, err_ang_acc_1, alpha=a, color=colors[1])
        fig.line(est_time, err_ang_acc_2, alpha=a, color=colors[2])
    return fig


def plot_body_pos_cov(body_state_dfs):
    """Plot body covariances for body position."""
    fig = figure(width=800, height=300, x_axis_label='Time [s]',
                 y_axis_label='Position [m]', title='Body Position Covariance')
    a = calculate_alpha(len(body_state_dfs))
    for body_df in body_state_dfs:
        time = body_df['time'].to_list()
        fig.line(time, body_df['body_cov_0'].to_list(), alpha=a, color=colors[0])
        fig.line(time, body_df['body_cov_1'].to_list(), alpha=a, color=colors[1])
        fig.line(time, body_df['body_cov_2'].to_list(), alpha=a, color=colors[2])
    return fig


def plot_body_vel_cov(body_state_dfs):
    """Plot body covariances for body velocity."""
    fig = figure(width=800, height=300, x_axis_label='Time [s]',
                 y_axis_label='Velocity [m/s]', title='Body Velocity Covariance')
    a = calculate_alpha(len(body_state_dfs))
    for body_df in body_state_dfs:
        time = body_df['time'].to_list()
        fig.line(time, body_df['body_cov_3'].to_list(), alpha=a, color=colors[0])
        fig.line(time, body_df['body_cov_4'].to_list(), alpha=a, color=colors[1])
        fig.line(time, body_df['body_cov_5'].to_list(), alpha=a, color=colors[2])
    return fig


def plot_body_acc_cov(body_state_dfs):
    """Plot body covariances for body acceleration."""
    fig = figure(width=800, height=300, x_axis_label='Time [s]',
                 y_axis_label='Acceleration [m/s/s]', title='Body Acceleration Covariance')
    a = calculate_alpha(len(body_state_dfs))
    for body_df in body_state_dfs:
        time = body_df['time'].to_list()
        fig.line(time, body_df['body_cov_6'].to_list(), alpha=a, color=colors[0])
        fig.line(time, body_df['body_cov_7'].to_list(), alpha=a, color=colors[1])
        fig.line(time, body_df['body_cov_8'].to_list(), alpha=a, color=colors[2])
    return fig


def plot_body_ang_cov(body_state_dfs):
    """Plot body covariances for body angles."""
    fig = figure(width=800, height=300, x_axis_label='Time [s]',
                 y_axis_label='', title='Body Angular Covariance')
    a = calculate_alpha(len(body_state_dfs))
    for body_df in body_state_dfs:
        time = body_df['time'].to_list()
        fig.line(time, body_df['body_cov_9'].to_list(), alpha=a, color=colors[0])
        fig.line(time, body_df['body_cov_10'].to_list(), alpha=a, color=colors[1])
        fig.line(time, body_df['body_cov_11'].to_list(), alpha=a, color=colors[2])
    return fig


def plot_body_ang_vel_cov(body_state_dfs):
    """Plot body covariances for body angular rate."""
    fig = figure(width=800, height=300, x_axis_label='Time [s]',
                 y_axis_label='Angular Rate [rad/s]', title='Body Angular Rate Covariance')
    a = calculate_alpha(len(body_state_dfs))
    for body_df in body_state_dfs:
        time = body_df['time'].to_list()
        fig.line(time, body_df['body_cov_12'].to_list(), alpha=a, color=colors[0])
        fig.line(time, body_df['body_cov_13'].to_list(), alpha=a, color=colors[1])
        fig.line(time, body_df['body_cov_14'].to_list(), alpha=a, color=colors[2])
    return fig


def plot_body_ang_acc_cov(body_state_dfs):
    """Plot body covariances for body angular acceleration."""
    fig = figure(width=800, height=300, x_axis_label='Time [s]',
                 y_axis_label='Angular Acceleration [rad/s/s]',
                 title='Body Angular Acceleration Covariance')
    a = calculate_alpha(len(body_state_dfs))
    for body_df in body_state_dfs:
        time = body_df['time'].to_list()
        fig.line(time, body_df['body_cov_15'].to_list(), alpha=a, color=colors[0])
        fig.line(time, body_df['body_cov_16'].to_list(), alpha=a, color=colors[1])
        fig.line(time, body_df['body_cov_17'].to_list(), alpha=a, color=colors[2])
    return fig


def tab_body(body_state_dfs, body_truth_dfs):
    layout_plots = [
        [
            plot_body_pos(body_state_dfs),
            plot_body_ang(body_state_dfs)],
        [
            plot_body_err_pos(body_state_dfs, body_truth_dfs),
            plot_body_err_ang(body_state_dfs, body_truth_dfs)],
        [
            plot_body_pos_cov(body_state_dfs),
            plot_body_ang_cov(body_state_dfs)],
        [
            plot_body_vel(body_state_dfs),
            plot_body_ang_vel(body_state_dfs)],
        [
            plot_body_err_vel(body_state_dfs, body_truth_dfs),
            plot_body_err_ang_vel(body_state_dfs, body_truth_dfs)],
        [
            plot_body_vel_cov(body_state_dfs),
            plot_body_ang_vel_cov(body_state_dfs)],
        [
            plot_body_acc(body_state_dfs),
            plot_body_ang_acc(body_state_dfs)],
        [
            plot_body_err_acc(body_state_dfs, body_truth_dfs),
            plot_body_err_ang_acc(body_state_dfs, body_truth_dfs)],
        [
            plot_body_acc_cov(body_state_dfs),
            plot_body_ang_acc_cov(body_state_dfs)],
        [
            plot_update_timing(body_state_dfs),
            Spacer()]
    ]

    tab_layout = layout(layout_plots, sizing_mode='stretch_width')
    tab = TabPanel(child=tab_layout, title='Body')

    return tab
