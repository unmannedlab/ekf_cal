#!/usr/bin/env python3

# Copyright 2023 Jacob Hartzer
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

"""
plot_sim.py.

A collection of functions for plotting results from the multi-IMU, multi-Camera simulation.

Typical usage is:
```
python3 eval/plot_sim.py config/example.yaml
```

To get help:
```
python3 eval/plot_sim.py --help
```
"""


import argparse
import collections
import functools
import glob
import math
import multiprocessing
import os
import re
import traceback

import matplotlib.animation as animation
import matplotlib.pyplot as plt

import numpy as np

import pandas as pd

from scipy.spatial.transform import Rotation as R

import yaml

plt.style.use('ggplot')
plt.rcParams.update({'figure.max_open_warning': 0})


def interpolate_error(true_t, true_x, estimate_t, estimate_x):
    """Calculate an interpolated vector error using truth and estimate points."""
    interp_x = np.interp(estimate_t, true_t, true_x)
    errors = [estimate - interp for estimate, interp in zip(estimate_x, interp_x)]
    return errors


def set_plot_titles(fig, name):
    """Set plot suptitle and canvas window title."""
    fig.suptitle(name)
    fig.canvas.manager.set_window_title(name)


def format_prefix(prefix):
    """Generate formatted prefix from string."""
    if (prefix == 'imu'):
        return 'IMU'
    elif (prefix == 'camera'):
        return 'Camera'
    elif (prefix == 'body_state'):
        return 'Body'
    else:
        return ''


def calculate_alpha(line_count: int):
    """Calculate transparency value from number of plots."""
    alpha = 1.0 / math.pow(line_count, 0.75)
    return alpha


def parse_yaml(config):
    """Collect sensor configuration data from input yaml."""
    config_data = {}
    config_data['imu_rates'] = {}
    config_data['camera_rates'] = {}
    with open(config, 'r') as stream:
        try:
            yaml_dict = yaml.safe_load(stream)
            imu_list = yaml_dict['/EkfCalNode']['ros__parameters']['imu_list']
            cam_list = yaml_dict['/EkfCalNode']['ros__parameters']['camera_list']
            id_counter = 1
            if imu_list:
                imu_dict = yaml_dict['/EkfCalNode']['ros__parameters']['imu']
                for imu_name in imu_list:
                    config_data['imu_rates'][id_counter] = imu_dict[imu_name]['rate']
                    id_counter += 1
            if cam_list:
                cam_dict = yaml_dict['/EkfCalNode']['ros__parameters']['camera']
                for cam_name in cam_list:
                    config_data['camera_rates'][id_counter] = cam_dict[cam_name]['rate']
                    id_counter += 1

        except yaml.YAMLError as exc:
            print(exc)

    return config_data


def find_and_read_data_frames(directories, prefix):
    """Find matching dataframes and read using pandas."""
    data_frame_sets = collections.defaultdict(list)
    for directory in directories:
        file_paths_id = glob.glob(os.path.join(directory, prefix + '*.csv'))
        for file_path in file_paths_id:
            file_name = os.path.basename(file_path)
            df = pd.read_csv(file_path)
            df.attrs['prefix'] = format_prefix(prefix)
            matches = re.findall(r'_[0-9]*\.csv', file_name)
            if matches:
                file_id = int(matches[0].split('_')[1].split('.csv')[0])
            else:
                file_id = 0
            df.attrs['id'] = file_id
            data_frame_sets[file_id].append(df)
    return data_frame_sets


def lists_to_rot(w_list, x_list, y_list, z_list):
    """Convert lists of quaternion elements to a list of scipy rotations."""
    r_list = []
    for w, x, y, z in zip(w_list, x_list, y_list, z_list):
        r = R.from_quat([w, x, y, z])
        r_list.append(r)
    return r_list


def calculate_rotation_errors(estimate, truth):
    """Calculate errors from two lists of quaternions."""
    w = []
    x = []
    y = []
    z = []
    for est, true in zip(estimate, truth):
        r = est * true.inv()
        q = r.as_quat()
        x.append(q[0])
        y.append(q[1])
        z.append(q[2])
        w.append(q[3])
    return w, x, y, z


def RMSE_from_vectors(x_list, y_list, z_list):
    """Calculate the root mean square errors from list of vector elements."""
    x_err = np.array(x_list)
    y_err = np.array(y_list)
    z_err = np.array(z_list)
    rmse = np.sqrt(np.mean(x_err * x_err + y_err * y_err + z_err * z_err))
    return rmse


class Plotter():
    """Class for containing plotting methods and options."""

    def __init__(self, show=False, rate_line=False, ext='png', jobs=None):
        """Plotter class initializer."""
        self.show = show
        self.rate_line = rate_line
        self.ext = ext
        cpu_count = jobs if (jobs) else multiprocessing.cpu_count() - 1
        self.pool = multiprocessing.Pool(cpu_count)
        self.statistics = collections.defaultdict(list)

    def print_err(err):
        """Print errors experienced in asynchronous pool."""
        print('error_callback()', err)
        traceback.print_exception(type(err), err, err.__traceback__)

    def plot_imu_measurements(self, imu_dfs):
        """Plot detected IMU measurements."""
        fig, (axs_1, axs_2) = plt.subplots(2, 1)
        alpha = calculate_alpha(len(imu_dfs))
        for imu_df in imu_dfs:
            t_imu = imu_df['time'].to_list()
            axs_1.plot(t_imu, imu_df['acc_0'].to_list(), alpha=alpha, color='tab:blue')
            axs_1.plot(t_imu, imu_df['acc_1'].to_list(), alpha=alpha, color='tab:orange')
            axs_1.plot(t_imu, imu_df['acc_2'].to_list(), alpha=alpha, color='tab:green')
            axs_2.plot(t_imu, imu_df['omg_0'].to_list(), alpha=alpha, color='tab:blue')
            axs_2.plot(t_imu, imu_df['omg_1'].to_list(), alpha=alpha, color='tab:orange')
            axs_2.plot(t_imu, imu_df['omg_2'].to_list(), alpha=alpha, color='tab:green')
        set_plot_titles(fig, f'IMU {imu_df.attrs["id"]} Measurements')
        axs_1.set_ylabel('Acceleration \nMeasurements ' + r'[$\frac{m}{s^2}$]')
        axs_2.set_ylabel('Angular Rate \nMeasurements [rad]')
        axs_2.set_xlabel('Time [s]')
        fig.tight_layout()
        return fig

    def plot_imu_residuals(self, imu_dfs):
        """Plot calculated IMU residuals."""
        fig, (axs_1, axs_2) = plt.subplots(2, 1)
        alpha = calculate_alpha(len(imu_dfs))
        for imu_df in imu_dfs:
            t_imu = imu_df['time'].to_list()
            axs_1.plot(t_imu, imu_df['residual_0'].to_list(), alpha=alpha, color='tab:blue')
            axs_1.plot(t_imu, imu_df['residual_1'].to_list(), alpha=alpha, color='tab:orange')
            axs_1.plot(t_imu, imu_df['residual_2'].to_list(), alpha=alpha, color='tab:green')
            axs_2.plot(t_imu, imu_df['residual_3'].to_list(), alpha=alpha, color='tab:blue')
            axs_2.plot(t_imu, imu_df['residual_4'].to_list(), alpha=alpha, color='tab:orange')
            axs_2.plot(t_imu, imu_df['residual_5'].to_list(), alpha=alpha, color='tab:green')
        set_plot_titles(fig, f'IMU {imu_df.attrs["id"]} Residuals')
        axs_1.set_ylabel('Acceleration \nResidual ' + r'[$\frac{m}{s^2}$]')
        axs_2.set_ylabel('Angular Rate \n Residual [rad]')
        axs_2.set_xlabel('Time [s]')
        fig.tight_layout()
        return fig

    def plot_imu_offset_updates(self, imu_dfs):
        """Plot IMU updates to extrinsic offsets."""
        fig, (axs_1, axs_2) = plt.subplots(2, 1)
        alpha = calculate_alpha(len(imu_dfs))
        for imu_df in imu_dfs:
            t_imu = imu_df['time'].to_list()
            axs_1.plot(t_imu, imu_df['imu_update_0'].to_list(), alpha=alpha, color='tab:blue')
            axs_1.plot(t_imu, imu_df['imu_update_1'].to_list(), alpha=alpha, color='tab:orange')
            axs_1.plot(t_imu, imu_df['imu_update_2'].to_list(), alpha=alpha, color='tab:green')
            axs_2.plot(t_imu, imu_df['imu_update_3'].to_list(), alpha=alpha, color='tab:blue')
            axs_2.plot(t_imu, imu_df['imu_update_4'].to_list(), alpha=alpha, color='tab:orange')
            axs_2.plot(t_imu, imu_df['imu_update_5'].to_list(), alpha=alpha, color='tab:green')
        set_plot_titles(fig, f'IMU {imu_df.attrs["id"]} Offset Updates')
        axs_1.set_ylabel('Position Offset\nUpdate [m]')
        axs_2.set_ylabel('Angular Offset\nUpdate [rad]')
        axs_2.set_xlabel('Time [s]')
        fig.tight_layout()
        return fig

    def plot_imu_bias_updates(self, imu_dfs):
        """Plot IMU updates to biases."""
        fig, (axs_1, axs_2) = plt.subplots(2, 1)
        alpha = calculate_alpha(len(imu_dfs))
        for imu_df in imu_dfs:
            t_imu = imu_df['time'].to_list()
            axs_1.plot(t_imu, imu_df['imu_update_6'].to_list(), alpha=alpha, color='tab:blue')
            axs_1.plot(t_imu, imu_df['imu_update_7'].to_list(), alpha=alpha, color='tab:orange')
            axs_1.plot(t_imu, imu_df['imu_update_8'].to_list(), alpha=alpha, color='tab:green')
            axs_2.plot(t_imu, imu_df['imu_update_9'].to_list(), alpha=alpha, color='tab:blue')
            axs_2.plot(t_imu, imu_df['imu_update_10'].to_list(), alpha=alpha, color='tab:orange')
            axs_2.plot(t_imu, imu_df['imu_update_11'].to_list(), alpha=alpha, color='tab:green')
        set_plot_titles(fig, f'IMU {imu_df.attrs["id"]} Bias Updates')
        axs_1.set_ylabel('Acceleration\nBias Update\n' + r'[$\frac{m}{s^2}$]')
        axs_2.set_ylabel('Angular Rate\nBias Update\n[rad/s]')
        axs_2.set_xlabel('Time [s]')
        fig.tight_layout()
        return fig

    def plot_imu_pos(self, imu_dfs):
        """TODO."""
        fig, (axs_1, axs_2, axs_3) = plt.subplots(3, 1)
        alpha = calculate_alpha(len(imu_dfs))
        t_imu = np.zeros([len(imu_dfs), len(imu_dfs[0]['time'])])
        pos_0 = np.zeros([len(imu_dfs), len(imu_dfs[0]['time'])])
        pos_1 = np.zeros([len(imu_dfs), len(imu_dfs[0]['time'])])
        pos_2 = np.zeros([len(imu_dfs), len(imu_dfs[0]['time'])])

        for i in range(len(imu_dfs)):
            t_imu[i, :] = imu_dfs[i]['time'].to_list()
            pos_0[i, :] = np.array(imu_dfs[i]['imu_pos_0'].to_list()) * 1000
            pos_1[i, :] = np.array(imu_dfs[i]['imu_pos_1'].to_list()) * 1000
            pos_2[i, :] = np.array(imu_dfs[i]['imu_pos_2'].to_list()) * 1000
            axs_1.plot(t_imu[i, :], pos_0[i, :], alpha=alpha, color='tab:blue')
            axs_2.plot(t_imu[i, :], pos_1[i, :], alpha=alpha, color='tab:orange')
            axs_3.plot(t_imu[i, :], pos_2[i, :], alpha=alpha, color='tab:green')

        pos_0_std = np.std(pos_0, axis=0)
        pos_1_std = np.std(pos_1, axis=0)
        pos_2_std = np.std(pos_2, axis=0)
        axs_1.plot(t_imu[0, :], +3.0 * pos_0_std, linestyle='--', color='tab:red', alpha=0.5)
        axs_2.plot(t_imu[0, :], +3.0 * pos_1_std, linestyle='--', color='tab:red', alpha=0.5)
        axs_3.plot(t_imu[0, :], +3.0 * pos_2_std, linestyle='--', color='tab:red', alpha=0.5)
        axs_1.plot(t_imu[0, :], -3.0 * pos_0_std, linestyle='--', color='tab:red', alpha=0.5)
        axs_2.plot(t_imu[0, :], -3.0 * pos_1_std, linestyle='--', color='tab:red', alpha=0.5)
        axs_3.plot(t_imu[0, :], -3.0 * pos_2_std, linestyle='--', color='tab:red', alpha=0.5)

        set_plot_titles(fig, f'IMU {imu_dfs[i].attrs["id"]} Position')
        axs_1.set_ylim([-25, 25])
        axs_2.set_ylim([-25, 25])
        axs_3.set_ylim([-25, 25])
        axs_1.set_ylabel('$p_x$ $[mm]$')
        axs_2.set_ylabel('$p_y$ $[mm]$')
        axs_3.set_ylabel('$p_z$ $[mm]$')
        axs_3.set_xlabel('Time [s]')
        fig.tight_layout()
        return fig

    def plot_imu_ang(self, imu_dfs):
        """Plot the body state angular error."""
        fig, (axs_1, axs_2, axs_3, axs_4) = plt.subplots(4, 1)
        alpha = calculate_alpha(len(imu_dfs))

        for i in range(len(imu_dfs)):
            time = imu_dfs[i]['time'].to_list()
            err_img_ang_w = imu_dfs[i]['imu_ang_0'].to_list()
            err_img_ang_x = imu_dfs[i]['imu_ang_1'].to_list()
            err_img_ang_y = imu_dfs[i]['imu_ang_2'].to_list()
            err_img_ang_z = imu_dfs[i]['imu_ang_3'].to_list()

            axs_1.plot(time, err_img_ang_w, alpha=alpha, color='tab:blue')
            axs_2.plot(time, err_img_ang_x, alpha=alpha, color='tab:orange')
            axs_3.plot(time, err_img_ang_y, alpha=alpha, color='tab:green')
            axs_4.plot(time, err_img_ang_z, alpha=alpha, color='tab:red')

        set_plot_titles(fig, f'IMU {imu_dfs[i].attrs["id"]} Angular Position')
        axs_1.set_ylabel('$q_w$')
        axs_2.set_ylabel('$q_x$')
        axs_3.set_ylabel('$q_y$')
        axs_4.set_ylabel('$q_z$')
        axs_4.set_xlabel('Time [s]')
        fig.tight_layout()
        return fig

    def plot_imu_bias_acc(self, imu_dfs):
        """TODO."""
        fig, (axs_1, axs_2, axs_3) = plt.subplots(3, 1)
        alpha = calculate_alpha(len(imu_dfs))
        t_imu = np.zeros([len(imu_dfs), len(imu_dfs[0]['time'])])
        acc_bias_0 = np.zeros([len(imu_dfs), len(imu_dfs[0]['time'])])
        acc_bias_1 = np.zeros([len(imu_dfs), len(imu_dfs[0]['time'])])
        acc_bias_2 = np.zeros([len(imu_dfs), len(imu_dfs[0]['time'])])

        for i in range(len(imu_dfs)):
            t_imu[i, :] = imu_dfs[i]['time'].to_list()
            acc_bias_0[i, :] = np.array(imu_dfs[i]['imu_acc_bias_0'].to_list()) * 1000
            acc_bias_1[i, :] = np.array(imu_dfs[i]['imu_acc_bias_1'].to_list()) * 1000
            acc_bias_2[i, :] = np.array(imu_dfs[i]['imu_acc_bias_2'].to_list()) * 1000
            axs_1.plot(t_imu[i, :], acc_bias_0[i, :], alpha=alpha, color='tab:blue')
            axs_2.plot(t_imu[i, :], acc_bias_1[i, :], alpha=alpha, color='tab:orange')
            axs_3.plot(t_imu[i, :], acc_bias_2[i, :], alpha=alpha, color='tab:green')

        acc_bias_0_std = np.std(acc_bias_0, axis=0)
        acc_bias_1_std = np.std(acc_bias_1, axis=0)
        acc_bias_2_std = np.std(acc_bias_2, axis=0)
        axs_1.plot(t_imu[0, :], +3.0 * acc_bias_0_std, linestyle='--', color='tab:red', alpha=0.5)
        axs_2.plot(t_imu[0, :], +3.0 * acc_bias_1_std, linestyle='--', color='tab:red', alpha=0.5)
        axs_3.plot(t_imu[0, :], +3.0 * acc_bias_2_std, linestyle='--', color='tab:red', alpha=0.5)
        axs_1.plot(t_imu[0, :], -3.0 * acc_bias_0_std, linestyle='--', color='tab:red', alpha=0.5)
        axs_2.plot(t_imu[0, :], -3.0 * acc_bias_1_std, linestyle='--', color='tab:red', alpha=0.5)
        axs_3.plot(t_imu[0, :], -3.0 * acc_bias_2_std, linestyle='--', color='tab:red', alpha=0.5)

        set_plot_titles(fig, f'IMU {imu_dfs[i].attrs["id"]} Accelerometer Bias')
        axs_1.set_ylim([-25, 25])
        axs_2.set_ylim([-25, 25])
        axs_3.set_ylim([-25, 25])
        axs_1.set_ylabel(r'$b_{a_x}$ $\left[ \frac{mm}{s^2} \right]$')
        axs_2.set_ylabel(r'$b_{a_y}$ $\left[ \frac{mm}{s^2} \right]$')
        axs_3.set_ylabel(r'$b_{a_z}$ $\left[ \frac{mm}{s^2} \right]$')
        axs_3.set_xlabel('Time [s]')
        fig.tight_layout()
        return fig

    def plot_imu_bias_omg(self, imu_dfs):
        """TODO."""
        fig, (axs_1, axs_2, axs_3) = plt.subplots(3, 1)
        alpha = calculate_alpha(len(imu_dfs))
        t_imu = np.zeros([len(imu_dfs), len(imu_dfs[0]['time'])])
        omg_bias_0 = np.zeros([len(imu_dfs), len(imu_dfs[0]['time'])])
        omg_bias_1 = np.zeros([len(imu_dfs), len(imu_dfs[0]['time'])])
        omg_bias_2 = np.zeros([len(imu_dfs), len(imu_dfs[0]['time'])])

        for i in range(len(imu_dfs)):
            t_imu[i, :] = imu_dfs[i]['time'].to_list()
            omg_bias_0[i, :] = np.array(imu_dfs[i]['imu_omg_bias_0'].to_list()) * 1000
            omg_bias_1[i, :] = np.array(imu_dfs[i]['imu_omg_bias_1'].to_list()) * 1000
            omg_bias_2[i, :] = np.array(imu_dfs[i]['imu_omg_bias_2'].to_list()) * 1000
            axs_1.plot(t_imu[i, :], omg_bias_0[i, :], alpha=alpha, color='tab:blue')
            axs_2.plot(t_imu[i, :], omg_bias_1[i, :], alpha=alpha, color='tab:orange')
            axs_3.plot(t_imu[i, :], omg_bias_2[i, :], alpha=alpha, color='tab:green')

        omg_bias_0_std = np.std(omg_bias_0, axis=0)
        omg_bias_1_std = np.std(omg_bias_1, axis=0)
        omg_bias_2_std = np.std(omg_bias_2, axis=0)
        axs_1.plot(t_imu[0, :], +3.0 * omg_bias_0_std, linestyle='--', color='tab:red', alpha=0.5)
        axs_2.plot(t_imu[0, :], +3.0 * omg_bias_1_std, linestyle='--', color='tab:red', alpha=0.5)
        axs_3.plot(t_imu[0, :], +3.0 * omg_bias_2_std, linestyle='--', color='tab:red', alpha=0.5)
        axs_1.plot(t_imu[0, :], -3.0 * omg_bias_0_std, linestyle='--', color='tab:red', alpha=0.5)
        axs_2.plot(t_imu[0, :], -3.0 * omg_bias_1_std, linestyle='--', color='tab:red', alpha=0.5)
        axs_3.plot(t_imu[0, :], -3.0 * omg_bias_2_std, linestyle='--', color='tab:red', alpha=0.5)

        set_plot_titles(fig, f'IMU {imu_dfs[i].attrs["id"]} Gyroscope Bias')
        axs_1.set_ylabel(r'$b_{\omega_x}$ $\left[ \frac{mrad}{s} \right]$')
        axs_2.set_ylabel(r'$b_{\omega_y}$ $\left[ \frac{mrad}{s} \right]$')
        axs_3.set_ylabel(r'$b_{\omega_z}$ $\left[ \frac{mrad}{s} \right]$')
        axs_3.set_xlabel('Time [s]')
        fig.tight_layout()
        return fig

    def plot_camera_body_pos_updates(self, cam_dfs):
        """Plot updates to the body position states from camera measurements."""
        fig, (axs_1, axs_2, axs_3) = plt.subplots(3, 1)
        alpha = calculate_alpha(len(cam_dfs))
        for cam_df in cam_dfs:
            time = cam_df['time'].to_list()
            axs_1.plot(time, cam_df['body_update_0'].to_list(), alpha=alpha, color='tab:blue')
            axs_1.plot(time, cam_df['body_update_1'].to_list(), alpha=alpha, color='tab:orange')
            axs_1.plot(time, cam_df['body_update_2'].to_list(), alpha=alpha, color='tab:green')
            axs_2.plot(time, cam_df['body_update_3'].to_list(), alpha=alpha, color='tab:blue')
            axs_2.plot(time, cam_df['body_update_4'].to_list(), alpha=alpha, color='tab:orange')
            axs_2.plot(time, cam_df['body_update_5'].to_list(), alpha=alpha, color='tab:green')
            axs_3.plot(time, cam_df['body_update_6'].to_list(), alpha=alpha, color='tab:blue')
            axs_3.plot(time, cam_df['body_update_7'].to_list(), alpha=alpha, color='tab:orange')
            axs_3.plot(time, cam_df['body_update_8'].to_list(), alpha=alpha, color='tab:green')
        set_plot_titles(fig, f'Camera {cam_df.attrs["id"]} Body State Position Updates')
        axs_1.set_ylabel('Position [m]')
        axs_2.set_ylabel(r'Velocity [$\frac{m}{s}$]')
        axs_3.set_ylabel(r'Acceleration [$\frac{m}{s^2}$]')
        axs_3.set_xlabel('Time [s]')
        fig.tight_layout()
        return fig

    def plot_camera_body_ang_updates(self, cam_dfs):
        """Plot updates to the body angular states from camera measurements."""
        fig, (axs_1, axs_2, axs_3) = plt.subplots(3, 1)
        alpha = calculate_alpha(len(cam_dfs))
        for cam_df in cam_dfs:
            time = cam_df['time'].to_list()
            axs_1.plot(time, cam_df['body_update_9'].to_list(), alpha=alpha, color='tab:blue')
            axs_1.plot(time, cam_df['body_update_10'].to_list(), alpha=alpha, color='tab:orange')
            axs_1.plot(time, cam_df['body_update_11'].to_list(), alpha=alpha, color='tab:green')
            axs_2.plot(time, cam_df['body_update_12'].to_list(), alpha=alpha, color='tab:blue')
            axs_2.plot(time, cam_df['body_update_13'].to_list(), alpha=alpha, color='tab:orange')
            axs_2.plot(time, cam_df['body_update_14'].to_list(), alpha=alpha, color='tab:green')
            axs_3.plot(time, cam_df['body_update_15'].to_list(), alpha=alpha, color='tab:blue')
            axs_3.plot(time, cam_df['body_update_16'].to_list(), alpha=alpha, color='tab:orange')
            axs_3.plot(time, cam_df['body_update_17'].to_list(), alpha=alpha, color='tab:green')
        set_plot_titles(fig, f'Camera {cam_df.attrs["id"]} Body State Angular Updates')
        axs_1.set_ylabel('Euler Angle\n[rad]')
        axs_2.set_ylabel('Angular Velocity\n[rad/s]')
        axs_3.set_ylabel('Angular Acceleration\n[rad/s/s]')
        axs_3.set_xlabel('Time [s]')
        fig.tight_layout()
        return fig

    def plot_camera_offset_updates(self, cam_dfs):
        """Plot camera updates to extrinsic offsets."""
        fig, (axs_1, axs_2) = plt.subplots(2, 1)
        alpha = calculate_alpha(len(cam_dfs))
        for cam_df in cam_dfs:
            t_cam = cam_df['time'].to_list()
            axs_1.plot(t_cam, cam_df['cam_update_0'].to_list(), alpha=alpha, color='tab:blue')
            axs_1.plot(t_cam, cam_df['cam_update_1'].to_list(), alpha=alpha, color='tab:orange')
            axs_1.plot(t_cam, cam_df['cam_update_2'].to_list(), alpha=alpha, color='tab:green')
            axs_2.plot(t_cam, cam_df['cam_update_3'].to_list(), alpha=alpha, color='tab:blue')
            axs_2.plot(t_cam, cam_df['cam_update_4'].to_list(), alpha=alpha, color='tab:orange')
            axs_2.plot(t_cam, cam_df['cam_update_5'].to_list(), alpha=alpha, color='tab:green')
        set_plot_titles(fig, f'Camera {cam_df.attrs["id"]} Offset Updates')
        axs_1.set_ylabel('Position Offset\nUpdates [m]')
        axs_2.set_ylabel('Angular Offset\nUpdates [rad]')
        axs_2.set_xlabel('Time [s]')
        fig.tight_layout()
        return fig

    def plot_camera_pos(self, cam_dfs):
        """Plot camera position offsets."""
        fig, (axs_1, axs_2, axs_3) = plt.subplots(3, 1)
        alpha = calculate_alpha(len(cam_dfs))
        for cam_df in cam_dfs:
            t_cam = cam_df['time'].to_list()
            axs_1.plot(t_cam, cam_df['cam_state_0'].to_list(), alpha=alpha, color='tab:blue')
            axs_2.plot(t_cam, cam_df['cam_state_1'].to_list(), alpha=alpha, color='tab:orange')
            axs_3.plot(t_cam, cam_df['cam_state_2'].to_list(), alpha=alpha, color='tab:green')
        set_plot_titles(fig, f'Camera {cam_df.attrs["id"]} Offset Position')
        axs_1.set_ylabel('X [m]')
        axs_2.set_ylabel('Y [m]')
        axs_3.set_ylabel('Z [m]')
        axs_3.set_xlabel('Time [s]')
        fig.tight_layout()
        return fig

    def plot_camera_ang(self, cam_dfs):
        """Plot camera angular offsets."""
        fig, (axs_1, axs_2, axs_3) = plt.subplots(3, 1)
        alpha = calculate_alpha(len(cam_dfs))
        for cam_df in cam_dfs:
            t_cam = cam_df['time'].to_list()
            axs_1.plot(t_cam, cam_df['cam_state_3'].to_list(), alpha=alpha, color='tab:blue')
            axs_2.plot(t_cam, cam_df['cam_state_4'].to_list(), alpha=alpha, color='tab:orange')
            axs_3.plot(t_cam, cam_df['cam_state_5'].to_list(), alpha=alpha, color='tab:green')
        set_plot_titles(fig, f'Camera {cam_df.attrs["id"]} Offset Rotations')
        axs_1.set_ylabel(r'$\theta_x$ [rad]')
        axs_2.set_ylabel(r'$\theta_y$ [rad]')
        axs_3.set_ylabel(r'$\theta_z$ [rad]')
        axs_3.set_xlabel('Time [s]')
        fig.tight_layout()
        return fig

    def plot_update_timing(self, data_frames, rate=None):
        """Plot histogram of update execution durations."""
        durations = np.array([])
        for df in data_frames:
            durations = np.append(durations, df['time_0'])
        fig, (axs) = plt.subplots(1, 1)
        counts, bins = np.histogram(durations / 1e6)
        axs.hist(bins[:-1], bins, weights=counts)
        if rate and self.rate_line:
            axs.axvline(x=1000.0 / rate, color='red', linestyle='--')
        axs.set_ylabel('Count')
        df_prefix = df.attrs['prefix']
        df_id = str(df.attrs['id'])
        set_plot_titles(fig, f'{df_prefix} {df_id} Update Time')
        axs.set_xlabel('Duration [ms]')
        fig.tight_layout()
        return fig

    def plot_body_pos(self, body_dfs):
        """Plot body position."""
        fig, (axs_1, axs_2, axs_3) = plt.subplots(3, 1)
        alpha = calculate_alpha(len(body_dfs))
        for body_df in body_dfs:
            time = body_df['time'].to_list()
            axs_1.plot(time, body_df['body_pos_0'].to_list(), alpha=alpha, color='tab:blue')
            axs_2.plot(time, body_df['body_pos_1'].to_list(), alpha=alpha, color='tab:orange')
            axs_3.plot(time, body_df['body_pos_2'].to_list(), alpha=alpha, color='tab:green')
        set_plot_titles(fig, 'Body Position')
        axs_1.set_ylabel('X [m]')
        axs_2.set_ylabel('Y [m]')
        axs_3.set_ylabel('Z [m]')
        axs_3.set_xlabel('Time [s]')
        fig.tight_layout()
        return fig

    def plot_body_pos_3d(self, body_dfs):
        """Plot body position in 3D."""
        fig = plt.figure()
        axs = fig.add_subplot(projection='3d')
        alpha = calculate_alpha(len(body_dfs))
        for body_df in body_dfs:
            x_pos = body_df['body_pos_0'].to_list()
            y_pos = body_df['body_pos_1'].to_list()
            z_pos = body_df['body_pos_2'].to_list()
            axs.plot(x_pos, y_pos, z_pos, alpha=alpha, color='tab:blue')
        set_plot_titles(fig, 'Body Position 3D')
        axs.set_xlabel('X [m]')
        axs.set_ylabel('Y [m]')
        axs.set_zlabel('Z [m]')
        fig.tight_layout()
        return fig

    def update_3d_pos_graph(
            self,
            num,
            graph_list,
            time_list,
            x_pos_list,
            y_pos_list,
            z_pos_list,
            frame_count):
        """3D body animation update function."""
        n_points = int(math.floor(len(x_pos_list[0]) * num / frame_count))
        for i in range(len(graph_list)):
            graph_list[i].set_data(x_pos_list[i][0:n_points], y_pos_list[i][0:n_points])
            graph_list[i].set_3d_properties(z_pos_list[i][0:n_points])
        # legend.get_texts()[0].set_text('t = {: 3.2f}'.format(t[n_points]))
        return graph_list

    def plot_body_pos_3d_anim(self, body_dfs, duration: float = 2):
        """Plot animated body position in 3D."""
        fig = plt.figure()
        axs = fig.add_subplot(111, projection='3d')

        # TODO(jhartzer): get from input
        # TODO(jhartzer): Add current time to plot
        interval = 40  # ms
        frame_count = interval * duration

        time_list = []
        x_pos_list = []
        y_pos_list = []
        z_pos_list = []
        graph_list = []
        alpha = calculate_alpha(len(body_dfs))
        for body_df in body_dfs:
            time = body_df['time'].to_list()
            x_pos = body_df['body_pos_0'].to_list()
            y_pos = body_df['body_pos_1'].to_list()
            z_pos = body_df['body_pos_2'].to_list()
            graph, = axs.plot(x_pos, y_pos, z_pos, alpha=alpha, color='tab:blue')
            time_list.append(time)
            x_pos_list.append(x_pos)
            y_pos_list.append(y_pos)
            z_pos_list.append(z_pos)
            graph_list.append(graph)

        title = axs.set_title('Body Position 3D')
        title.set_text('Body Position 3D')
        axs.set_xlabel('X [m]')
        axs.set_ylabel('Y [m]')
        axs.set_zlabel('Z [m]')
        ani = animation.FuncAnimation(
            fig,
            functools.partial(
                self.update_3d_pos_graph,
                graph_list=graph_list,
                time_list=time_list,
                x_pos_list=x_pos_list,
                y_pos_list=y_pos_list,
                z_pos_list=z_pos_list,
                frame_count=frame_count),
            frame_count,
            interval=interval,
            blit=True)
        name = 'Body Position 3D'
        ani._title = name
        fig.canvas.manager.set_window_title(name)
        return ani

    def plot_body_vel(self, body_dfs):
        """Plot body velocity."""
        fig, (axs_1, axs_2, axs_3) = plt.subplots(3, 1)
        alpha = calculate_alpha(len(body_dfs))
        for body_df in body_dfs:
            time = body_df['time'].to_list()
            axs_1.plot(time, body_df['body_vel_0'].to_list(), alpha=alpha, color='tab:blue')
            axs_2.plot(time, body_df['body_vel_1'].to_list(), alpha=alpha, color='tab:orange')
            axs_3.plot(time, body_df['body_vel_2'].to_list(), alpha=alpha, color='tab:green')
        set_plot_titles(fig, 'Body Velocity')
        axs_1.set_ylabel(r'X [$\frac{m}{s}$]')
        axs_2.set_ylabel(r'Y [$\frac{m}{s}$]')
        axs_3.set_ylabel(r'Z [$\frac{m}{s}$]')
        axs_3.set_xlabel('Time [s]')
        fig.tight_layout()
        return fig

    def plot_body_acc(self, body_dfs):
        """Plot body acceleration."""
        fig, (axs_1, axs_2, axs_3) = plt.subplots(3, 1)
        alpha = calculate_alpha(len(body_dfs))
        for body_df in body_dfs:
            time = body_df['time'].to_list()
            axs_1.plot(time, body_df['body_acc_0'].to_list(), alpha=alpha, color='tab:blue')
            axs_2.plot(time, body_df['body_acc_1'].to_list(), alpha=alpha, color='tab:orange')
            axs_3.plot(time, body_df['body_acc_2'].to_list(), alpha=alpha, color='tab:green')
        set_plot_titles(fig, 'Body Acceleration')
        axs_1.set_ylabel(r'X [$\frac{m}{s^2}$]')
        axs_2.set_ylabel(r'Y [$\frac{m}{s^2}$]')
        axs_3.set_ylabel(r'Z [$\frac{m}{s^2}$]')
        axs_3.set_xlabel('Time [s]')
        fig.tight_layout()
        return fig

    def plot_body_ang(self, body_dfs):
        """Plot body angular position."""
        fig, (axs_1, axs_2, axs_3, axs_4) = plt.subplots(4, 1)
        alpha = calculate_alpha(len(body_dfs))
        for body_df in body_dfs:
            time = body_df['time'].to_list()
            axs_1.plot(time, body_df['body_ang_pos_0'].to_list(), alpha=alpha, color='tab:blue')
            axs_2.plot(time, body_df['body_ang_pos_1'].to_list(), alpha=alpha, color='tab:orange')
            axs_3.plot(time, body_df['body_ang_pos_2'].to_list(), alpha=alpha, color='tab:green')
            axs_4.plot(time, body_df['body_ang_pos_3'].to_list(), alpha=alpha, color='tab:red')
        set_plot_titles(fig, 'Body Angles')
        axs_1.set_ylabel('W')
        axs_2.set_ylabel('X')
        axs_3.set_ylabel('Y')
        axs_4.set_ylabel('Z')
        axs_4.set_xlabel('Time [s]')
        fig.tight_layout()
        return fig

    def plot_body_ang_vel(self, body_dfs):
        """Plot body angular position."""
        fig, (axs_1, axs_2, axs_3) = plt.subplots(3, 1)
        alpha = calculate_alpha(len(body_dfs))
        for body_df in body_dfs:
            time = body_df['time'].to_list()
            axs_1.plot(time, body_df['body_ang_vel_0'].to_list(), alpha=alpha, color='tab:blue')
            axs_2.plot(time, body_df['body_ang_vel_1'].to_list(), alpha=alpha, color='tab:orange')
            axs_3.plot(time, body_df['body_ang_vel_2'].to_list(), alpha=alpha, color='tab:green')
        set_plot_titles(fig, 'Body Angular Velocity')
        axs_1.set_ylabel('X [rad/s]')
        axs_2.set_ylabel('Y [rad/s]')
        axs_3.set_ylabel('Z [rad/s]')
        axs_3.set_xlabel('Time [s]')
        fig.tight_layout()
        return fig

    def plot_body_ang_acc(self, body_dfs):
        """Plot body angular position."""
        fig, (axs_1, axs_2, axs_3) = plt.subplots(3, 1)
        alpha = calculate_alpha(len(body_dfs))
        for body_df in body_dfs:
            time = body_df['time'].to_list()
            axs_1.plot(time, body_df['body_ang_acc_0'].to_list(), alpha=alpha, color='tab:blue')
            axs_2.plot(time, body_df['body_ang_acc_1'].to_list(), alpha=alpha, color='tab:orange')
            axs_3.plot(time, body_df['body_ang_acc_2'].to_list(), alpha=alpha, color='tab:green')
        set_plot_titles(fig, 'Body Angular Acceleration')
        axs_1.set_ylabel('X [rad/s/s]')
        axs_2.set_ylabel('Y [rad/s/s]')
        axs_3.set_ylabel('Z [rad/s/s]')
        axs_3.set_xlabel('Time [s]')
        fig.tight_layout()
        return fig

    def plot_body_pos_cov(self, body_dfs):
        """Plot body covariances for position and derivatives."""
        fig, (axs_1, axs_2, axs_3) = plt.subplots(3, 1)
        alpha = calculate_alpha(len(body_dfs))
        for body_df in body_dfs:
            time = body_df['time'].to_list()
            axs_1.plot(time, body_df['body_cov_0'].to_list(), alpha=alpha, color='tab:blue')
            axs_1.plot(time, body_df['body_cov_1'].to_list(), alpha=alpha, color='tab:orange')
            axs_1.plot(time, body_df['body_cov_2'].to_list(), alpha=alpha, color='tab:green')
            axs_2.plot(time, body_df['body_cov_3'].to_list(), alpha=alpha, color='tab:blue')
            axs_2.plot(time, body_df['body_cov_4'].to_list(), alpha=alpha, color='tab:orange')
            axs_2.plot(time, body_df['body_cov_5'].to_list(), alpha=alpha, color='tab:green')
            axs_3.plot(time, body_df['body_cov_6'].to_list(), alpha=alpha, color='tab:blue')
            axs_3.plot(time, body_df['body_cov_7'].to_list(), alpha=alpha, color='tab:orange')
            axs_3.plot(time, body_df['body_cov_8'].to_list(), alpha=alpha, color='tab:green')
        set_plot_titles(fig, 'Body Position Covariance')
        axs_1.set_ylabel('Position [m]')
        axs_2.set_ylabel(r'Velocity [$\frac{m}{s}$]')
        axs_3.set_ylabel(r'Acceleration [$\frac{m}{s^2}$]')
        axs_3.set_xlabel('Time [s]')
        fig.tight_layout()
        return fig

    def plot_body_ang_cov(self, body_dfs):
        """Plot body covariances for angles and derivatives."""
        fig, (axs_1, axs_2, axs_3) = plt.subplots(3, 1)
        alpha = calculate_alpha(len(body_dfs))
        for body_df in body_dfs:
            time = body_df['time'].to_list()
            axs_1.plot(time, body_df['body_cov_9'].to_list(), alpha=alpha, color='tab:blue')
            axs_1.plot(time, body_df['body_cov_10'].to_list(), alpha=alpha, color='tab:orange')
            axs_1.plot(time, body_df['body_cov_11'].to_list(), alpha=alpha, color='tab:green')
            axs_2.plot(time, body_df['body_cov_12'].to_list(), alpha=alpha, color='tab:blue')
            axs_2.plot(time, body_df['body_cov_13'].to_list(), alpha=alpha, color='tab:orange')
            axs_2.plot(time, body_df['body_cov_14'].to_list(), alpha=alpha, color='tab:green')
            axs_3.plot(time, body_df['body_cov_15'].to_list(), alpha=alpha, color='tab:blue')
            axs_3.plot(time, body_df['body_cov_16'].to_list(), alpha=alpha, color='tab:orange')
            axs_3.plot(time, body_df['body_cov_17'].to_list(), alpha=alpha, color='tab:green')
        set_plot_titles(fig, 'Body Angular Covariance')
        axs_1.set_ylabel('Orientation\n[rad]')
        axs_2.set_ylabel('Angular Rate\n[rad/s]')
        axs_3.set_ylabel('Angular Acceleration\n[rad/s/s]')
        axs_3.set_xlabel('Time [s]')
        fig.tight_layout()
        return fig

    def plot_body_err_pos(self, body_state_dfs, body_truth_dfs):
        """Plot the body state position error."""
        fig, (axs_1, axs_2, axs_3) = plt.subplots(3, 1)
        alpha = calculate_alpha(len(body_state_dfs))
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
            self.statistics['Body Position'].append(
                RMSE_from_vectors(err_pos_0, err_pos_1, err_pos_2))

            axs_1.plot(est_time, err_pos_0, alpha=alpha, color='tab:blue')
            axs_2.plot(est_time, err_pos_1, alpha=alpha, color='tab:orange')
            axs_3.plot(est_time, err_pos_2, alpha=alpha, color='tab:green')

        set_plot_titles(fig, 'Body Position Error')
        axs_1.set_ylabel('X Error [m]')
        axs_2.set_ylabel('Y Error [m]')
        axs_3.set_ylabel('Z Error [m]')
        axs_3.set_xlabel('Time [s]')
        fig.tight_layout()

        return fig

    def plot_body_err_vel(self, body_state_dfs, body_truth_dfs):
        """Plot the body state velocity error."""
        fig, (axs_1, axs_2, axs_3) = plt.subplots(3, 1)
        alpha = calculate_alpha(len(body_state_dfs))
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
            self.statistics['Body Velocity'].append(
                RMSE_from_vectors(err_vel_0, err_vel_1, err_vel_2))

            axs_1.plot(est_time, err_vel_0, alpha=alpha, color='tab:blue')
            axs_2.plot(est_time, err_vel_1, alpha=alpha, color='tab:orange')
            axs_3.plot(est_time, err_vel_2, alpha=alpha, color='tab:green')

        set_plot_titles(fig, 'Body Velocity Error')
        axs_1.set_ylabel(r'X Error [$\frac{m}{s}$]')
        axs_2.set_ylabel(r'Y Error [$\frac{m}{s}$]')
        axs_3.set_ylabel(r'Z Error [$\frac{m}{s}$]')
        axs_3.set_xlabel('Time [s]')
        fig.tight_layout()

        return fig

    def plot_body_err_acc(self, body_state_dfs, body_truth_dfs):
        """Plot the body state acceleration error."""
        fig, (axs_1, axs_2, axs_3) = plt.subplots(3, 1)
        alpha = calculate_alpha(len(body_state_dfs))
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
            self.statistics['Body Acceleration'].append(
                RMSE_from_vectors(err_acc_0, err_acc_1, err_acc_2))

            axs_1.plot(est_time, err_acc_0, alpha=alpha, color='tab:blue')
            axs_2.plot(est_time, err_acc_1, alpha=alpha, color='tab:orange')
            axs_3.plot(est_time, err_acc_2, alpha=alpha, color='tab:green')

        set_plot_titles(fig, 'Body Acceleration Error')
        axs_1.set_ylabel(r'X Error [$\frac{m}{s^2}$]')
        axs_2.set_ylabel(r'Y Error [$\frac{m}{s^2}$]')
        axs_3.set_ylabel(r'Z Error [$\frac{m}{s^2}$]')
        axs_3.set_xlabel('Time [s]')
        fig.tight_layout()

        return fig

    def plot_body_err_ang(self, body_state_dfs, body_truth_dfs):
        """Plot the body state angular error."""
        fig, (axs_1, axs_2, axs_3, axs_4) = plt.subplots(4, 1)
        alpha = calculate_alpha(len(body_state_dfs))
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
            est_ang_pos_r = lists_to_rot(
                est_ang_pos_w, est_ang_pos_x, est_ang_pos_y, est_ang_pos_z)

            interp_w = np.interp(est_time, true_time, true_ang_pos_w)
            interp_x = np.interp(est_time, true_time, true_ang_pos_x)
            interp_y = np.interp(est_time, true_time, true_ang_pos_y)
            interp_z = np.interp(est_time, true_time, true_ang_pos_z)
            interp_r = lists_to_rot(interp_w, interp_x, interp_y, interp_z)

            err_ang_pos_w, err_ang_pos_x, err_ang_pos_y, err_ang_pos_z = \
                calculate_rotation_errors(est_ang_pos_r, interp_r)

            axs_1.plot(est_time, err_ang_pos_w, alpha=alpha, color='tab:blue')
            axs_2.plot(est_time, err_ang_pos_x, alpha=alpha, color='tab:orange')
            axs_3.plot(est_time, err_ang_pos_y, alpha=alpha, color='tab:green')
            axs_4.plot(est_time, err_ang_pos_z, alpha=alpha, color='tab:red')

        set_plot_titles(fig, 'Body Angular Error')
        axs_1.set_ylabel('W Error')
        axs_2.set_ylabel('X Error')
        axs_3.set_ylabel('Y Error')
        axs_4.set_ylabel('Z Error')
        axs_4.set_xlabel('Time [s]')
        fig.tight_layout()

        return fig

    def plot_body_err_ang_vel(self, body_state_dfs, body_truth_dfs):
        """Plot the body state angular velocity error."""
        fig, (axs_1, axs_2, axs_3) = plt.subplots(3, 1)
        alpha = calculate_alpha(len(body_state_dfs))
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
            self.statistics['Body Angular Velocity'].append(
                RMSE_from_vectors(err_ang_vel_0, err_ang_vel_1, err_ang_vel_2))

            axs_1.plot(est_time, err_ang_vel_0, alpha=alpha, color='tab:blue')
            axs_2.plot(est_time, err_ang_vel_1, alpha=alpha, color='tab:orange')
            axs_3.plot(est_time, err_ang_vel_2, alpha=alpha, color='tab:green')

        set_plot_titles(fig, 'Body Angular Rate Error')
        axs_1.set_ylabel('X Error [rad/s]')
        axs_2.set_ylabel('Y Error [rad/s]')
        axs_3.set_ylabel('Z Error [rad/s]')
        axs_3.set_xlabel('Time [s]')
        fig.tight_layout()

        return fig

    def plot_body_err_ang_acc(self, body_state_dfs, body_truth_dfs):
        """Plot the body state angular acceleration error."""
        fig, (axs_1, axs_2, axs_3) = plt.subplots(3, 1)
        alpha = calculate_alpha(len(body_state_dfs))
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
            self.statistics['Body Angular Acceleration'].append(
                RMSE_from_vectors(err_ang_acc_0, err_ang_acc_1, err_ang_acc_2))

            axs_1.plot(est_time, err_ang_acc_0, alpha=alpha, color='tab:blue')
            axs_2.plot(est_time, err_ang_acc_1, alpha=alpha, color='tab:orange')
            axs_3.plot(est_time, err_ang_acc_2, alpha=alpha, color='tab:green')

        set_plot_titles(fig, 'Body Angular Acceleration Error')
        axs_1.set_ylabel('X Error [rad/s/s]')
        axs_2.set_ylabel('Y Error [rad/s/s]')
        axs_3.set_ylabel('Z Error [rad/s/s]')
        axs_3.set_xlabel('Time [s]')
        fig.tight_layout()

        return fig

    # TODO include camera ID
    def plot_triangulation_error(self, tri_dfs, feat_dfs):
        """Plot MSCKF feature point triangulation error."""
        fig, (axs_1, axs_2, axs_3) = plt.subplots(3, 1)

        err_x = collections.defaultdict(list)
        err_y = collections.defaultdict(list)
        err_z = collections.defaultdict(list)

        alpha = calculate_alpha(len(tri_dfs))
        for tri_df, feat_df in zip(tri_dfs, feat_dfs):
            time = tri_df['time'].to_list()
            feature = tri_df['feature'].to_list()
            feat_x = tri_df['x'].to_list()
            feat_y = tri_df['y'].to_list()
            feat_z = tri_df['z'].to_list()

            true_x = feat_df['x'].to_list()
            true_y = feat_df['y'].to_list()
            true_z = feat_df['z'].to_list()

            for (t, f, x, y, z) in zip(time, feature, feat_x, feat_y, feat_z):
                err_x[t].append(x - true_x[int(f)])
                err_y[t].append(y - true_y[int(f)])
                err_z[t].append(z - true_z[int(f)])
        times = []
        mean_x = []
        mean_y = []
        mean_z = []
        stddev_x = []
        stddev_y = []
        stddev_z = []

        for time in err_x:
            times.append(time)
            mean_x.append(np.mean(err_x[time]))
            mean_y.append(np.mean(err_y[time]))
            mean_z.append(np.mean(err_z[time]))
            stddev_x.append(np.std(err_x[time]))
            stddev_y.append(np.std(err_y[time]))
            stddev_z.append(np.std(err_z[time]))

        times = np.array(times)
        mean_x = np.array(mean_x)
        mean_y = np.array(mean_y)
        mean_z = np.array(mean_z)
        stddev_x = np.array(stddev_x)
        stddev_y = np.array(stddev_y)
        stddev_z = np.array(stddev_z)

        t_indices = times.argsort()
        times = times[t_indices]
        mean_x = mean_x[t_indices]
        mean_y = mean_y[t_indices]
        mean_z = mean_z[t_indices]
        stddev_x = stddev_x[t_indices]
        stddev_y = stddev_y[t_indices]
        stddev_z = stddev_z[t_indices]

        axs_1.plot(times, mean_x, color='tab:blue')
        axs_2.plot(times, mean_y, color='tab:orange')
        axs_3.plot(times, mean_z, color='tab:green')
        axs_1.fill_between(times, mean_x - stddev_x, mean_x + stddev_x,
                           alpha=alpha, color='tab:blue')
        axs_2.fill_between(times, mean_y - stddev_y, mean_y + stddev_y,
                           alpha=alpha, color='tab:orange')
        axs_3.fill_between(times, mean_z - stddev_z, mean_z + stddev_z,
                           alpha=alpha, color='tab:green')
        set_plot_titles(fig, 'Triangulation Errors')
        axs_1.set_ylabel('X Error [m]')
        axs_2.set_ylabel('Y Error [m]')
        axs_3.set_ylabel('Z Error [m]')
        axs_3.set_xlabel('Time [s]')
        return fig

    def save_figures(self, save_dir, figures):
        """Save list of figures to directory."""
        if (self.show):
            plt.show()
        for fig in figures:
            title = fig._suptitle.get_text().replace(' ', '_').lower()
            fig.savefig(os.path.join(save_dir, f'{title}.{self.ext}'), format=self.ext)
            plt.close(fig)

    def save_animations(self, save_dir, animations):
        """Save list of animations to directory."""
        if (self.show):
            plt.show()
        for ani in animations:
            title = ani._title.replace(' ', '_').lower()
            ani.save(filename=os.path.join(save_dir, f'{title}.gif'), writer='pillow')

    def plot_body_data(self, body_state_dfs, body_truth_dfs=None):
        """Generate plots for body data."""
        # TODO(jhartzer): Skip plot_body_ang_acc if prediction IMU
        figures = [
            self.plot_body_pos(body_state_dfs),
            self.plot_body_pos_3d(body_state_dfs),
            self.plot_body_vel(body_state_dfs),
            self.plot_body_acc(body_state_dfs),
            self.plot_body_ang(body_state_dfs),
            self.plot_body_ang_vel(body_state_dfs),
            self.plot_body_ang_acc(body_state_dfs),
            self.plot_body_pos_cov(body_state_dfs),
            self.plot_body_ang_cov(body_state_dfs),
            self.plot_body_err_pos(body_state_dfs, body_truth_dfs),
            self.plot_body_err_vel(body_state_dfs, body_truth_dfs),
            self.plot_body_err_acc(body_state_dfs, body_truth_dfs),
            self.plot_body_err_ang(body_state_dfs, body_truth_dfs),
            self.plot_body_err_ang_vel(body_state_dfs, body_truth_dfs),
            self.plot_body_err_ang_acc(body_state_dfs, body_truth_dfs),
        ]
        animations = [
            self.plot_body_pos_3d_anim(body_state_dfs)
        ]
        return figures, animations

    def plot_imu_data(self, imu_dfs, config_data, i):
        """Generate IMU plots from sets of dataframes."""
        figures = [
            self.plot_imu_measurements(imu_dfs),
            self.plot_imu_residuals(imu_dfs),
            self.plot_imu_offset_updates(imu_dfs),
            self.plot_imu_bias_updates(imu_dfs),
            self.plot_update_timing(imu_dfs, config_data['imu_rates'][i]),
            self.plot_imu_pos(imu_dfs),
            self.plot_imu_ang(imu_dfs),
            self.plot_imu_bias_acc(imu_dfs),
            self.plot_imu_bias_omg(imu_dfs)
        ]
        return figures

    def plot_cam_data(self, cam_dfs, config_data, i):
        """Generate camera plots from sets of dataframes."""
        figures = [
            self.plot_camera_body_pos_updates(cam_dfs),
            self.plot_camera_body_ang_updates(cam_dfs),
            self.plot_camera_offset_updates(cam_dfs),
            self.plot_camera_pos(cam_dfs),
            self.plot_camera_ang(cam_dfs),
            self.plot_update_timing(cam_dfs, config_data['camera_rates'][i])
        ]
        return figures

    def plot_triangulation_data(self, tri_dfs, feat_dfs, i):
        """Generate triangulation plots from sets of dataframes."""
        figures = [
            self.plot_triangulation_error(tri_dfs, feat_dfs)
        ]
        return figures

    # TODO(jhartzer): Split for loop into thread pool
    def plot_sim_results(self, config_sets):
        """Top level function to plot simulation results from sets of config files."""
        for config_set in config_sets:
            config_data = parse_yaml(config_set[0])

            data_dirs = [config.split('.yaml')[0] for config in config_set]
            if len(config_set) > 1:
                plot_dir = os.path.join(
                    os.path.dirname(os.path.dirname(config_set[0])), 'plots')
                stat_dir = os.path.dirname(os.path.dirname(config_set[0]))
            else:
                plot_dir = os.path.join(data_dirs[0], 'plots')
                stat_dir = data_dirs[0]

            if not os.path.isdir(plot_dir):
                os.mkdir(plot_dir)

            body_state_dfs_dict = find_and_read_data_frames(data_dirs, 'body_state')
            body_truth_dfs_dict = find_and_read_data_frames(data_dirs, 'body_truth')
            for key in body_state_dfs_dict:
                body_state_dfs = body_state_dfs_dict[key]
                body_truth_dfs = body_truth_dfs_dict[key]
                figures, animations = self.plot_body_data(body_state_dfs, body_truth_dfs)
                self.save_figures(plot_dir, figures)
                self.save_animations(plot_dir, animations)

            imu_dfs_dict = find_and_read_data_frames(data_dirs, 'imu')
            for key in imu_dfs_dict:
                imu_dfs = imu_dfs_dict[key]
                figures = self.plot_imu_data(imu_dfs, config_data, key)
                self.save_figures(plot_dir, figures)

            cam_dfs_dict = find_and_read_data_frames(data_dirs, 'camera')
            for key in cam_dfs_dict:
                cam_dfs = cam_dfs_dict[key]
                figures = self.plot_cam_data(cam_dfs, config_data, key)
                self.save_figures(plot_dir, figures)

            tri_dfs_dict = find_and_read_data_frames(data_dirs, 'triangulation')
            feat_dfs_dict = find_and_read_data_frames(data_dirs, 'feature_points')
            for key in tri_dfs_dict:
                tri_dfs = tri_dfs_dict[key]
                feat_dfs = feat_dfs_dict[0]
                figures = self.plot_triangulation_data(tri_dfs, feat_dfs, key)
                self.save_figures(plot_dir, figures)

            self.write_summary(stat_dir)

    def write_summary(self, directory):
        """Write the error summary statistics to a file."""
        with open(os.path.join(directory, 'stats.txt'), 'w') as f:
            f.write('Statistic,RMSE\n')
            for key in self.statistics:
                vals = np.array(self.statistics[key])
                mu = np.mean(vals)
                sig = np.std(vals)
                f.write('{}: {:0.4f}, {:0.4f}\n'.format(key, mu, sig))
        return


def generate_mc_lists(input_files, runs=None):
    """Generate sets of yaml configuration files to plot."""
    mc_lists = []
    for input_file in input_files:
        yaml_files = []
        with open(input_file, 'r') as input_stream:
            try:
                top_yaml = yaml.safe_load(input_stream)
                sim_yaml = top_yaml['/EkfCalNode']['ros__parameters']['sim_params']
                num_runs = sim_yaml['number_of_runs']
                if (runs):
                    num_runs = runs
                if (num_runs > 1):
                    top_name = os.path.basename(input_file).split('.yaml')[0]
                    yaml_dir = input_file.split('.yaml')[0] + os.sep
                    runs_dir = os.path.join(yaml_dir, 'runs')
                    n_digits = math.ceil(math.log10(num_runs - 1))
                    for i in range(num_runs):
                        sub_file = os.path.join(
                            runs_dir, '{}_{:0{:d}.0f}.yaml'.format(top_name, i, n_digits))
                        yaml_files.append(sub_file)
                else:
                    yaml_files.append(input_file)
            except yaml.YAMLError as exc:
                print(exc)
        mc_lists.append(yaml_files)
    return mc_lists


# TODO(jhartzer): Write tests
# TODO(jhartzer): Add flag for low-memory usage (load single df at a time)
# TODO(jhartzer): Add option for saving with no title (for papers)
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('configs', nargs='+', type=str)
    parser.add_argument('--show', action='store_true')
    parser.add_argument('--rate_line', action='store_true')
    parser.add_argument('-ext', default='png', type=str)
    parser.add_argument('-j', '--jobs', default=None, type=int)
    parser.add_argument('-n', '--runs', default=None, type=int)
    args = parser.parse_args()
    plotter = Plotter(
        show=args.show,
        ext=args.ext,
        rate_line=args.rate_line,
        jobs=args.jobs
    )
    config_files = generate_mc_lists(args.configs, runs=args.runs)
    plotter.plot_sim_results(config_files)
