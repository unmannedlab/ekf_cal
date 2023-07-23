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
import os

import matplotlib.animation as animation
import matplotlib.pyplot as plt

import numpy as np

import pandas as pd

import yaml

# import matplotlib as mpl
# mpl.rcParams['lines.marker'] = '.'
plt.style.use('ggplot')


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
    alpha = 1.0 / math.sqrt(line_count)
    return alpha


def parse_yaml(config):
    """Collect sensor configuration data from input yaml."""
    config_data = {}
    config_data['IMU_rates'] = {}
    config_data['Camera_rates'] = {}
    with open(config, 'r') as stream:
        try:
            yaml_dict = yaml.safe_load(stream)
            imu_dict = yaml_dict['/EkfCalNode']['ros__parameters']['IMU']
            cam_dict = yaml_dict['/EkfCalNode']['ros__parameters']['Camera']
            id_counter = 1
            for imu_name in imu_dict:
                config_data['IMU_rates'][id_counter] = imu_dict[imu_name]['Rate']
                id_counter += 1

            for cam_name in cam_dict:
                config_data['Camera_rates'][id_counter] = cam_dict[cam_name]['Rate']
                id_counter += 1

        except yaml.YAMLError as exc:
            print(exc)

    return config_data


def find_and_read_data_frames(directories, prefix):
    """Find matching dataframes and read using pandas."""
    data_frame_sets = collections.defaultdict(list)
    for directory in directories:
        file_paths = glob.glob(os.path.join(directory, prefix + '_*.csv'))
        for file_path in file_paths:
            file_name = os.path.basename(file_path)
            file_id = int(file_name.split('_')[-1].split('.csv')[0])
            df = pd.read_csv(file_path)
            df.attrs['prefix'] = format_prefix(prefix)
            df.attrs['id'] = file_id
            data_frame_sets[file_id].append(df)
    return data_frame_sets


class Plotter():
    """Class for containing plotting methods and options."""

    def __init__(self, no_show=False, rate_line=False, ext='png'):
        """Plotter class initializer."""
        self.no_show = no_show
        self.rate_line = rate_line
        self.ext = ext

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
        axs_1.set_ylabel('Acceleration \nMeasurements [m]')
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
        axs_1.set_ylabel('Acceleration Residual\n[m]')
        axs_2.set_ylabel('Angular Rate Residual\n[rad]')
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
        axs_1.set_ylabel('Acceleration\nBias Update\n[m/s/s]')
        axs_2.set_ylabel('Angular Rate\nBias Update\n[rad/s]')
        axs_2.set_xlabel('Time [s]')
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
        axs_2.set_ylabel('Velocity [m/s]')
        axs_3.set_ylabel('Acceleration [m/s/s]')
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

    # @todo Add update rate dashed-line from config file
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
            axs_1.plot(time, body_df['body_state_0'].to_list(), alpha=alpha, color='tab:blue')
            axs_2.plot(time, body_df['body_state_1'].to_list(), alpha=alpha, color='tab:orange')
            axs_3.plot(time, body_df['body_state_2'].to_list(), alpha=alpha, color='tab:green')
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
            x_pos = body_df['body_state_0'].to_list()
            y_pos = body_df['body_state_1'].to_list()
            z_pos = body_df['body_state_2'].to_list()
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

        # @todo(jhartzer): get from input
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
            x_pos = body_df['body_state_0'].to_list()
            y_pos = body_df['body_state_1'].to_list()
            z_pos = body_df['body_state_2'].to_list()
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
            axs_1.plot(time, body_df['body_state_3'].to_list(), alpha=alpha, color='tab:blue')
            axs_2.plot(time, body_df['body_state_4'].to_list(), alpha=alpha, color='tab:orange')
            axs_3.plot(time, body_df['body_state_5'].to_list(), alpha=alpha, color='tab:green')
        set_plot_titles(fig, 'Body Velocity')
        axs_1.set_ylabel('X [m/s]')
        axs_2.set_ylabel('Y [m/s]')
        axs_3.set_ylabel('Z [m/s]')
        axs_3.set_xlabel('Time [s]')
        fig.tight_layout()
        return fig

    def plot_body_acc(self, body_dfs):
        """Plot body acceleration."""
        fig, (axs_1, axs_2, axs_3) = plt.subplots(3, 1)
        alpha = calculate_alpha(len(body_dfs))
        for body_df in body_dfs:
            time = body_df['time'].to_list()
            axs_1.plot(time, body_df['body_state_6'].to_list(), alpha=alpha, color='tab:blue')
            axs_2.plot(time, body_df['body_state_7'].to_list(), alpha=alpha, color='tab:orange')
            axs_3.plot(time, body_df['body_state_8'].to_list(), alpha=alpha, color='tab:green')
        set_plot_titles(fig, 'Body Acceleration')
        axs_1.set_ylabel('X [m/s/s]')
        axs_2.set_ylabel('Y [m/s/s]')
        axs_3.set_ylabel('Z [m/s/s]')
        axs_3.set_xlabel('Time [s]')
        fig.tight_layout()
        return fig

    def plot_body_ang(self, body_dfs):
        """Plot body angular position."""
        fig, (axs_1, axs_2, axs_3) = plt.subplots(3, 1)
        alpha = calculate_alpha(len(body_dfs))
        for body_df in body_dfs:
            time = body_df['time'].to_list()
            axs_1.plot(time, body_df['body_state_9'].to_list(), alpha=alpha, color='tab:blue')
            axs_2.plot(time, body_df['body_state_10'].to_list(), alpha=alpha, color='tab:orange')
            axs_3.plot(time, body_df['body_state_11'].to_list(), alpha=alpha, color='tab:green')
        set_plot_titles(fig, 'Body Angles')
        axs_1.set_ylabel(r'$\theta_x$ [rad]')
        axs_2.set_ylabel(r'$\theta_y$ [rad]')
        axs_3.set_ylabel(r'$\theta_z$ [rad]')
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
        axs_2.set_ylabel('Velocity [m/s]')
        axs_3.set_ylabel('Acceleration [m/s/s]')
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

    # @todo include camera ID
    def plot_triangulation_error(self, tri_dfs, feat_dfs):
        """Plot MSCKF feature point triangulation error."""
        fig, (axs_1, axs_2, axs_3) = plt.subplots(3, 1)

        alpha = calculate_alpha(len(tri_dfs))
        for tri_df, feat_df in zip(tri_dfs, feat_dfs):
            feature = tri_df['feature'].to_list()
            feat_x = tri_df['x'].to_list()
            feat_y = tri_df['y'].to_list()
            feat_z = tri_df['z'].to_list()

            true_x = feat_df['x'].to_list()
            true_y = feat_df['y'].to_list()
            true_z = feat_df['z'].to_list()

            err_x = []
            err_y = []
            err_z = []
            for (f, x, y, z) in zip(feature, feat_x, feat_y, feat_z):
                err_x.append(x - true_x[int(f)])
                err_y.append(y - true_y[int(f)])
                err_z.append(z - true_z[int(f)])

            axs_1.plot(err_x, alpha=alpha, color='tab:blue')
            axs_2.plot(err_y, alpha=alpha, color='tab:orange')
            axs_3.plot(err_z, alpha=alpha, color='tab:green')
        set_plot_titles(fig, 'Triangulation Errors')

        return fig

    def save_figures(self, save_dir, figures):
        """Save list of figures to directory."""
        if (not self.no_show):
            plt.show()
        for fig in figures:
            title = fig._suptitle.get_text().replace(' ', '_').lower()
            fig.savefig(os.path.join(save_dir, f'{title}.{self.ext}'), format=self.ext)
            plt.close(fig)

    def save_animations(self, save_dir, animations):
        """Save list of animations to directory."""
        if (not self.no_show):
            plt.show()
        for ani in animations:
            title = ani._title.replace(' ', '_').lower()
            ani.save(filename=os.path.join(save_dir, f'{title}.gif'), writer='pillow')

    def plot_body_data(self, body_dfs):
        """Generate plots for body data."""
        figures = [
            self.plot_body_pos(body_dfs),
            self.plot_body_pos_3d(body_dfs),
            self.plot_body_vel(body_dfs),
            self.plot_body_acc(body_dfs),
            self.plot_body_ang(body_dfs),
            self.plot_body_pos_cov(body_dfs),
            self.plot_body_ang_cov(body_dfs),
        ]
        animations = [
            self.plot_body_pos_3d_anim(body_dfs)
        ]
        return figures, animations

    def plot_imu_data(self, imu_dfs, config_data, i):
        """Generate IMU plots from sets of dataframes."""
        figures = [
            self.plot_imu_measurements(imu_dfs),
            self.plot_imu_residuals(imu_dfs),
            self.plot_imu_offset_updates(imu_dfs),
            self.plot_imu_bias_updates(imu_dfs),
            self.plot_update_timing(imu_dfs, config_data['IMU_rates'][i])
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
            self.plot_update_timing(cam_dfs, config_data['Camera_rates'][i])
        ]
        return figures

    def plot_triangulation_data(self, tri_dfs, feat_dfs, i):
        """Generate triangulation plots from sets of dataframes."""
        figures = [
            self.plot_triangulation_error(tri_dfs, feat_dfs)
        ]
        return figures

    def plot_sim_results(self, config_sets):
        """Top level function to plot simulation results from sets of config files."""
        for config_set in config_sets:
            config_data = parse_yaml(config_set[0])

            data_dirs = [config.split('.yaml')[0] for config in config_set]
            if len(config_set) > 1:
                plot_dir = os.path.join(
                    os.path.dirname(os.path.dirname(config_set[0])), 'plots')
            else:
                plot_dir = os.path.join(data_dirs[0], 'plots')
            if not os.path.isdir(plot_dir):
                os.mkdir(plot_dir)

            body_dfs_dict = find_and_read_data_frames(data_dirs, 'body_state')

            # body_df = pd.read_csv(os.path.join(data_dirs[0], 'body_state.csv'))
            for key in body_dfs_dict:
                body_dfs = body_dfs_dict[key]
                figures, animations = self.plot_body_data(body_dfs)
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


def generate_mc_lists(input_files):
    """Generate sets of yaml configuration files to plot."""
    mc_lists = []
    for input_file in input_files:
        yaml_files = []
        with open(input_file, 'r') as input_stream:
            try:
                top_yaml = yaml.safe_load(input_stream)
                num_runs = top_yaml['/EkfCalNode']['ros__parameters']['SimParams']['NumberOfRuns']
                if (num_runs > 1):
                    top_name = os.path.basename(input_file).split('.yaml')[0]
                    yaml_dir = input_file.split('.yaml')[0] + os.sep
                    runs_dir = os.path.join(yaml_dir, 'runs')
                    n_digits = math.ceil(math.log10(num_runs + 1))
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


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('configs', nargs='+', type=str)
    parser.add_argument('--no_show', action='store_true')
    parser.add_argument('--rate_line', action='store_true')
    parser.add_argument('-ext', default='png', type=str)
    args = parser.parse_args()
    plotter = Plotter(no_show=args.no_show, ext=args.ext, rate_line=args.rate_line)
    config_files = generate_mc_lists(args.configs)
    plotter.plot_sim_results(config_files)
