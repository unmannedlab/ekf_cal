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

import argparse
import functools
import glob
import math
import os

import matplotlib.animation as animation
import matplotlib.pyplot as plt

import numpy as np

import pandas as pd

import yaml

plt.style.use('ggplot')


def set_plot_titles(fig, name):
    fig.suptitle(name)
    fig.canvas.manager.set_window_title(name)


def plot_imu_residuals(imu_df):
    t_imu = imu_df['time'].to_list()
    fig, (axs_1, axs_2) = plt.subplots(2, 1)
    axs_1.plot(t_imu, imu_df['residual_0'].to_list(), label=r'$p_x$')
    axs_1.plot(t_imu, imu_df['residual_1'].to_list(), label=r'$p_y$')
    axs_1.plot(t_imu, imu_df['residual_2'].to_list(), label=r'$p_z$')
    axs_2.plot(t_imu, imu_df['residual_3'].to_list(), label=r'$\theta_x$')
    axs_2.plot(t_imu, imu_df['residual_4'].to_list(), label=r'$\theta_y$')
    axs_2.plot(t_imu, imu_df['residual_5'].to_list(), label=r'$\theta_z$')
    set_plot_titles(fig, f'IMU {imu_df.attrs["id"]} Residuals')
    axs_1.set_ylabel('Position Residual\n[m]')
    axs_2.set_ylabel('Angular Residual\n[red]')
    axs_2.set_xlabel('Time [s]')
    axs_1.legend()
    axs_2.legend()
    fig.tight_layout()
    return fig


def plot_imu_offset_updates(imu_df):
    t_imu = imu_df['time'].to_list()
    fig, (axs_1, axs_2) = plt.subplots(2, 1)
    axs_1.plot(t_imu, imu_df['imu_update_0'].to_list(), label=r'$p_x$')
    axs_1.plot(t_imu, imu_df['imu_update_1'].to_list(), label=r'$p_y$')
    axs_1.plot(t_imu, imu_df['imu_update_2'].to_list(), label=r'$p_z$')
    axs_2.plot(t_imu, imu_df['imu_update_3'].to_list(), label=r'$\theta_x$')
    axs_2.plot(t_imu, imu_df['imu_update_4'].to_list(), label=r'$\theta_y$')
    axs_2.plot(t_imu, imu_df['imu_update_5'].to_list(), label=r'$\theta_z$')
    set_plot_titles(fig, f'IMU {imu_df.attrs["id"]} Offset Updates')
    axs_1.set_ylabel('Position Offset\nUpdate [m]')
    axs_2.set_ylabel('Angular Offset\nUpdate [red]')
    axs_2.set_xlabel('Time [s]')
    axs_1.legend()
    axs_2.legend()
    fig.tight_layout()
    return fig


def plot_imu_bias_updates(imu_df):
    t_imu = imu_df['time'].to_list()
    fig, (axs_1, axs_2) = plt.subplots(2, 1)
    axs_1.plot(t_imu, imu_df['imu_update_6'].to_list(), label=r'$a_x$')
    axs_1.plot(t_imu, imu_df['imu_update_7'].to_list(), label=r'$a_y$')
    axs_1.plot(t_imu, imu_df['imu_update_8'].to_list(), label=r'$a_z$')
    axs_2.plot(t_imu, imu_df['imu_update_9'].to_list(), label=r'$\omega_x$')
    axs_2.plot(t_imu, imu_df['imu_update_10'].to_list(), label=r'$\omega_y$')
    axs_2.plot(t_imu, imu_df['imu_update_11'].to_list(), label=r'$\omega_z$')
    set_plot_titles(fig, f'IMU {imu_df.attrs["id"]} Bias Updates')
    axs_1.set_ylabel('Acceleration\nBias Update\n[m/s/s]')
    axs_2.set_ylabel('Angular Rate\nBias Update\n[rad/s]')
    axs_2.set_xlabel('Time [s]')
    axs_1.legend()
    axs_2.legend()
    fig.tight_layout()
    return fig


def plot_camera_body_pos_updates(cam_df):
    fig, (axs_1, axs_2, axs_3) = plt.subplots(3, 1)
    time = cam_df['time'].to_list()
    axs_1.plot(time, cam_df['body_update_0'].to_list(), label=r'$p_x$')
    axs_1.plot(time, cam_df['body_update_1'].to_list(), label=r'$p_y$')
    axs_1.plot(time, cam_df['body_update_2'].to_list(), label=r'$p_z$')
    axs_2.plot(time, cam_df['body_update_3'].to_list(), label=r'$v_x$')
    axs_2.plot(time, cam_df['body_update_4'].to_list(), label=r'$v_y$')
    axs_2.plot(time, cam_df['body_update_5'].to_list(), label=r'$v_z$')
    axs_3.plot(time, cam_df['body_update_6'].to_list(), label=r'$a_x$')
    axs_3.plot(time, cam_df['body_update_7'].to_list(), label=r'$a_y$')
    axs_3.plot(time, cam_df['body_update_8'].to_list(), label=r'$a_z$')
    set_plot_titles(fig, 'Body State Position Updates')
    axs_1.set_ylabel('Position [m]')
    axs_2.set_ylabel('Velocity [m/s]')
    axs_3.set_ylabel('Acceleration [m/s/s]')
    axs_3.set_xlabel('Time [s]')
    axs_1.legend()
    axs_2.legend()
    axs_3.legend()
    fig.tight_layout()
    return fig


def plot_camera_body_ang_updates(cam_df):
    fig, (axs_1, axs_2, axs_3) = plt.subplots(3, 1)
    time = cam_df['time'].to_list()
    axs_1.plot(time, cam_df['body_update_9'].to_list(), label=r'$\theta_x$')
    axs_1.plot(time, cam_df['body_update_10'].to_list(), label=r'$\theta_y$')
    axs_1.plot(time, cam_df['body_update_11'].to_list(), label=r'$\theta_z$')
    axs_2.plot(time, cam_df['body_update_12'].to_list(), label=r'$\omega_x$')
    axs_2.plot(time, cam_df['body_update_13'].to_list(), label=r'$\omega_y$')
    axs_2.plot(time, cam_df['body_update_14'].to_list(), label=r'$\omega_z$')
    axs_3.plot(time, cam_df['body_update_15'].to_list(), label=r'$\alpha_x$')
    axs_3.plot(time, cam_df['body_update_16'].to_list(), label=r'$\alpha_y$')
    axs_3.plot(time, cam_df['body_update_17'].to_list(), label=r'$\alpha_z$')
    set_plot_titles(fig, 'Body State Angular Updates')
    axs_1.set_ylabel('Euler Angle\n[rad]')
    axs_2.set_ylabel('Angular Velocity\n[rad/s]')
    axs_3.set_ylabel('Angular Acceleration\n[rad/s/s]')
    axs_3.set_xlabel('Time [s]')
    axs_1.legend()
    axs_2.legend()
    axs_3.legend()
    fig.tight_layout()
    return fig


def plot_camera_offset_updates(cam_df):
    t_cam = cam_df['time'].to_list()
    fig, (axs_1, axs_2) = plt.subplots(2, 1)
    axs_1.plot(t_cam, cam_df['cam_update_0'].to_list(), label=r'$p_x$')
    axs_1.plot(t_cam, cam_df['cam_update_1'].to_list(), label=r'$p_y$')
    axs_1.plot(t_cam, cam_df['cam_update_2'].to_list(), label=r'$p_z$')
    axs_2.plot(t_cam, cam_df['cam_update_3'].to_list(), label=r'$\theta_x$')
    axs_2.plot(t_cam, cam_df['cam_update_4'].to_list(), label=r'$\theta_y$')
    axs_2.plot(t_cam, cam_df['cam_update_5'].to_list(), label=r'$\theta_z$')
    set_plot_titles(fig, f'Camera {cam_df.attrs["id"]} Offset Updates')
    axs_1.set_ylabel('Position Offset\nUpdates [m]')
    axs_2.set_ylabel('Angular Offset\nUpdates [red]')
    axs_2.set_xlabel('Time [s]')
    axs_1.legend()
    axs_2.legend()
    fig.tight_layout()
    return fig


# @todo Add update rate dashed-line from config file
def plot_update_timing(df, rate=None):
    fig, (axs) = plt.subplots(1, 1)
    counts, bins = np.histogram(df['time_0'] / 1e6)
    axs.hist(bins[:-1], bins, weights=counts)
    if rate:
        axs.axvline(x=1000.0 / rate, color='red', linestyle='--')
    axs.set_ylabel('Count')
    df_prefix = df.attrs['prefix']
    df_id = str(df.attrs['id'])
    set_plot_titles(fig, f'{df_prefix} {df_id} Update Time')
    axs.set_xlabel('Duration [ms]')
    fig.tight_layout()
    return fig


def plot_body_pos(df):
    time = df['time'].to_list()
    fig, (axs_1, axs_2, axs_3) = plt.subplots(3, 1)
    axs_1.plot(time, df['body_state_0'].to_list(), label=r'$pos_x$')
    axs_2.plot(time, df['body_state_1'].to_list(), label=r'$pos_y$')
    axs_3.plot(time, df['body_state_2'].to_list(), label=r'$pos_z$')
    set_plot_titles(fig, 'Body Position')
    axs_1.set_ylabel('X [m]')
    axs_2.set_ylabel('Y [m]')
    axs_3.set_ylabel('Z [m]')
    axs_3.set_xlabel('Time [s]')
    fig.tight_layout()
    return fig


def plot_body_pos_3d(df):
    fig = plt.figure()
    axs = fig.add_subplot(projection='3d')
    x_pos = df['body_state_0'].to_list()
    y_pos = df['body_state_1'].to_list()
    z_pos = df['body_state_2'].to_list()
    axs.plot(x_pos, y_pos, z_pos)
    set_plot_titles(fig, 'Body Position 3D')
    axs.set_xlabel('X [m]')
    axs.set_ylabel('Y [m]')
    axs.set_zlabel('Z [m]')
    fig.tight_layout()
    return fig


def update_3d_pos_graph(num, graph, legend, t, x, y, z, frame_count):
    n_points = int(math.floor(len(x) * num / frame_count))
    graph.set_data(x[0:n_points], y[0:n_points])
    graph.set_3d_properties(z[0:n_points])
    legend.get_texts()[0].set_text('t = {: 3.2f}'.format(t[n_points]))
    return legend, graph


def plot_body_pos_3d_anim(df, duration: float = 2):
    interval = 40  # ms
    frame_count = interval * duration
    time = df['time'].to_list()
    x_pos = df['body_state_0'].to_list()
    y_pos = df['body_state_1'].to_list()
    z_pos = df['body_state_2'].to_list()
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    title = ax.set_title('Body Position 3D')
    graph, = ax.plot(x_pos, y_pos, z_pos, label='t = 0.00')
    title.set_text('Body Position 3D')
    legend = ax.legend()
    ani = animation.FuncAnimation(fig, functools.partial(update_3d_pos_graph, graph=graph, legend=legend, t=time, x=x_pos, y=y_pos, z=z_pos, frame_count=frame_count), frame_count, interval=interval, blit=True)
    ani._title = 'Body Position 3D'
    return ani


def plot_body_vel(df):
    time = df['time'].to_list()
    fig, (axs_1, axs_2, axs_3) = plt.subplots(3, 1)
    axs_1.plot(time, df['body_state_3'].to_list(), label=r'$vel_x$')
    axs_2.plot(time, df['body_state_4'].to_list(), label=r'$vel_y$')
    axs_3.plot(time, df['body_state_5'].to_list(), label=r'$vel_z$')
    set_plot_titles(fig, 'Body Velocity')
    axs_1.set_ylabel('X [m/s]')
    axs_2.set_ylabel('Y [m/s]')
    axs_3.set_ylabel('Z [m/s]')
    axs_3.set_xlabel('Time [s]')
    fig.tight_layout()
    return fig


def plot_body_acc(df):
    time = df['time'].to_list()
    fig, (axs_1, axs_2, axs_3) = plt.subplots(3, 1)
    axs_1.plot(time, df['body_state_6'].to_list(), label=r'$acc_x$')
    axs_2.plot(time, df['body_state_7'].to_list(), label=r'$acc_y$')
    axs_3.plot(time, df['body_state_8'].to_list(), label=r'$acc_z$')
    set_plot_titles(fig, 'Body Acceleration')
    axs_1.set_ylabel('X [m/s/s]')
    axs_2.set_ylabel('Y [m/s/s]')
    axs_3.set_ylabel('Z [m/s/s]')
    axs_3.set_xlabel('Time [s]')
    fig.tight_layout()
    return fig


def plot_body_ang(df):
    time = df['time'].to_list()
    fig, (axs_1, axs_2, axs_3) = plt.subplots(3, 1)
    axs_1.plot(time, df['body_state_9'].to_list(), label=r'$\theta_x$')
    axs_2.plot(time, df['body_state_10'].to_list(), label=r'$\theta_y$')
    axs_3.plot(time, df['body_state_11'].to_list(), label=r'$\theta_z$')
    set_plot_titles(fig, 'Body Angles')
    axs_1.set_ylabel(r'$\theta_x$ [rad]')
    axs_2.set_ylabel(r'$\theta_y$ [rad]')
    axs_3.set_ylabel(r'$\theta_z$ [rad]')
    axs_3.set_xlabel('Time [s]')
    fig.tight_layout()
    return fig


def plot_camera_pos(cam_df):
    fig, (axs_1, axs_2, axs_3) = plt.subplots(3, 1)
    axs_1.plot(cam_df['time'].to_list(), cam_df['cam_state_0'].to_list())
    axs_2.plot(cam_df['time'].to_list(), cam_df['cam_state_1'].to_list())
    axs_3.plot(cam_df['time'].to_list(), cam_df['cam_state_2'].to_list())
    set_plot_titles(fig, f'Camera {cam_df.attrs["id"]} Offset Position')
    axs_1.set_ylabel('X [m]')
    axs_2.set_ylabel('Y [m]')
    axs_3.set_ylabel('Z [m]')
    axs_3.set_xlabel('Time [s]')
    fig.tight_layout()
    return fig


def plot_camera_ang(cam_df):
    fig, (axs_1, axs_2, axs_3) = plt.subplots(3, 1)
    axs_1.plot(cam_df['time'].to_list(), cam_df['cam_state_3'].to_list())
    axs_2.plot(cam_df['time'].to_list(), cam_df['cam_state_4'].to_list())
    axs_3.plot(cam_df['time'].to_list(), cam_df['cam_state_5'].to_list())
    set_plot_titles(fig, f'Camera {cam_df.attrs["id"]} Offset Rotations')
    axs_1.set_ylabel(r'$\theta_x$ [rad]')
    axs_2.set_ylabel(r'$\theta_y$ [rad]')
    axs_3.set_ylabel(r'$\theta_z$ [rad]')
    axs_3.set_xlabel('Time [s]')
    fig.tight_layout()
    return fig


# @todo include camera ID
def total_triangulation_error(tri_df, feat_df):
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

    fig, (axs_1, axs_2, axs_3) = plt.subplots(3, 1)
    axs_1.plot(err_x)
    axs_2.plot(err_y)
    axs_3.plot(err_z)
    set_plot_titles(fig, 'Triangulation Errors')

    return fig


def format_prefix(prefix):
    if (prefix == 'imu'):
        return 'IMU'
    elif (prefix == 'camera'):
        return 'Camera'
    else:
        return ''


def find_and_read_data_frames(directory, prefix):
    file_paths = glob.glob(os.path.join(directory, prefix + '_*.csv'))
    data_frames = []
    for file_path in file_paths:
        file_name = os.path.basename(file_path)
        file_id = file_name.split('_')[-1].split('.csv')[0]
        df = pd.read_csv(file_path)
        df.attrs['prefix'] = format_prefix(prefix)
        df.attrs['id'] = file_id
        data_frames.append(df)

    return data_frames


def parse_yaml(config):
    config_data = {}
    config_data['IMU_rates'] = []
    config_data['Camera_rates'] = []
    with open(config, 'r') as stream:
        try:
            yaml_dict = yaml.safe_load(stream)
            imu_dict = yaml_dict['/EkfCalNode']['ros__parameters']['IMU']
            cam_dict = yaml_dict['/EkfCalNode']['ros__parameters']['Camera']

            for imu_name in imu_dict:
                config_data['IMU_rates'].append(imu_dict[imu_name]['Rate'])

            for cam_name in cam_dict:
                config_data['Camera_rates'].append(cam_dict[cam_name]['Rate'])

        except yaml.YAMLError as exc:
            print(exc)

    return config_data


def save_figures(save_dir, figures, ext):
    for fig in figures:
        title = fig._suptitle.get_text().replace(' ', '_').lower()
        fig.savefig(os.path.join(save_dir, f'{title}.{ext}'), format=ext)
        plt.close(fig)


def save_animations(save_dir, animations):
    for ani in animations:
        title = ani._title.replace(' ', '_').lower()
        ani.save(filename=os.path.join(save_dir, f'{title}.gif'), writer='pillow')

def plot_body_data(body_df):
    figures = [
        plot_body_pos(body_df),
        plot_body_pos_3d(body_df),
        plot_body_vel(body_df),
        plot_body_acc(body_df),
        plot_body_ang(body_df)
    ]
    animations = [
        plot_body_pos_3d_anim(body_df)
    ]
    return figures, animations


def plot_imu_data(imu_df, config_data, i):
    figures = [
        plot_imu_residuals(imu_df),
        plot_imu_offset_updates(imu_df),
        plot_imu_bias_updates(imu_df),
        plot_update_timing(imu_df, config_data['IMU_rates'][i])
    ]
    return figures


def plot_cam_data(cam_df, config_data, i):
    figures = [
        plot_camera_body_pos_updates(cam_df),
        plot_camera_body_ang_updates(cam_df),
        plot_camera_offset_updates(cam_df),
        plot_camera_pos(cam_df),
        plot_camera_ang(cam_df),
        plot_update_timing(cam_df, config_data['Camera_rates'][i])
    ]
    return figures


def plot_triangulation_data(tri_df, feat_df, i):
    figures = [
        total_triangulation_error(tri_df, feat_df)
    ]
    return figures


def plot_sim_results(configs, no_show=False, ext='png'):
    for config in configs:
        config_data = parse_yaml(config)

        directory = config.split('.yaml')[0]
        save_dir = os.path.join(directory, 'plots')
        if not os.path.isdir(save_dir):
            os.mkdir(save_dir)

        body_df = pd.read_csv(os.path.join(directory, 'body_state.csv'))
        figures, animations = plot_body_data(body_df)
        save_figures(save_dir, figures, ext)
        save_animations(save_dir, animations)

        imu_dfs = find_and_read_data_frames(directory, 'imu')
        for i, imu_df in enumerate(imu_dfs):
            figures = plot_imu_data(imu_df, config_data, i)
            save_figures(save_dir, figures, ext)

        cam_dfs = find_and_read_data_frames(directory, 'camera')
        for i, cam_df in enumerate(cam_dfs):
            figures = plot_cam_data(cam_df, config_data, i)
            save_figures(save_dir, figures, ext)

        feat_df = pd.read_csv(os.path.join(directory, 'feature_points.csv'))
        tri_dfs = find_and_read_data_frames(directory, 'triangulation')
        for i, tri_df in enumerate(tri_dfs):
            figures = plot_triangulation_data(tri_df, feat_df, i)
            save_figures(save_dir, figures, ext)

        if (not no_show):
            plt.show()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('configs', nargs='+', type=str)
    parser.add_argument('--no_show', action='store_true')
    parser.add_argument('-ext', default='png', type=str)
    args = parser.parse_args()
    plot_sim_results(args.configs, no_show=args.no_show, ext=args.ext)
