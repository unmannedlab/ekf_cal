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
import glob
import os

import matplotlib.pyplot as plt

import pandas as pd

plt.style.use('ggplot')


def set_plot_titles(fig, name):
    fig.suptitle(name)
    fig.canvas.manager.set_window_title(name)


def plot_imu_residuals(imu_df):
    fig, (axs_1, axs_2) = plt.subplots(2, 1)
    axs_1.plot(imu_df['time'], imu_df['residual_0'], label=r'$p_x$')
    axs_1.plot(imu_df['time'], imu_df['residual_1'], label=r'$p_y$')
    axs_1.plot(imu_df['time'], imu_df['residual_2'], label=r'$p_z$')
    axs_2.plot(imu_df['time'], imu_df['residual_3'], label=r'$\theta_x$')
    axs_2.plot(imu_df['time'], imu_df['residual_4'], label=r'$\theta_y$')
    axs_2.plot(imu_df['time'], imu_df['residual_5'], label=r'$\theta_z$')
    set_plot_titles(fig, 'IMU Residuals')
    axs_1.set_ylabel('Position Residual\n[m]')
    axs_2.set_ylabel('Angular Residual\n[red]')
    axs_2.set_xlabel('Time [s]')
    axs_1.legend()
    axs_2.legend()
    fig.tight_layout()


def plot_imu_pos_updates(imu_df):
    fig, (axs_1, axs_2) = plt.subplots(2, 1)
    axs_1.plot(imu_df['time'], imu_df['imu_update_0'], label=r'$p_x$')
    axs_1.plot(imu_df['time'], imu_df['imu_update_1'], label=r'$p_y$')
    axs_1.plot(imu_df['time'], imu_df['imu_update_2'], label=r'$p_z$')
    axs_2.plot(imu_df['time'], imu_df['imu_update_3'], label=r'$\theta_x$')
    axs_2.plot(imu_df['time'], imu_df['imu_update_4'], label=r'$\theta_y$')
    axs_2.plot(imu_df['time'], imu_df['imu_update_5'], label=r'$\theta_z$')
    set_plot_titles(fig, 'IMU State Offset Updates')
    axs_1.set_ylabel('Position Offset Update\n[m]')
    axs_2.set_ylabel('Angular Offset Update\n[red]')
    axs_2.set_xlabel('Time [s]')
    axs_1.legend()
    axs_2.legend()
    fig.tight_layout()


def plot_imu_vel_updates(imu_df):
    fig, (axs_1, axs_2) = plt.subplots(2, 1)
    axs_1.plot(imu_df['time'], imu_df['imu_update_6'], label=r'$a_x$')
    axs_1.plot(imu_df['time'], imu_df['imu_update_7'], label=r'$a_y$')
    axs_1.plot(imu_df['time'], imu_df['imu_update_8'], label=r'$a_z$')
    axs_2.plot(imu_df['time'], imu_df['imu_update_9'], label=r'$\omega_x$')
    axs_2.plot(imu_df['time'], imu_df['imu_update_10'], label=r'$\omega_y$')
    axs_2.plot(imu_df['time'], imu_df['imu_update_11'], label=r'$\omega_z$')
    set_plot_titles(fig, 'IMU State Bias Updates')
    axs_1.set_ylabel('Acceleration Bias Update\n[m/s/s]')
    axs_2.set_ylabel('Angular Rate Bias Update\n[rad/s]')
    axs_2.set_xlabel('Time [s]')
    axs_1.legend()
    axs_2.legend()
    fig.tight_layout()


def plot_msckf_body_pos_updates(msckf_df):
    fig, (axs_1, axs_2, axs_3) = plt.subplots(3, 1)
    time = msckf_df['time']
    axs_1.plot(time, msckf_df['body_update_0'], label=r'$p_x$')
    axs_1.plot(time, msckf_df['body_update_1'], label=r'$p_y$')
    axs_1.plot(time, msckf_df['body_update_2'], label=r'$p_z$')
    axs_2.plot(time, msckf_df['body_update_3'], label=r'$v_x$')
    axs_2.plot(time, msckf_df['body_update_4'], label=r'$v_y$')
    axs_2.plot(time, msckf_df['body_update_5'], label=r'$v_z$')
    axs_3.plot(time, msckf_df['body_update_6'], label=r'$a_x$')
    axs_3.plot(time, msckf_df['body_update_7'], label=r'$a_y$')
    axs_3.plot(time, msckf_df['body_update_8'], label=r'$a_z$')
    set_plot_titles(fig, 'Body State Position Updates')
    axs_1.set_ylabel('Position [m]')
    axs_2.set_ylabel('Velocity [m/s]')
    axs_3.set_ylabel('Acceleration [m/s/s]')
    axs_3.set_xlabel('Time [s]')
    axs_1.legend()
    axs_2.legend()
    axs_3.legend()
    fig.tight_layout()


def plot_msckf_body_ang_updates(msckf_df):
    fig, (axs_1, axs_2, axs_3) = plt.subplots(3, 1)
    time = msckf_df['time']
    axs_1.plot(time, msckf_df['body_update_9'], label=r'$\theta_x$')
    axs_1.plot(time, msckf_df['body_update_10'], label=r'$\theta_y$')
    axs_1.plot(time, msckf_df['body_update_11'], label=r'$\theta_z$')
    axs_2.plot(time, msckf_df['body_update_12'], label=r'$\omega_x$')
    axs_2.plot(time, msckf_df['body_update_13'], label=r'$\omega_y$')
    axs_2.plot(time, msckf_df['body_update_14'], label=r'$\omega_z$')
    axs_3.plot(time, msckf_df['body_update_15'], label=r'$\alpha_x$')
    axs_3.plot(time, msckf_df['body_update_16'], label=r'$\alpha_y$')
    axs_3.plot(time, msckf_df['body_update_17'], label=r'$\alpha_z$')
    set_plot_titles(fig, 'Body State Angular Updates')
    axs_1.set_ylabel('Euler Angle [rad]')
    axs_2.set_ylabel('Angular Velocity [rad/s]')
    axs_3.set_ylabel('Angular Acceleration [rad/s/s]')
    axs_3.set_xlabel('Time [s]')
    axs_1.legend()
    axs_2.legend()
    axs_3.legend()
    fig.tight_layout()


def plot_msckf_cam_pos_updates(msckf_df):
    fig, (axs_1, axs_2, axs_3) = plt.subplots(3, 1)
    axs_1.plot(msckf_df['time'], msckf_df['cam_update_0'])
    axs_2.plot(msckf_df['time'], msckf_df['cam_update_1'])
    axs_3.plot(msckf_df['time'], msckf_df['cam_update_2'])
    set_plot_titles(fig, 'Camera Position Offset Updates')
    axs_1.set_ylabel('X Update [m]')
    axs_2.set_ylabel('Y Update [m]')
    axs_3.set_ylabel('Z Update [m]')
    axs_3.set_xlabel('Time [s]')
    fig.tight_layout()


def plot_msckf_cam_ang_updates(msckf_df):
    fig, (axs_1, axs_2, axs_3) = plt.subplots(3, 1)
    axs_1.plot(msckf_df['time'], msckf_df['cam_update_3'])
    axs_2.plot(msckf_df['time'], msckf_df['cam_update_4'])
    axs_3.plot(msckf_df['time'], msckf_df['cam_update_5'])
    set_plot_titles(fig, 'Camera Angular Offset Updates')
    axs_1.set_ylabel(r'$\theta_x$ Update [rad]')
    axs_2.set_ylabel(r'$\theta_y$ Update [rad]')
    axs_3.set_ylabel(r'$\theta_z$ Update [rad]')
    axs_3.set_xlabel('Time [s]')
    fig.tight_layout()


def plot_update_timing(df):
    fig, (axs_1) = plt.subplots(1, 1)
    axs_1.hist(df['time_0'] / 1e3, bins=10)
    axs_1.set_ylabel('Count')
    df_prefix = df.attrs['prefix']
    df_id = str(df.attrs['id'])
    set_plot_titles(fig, f'{df_prefix} {df_id} Update Time')
    axs_1.set_xlabel('Duration [ms]')
    fig.tight_layout()


def plot_body_pos(df):
    fig, (axs_1, axs_2, axs_3) = plt.subplots(3, 1)
    axs_1.plot(df['time'], df['body_state_0'], label=r'$pos_x$')
    axs_2.plot(df['time'], df['body_state_1'], label=r'$pos_y$')
    axs_3.plot(df['time'], df['body_state_2'], label=r'$pos_z$')
    set_plot_titles(fig, 'Body Position')
    axs_1.set_ylabel('X [m]')
    axs_2.set_ylabel('Y [m]')
    axs_3.set_ylabel('Z [m]')
    axs_3.set_xlabel('Time [s]')
    fig.tight_layout()


def plot_body_vel(df):
    fig, (axs_1, axs_2, axs_3) = plt.subplots(3, 1)
    axs_1.plot(df['time'], df['body_state_3'], label=r'$vel_x$')
    axs_2.plot(df['time'], df['body_state_4'], label=r'$vel_y$')
    axs_3.plot(df['time'], df['body_state_5'], label=r'$vel_z$')
    set_plot_titles(fig, 'Body Velocity')
    axs_1.set_ylabel('X [m/s]')
    axs_2.set_ylabel('Y [m/s]')
    axs_3.set_ylabel('Z [m/s]')
    axs_3.set_xlabel('Time [s]')
    fig.tight_layout()


def plot_body_acc(df):
    fig, (axs_1, axs_2, axs_3) = plt.subplots(3, 1)
    axs_1.plot(df['time'], df['body_state_6'], label=r'$acc_x$')
    axs_2.plot(df['time'], df['body_state_7'], label=r'$acc_y$')
    axs_3.plot(df['time'], df['body_state_8'], label=r'$acc_z$')
    set_plot_titles(fig, 'Body Acceleration')
    axs_1.set_ylabel('X [m/s/s]')
    axs_2.set_ylabel('Y [m/s/s]')
    axs_3.set_ylabel('Z [m/s/s]')
    axs_3.set_xlabel('Time [s]')
    fig.tight_layout()


def plot_body_ang(df):
    fig, (axs_1, axs_2, axs_3) = plt.subplots(3, 1)
    axs_1.plot(df['time'], df['body_state_9'], label=r'$\theta_x$')
    axs_2.plot(df['time'], df['body_state_10'], label=r'$\theta_y$')
    axs_3.plot(df['time'], df['body_state_11'], label=r'$\theta_z$')
    set_plot_titles(fig, 'Body Angles')
    axs_1.set_ylabel(r'$\theta_x$ [rad]')
    axs_2.set_ylabel(r'$\theta_y$ [rad]')
    axs_3.set_ylabel(r'$\theta_z$ [rad]')
    axs_3.set_xlabel('Time [s]')
    fig.tight_layout()


def plot_msckf_cam_pos(msckf_df):
    fig, (axs_1, axs_2, axs_3) = plt.subplots(3, 1)
    axs_1.plot(msckf_df['time'], msckf_df['cam_state_0'])
    axs_2.plot(msckf_df['time'], msckf_df['cam_state_1'])
    axs_3.plot(msckf_df['time'], msckf_df['cam_state_2'])
    set_plot_titles(fig, 'Camera Offset Position')
    axs_1.set_ylabel('X [m]')
    axs_2.set_ylabel('Y [m]')
    axs_3.set_ylabel('Z [m]')
    axs_3.set_xlabel('Time [s]')
    fig.tight_layout()


def plot_msckf_cam_ang(msckf_df):
    fig, (axs_1, axs_2, axs_3) = plt.subplots(3, 1)
    axs_1.plot(msckf_df['time'], msckf_df['cam_state_3'])
    axs_2.plot(msckf_df['time'], msckf_df['cam_state_4'])
    axs_3.plot(msckf_df['time'], msckf_df['cam_state_5'])
    set_plot_titles(fig, 'Camera Offset Rotations')
    axs_1.set_ylabel(r'$\theta_x$ [rad]')
    axs_2.set_ylabel(r'$\theta_y$ [rad]')
    axs_3.set_ylabel(r'$\theta_z$ [rad]')
    axs_3.set_xlabel('Time [s]')
    fig.tight_layout()


def find_and_read_data_frames(directory, prefix):
    file_paths = glob.glob(os.path.join(directory, prefix + '_*.csv'))
    data_frames = []
    for file_path in file_paths:
        file_name = os.path.basename(file_path)
        file_id = file_name.split('_')[-1].split('.csv')[0]
        df = pd.read_csv(file_path)
        df.attrs['prefix'] = prefix
        df.attrs['id'] = file_id
        data_frames.append(df)

    return data_frames


def plot_sim_results(directories, no_show):
    for directory in directories:

        imu_dfs = find_and_read_data_frames(directory, 'imu')
        msckf_dfs = find_and_read_data_frames(directory, 'msckf')

        for imu_df in imu_dfs:
            plot_imu_residuals(imu_df)
            plot_imu_pos_updates(imu_df)
            plot_imu_vel_updates(imu_df)
            plot_update_timing(imu_df)
            plot_body_pos(imu_df)
            plot_body_vel(imu_df)
            plot_body_acc(imu_df)
            plot_body_ang(imu_df)

        for msckf_df in msckf_dfs:
            plot_msckf_body_pos_updates(msckf_df)
            plot_msckf_body_ang_updates(msckf_df)
            plot_msckf_cam_pos_updates(msckf_df)
            plot_msckf_cam_ang_updates(msckf_df)
            plot_msckf_cam_pos(msckf_df)
            plot_msckf_cam_ang(msckf_df)
            plot_update_timing(msckf_df)

        saveDir = os.path.join(directory, 'plots')
        if not os.path.isdir(saveDir):
            os.mkdir(saveDir)

        for i in plt.get_fignums():
            fig = plt.figure(i)
            title = fig._suptitle.get_text().replace(' ', '_').lower()
            fig.savefig(os.path.join(saveDir, title))

        if (not no_show):
            plt.show()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('dirs', nargs='+', type=str)
    parser.add_argument('--no_show', action='store_true')
    args = parser.parse_args()
    plot_sim_results(args.dirs, no_show=args.no_show)
