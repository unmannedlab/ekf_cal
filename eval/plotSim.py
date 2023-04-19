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
import os

import matplotlib.pyplot as plt
from matplotlib.ticker import PercentFormatter

import pandas as pd


def plot_imu_residuals(imu_df):
    fig, (axs_1, axs_2) = plt.subplots(2, 1)

    axs_1.plot(imu_df['time'], imu_df['residual_0'], label=r'$p_x$')
    axs_1.plot(imu_df['time'], imu_df['residual_1'], label=r'$p_y$')
    axs_1.plot(imu_df['time'], imu_df['residual_2'], label=r'$p_z$')
    axs_2.plot(imu_df['time'], imu_df['residual_3'], label=r'$\theta_x$')
    axs_2.plot(imu_df['time'], imu_df['residual_4'], label=r'$\theta_y$')
    axs_2.plot(imu_df['time'], imu_df['residual_5'], label=r'$\theta_z$')
    axs_1.set_title('IMU Residuals')
    axs_1.set_ylabel('Position Residual\n[m]')
    axs_2.set_ylabel('Angular Residual\n[red]')
    axs_2.set_xlabel('Time [s]')
    axs_1.grid(True)
    axs_2.grid(True)
    axs_1.legend()
    axs_2.legend()
    fig.tight_layout()

    return fig


def plot_imu_pos_updates(imu_df):
    fig, (axs_1, axs_2) = plt.subplots(2, 1)

    axs_1.plot(imu_df['time'], imu_df['update_0'], label=r'$p_x$')
    axs_1.plot(imu_df['time'], imu_df['update_1'], label=r'$p_y$')
    axs_1.plot(imu_df['time'], imu_df['update_2'], label=r'$p_z$')
    axs_2.plot(imu_df['time'], imu_df['update_3'], label=r'$\theta_x$')
    axs_2.plot(imu_df['time'], imu_df['update_4'], label=r'$\theta_y$')
    axs_2.plot(imu_df['time'], imu_df['update_5'], label=r'$\theta_z$')
    axs_1.set_title('IMU State Offset Updates')
    axs_1.set_ylabel('Position Offset Update\n[m]')
    axs_2.set_ylabel('Angular Offset Update\n[red]')
    axs_2.set_xlabel('Time [s]')
    axs_1.grid(True)
    axs_2.grid(True)
    axs_1.legend()
    axs_2.legend()
    fig.tight_layout()

    return fig


def plot_imu_vel_updates(imu_df):
    fig, (axs_1, axs_2) = plt.subplots(2, 1)

    axs_1.plot(imu_df['time'], imu_df['update_6'], label=r'$a_x$')
    axs_1.plot(imu_df['time'], imu_df['update_7'], label=r'$a_y$')
    axs_1.plot(imu_df['time'], imu_df['update_8'], label=r'$a_z$')
    axs_2.plot(imu_df['time'], imu_df['update_9'], label=r'$\omega_x$')
    axs_2.plot(imu_df['time'], imu_df['update_10'], label=r'$\omega_y$')
    axs_2.plot(imu_df['time'], imu_df['update_11'], label=r'$\omega_z$')
    axs_1.set_title('IMU State Bias Updates')
    axs_1.set_ylabel('Acceleration Bias Update\n[m/s/s]')
    axs_2.set_ylabel('Angular Rate Bias Update\n[rad/s]')
    axs_2.set_xlabel('Time [s]')
    axs_1.grid(True)
    axs_2.grid(True)
    axs_1.legend()
    axs_2.legend()
    fig.tight_layout()

    return fig


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
    axs_1.set_title('Body State Position Updates')
    axs_1.set_ylabel('Position [m]')
    axs_2.set_ylabel('Velocity [m/s]')
    axs_3.set_ylabel('Acceleration [m/s/s]')
    axs_3.set_xlabel('Time [s]')
    axs_1.grid(True)
    axs_2.grid(True)
    axs_3.grid(True)
    axs_1.legend()
    axs_2.legend()
    axs_3.legend()
    fig.tight_layout()

    return fig


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
    axs_1.set_title('Body State Angular Updates')
    axs_1.set_ylabel('Euler Angle [rad]')
    axs_2.set_ylabel('Angular Velocity [rad/s]')
    axs_3.set_ylabel('Angular Acceleration [rad/s/s]')
    axs_3.set_xlabel('Time [s]')
    axs_1.grid(True)
    axs_2.grid(True)
    axs_3.grid(True)
    axs_1.legend()
    axs_2.legend()
    axs_3.legend()
    fig.tight_layout()

    return fig


def plot_msckf_cam_pos_updates(msckf_df):
    fig, (axs) = plt.subplots(1, 1)

    axs.plot(msckf_df['time'], msckf_df['cam_update_0'], label=r'$p_x$')
    axs.plot(msckf_df['time'], msckf_df['cam_update_1'], label=r'$p_y$')
    axs.plot(msckf_df['time'], msckf_df['cam_update_2'], label=r'$p_z$')
    axs.set_title('Camera Position Offset')
    axs.set_ylabel('Position Offset [m]')
    axs.set_xlabel('Time [s]')
    axs.grid(True)
    axs.legend()
    fig.tight_layout()

    return fig


def plot_msckf_cam_ang_updates(msckf_df):
    fig, (axs) = plt.subplots(1, 1)

    axs.plot(msckf_df['time'], msckf_df['cam_update_3'], label=r'$\theta_x$')
    axs.plot(msckf_df['time'], msckf_df['cam_update_4'], label=r'$\theta_y$')
    axs.plot(msckf_df['time'], msckf_df['cam_update_5'], label=r'$\theta_z$')
    axs.set_title('Camera Angular Offset')
    axs.set_ylabel('Angular Offset [rad]')
    axs.set_xlabel('Time [s]')
    axs.grid(True)
    axs.legend()
    fig.tight_layout()

    return fig


def plot_update_timing(imu_df, msckf_df):
    fig, (axs_1, axs_2) = plt.subplots(1, 2)
    axs_1.hist(imu_df['time_0']/1e3, bins=10)
    axs_2.hist(msckf_df['time_0']/1e3, bins=10)
    axs_1.set_ylabel('Count')
    axs_2.set_ylabel('Count')
    axs_1.set_title('IMU Update Time')
    axs_2.set_title('MSCKF Update Time')
    axs_1.set_xlabel('Duration [ms]')
    axs_2.set_xlabel('Duration [ms]')
    axs_1.grid(True)
    axs_2.grid(True)
    fig.tight_layout()

    return fig


def plot_sim_results(directories):
    for directory in directories:
        imu_df = pd.read_csv(os.path.join(directory, 'imu_1.csv'))
        msckf_df = pd.read_csv(os.path.join(directory, 'msckf_2.csv'))

        fig1 = plot_imu_residuals(imu_df)
        fig2 = plot_imu_pos_updates(imu_df)
        fig3 = plot_imu_vel_updates(imu_df)
        fig4 = plot_msckf_body_pos_updates(msckf_df)
        fig5 = plot_msckf_body_ang_updates(msckf_df)
        fig6 = plot_msckf_cam_pos_updates(msckf_df)
        fig7 = plot_msckf_cam_ang_updates(msckf_df)
        fig8 = plot_update_timing(imu_df, msckf_df)

        saveDir = os.path.join(directory, 'plots')
        if not os.path.isdir(saveDir):
            os.mkdir(saveDir)

        fig1.savefig(os.path.join(saveDir, 'imu_residuals.png'))
        fig2.savefig(os.path.join(saveDir, 'imu_pos_updates.png'))
        fig3.savefig(os.path.join(saveDir, 'imu_vel_updates.png'))
        fig4.savefig(os.path.join(saveDir, 'msckf_body_pos_updates.png'))
        fig5.savefig(os.path.join(saveDir, 'msckf_body_ang_updates.png'))
        fig6.savefig(os.path.join(saveDir, 'msckf_cam_pos_updates.png'))
        fig7.savefig(os.path.join(saveDir, 'msckf_cam_ang_updates.png'))
        fig8.savefig(os.path.join(saveDir, 'update_timing.png'))

        plt.show()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('dirs', nargs='+', type=str)
    args = parser.parse_args()
    plot_sim_results(args.dirs)
