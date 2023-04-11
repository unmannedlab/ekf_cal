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


def plot_sim_results(directories):
    for directory in directories:
        imu_df = pd.read_csv(os.path.join(directory, 'imu.csv'))

        fig1 = plot_imu_residuals(imu_df)
        fig2 = plot_imu_pos_updates(imu_df)
        fig3 = plot_imu_vel_updates(imu_df)

        saveDir = os.path.join(directory, 'plots')
        if not os.path.isdir(saveDir):
            os.mkdir(saveDir)

        fig1.savefig(os.path.join(saveDir, 'imu_residuals.png'))
        fig2.savefig(os.path.join(saveDir, 'imu_pos_updates.png'))
        fig3.savefig(os.path.join(saveDir, 'imu_vel_updates.png'))

        plt.show()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('dirs', nargs='+', type=str)
    args = parser.parse_args()
    plot_sim_results(args.dirs)
