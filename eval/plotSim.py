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

import matplotlib.pyplot as plt
import argparse
import pandas as pd
import os


def plot_sim_results(directories):
    # for directory in directories:
    directory = directories[0]
    print(directory)
    # @todo switch to glob
    imu_df = pd.read_csv(os.path.join(directory, 'imu.csv'))

    fig1, (axs1_1, axs1_2) = plt.subplots(2, 1)
    axs1_1.plot(imu_df['time'], imu_df['residual_0'], label=r'$p_x$')
    axs1_1.plot(imu_df['time'], imu_df['residual_1'], label=r'$p_y$')
    axs1_1.plot(imu_df['time'], imu_df['residual_2'], label=r'$p_z$')
    axs1_2.plot(imu_df['time'], imu_df['residual_3'], label=r'$\theta_x$')
    axs1_2.plot(imu_df['time'], imu_df['residual_4'], label=r'$\theta_y$')
    axs1_2.plot(imu_df['time'], imu_df['residual_5'], label=r'$\theta_z$')
    axs1_1.set_title('IMU Residuals')
    axs1_1.set_ylabel('Position Residual\n[m]')
    axs1_2.set_ylabel('Angular Residual\n[red]')
    axs1_2.set_xlabel('Time [s]')
    axs1_1.grid(True)
    axs1_2.grid(True)
    axs1_1.legend()
    axs1_2.legend()
    fig1.tight_layout()

    fig2, (axs2_1, axs2_2) = plt.subplots(2, 1)
    axs2_1.plot(imu_df['time'], imu_df['update_0'], label=r'$p_x$')
    axs2_1.plot(imu_df['time'], imu_df['update_1'], label=r'$p_y$')
    axs2_1.plot(imu_df['time'], imu_df['update_2'], label=r'$p_z$')
    axs2_2.plot(imu_df['time'], imu_df['update_3'], label=r'$\theta_x$')
    axs2_2.plot(imu_df['time'], imu_df['update_4'], label=r'$\theta_y$')
    axs2_2.plot(imu_df['time'], imu_df['update_5'], label=r'$\theta_z$')
    axs2_1.set_title('IMU State Offset Updates')
    axs2_1.set_ylabel('Position Offset Update\n[m]')
    axs2_2.set_ylabel('Angular Offset Update\n[red]')
    axs2_2.set_xlabel('Time [s]')
    axs2_1.grid(True)
    axs2_2.grid(True)
    axs2_1.legend()
    axs2_2.legend()
    fig2.tight_layout()

    fig3, (axs3_1, axs3_2) = plt.subplots(2, 1)
    axs3_1.plot(imu_df['time'], imu_df['update_6'], label=r'$a_x$')
    axs3_1.plot(imu_df['time'], imu_df['update_7'], label=r'$a_y$')
    axs3_1.plot(imu_df['time'], imu_df['update_8'], label=r'$a_z$')
    axs3_2.plot(imu_df['time'], imu_df['update_9'], label=r'$\omega_x$')
    axs3_2.plot(imu_df['time'], imu_df['update_10'], label=r'$\omega_y$')
    axs3_2.plot(imu_df['time'], imu_df['update_11'], label=r'$\omega_z$')
    axs3_1.set_title('IMU State Bias Updates')
    axs3_1.set_ylabel('Acceleration Bias Update\n[m/s/s]')
    axs3_2.set_ylabel('Angular Rate Bias Update\n[rad/s]')
    axs3_2.set_xlabel('Time [s]')
    axs3_1.grid(True)
    axs3_2.grid(True)
    axs3_1.legend()
    axs3_2.legend()
    fig3.tight_layout()

    plt.show()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('dirs', nargs='+', type=str)
    args = parser.parse_args()
    plot_sim_results(args.dirs)
