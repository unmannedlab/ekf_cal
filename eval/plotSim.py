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
    axs1_1.plot(imu_df['time'], imu_df['residual_0'], label='Px')
    axs1_1.plot(imu_df['time'], imu_df['residual_1'], label='Py')
    axs1_1.plot(imu_df['time'], imu_df['residual_2'], label='Pz')
    axs1_2.plot(imu_df['time'], imu_df['residual_3'], label='\theta x')
    axs1_2.plot(imu_df['time'], imu_df['residual_4'], label='\theta y')
    axs1_2.plot(imu_df['time'], imu_df['residual_5'], label='\theta z')

    fig2, (axs2_1, axs2_2) = plt.subplots(2, 1)
    axs2_1.plot(imu_df['time'], imu_df['update_0'], label='x')
    axs2_1.plot(imu_df['time'], imu_df['update_1'], label='y')
    axs2_1.plot(imu_df['time'], imu_df['update_2'], label='z')
    axs2_2.plot(imu_df['time'], imu_df['update_3'], label='\theta x')
    axs2_2.plot(imu_df['time'], imu_df['update_4'], label='\theta y')
    axs2_2.plot(imu_df['time'], imu_df['update_5'], label='\theta z')

    fig3, (axs3_1, axs3_2) = plt.subplots(2, 1)
    axs3_1.plot(imu_df['time'], imu_df['update_6'], label='x')
    axs3_1.plot(imu_df['time'], imu_df['update_7'], label='y')
    axs3_1.plot(imu_df['time'], imu_df['update_8'], label='z')
    axs3_2.plot(imu_df['time'], imu_df['update_9'], label='x')
    axs3_2.plot(imu_df['time'], imu_df['update_10'], label='y')
    axs3_2.plot(imu_df['time'], imu_df['update_11'], label='z')
    plt.show()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('dirs', nargs='+', type=str)
    args = parser.parse_args()
    plot_sim_results(args.dirs)

