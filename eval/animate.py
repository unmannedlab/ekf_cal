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

"""
Generates animation using data from runs outlined in input files.

Usage is:
```
python3 eval/animate.py config/example.yaml
```

To get help:
```
python3 eval/animate.py --help
```
"""

import matplotlib.pyplot as plt
import os
import numpy as np
from matplotlib import animation
from input_parser import InputParser
from utilities import find_and_read_data_frames, generate_mc_lists
from functools import partial

# TODO: Plot keypoints
# TODO: Plot augmented states
# TODO: Plot polygon for fiducial board
def generate_animation(config_sets, args):
    """Top level function to plot simulation results from sets of config files."""
    for config_set in config_sets:
        data_dirs = [config.split('.yaml')[0] for config in config_set]
        if len(config_set) > 1:
            plot_dir = os.path.dirname(os.path.dirname(config_set[0]))
        else:
            plot_dir = data_dirs[0]

        file_name = os.path.basename(plot_dir)
        if not os.path.isdir(plot_dir):
            os.mkdir(plot_dir)

        body_state_dfs_dict = find_and_read_data_frames(data_dirs, 'body_state')
        # body_truth_dfs_dict = find_and_read_data_frames(data_dirs, 'body_truth')
        # tri_dfs_dict = find_and_read_data_frames(data_dirs, 'triangulation')
        # feat_dfs_dict = find_and_read_data_frames(data_dirs, 'feature_points')
        # board_truth_dfs_dict = find_and_read_data_frames(data_dirs, 'board_truth')

        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        x_lim = [1e9, -1e9]
        y_lim = [1e9, -1e9]
        z_lim = [1e9, -1e9]
        
        for key in body_state_dfs_dict:
            body_dfs = body_state_dfs_dict[key]
            for body_df in body_dfs:
                x_lim[0] = min(x_lim[0], np.min(body_df['body_pos_0']))
                y_lim[0] = min(y_lim[0], np.min(body_df['body_pos_1']))
                z_lim[0] = min(z_lim[0], np.min(body_df['body_pos_2']))
                x_lim[1] = max(x_lim[1], np.max(body_df['body_pos_0']))
                y_lim[1] = max(y_lim[1], np.max(body_df['body_pos_1']))
                z_lim[1] = max(z_lim[1], np.max(body_df['body_pos_2']))

        fps = 30
        reset_time = 5.0
        tail_time = 1.0
        frame_count = int(reset_time * fps)

        def gen_frame(t, body_dfs_dict):
            ax.cla()
            for key in body_dfs_dict:
                body_dfs = body_dfs_dict[key]
                for body_df in body_dfs:
                    ppf = int(len(body_df['body_pos_0']) / frame_count)
                    max_n = int(tail_time * fps * ppf)
                    n = int(t * ppf) + 1

                    pos_x = np.array(body_df['body_pos_0'][max(0,n-max_n):n])
                    pos_y = np.array(body_df['body_pos_1'][max(0,n-max_n):n])
                    pos_z = np.array(body_df['body_pos_2'][max(0,n-max_n):n])
                    ax.plot(pos_x, pos_y, pos_z)
                    ax.set_xlim(x_lim)
                    ax.set_ylim(y_lim)
                    ax.set_zlim(z_lim)

            return fig,

        ani = animation.FuncAnimation(fig, partial(
            gen_frame, body_dfs_dict=body_state_dfs_dict), frame_count, interval=1000 / fps)
        plt.show()

        ani.save(os.path.join(plot_dir, f'{file_name}.gif'), writer='imagemagick')


# TODO(jhartzer): Write tests
if __name__ == '__main__':
    parser = InputParser()
    args = parser.parse_args()

    config_files = generate_mc_lists(args)
    generate_animation(config_files, args)