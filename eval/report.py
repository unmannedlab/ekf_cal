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
plot-bokeh.py.

A collection of functions for plotting results from the multi-IMU, multi-Camera simulation.

Typical usage is:
```
python3 eval/plot-bokeh.py config/example.yaml
```

To get help:
```
python3 eval/plot-bokeh.py --help
```
"""

import argparse
import os

from bokeh.models import Tabs
from bokeh.plotting import save
from tab_body import tab_body
from tab_fiducial import tab_fiducial
from tab_imu import tab_imu
from tab_msckf import tab_msckf
from utilities import find_and_read_data_frames, generate_mc_lists, parse_yaml


# TODO(jhartzer): Split for loop into thread pool
def plot_sim_results(config_sets, settings):
    """Top level function to plot simulation results from sets of config files."""
    for config_set in config_sets:
        config_data = parse_yaml(config_set[0])

        data_dirs = [config.split('.yaml')[0] for config in config_set]
        config_name = os.path.basename(os.path.dirname(os.path.dirname(config_set[0])))
        if len(config_set) > 1:
            plot_dir = os.path.dirname(os.path.dirname(config_set[0]))
        else:
            plot_dir = data_dirs[0]

        if not os.path.isdir(plot_dir):
            os.mkdir(plot_dir)

        tabs = []
        body_state_dfs_dict = find_and_read_data_frames(data_dirs, 'body_state')
        body_truth_dfs_dict = find_and_read_data_frames(data_dirs, 'body_truth')
        for key in body_state_dfs_dict:
            body_state_dfs = body_state_dfs_dict[key]
            body_truth_dfs = body_truth_dfs_dict[key]
            tabs.append(tab_body(body_state_dfs, body_truth_dfs))

        imu_dfs_dict = find_and_read_data_frames(data_dirs, 'imu')
        for key in sorted(imu_dfs_dict.keys()):
            imu_dfs = imu_dfs_dict[key]
            tabs.append(tab_imu(imu_dfs, config_data, key))

        mskcf_dfs_dict = find_and_read_data_frames(data_dirs, 'msckf')
        tri_dfs_dict = find_and_read_data_frames(data_dirs, 'triangulation')
        feat_dfs_dict = find_and_read_data_frames(data_dirs, 'feature_points')
        for key in sorted(mskcf_dfs_dict.keys()):
            mskcf_dfs = mskcf_dfs_dict[key]
            tri_dfs = tri_dfs_dict[key]
            feat_dfs = feat_dfs_dict[0]
            tabs.append(tab_msckf(mskcf_dfs, tri_dfs, feat_dfs))

        fiducial_dfs_dict = find_and_read_data_frames(data_dirs, 'fiducial')
        tri_dfs_dict = find_and_read_data_frames(data_dirs, 'triangulation')
        board_dfs_dict = find_and_read_data_frames(data_dirs, 'boards')
        for key in sorted(fiducial_dfs_dict.keys()):
            fiducial_dfs = fiducial_dfs_dict[key]
            tri_dfs = tri_dfs_dict[key]
            board_dfs = board_dfs_dict[0]
            tabs.append(tab_fiducial(fiducial_dfs, tri_dfs, board_dfs))

        save(
            obj=Tabs(tabs=tabs, sizing_mode='stretch_width'),
            filename=os.path.join(plot_dir, f'{config_name}-report.html'),
            resources='cdn',
            title=f'{config_name}-report.html'
            )


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

    settings = {}
    settings['show'] = args.show
    settings['ext'] = args.ext
    settings['jobs'] = args.jobs

    config_files = generate_mc_lists(args.configs, runs=args.runs)
    plot_sim_results(config_files, settings)
