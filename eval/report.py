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
import argparse
import collections
import functools
import glob
import math
import multiprocessing
import numpy as np
import os
import pandas as pd
import re
import yaml

from scipy.spatial.transform import Rotation

from bokeh.models import Tabs
from bokeh.plotting import save

from tab_body import tab_body
from tab_fiducial import tab_fiducial
from tab_imu import tab_imu
from tab_msckf import tab_msckf


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
        r = Rotation.from_quat([w, x, y, z])
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
        for key in sorted(fiducial_dfs_dict.keys()):
            fiducial_dfs = fiducial_dfs_dict[key]
            tabs.append(tab_fiducial(fiducial_dfs, config_data, key))

        save(
            obj=Tabs(tabs=tabs, sizing_mode="stretch_width"), 
            filename=os.path.join(plot_dir, f'{config_name}-plots.html'),
            resources='cdn',
            title=f'{config_name}-plots.html'
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
