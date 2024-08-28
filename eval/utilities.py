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

import collections
import glob
import math
import os
import re

from bokeh.plotting import figure
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation
import yaml


def get_colors(args):
    if args.light:
        return ['red', 'green', 'blue', 'magenta']
    else:
        return ['cyan', 'yellow', 'magenta', 'white']


def calculate_alpha(line_count: int):
    """Calculate transparency value from number of plots."""
    alpha = 1.0 / math.pow(line_count, 0.5)
    return alpha


def interpolate_error(true_t, true_x, estimate_t, estimate_x):
    """Calculate an interpolated vector error using truth and estimate points."""
    interp_x = np.interp(estimate_t, true_t, true_x)
    errors = [estimate - interp for estimate, interp in zip(estimate_x, interp_x)]
    return errors


def lists_to_rot(w_list, x_list, y_list, z_list):
    """Convert lists of quaternion elements to a list of scipy rotations."""
    r_list = []
    for w, x, y, z in zip(w_list, x_list, y_list, z_list):
        r = Rotation.from_quat([w, x, y, z], scalar_first=True)
        r_list.append(r)
    return r_list


def calculate_rotation_errors(estimate, truth):
    """Calculate errors from two lists of quaternions."""
    x = []
    y = []
    z = []
    for est, true in zip(estimate, truth):
        rotation = est * true.inv()
        error_eul = rotation.as_euler('XYZ')
        x.append(error_eul[0])
        y.append(error_eul[1])
        z.append(error_eul[2])

    return x, y, z


def RMSE_from_vectors(x_list, y_list, z_list):
    """Calculate the root mean square errors from list of vector elements."""
    x_err = np.array(x_list)
    y_err = np.array(y_list)
    z_err = np.array(z_list)
    rmse = np.sqrt(np.mean(x_err * x_err + y_err * y_err + z_err * z_err))
    return rmse


def plot_update_timing(data_frames, rate=None):
    """Plot histogram of update execution durations."""
    df_prefix = data_frames[0].attrs['prefix']
    df_id = str(data_frames[0].attrs['id'])
    fig = figure(width=800, height=300, x_axis_label='time [us]',
                 y_axis_label='Count', title=f'{df_prefix} {df_id} Update Time')
    durations = np.array([])
    for df in data_frames:
        durations = np.append(durations, df['duration_0'])
    hist, edges = np.histogram(durations / 1e3)
    fig.quad(top=hist, bottom=0, left=edges[:-1], right=edges[1:], legend_label='Duration [us]')
    if rate:
        pass
        # TODO(jhartzer): Add max duration line
        # axs.axvline(x=1000.0 / rate, color='red', linestyle='--')
    return fig


def format_prefix(prefix):
    """Generate formatted prefix from string."""
    if (prefix == 'imu'):
        return 'IMU'
    elif (prefix == 'camera'):
        return 'Camera'
    elif (prefix == 'body_state'):
        return 'Body'
    elif (prefix == 'fiducial'):
        return 'Fiducial'
    elif (prefix == 'msckf'):
        return 'MSCKF'
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
        file_paths_id = glob.glob(os.path.join(directory, prefix + '.csv'))
        file_paths_id.extend(glob.glob(os.path.join(directory, prefix + '_[0-9].csv')))
        for file_path in file_paths_id:
            file_name = os.path.basename(file_path)
            df = pd.read_csv(file_path).dropna()
            df.attrs['prefix'] = format_prefix(prefix)
            matches = re.findall(r'_[0-9]*\.csv', file_name)
            if matches:
                file_id = int(matches[0].split('_')[1].split('.csv')[0])
            else:
                file_id = 0
            df.attrs['id'] = file_id
            data_frame_sets[file_id].append(df)
    return data_frame_sets


def generate_mc_lists(args):
    """Generate sets of yaml configuration files to plot."""
    mc_lists = []
    for input_file in args.inputs:
        yaml_files = []
        with open(input_file, 'r') as input_stream:
            try:
                top_yaml = yaml.safe_load(input_stream)
                sim_yaml = top_yaml['/EkfCalNode']['ros__parameters']['sim_params']
                num_runs = sim_yaml['number_of_runs']
                if (args.runs):
                    num_runs = args.runs
                if (num_runs > 1):
                    top_name = os.path.basename(input_file).split('.yaml')[0]
                    yaml_dir = input_file.split('.yaml')[0] + os.sep
                    runs_dir = os.path.join(yaml_dir, 'runs')
                    n_digits = math.ceil(math.log10(num_runs))
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
