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
stats.py.

A collection of functions for calculating statistics from the multi-IMU, multi-Camera simulation.

Typical usage is:
```
python3 eval/stats.py config/example.yaml
```

To get help:
```
python3 eval/stats.py --help
```
"""

import argparse
import os

import numpy as np
from scipy.spatial.transform import Rotation
from utilities import find_and_read_data_frames, generate_mc_lists, interpolate_error


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


def body_err_pos(body_state_dfs, body_truth_dfs):
    """Plot the body state position error."""
    RMSE_list = []
    for body_state, body_truth in zip(body_state_dfs, body_truth_dfs):
        true_time = body_truth['time'].to_list()
        true_pos_0 = body_truth['body_pos_0'].to_list()
        true_pos_1 = body_truth['body_pos_1'].to_list()
        true_pos_2 = body_truth['body_pos_2'].to_list()

        est_time = body_state['time'].to_list()
        est_pos_0 = body_state['body_pos_0'].to_list()
        est_pos_1 = body_state['body_pos_1'].to_list()
        est_pos_2 = body_state['body_pos_2'].to_list()

        err_pos_0 = interpolate_error(true_time, true_pos_0, est_time, est_pos_0)
        err_pos_1 = interpolate_error(true_time, true_pos_1, est_time, est_pos_1)
        err_pos_2 = interpolate_error(true_time, true_pos_2, est_time, est_pos_2)
        RMSE_list.append(RMSE_from_vectors(err_pos_0, err_pos_1, err_pos_2))
    return RMSE_list


def body_err_vel(body_state_dfs, body_truth_dfs):
    """Plot the body state velocity error."""
    RMSE_list = []
    for body_state, body_truth in zip(body_state_dfs, body_truth_dfs):
        true_time = body_truth['time'].to_list()
        true_vel_0 = body_truth['body_vel_0'].to_list()
        true_vel_1 = body_truth['body_vel_1'].to_list()
        true_vel_2 = body_truth['body_vel_2'].to_list()

        est_time = body_state['time'].to_list()
        est_vel_0 = body_state['body_vel_0'].to_list()
        est_vel_1 = body_state['body_vel_1'].to_list()
        est_vel_2 = body_state['body_vel_2'].to_list()

        err_vel_0 = interpolate_error(true_time, true_vel_0, est_time, est_vel_0)
        err_vel_1 = interpolate_error(true_time, true_vel_1, est_time, est_vel_1)
        err_vel_2 = interpolate_error(true_time, true_vel_2, est_time, est_vel_2)
        RMSE_list.append(RMSE_from_vectors(err_vel_0, err_vel_1, err_vel_2))
    return RMSE_list


def body_err_acc(body_state_dfs, body_truth_dfs):
    """Plot the body state acceleration error."""
    RMSE_list = []
    for body_state, body_truth in zip(body_state_dfs, body_truth_dfs):
        true_time = body_truth['time'].to_list()
        true_acc_0 = body_truth['body_acc_0'].to_list()
        true_acc_1 = body_truth['body_acc_1'].to_list()
        true_acc_2 = body_truth['body_acc_2'].to_list()

        est_time = body_state['time'].to_list()
        est_acc_0 = body_state['body_acc_0'].to_list()
        est_acc_1 = body_state['body_acc_1'].to_list()
        est_acc_2 = body_state['body_acc_2'].to_list()

        err_acc_0 = interpolate_error(true_time, true_acc_0, est_time, est_acc_0)
        err_acc_1 = interpolate_error(true_time, true_acc_1, est_time, est_acc_1)
        err_acc_2 = interpolate_error(true_time, true_acc_2, est_time, est_acc_2)
        RMSE_list.append(RMSE_from_vectors(err_acc_0, err_acc_1, err_acc_2))
    return RMSE_list


def body_err_ang(body_state_dfs, body_truth_dfs):
    """Plot the body state angular velocity error."""
    RMSE_list = []
    for body_state, body_truth in zip(body_state_dfs, body_truth_dfs):
        true_time = body_truth['time'].to_list()
        true_ang_vel_0 = body_truth['body_ang_vel_0'].to_list()
        true_ang_vel_1 = body_truth['body_ang_vel_1'].to_list()
        true_ang_vel_2 = body_truth['body_ang_vel_2'].to_list()

        est_time = body_state['time'].to_list()
        est_ang_vel_0 = body_state['body_ang_vel_0'].to_list()
        est_ang_vel_1 = body_state['body_ang_vel_1'].to_list()
        est_ang_vel_2 = body_state['body_ang_vel_2'].to_list()

        err_ang_vel_0 = interpolate_error(true_time, true_ang_vel_0, est_time, est_ang_vel_0)
        err_ang_vel_1 = interpolate_error(true_time, true_ang_vel_1, est_time, est_ang_vel_1)
        err_ang_vel_2 = interpolate_error(true_time, true_ang_vel_2, est_time, est_ang_vel_2)
        RMSE_list.append(RMSE_from_vectors(err_ang_vel_0, err_ang_vel_1, err_ang_vel_2))
    return RMSE_list


def body_err_ang_vel(body_state_dfs, body_truth_dfs):
    """Plot the body state angular velocity error."""
    RMSE_list = []
    for body_state, body_truth in zip(body_state_dfs, body_truth_dfs):
        true_time = body_truth['time'].to_list()
        true_ang_vel_0 = body_truth['body_ang_vel_0'].to_list()
        true_ang_vel_1 = body_truth['body_ang_vel_1'].to_list()
        true_ang_vel_2 = body_truth['body_ang_vel_2'].to_list()

        est_time = body_state['time'].to_list()
        est_ang_vel_0 = body_state['body_ang_vel_0'].to_list()
        est_ang_vel_1 = body_state['body_ang_vel_1'].to_list()
        est_ang_vel_2 = body_state['body_ang_vel_2'].to_list()

        err_ang_vel_0 = interpolate_error(true_time, true_ang_vel_0, est_time, est_ang_vel_0)
        err_ang_vel_1 = interpolate_error(true_time, true_ang_vel_1, est_time, est_ang_vel_1)
        err_ang_vel_2 = interpolate_error(true_time, true_ang_vel_2, est_time, est_ang_vel_2)
        RMSE_list.append(RMSE_from_vectors(err_ang_vel_0, err_ang_vel_1, err_ang_vel_2))
    return RMSE_list


def body_err_ang_acc(body_state_dfs, body_truth_dfs):
    """Plot the body state angular acceleration error."""
    RMSE_list = []
    for body_state, body_truth in zip(body_state_dfs, body_truth_dfs):
        true_time = body_truth['time'].to_list()
        true_ang_acc_0 = body_truth['body_ang_acc_0'].to_list()
        true_ang_acc_1 = body_truth['body_ang_acc_1'].to_list()
        true_ang_acc_2 = body_truth['body_ang_acc_2'].to_list()

        est_time = body_state['time'].to_list()
        est_ang_acc_0 = body_state['body_ang_acc_0'].to_list()
        est_ang_acc_1 = body_state['body_ang_acc_1'].to_list()
        est_ang_acc_2 = body_state['body_ang_acc_2'].to_list()

        err_ang_acc_0 = interpolate_error(true_time, true_ang_acc_0, est_time, est_ang_acc_0)
        err_ang_acc_1 = interpolate_error(true_time, true_ang_acc_1, est_time, est_ang_acc_1)
        err_ang_acc_2 = interpolate_error(true_time, true_ang_acc_2, est_time, est_ang_acc_2)
        RMSE_list.append(RMSE_from_vectors(err_ang_acc_0, err_ang_acc_1, err_ang_acc_2))
    return RMSE_list


def write_summary(directory, stats):
    """Write the error summary statistics to a file."""
    with open(os.path.join(directory, 'stats.txt'), 'w') as f:
        f.write('Statistic,RMSE\n')
        for key in stats:
            vals = np.array(stats[key])
            mu = np.mean(vals)
            sig = np.std(vals)
            f.write('{}: {:0.4f}, {:0.4f}\n'.format(key, mu, sig))
    return


# TODO(jhartzer): Split for loop into thread pool
def calc_sim_stats(config_sets, settings):
    """Top level function to plot simulation results from sets of config files."""
    for config_set in config_sets:

        data_dirs = [config.split('.yaml')[0] for config in config_set]
        if len(config_set) > 1:
            stat_dir = os.path.dirname(os.path.dirname(config_set[0]))
        else:
            stat_dir = data_dirs[0]

        stats = {}
        body_state_dfs_dict = find_and_read_data_frames(data_dirs, 'body_state')
        body_truth_dfs_dict = find_and_read_data_frames(data_dirs, 'body_truth')
        for key in body_state_dfs_dict:
            body_state_dfs = body_state_dfs_dict[key]
            body_truth_dfs = body_truth_dfs_dict[key]
            stats[f'body_{key}_err_pos'] = body_err_pos(body_state_dfs, body_truth_dfs)
            stats[f'body_{key}_err_vel'] = body_err_vel(body_state_dfs, body_truth_dfs)
            stats[f'body_{key}_err_acc'] = body_err_acc(body_state_dfs, body_truth_dfs)
            stats[f'body_{key}_err_ang'] = body_err_ang(body_state_dfs, body_truth_dfs)
            stats[f'body_{key}_err_ang_vel'] = body_err_ang_vel(body_state_dfs, body_truth_dfs)
            stats[f'body_{key}_err_ang_acc'] = body_err_ang_acc(body_state_dfs, body_truth_dfs)

        # TODO(jhartzer): Generate IMU errors
        # imu_dfs_dict = find_and_read_data_frames(data_dirs, 'imu')
        # for key in sorted(imu_dfs_dict.keys()):
        #     imu_dfs = imu_dfs_dict[key]
        #     # stats[f'imu_{}_']

        # TODO(jhartzer): Generate MSKCF errors
        # mskcf_dfs_dict = find_and_read_data_frames(data_dirs, 'msckf')
        # tri_dfs_dict = find_and_read_data_frames(data_dirs, 'triangulation')
        # feat_dfs_dict = find_and_read_data_frames(data_dirs, 'feature_points')
        # for key in sorted(mskcf_dfs_dict.keys()):
        #     mskcf_dfs = mskcf_dfs_dict[key]
        #     tri_dfs = tri_dfs_dict[key]
        #     feat_dfs = feat_dfs_dict[0]
        #     # stats[f'mskcf_{}_']

        # TODO(jhartzer): Generate fiducial errors
        # fiducial_dfs_dict = find_and_read_data_frames(data_dirs, 'fiducial')
        # tri_dfs_dict = find_and_read_data_frames(data_dirs, 'triangulation')
        # board_dfs_dict = find_and_read_data_frames(data_dirs, 'boards')
        # for key in sorted(fiducial_dfs_dict.keys()):
        #     fiducial_dfs = fiducial_dfs_dict[key]
        #     tri_dfs = tri_dfs_dict[key]
        #     board_dfs = board_dfs_dict[0]
        #     # stats[f'fiducial_{}_']

        write_summary(stat_dir, stats)


# TODO(jhartzer): Write tests
# TODO(jhartzer): Add flag for low-memory usage (load single df at a time)
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('yaml_files', nargs='+', type=str)
    parser.add_argument('-j', '--jobs', default=None, type=int)
    parser.add_argument('-n', '--runs', default=None, type=int)
    args = parser.parse_args()

    settings = {}
    settings['jobs'] = args.jobs

    config_files = generate_mc_lists(args.yaml_files, runs=args.runs)
    calc_sim_stats(config_files, settings)
