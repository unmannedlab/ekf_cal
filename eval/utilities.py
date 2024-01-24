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

from math import pow

from bokeh.plotting import figure
import numpy as np
from scipy.spatial.transform import Rotation


def calculate_alpha(line_count: int):
    """Calculate transparency value from number of plots."""
    alpha = 1.0 / pow(line_count, 0.5)
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


def plot_update_timing(data_frames, rate=None):
    """Plot histogram of update execution durations."""
    df_prefix = data_frames[0].attrs['prefix']
    df_id = str(data_frames[0].attrs['id'])
    fig = figure(width=800, height=300, x_axis_label='time [s]',
                 y_axis_label='Count', title=f'{df_prefix} {df_id} Update Time')
    durations = np.array([])
    for df in data_frames:
        durations = np.append(durations, df['duration_0'])
    hist, edges = np.histogram(durations / 1e6)
    fig.quad(top=hist, bottom=0, left=edges[:-1], right=edges[1:], legend_label='Duration [ms]')
    if rate:
        pass
        # TODO(jhartzer): Add max duration line
        # axs.axvline(x=1000.0 / rate, color='red', linestyle='--')
    return fig
