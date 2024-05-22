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

from bokeh.layouts import layout
from bokeh.models import Band, Spacer, TabPanel
from bokeh.plotting import ColumnDataSource, figure
import numpy as np
from utilities import calculate_alpha, plot_update_timing


def plot_camera_pos(fiducial_dfs):
    """Plot camera position offsets."""
    fig = figure(width=800, height=300, x_axis_label='Time [s]',
                 y_axis_label='Position [m]', title='Camera Position')
    a = calculate_alpha(len(fiducial_dfs))
    for mskcf_df in fiducial_dfs:
        t_cam = mskcf_df['time'].to_list()
        fig.line(t_cam, mskcf_df['cam_pos_0'].to_list(), alpha=a, color='cyan')
        fig.line(t_cam, mskcf_df['cam_pos_1'].to_list(), alpha=a, color='yellow')
        fig.line(t_cam, mskcf_df['cam_pos_2'].to_list(), alpha=a, color='magenta')
    return fig


def plot_camera_ang(fiducial_dfs):
    """Plot camera angular offsets."""
    fig = figure(width=800, height=300, x_axis_label='Time [s]',
                 y_axis_label='Orientation', title='Camera Orientation')
    a = calculate_alpha(len(fiducial_dfs))
    for mskcf_df in fiducial_dfs:
        t_cam = mskcf_df['time'].to_list()
        fig.line(t_cam, mskcf_df['cam_ang_pos_0'].to_list(), alpha=a, color='cyan')
        fig.line(t_cam, mskcf_df['cam_ang_pos_1'].to_list(), alpha=a, color='yellow')
        fig.line(t_cam, mskcf_df['cam_ang_pos_2'].to_list(), alpha=a, color='magenta')
    return fig


def plot_cam_pos_cov(fiducial_dfs):
    """Plot extrinsic position covariance."""
    fig = figure(width=800, height=300, x_axis_label='Time [s]',
                 y_axis_label='Position Covariance [m]', title='Camera Position Covariance')
    a = calculate_alpha(len(fiducial_dfs))
    for mskcf_df in fiducial_dfs:
        t_cam = mskcf_df['time'].to_list()
        cam_cov_0 = mskcf_df['cam_cov_0'].to_list()
        cam_cov_1 = mskcf_df['cam_cov_1'].to_list()
        cam_cov_2 = mskcf_df['cam_cov_2'].to_list()
        fig.line(t_cam, cam_cov_0, alpha=a, color='cyan', legend_label='p_x')
        fig.line(t_cam, cam_cov_1, alpha=a, color='yellow', legend_label='p_y')
        fig.line(t_cam, cam_cov_2, alpha=a, color='magenta', legend_label='p_z')
    return fig


def plot_cam_ang_cov(fiducial_dfs):
    """Plot extrinsic angle covariance."""
    fig = figure(width=800, height=300, x_axis_label='Time [s]',
                 y_axis_label='Angle Covariance [m]', title='Camera Angle Covariance')
    a = calculate_alpha(len(fiducial_dfs))
    for mskcf_df in fiducial_dfs:
        t_cam = mskcf_df['time'].to_list()
        cam_cov_3 = mskcf_df['cam_cov_3'].to_list()
        cam_cov_4 = mskcf_df['cam_cov_4'].to_list()
        cam_cov_5 = mskcf_df['cam_cov_5'].to_list()
        fig.line(t_cam, cam_cov_3, alpha=a, color='cyan', legend_label='\theta_x')
        fig.line(t_cam, cam_cov_4, alpha=a, color='yellow', legend_label='\theta_y')
        fig.line(t_cam, cam_cov_5, alpha=a, color='magenta', legend_label='\theta_z')
    return fig


def plot_fiducial_error_pos(tri_dfs, board_dfs):
    """Plot fiducial position error."""
    fig = figure(width=800, height=300, x_axis_label='Time [s]',
                 y_axis_label='Position Error [m]', title='Fiducial Position Error')

    err_px = collections.defaultdict(list)
    err_py = collections.defaultdict(list)
    err_pz = collections.defaultdict(list)

    for tri_df, board_df in zip(tri_dfs, board_dfs):
        time = tri_df['time'].to_list()
        board = tri_df['board'].to_list()
        board_px = tri_df['pos_x'].to_list()
        board_py = tri_df['pos_y'].to_list()
        board_pz = tri_df['pos_z'].to_list()

        true_px = board_df['pos_x'].to_list()
        true_py = board_df['pos_y'].to_list()
        true_pz = board_df['pos_z'].to_list()

        for (t, b, px, py, pz) in zip(time, board, board_px, board_py, board_pz):
            err_px[t].append(px - true_px[int(b)])
            err_py[t].append(py - true_py[int(b)])
            err_pz[t].append(pz - true_pz[int(b)])
    times = []
    mean_px = []
    mean_py = []
    mean_pz = []
    std_px = []
    std_py = []
    std_pz = []

    for time in err_px:
        times.append(time)
        mean_px.append(np.mean(err_px[time]))
        mean_py.append(np.mean(err_py[time]))
        mean_pz.append(np.mean(err_pz[time]))
        std_px.append(np.std(err_px[time]))
        std_py.append(np.std(err_py[time]))
        std_pz.append(np.std(err_pz[time]))

    times = np.array(times)
    mean_px = np.array(mean_px)
    mean_py = np.array(mean_py)
    mean_pz = np.array(mean_pz)
    std_px = np.array(std_px)
    std_py = np.array(std_py)
    std_pz = np.array(std_pz)

    t_indices = times.argsort()
    times = times[t_indices]
    mean_px = mean_px[t_indices]
    mean_py = mean_py[t_indices]
    mean_pz = mean_pz[t_indices]
    std_px = std_px[t_indices]
    std_py = std_py[t_indices]
    std_pz = std_pz[t_indices]

    fig.line(times, mean_px, color='cyan')
    fig.line(times, mean_py, color='yellow')
    fig.line(times, mean_pz, color='magenta')

    cds_x = ColumnDataSource({'base': times, 'lower': mean_px - std_px, 'upper': mean_px + std_px})
    cds_y = ColumnDataSource({'base': times, 'lower': mean_py - std_py, 'upper': mean_py + std_py})
    cds_z = ColumnDataSource({'base': times, 'lower': mean_pz - std_pz, 'upper': mean_pz + std_pz})

    fig.add_layout(Band(base='base', lower='lower', upper='upper', fill_alpha=0.3, source=cds_x,
                        fill_color='cyan', line_color='cyan'))
    fig.add_layout(Band(base='base', lower='lower', upper='upper', fill_alpha=0.3, source=cds_y,
                        fill_color='yellow', line_color='yellow'))
    fig.add_layout(Band(base='base', lower='lower', upper='upper', fill_alpha=0.3, source=cds_z,
                        fill_color='magenta', line_color='magenta'))

    return fig


def plot_fiducial_error_ang(tri_dfs, board_dfs):
    """Plot fiducial angular error."""
    fig = figure(width=800, height=300, x_axis_label='Time [s]',
                 y_axis_label='Angular Error', title='Fiducial Angular Error')

    err_qw = collections.defaultdict(list)
    err_qx = collections.defaultdict(list)
    err_qy = collections.defaultdict(list)
    err_qz = collections.defaultdict(list)

    for tri_df, board_df in zip(tri_dfs, board_dfs):
        time = tri_df['time'].to_list()
        board = tri_df['board'].to_list()
        board_qw = tri_df['quat_w'].to_list()
        board_qx = tri_df['quat_x'].to_list()
        board_qy = tri_df['quat_y'].to_list()
        board_qz = tri_df['quat_z'].to_list()

        true_qw = board_df['quat_w'].to_list()
        true_qx = board_df['quat_x'].to_list()
        true_qy = board_df['quat_y'].to_list()
        true_qz = board_df['quat_z'].to_list()

        for (t, b, qw, qx, qy, qz) in zip(time, board, board_qw, board_qx, board_qy, board_qz):
            err_qw[t].append(qw - true_qw[int(b)])
            err_qx[t].append(qx - true_qx[int(b)])
            err_qy[t].append(qy - true_qy[int(b)])
            err_qz[t].append(qz - true_qz[int(b)])
    times = []
    mean_qw = []
    mean_qx = []
    mean_qy = []
    mean_qz = []
    std_qw = []
    std_qx = []
    std_qy = []
    std_qz = []

    for time in err_qw:
        times.append(time)
        mean_qw.append(np.mean(err_qw[time]))
        mean_qx.append(np.mean(err_qx[time]))
        mean_qy.append(np.mean(err_qy[time]))
        mean_qz.append(np.mean(err_qz[time]))
        std_qw.append(np.std(err_qw[time]))
        std_qx.append(np.std(err_qx[time]))
        std_qy.append(np.std(err_qy[time]))
        std_qz.append(np.std(err_qz[time]))

    times = np.array(times)
    mean_qw = np.array(mean_qw)
    mean_qx = np.array(mean_qx)
    mean_qy = np.array(mean_qy)
    mean_qz = np.array(mean_qz)
    std_qw = np.array(std_qw)
    std_qx = np.array(std_qx)
    std_qy = np.array(std_qy)
    std_qz = np.array(std_qz)

    t_indices = times.argsort()
    times = times[t_indices]
    mean_qw = mean_qw[t_indices]
    mean_qx = mean_qx[t_indices]
    mean_qy = mean_qy[t_indices]
    mean_qz = mean_qz[t_indices]
    std_qw = std_qw[t_indices]
    std_qx = std_qx[t_indices]
    std_qy = std_qy[t_indices]
    std_qz = std_qz[t_indices]

    fig.line(times, mean_qw, color='cyan')
    fig.line(times, mean_qx, color='yellow')
    fig.line(times, mean_qy, color='magenta')
    fig.line(times, mean_qz, color='purple')

    cds_w = ColumnDataSource({'base': times, 'lower': mean_qw - std_qw, 'upper': mean_qw + std_qw})
    cds_x = ColumnDataSource({'base': times, 'lower': mean_qx - std_qx, 'upper': mean_qx + std_qx})
    cds_y = ColumnDataSource({'base': times, 'lower': mean_qy - std_qy, 'upper': mean_qy + std_qy})
    cds_z = ColumnDataSource({'base': times, 'lower': mean_qz - std_qz, 'upper': mean_qz + std_qz})

    fig.add_layout(Band(base='base', lower='lower', upper='upper', fill_alpha=0.3, source=cds_w,
                        fill_color='cyan', line_color='cyan'))
    fig.add_layout(Band(base='base', lower='lower', upper='upper', fill_alpha=0.3, source=cds_x,
                        fill_color='yellow', line_color='yellow'))
    fig.add_layout(Band(base='base', lower='lower', upper='upper', fill_alpha=0.3, source=cds_y,
                        fill_color='magenta', line_color='magenta'))
    fig.add_layout(Band(base='base', lower='lower', upper='upper', fill_alpha=0.3, source=cds_z,
                        fill_color='magenta', line_color='magenta'))

    return fig


def tab_fiducial(fiducial_dfs, tri_dfs, board_dfs):
    layout_plots = [
        [plot_camera_pos(fiducial_dfs), plot_camera_ang(fiducial_dfs)],
        [plot_cam_pos_cov(fiducial_dfs), plot_cam_ang_cov(fiducial_dfs)],
        [plot_fiducial_error_pos(tri_dfs, board_dfs), plot_fiducial_error_ang(tri_dfs, board_dfs)],
        [plot_update_timing(fiducial_dfs), Spacer()]
    ]

    tab_layout = layout(layout_plots, sizing_mode='stretch_width')
    tab = TabPanel(child=tab_layout, title=f"Fiducial {fiducial_dfs[0].attrs['id']}")

    return tab
