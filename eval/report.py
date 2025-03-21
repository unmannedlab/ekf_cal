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

import os

from bokeh.embed import components
from bokeh.models import BoxZoomTool, FullscreenTool, PanTool, Paragraph, ResetTool, \
    SaveTool, Spacer, Tabs, WheelZoomTool
from bokeh.plotting import curdoc, figure, save
from bokeh.themes import Theme
from input_parser import InputParser
from tab_body import tab_body
from tab_fiducial import tab_fiducial
from tab_gps import tab_gps
from tab_imu import tab_imu
from tab_msckf import tab_msckf
from utilities import find_and_read_data_frames, generate_mc_lists


# TODO(jhartzer): Split for loop into thread pool
def plot_sim_results(config_sets, args):
    """Top level function to plot simulation results from sets of config files."""
    for config_set in config_sets:
        data_dirs = [config.split('.yaml')[0] for config in config_set]
        if len(config_set) > 1:
            plot_dir = os.path.dirname(os.path.dirname(config_set[0]))
            config_name = os.path.basename(os.path.dirname(os.path.dirname(config_set[0])))
        else:
            plot_dir = data_dirs[0]
            config_name = os.path.splitext(os.path.basename(config_set[0]))[0]

        if not os.path.isdir(plot_dir):
            os.mkdir(plot_dir)

        tabs = []
        body_state_dfs_dict = find_and_read_data_frames(data_dirs, 'body_state')
        aug_state_dfs_dict = find_and_read_data_frames(data_dirs, 'aug_state')
        body_truth_dfs_dict = find_and_read_data_frames(data_dirs, 'body_truth')
        for key in body_state_dfs_dict:
            body_state_dfs = body_state_dfs_dict[key]
            aug_state_dfs = aug_state_dfs_dict[key]
            body_truth_dfs = body_truth_dfs_dict[key]
            tabs.append(tab_body(body_state_dfs, aug_state_dfs, body_truth_dfs, args).get_tab())

        imu_dfs_dict = find_and_read_data_frames(data_dirs, 'imu')
        for key in sorted(imu_dfs_dict.keys()):
            imu_dfs = imu_dfs_dict[key]
            body_truth_dfs = body_truth_dfs_dict[0]
            tabs.append(tab_imu(imu_dfs, body_truth_dfs, args).get_tab())

        mskcf_dfs_dict = find_and_read_data_frames(data_dirs, 'msckf')
        tri_dfs_dict = find_and_read_data_frames(data_dirs, 'triangulation')
        feat_dfs_dict = find_and_read_data_frames(data_dirs, 'feature_points')
        for key in sorted(mskcf_dfs_dict.keys()):
            mskcf_dfs = mskcf_dfs_dict[key]
            tri_dfs = tri_dfs_dict[key]
            feat_dfs = feat_dfs_dict[0]
            body_truth_dfs = body_truth_dfs_dict[0]
            tabs.append(tab_msckf(mskcf_dfs, tri_dfs, feat_dfs, body_truth_dfs, args).get_tab())

        fiducial_dfs_dict = find_and_read_data_frames(data_dirs, 'fiducial')
        board_truth_dfs_dict = find_and_read_data_frames(data_dirs, 'board_truth')
        for key in sorted(fiducial_dfs_dict.keys()):
            fiducial_dfs = fiducial_dfs_dict[key]
            board_dfs = board_truth_dfs_dict[0]
            body_truth_dfs = body_truth_dfs_dict[0]
            tabs.append(tab_fiducial(fiducial_dfs, board_dfs, body_truth_dfs, args).get_tab())

        gps_dfs_dict = find_and_read_data_frames(data_dirs, 'gps')
        for key in sorted(gps_dfs_dict.keys()):
            gps_dfs = gps_dfs_dict[key]
            body_truth_dfs = body_truth_dfs_dict[0]
            tabs.append(tab_gps(gps_dfs, body_truth_dfs, args).get_tab())

        # Hide legends on click
        for tab in tabs:
            for row in tab.child.children:
                for fig in row.children:
                    if (isinstance(fig, figure)):
                        fig.toolbar.logo = None
                        fig.tools = [
                            PanTool(),
                            BoxZoomTool(),
                            WheelZoomTool(),
                            FullscreenTool(),
                            ResetTool(),
                            SaveTool()
                        ]
                        if (fig.legend):
                            fig.legend.click_policy = 'hide'
        if not args.light:
            theme = Theme(os.path.join(os.path.dirname(
                os.path.abspath(__file__)), 'theme-dark.yaml'))
            template = """
            {% block preamble %}
            <style>
            body {
                background: #1C1D1F;
                color: #fff;
            }
            </style>
            {% endblock %}
            """
        else:
            theme = Theme(os.path.join(os.path.dirname(
                os.path.abspath(__file__)), 'theme-light.yaml'))
            template = """"""

        curdoc().theme = theme

        if (args.embed):
            if not os.path.exists(os.path.join(plot_dir, 'js')):
                os.makedirs(os.path.join(plot_dir, 'js'))
            if not os.path.exists(os.path.join(plot_dir, 'html')):
                os.makedirs(os.path.join(plot_dir, 'html'))
            for tab in tabs:
                for row in tab.child.children:
                    for fig in row.children:
                        if isinstance(fig, Spacer) or isinstance(fig, Paragraph):
                            continue
                        title = fig.title.text.replace(' ', '_')
                        script, div = components(fig, wrap_script=False, theme=theme)
                        with open(os.path.join(plot_dir, 'js', f'{title}.js'), 'w') as f:
                            f.write(script)

                        with open(os.path.join(plot_dir, 'html', f'{title}.html'), 'w') as f:
                            f.write(div)
        else:
            save(
                obj=Tabs(tabs=tabs, sizing_mode='stretch_width'),
                filename=os.path.join(plot_dir, f'{config_name}-report.html'),
                resources='cdn',
                title=f'{config_name}-report.html',
                template=template
            )


# TODO(jhartzer): Write tests
# TODO(jhartzer): Add option for saving with no title (for papers)
if __name__ == '__main__':
    parser = InputParser()
    args = parser.parse_args()

    config_files = generate_mc_lists(args)
    plot_sim_results(config_files, args)
