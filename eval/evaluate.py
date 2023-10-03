#!/usr/bin/env python3

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

"""
evaluate.py.

Runs and plots data in input yaml files using run_sim.py and plot.py

Usage is:
```
python3 eval/evaluate.py config/example.yaml
```

To get help:
```
python3 eval/plot.py --help
```
"""

import argparse

from plot import generate_mc_lists, Plotter

from run import add_jobs


# TODO(jhartzer): Write tests
# TODO(jhartzer): Fix import error
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('inputs', nargs='+', type=str)
    parser.add_argument('-j', '--jobs', default=None, type=int)
    parser.add_argument('-n', '--runs', default=None, type=int)
    parser.add_argument('-t', '--time', default=None, type=float)
    parser.add_argument('--show', action='store_true')
    parser.add_argument('--rate_line', action='store_true')
    parser.add_argument('-ext', default='png', type=str)
    args = parser.parse_args()
    add_jobs(
        args.inputs,
        jobs=args.jobs,
        runs=args.runs,
        time=args.time)
    config_files = generate_mc_lists(args.inputs, runs=args.runs)
    plotter = Plotter(show=args.show, ext=args.ext, rate_line=args.rate_line)
    plotter.plot_sim_results(config_files)
