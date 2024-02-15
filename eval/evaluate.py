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
Runs and plots data in input yaml files using run_sim.py and report.py.

Usage is:
```
python3 eval/evaluate.py config/example.yaml
```

To get help:
```
python3 eval/evaluate.py --help
```
"""

from input_parser import InputParser
from report import plot_sim_results
from run import add_jobs
from stats import calc_sim_stats
from utilities import generate_mc_lists

# TODO(jhartzer): Write tests
if __name__ == '__main__':
    parser = InputParser()
    args = parser.parse_args()

    add_jobs(
        args.inputs,
        jobs=args.jobs,
        runs=args.runs,
        time=args.time)

    config_files = generate_mc_lists(args.inputs, runs=args.runs)
    if (not args.no_plot):
        plot_sim_results(config_files, args.embed)
    calc_sim_stats(config_files, args)
