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
A collection of function for running the multi-IMU, multi-Camera simulation.

Typical usage is:
```
python3 eval/run_sim.py config/example.yaml
```

To get help:
```
python3 eval/run_sim.py --help
```
"""

import math
import multiprocessing
import os
import random
import subprocess
import traceback
from typing import List

from input_parser import InputParser
import yaml


def add_gitignore(directory):
    """Add gitignore to a directory."""
    f_path = os.path.join(directory, '.gitignore')
    with open(f_path, 'w') as file_id:
        file_id.write('*\n')


def print_err(err):
    """Print errors experienced in asynchronous pool."""
    print('error_callback()', err)
    traceback.print_exception(type(err), err, err.__traceback__)


def run_sim(yaml_path: str):
    """Run simulation given an input yaml."""
    # Get (and create) yaml directory
    yaml_dir = yaml_path.split('.yaml')[0] + os.sep
    if (not os.path.isdir(yaml_dir)):
        os.mkdir(yaml_dir)
        with open(os.path.join(yaml_dir, '.gitignore'), 'w') as f_git_ignore:
            f_git_ignore.write('*\n')

    # Run simulation
    base_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..')
    sim_path = os.path.join(base_path, '..', '..', 'build', 'ekf_cal', 'Release', 'sim')
    proc = subprocess.run([sim_path, yaml_path, yaml_dir],
                          stdout=subprocess.PIPE,
                          stderr=subprocess.PIPE)

    # Write stdout
    if (proc.stdout):
        with open(os.path.join(yaml_dir, 'sim.stdout'), 'wb') as output:
            output.write(proc.stdout)

    # Write stderr
    if (proc.stderr):
        with open(os.path.join(yaml_dir, 'sim.stderr'), 'wb') as output:
            output.write(proc.stderr)


def generate_mc_from_yaml(
    yaml_file,
    runs=None,
    time=None
):
    """Generate a Monte Carlo list of inputs given a top-level yaml."""
    with open(yaml_file, 'r') as yaml_stream:
        try:
            top_yaml = yaml.safe_load(yaml_stream)
            sim_yaml = top_yaml['/EkfCalNode']['ros__parameters']['sim_params']
            if (runs):
                num_runs = runs
            else:
                num_runs = sim_yaml['number_of_runs']
            if (num_runs > 1):
                yaml_files = []
                top_name = os.path.basename(yaml_file).split('.yaml')[0]
                yaml_dir = yaml_file.split('.yaml')[0] + os.sep
                if (not os.path.isdir(yaml_dir)):
                    os.mkdir(yaml_dir)
                add_gitignore(yaml_dir)
                runs_dir = os.path.join(yaml_dir, 'runs')
                if (not os.path.isdir(runs_dir)):
                    os.mkdir(runs_dir)

                use_seed = sim_yaml['use_seed']
                seed = sim_yaml['seed']
                if (use_seed):
                    random.seed(seed)

                n_digits = math.ceil(math.log10(num_runs - 1))
                for i in range(num_runs):
                    sub_yaml = top_yaml
                    if (use_seed):
                        new_seed = random.randint(0, 1e9)
                        sub_yaml['/EkfCalNode']['ros__parameters']['sim_params']['seed'] = new_seed
                    sub_yaml['/EkfCalNode']['ros__parameters']['sim_params']['number_of_runs'] = 1
                    sub_yaml['/EkfCalNode']['ros__parameters']['sim_params']['run_number'] += i
                    if (time):
                        sub_yaml['/EkfCalNode']['ros__parameters']['sim_params']['max_time'] = time
                    sub_file = os.path.join(
                        runs_dir, '{}_{:0{:d}.0f}.yaml'.format(top_name, i, n_digits))
                    yaml_files.append(sub_file)
                    with open(sub_file, 'w') as f:
                        yaml.dump(sub_yaml, f)
            else:
                yaml_files = [yaml_file]
        except yaml.YAMLError as exc:
            print(exc)

    return yaml_files


def add_jobs(
    inputs: List[str],
    jobs=None,
    runs=None,
    time=None
):
    """Add simulation jobs to pool given list of top-level input yaml files."""
    cpu_count = jobs if (jobs) else multiprocessing.cpu_count() - 1
    pool = multiprocessing.Pool(cpu_count)

    for yaml_file in inputs:
        input_yaml_path = os.path.abspath(yaml_file)
        list_of_runs = generate_mc_from_yaml(
            input_yaml_path,
            runs=runs,
            time=time
        )
        for single_run in list_of_runs:
            pool.apply_async(run_sim, args=(single_run,), error_callback=print_err)

    pool.close()
    pool.join()


# TODO(jhartzer): Write tests
# TODO(jhartzer): Add lock file when simulation begins and delete when complete
if __name__ == '__main__':
    parser = InputParser()
    args = parser.parse_args()
    add_jobs(
        args.inputs,
        jobs=args.jobs,
        runs=args.runs,
        time=args.time
    )
