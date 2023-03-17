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

import argparse
import multiprocessing
import os
import subprocess
import traceback
from typing import List


def print_err(err):
    print('error_callback()', err)
    traceback.print_exception(type(err), err, err.__traceback__)


def run_sim(yaml: str):
    yaml_dir = yaml.split('.yaml')[0] + os.sep
    if (not os.path.isdir(yaml_dir)):
        os.mkdir(yaml_dir)

    sim_path = os.path.join('..', '..', 'build', 'ekf_cal', 'sim_path')
    subprocess.run([sim_path, yaml, yaml_dir])
    return


def add_jobs(inputs: List[str]):
    cpu_count = multiprocessing.cpu_count() - 1
    pool = multiprocessing.Pool(cpu_count)

    for yaml in inputs:
        yaml_path = os.path.abspath(yaml)
        pool.apply_async(run_sim, args=(yaml_path,), error_callback=print_err)

    pool.close()
    pool.join()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('inputs', nargs='+', type=str)
    args = parser.parse_args()
    add_jobs(args.inputs)
