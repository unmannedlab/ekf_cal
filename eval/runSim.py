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
