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

"""Define a common input parser for ekf_cal tools."""

import argparse


class InputParser:
    """Handles parsing of common command-line arguments for EKF evaluation tools."""

    def __init__(self):
        """Initializes the argparse parser with common arguments."""
        self.parser = argparse.ArgumentParser()
        self.parser.add_argument('inputs', nargs='+', type=str)
        self.parser.add_argument('-j', '--jobs', default=None, type=int)
        self.parser.add_argument('-n', '--runs', default=None, type=int)
        self.parser.add_argument('-t', '--time', default=None, type=float)
        self.parser.add_argument('-e', '--ext', default='png', type=str)
        self.parser.add_argument('--no_plot', action='store_true')
        self.parser.add_argument('--embed', action='store_true')
        self.parser.add_argument('--rate_line', action='store_true')
        self.parser.add_argument('--light', action='store_true')

    def parse_args(self):
        """Parses generic command-line arguments."""
        self.args = self.parser.parse_args()
        return self.args
