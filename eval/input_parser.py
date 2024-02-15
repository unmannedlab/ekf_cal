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

"""Define a common input parser for EKF-CAL tools."""

import argparse


class InputParser:

    def __init__(self):
        self.parser = argparse.ArgumentParser()
        self.parser.add_argument('inputs', nargs='+', type=str)
        self.parser.add_argument('-j', '--jobs', default=None, type=int)
        self.parser.add_argument('-n', '--runs', default=None, type=int)
        self.parser.add_argument('-t', '--time', default=None, type=float)
        self.parser.add_argument('-e', '--ext', default='png', type=str)
        self.parser.add_argument('--show', action='store_true')
        self.parser.add_argument('--embed', action='store_true')
        self.parser.add_argument('--rate_line', action='store_true')

    def parse_args(self):
        self.args = self.parser.parse_args()
        return self.args
