// Copyright 2023 Jacob Hartzer
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include <gtest/gtest.h>

#include <eigen3/Eigen/Eigen>

#include "infrastructure/sim/truth_engine_spline.hpp"
#include "infrastructure/sim/truth_engine_cyclic.hpp"

TEST(test_TruthEngineCyclic, Constructor) {
  Eigen::Vector3d pos_frequency{1, 2, 3};
  Eigen::Vector3d ang_frequency{4, 5, 6};
  Eigen::Vector3d pos_offset{1, 2, 3};
  Eigen::Vector3d ang_offset{0.1, 0.2, 0.3};
  double pos_amplitude = 1.0;
  double ang_amplitude = 0.1;

  TruthEngineCyclic truth_engine_cyclic(
    pos_frequency,
    ang_frequency,
    pos_offset,
    ang_offset,
    pos_amplitude,
    ang_amplitude
  );
}

TEST(test_TruthEngineSpline, Constructor) {
  std::vector<Eigen::Vector3d> positions;
  std::vector<Eigen::Vector3d> angles;

  positions.push_back(Eigen::Vector3d{0, 0, 0});
  positions.push_back(Eigen::Vector3d{1, 1, 1});
  positions.push_back(Eigen::Vector3d{2, 2, 2});

  angles.push_back(Eigen::Vector3d{0, 0, 0});
  angles.push_back(Eigen::Vector3d{1, 1, 1});
  angles.push_back(Eigen::Vector3d{2, 2, 2});

  TruthEngineSpline truth_engine_spline(positions, angles);
}
