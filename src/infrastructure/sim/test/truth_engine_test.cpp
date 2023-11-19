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
  double max_time {2.0};
  std::vector<std::vector<double>> positions;
  std::vector<std::vector<double>> angles;

  positions.push_back(std::vector<double>{0, 0, 0});
  positions.push_back(std::vector<double>{1, 1, 1});
  positions.push_back(std::vector<double>{2, 2, 2});

  angles.push_back(std::vector<double>{0, 0, 0});
  angles.push_back(std::vector<double>{1, 1, 1});
  angles.push_back(std::vector<double>{2, 2, 2});

  TruthEngineSpline truth_engine_spline(max_time, positions, angles);

  Eigen::Vector3d pos_0 = truth_engine_spline.GetBodyPosition(0.0);
  Eigen::Vector3d pos_1 = truth_engine_spline.GetBodyPosition(1.0);
  Eigen::Vector3d pos_2 = truth_engine_spline.GetBodyPosition(2.0);
}
