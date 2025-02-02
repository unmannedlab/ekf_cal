// Copyright 2024 Jacob Hartzer
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

#include "infrastructure/sim/truth_engine_smoother.hpp"
#include "infrastructure/sim/truth_engine.hpp"
#include "utility/custom_assertions.hpp"
#include "utility/sim/sim_rng.hpp"
#include "utility/type_helper.hpp"

TEST(test_TruthEngineSmoother, Oscillating) {
  double stationary_time {0.0};
  double max_time {6.0};
  std::vector<double> times {0, 1, 2, 3, 4, 5, 6};
  std::vector<double> positions{
    0, 0, 0,
    1, 0, 0,
    1, 1, 1,
    1, 0, 1,
    0, 0, 1,
    0, 0, 0,
    0, 0, 0};
  std::vector<double> angles{
    0.0, 0.0, 0.0,
    0.1, 0.0, 0.0,
    0.1, 0.1, 0.1,
    0.1, 0.0, 0.1,
    0.0, 0.0, 0.1,
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0};

  auto pos_errs = std::vector<double>{0.0, 0.0, 0.0};
  auto ang_errs = std::vector<double>{0.0, 0.0, 0.0};

  auto logger = std::make_shared<DebugLogger>(LogLevel::DEBUG, "");

  SimRNG rng;

  TruthEngineSmoother truth_engine_spline(
    times, positions, angles, pos_errs, ang_errs, stationary_time, max_time, 10.0, logger, rng);

  for (unsigned int i = 0; i < 70; ++i) {
    double time = static_cast<double>(i) / 10.0;
    auto pos = truth_engine_spline.GetBodyPosition(time);
    std::cout << time << " " << pos.transpose() << std::endl;
  }
}
