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

#include "sensors/sim/SimIMU.hpp"
#include "infrastructure/sim/TruthEngine.hpp"


int main(int argc, char * argv[])
{
  // Define sensors to use (load config from yaml)

  auto truthEngine = std::make_shared<TruthEngine>();

  SimImuParams imuParams1;
  imuParams1.accError = 1e-3;
  imuParams1.omgError = 1e-3;
  SimIMU imu1(imuParams1, truthEngine);

  SimImuParams imuParams2;
  imuParams2.accError = 1e-3;
  imuParams2.omgError = 1e-3;
  imuParams2.posOffset = Eigen::Vector3d(0.1, 0, 0);
  SimIMU imu2(imuParams2, truthEngine);

  // Calculate sensor measurement times with errors

  // Calculate sensor measurements

  // Construct sensors and EKF

  // Run measurements through sensors and EKF

  // Plot results
  return 0;
}
