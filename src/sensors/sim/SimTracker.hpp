// Copyright 2022 Jacob Hartzer
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

#ifndef SENSORS__SIM__SIMTRACKER_HPP_
#define SENSORS__SIM__SIMTRACKER_HPP_

#include <string>

#include "infrastructure/Logger.hpp"
#include "sensors/Camera.hpp"
#include "sensors/Sensor.hpp"
#include "sensors/sim/SimTypes.hpp"
#include "utility/sim/SimRNG.hpp"


class SimTrackerMessage : public SimMessage
{
public:
  SimTrackerMessage() {}
};

class SimTracker : public Camera
{
public:
  SimTrackerMessage generateMeasurement(double measurementTime);

private:
  double tBias{0};
  double uvError{1e-9};
  double accBias{0};
  double accError{1};
  double omgBias{0};
  double omgError{1};
  SimRNG m_rng;
};


#endif  // SENSORS__SIM__SIMTRACKER_HPP_
