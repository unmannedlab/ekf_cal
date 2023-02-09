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

#include "ekf/Types.hpp"

#include <eigen3/Eigen/Eigen>
#include <vector>
#include <memory>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

#include "utility/TypeHelper.hpp"

State & operator+=(State & lState, State & rState)
{
  lState.bodyState.position += lState.bodyState.position;
  lState.bodyState.velocity += lState.bodyState.velocity;
  lState.bodyState.acceleration += lState.bodyState.acceleration;
  lState.bodyState.orientation *= lState.bodyState.orientation;
  lState.bodyState.angularVelocity += lState.bodyState.angularVelocity;
  lState.bodyState.angularAcceleration += lState.bodyState.angularAcceleration;

  for (auto & imuIter : lState.imuStates) {
    unsigned int imuID = imuIter.first;
    lState.imuStates[imuID].position += rState.imuStates[imuID].position;
    lState.imuStates[imuID].orientation *= rState.imuStates[imuID].orientation;
    lState.imuStates[imuID].accBias += rState.imuStates[imuID].accBias;
    lState.imuStates[imuID].omgBias += rState.imuStates[imuID].omgBias;
  }

  for (auto & camIter : lState.camStates) {
    unsigned int imuID = camIter.first;
    lState.camStates[imuID].position += rState.camStates[imuID].position;
    lState.camStates[imuID].orientation *= rState.camStates[imuID].orientation;
    for (unsigned int i = 0; i < lState.camStates[imuID].augmentedStates.size(); ++i) {
      AugmentedState & lAugState = lState.camStates[imuID].augmentedStates[i];
      AugmentedState & rAugState = rState.camStates[imuID].augmentedStates[i];
      lAugState.position += rAugState.position;
      lAugState.orientation *= rAugState.orientation;
    }
  }

  return lState;
}

State & operator+=(State & lState, Eigen::VectorXd & rVector)
{
  lState.bodyState.position += rVector.segment<3>(0);
  lState.bodyState.velocity += rVector.segment<3>(3);
  lState.bodyState.acceleration += rVector.segment<3>(6);
  lState.bodyState.orientation *= rotVecToQuat(rVector.segment<3>(9));
  lState.bodyState.angularVelocity += rVector.segment<3>(12);
  lState.bodyState.angularAcceleration += rVector.segment<3>(15);

  unsigned int n = 18;
  for (auto & imuIter : lState.imuStates) {
    unsigned int imuID = imuIter.first;
    lState.imuStates[imuID].position += rVector.segment<3>(n + 0);
    lState.imuStates[imuID].orientation *= rotVecToQuat(rVector.segment<3>(n + 3));
    lState.imuStates[imuID].accBias += rVector.segment<3>(n + 6);
    lState.imuStates[imuID].omgBias += rVector.segment<3>(n + 9);
    n += 12;
  }

  for (auto & camIter : lState.camStates) {
    camIter.second.position += rVector.segment<3>(n + 0);
    camIter.second.orientation *= rotVecToQuat(rVector.segment<3>(n + 3));
    n += 6;
    for (unsigned int i = 0; i < camIter.second.augmentedStates.size(); ++i) {
      camIter.second.augmentedStates[i].imuPosition += rVector.segment<3>(n + 0);
      camIter.second.augmentedStates[i].imuOrientation *= rotVecToQuat(rVector.segment<3>(n + 3));
      camIter.second.augmentedStates[i].position += rVector.segment<3>(n + 6);
      camIter.second.augmentedStates[i].orientation *= rotVecToQuat(rVector.segment<3>(n + 9));
      n += 12;
    }
  }

  return lState;
}


State::State() {}


Eigen::VectorXd State::toVector()
{
  Eigen::VectorXd outVec = Eigen::VectorXd::Zero(getStateSize());

  outVec.segment<3>(0) = bodyState.position;
  outVec.segment<3>(3) = bodyState.velocity;
  outVec.segment<3>(6) = bodyState.acceleration;
  outVec.segment<3>(9) = quatToRotVec(bodyState.orientation);
  outVec.segment<3>(12) = bodyState.angularVelocity;
  outVec.segment<3>(15) = bodyState.angularAcceleration;
  unsigned int n = 18;

  for (auto const & imuIter : imuStates) {
    outVec.segment<3>(n + 0) = imuIter.second.position;
    outVec.segment<3>(n + 3) = quatToRotVec(imuIter.second.orientation);
    outVec.segment<3>(n + 6) = imuIter.second.accBias;
    outVec.segment<3>(n + 9) = imuIter.second.omgBias;
    n += 12;
  }

  for (auto const & camIter : camStates) {
    outVec.segment<3>(n + 0) = camIter.second.position;
    outVec.segment<3>(n + 3) = quatToRotVec(camIter.second.orientation);
    n += 6;
    for (auto const & augState : camIter.second.augmentedStates) {
      outVec.segment<3>(n + 0) = augState.position;
      outVec.segment<3>(n + 3) = quatToRotVec(augState.orientation);
      n += 6;
    }
  }

  return outVec;
}


unsigned int State::getStateSize()
{
  unsigned int stateSize = 18;
  stateSize += 6 * imuStates.size();

  for (auto const & camIter : camStates) {
    stateSize += 6 + 12 * camIter.second.augmentedStates.size();
  }
  return stateSize;
}
