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

#include <gtest/gtest.h>

#include <string>

#include "infrastructure/debug_logger.hpp"

TEST(test_ros_debug_logger, debug_logger) {
  auto logger = std::make_shared<DebugLogger>(LogLevel::DEBUG, "");

  logger->SetLogLevel(0);
  logger->SetLogLevel(1);
  logger->SetLogLevel(2);
  logger->SetLogLevel(3);
  logger->SetLogLevel(4);

  logger->SetLogLevel(LogLevel::FATAL);
  logger->SetLogLevel(LogLevel::ERROR);
  logger->SetLogLevel(LogLevel::WARN);
  logger->SetLogLevel(LogLevel::INFO);
  logger->SetLogLevel(LogLevel::DEBUG);

  logger->Log(LogLevel::DEBUG, "DEBUG");
  logger->Log(LogLevel::INFO, "INFO");
  logger->Log(LogLevel::WARN, "WARN");
  logger->Log(LogLevel::ERROR, "ERROR");
  logger->Log(LogLevel::FATAL, "FATAL");
}
