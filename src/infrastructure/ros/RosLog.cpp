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

#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <string>

#include "../Logger.hpp"

Logger::Logger(LogLevel level)
{
  logLevel = level;
  if (logLevel <= LogLevel::INFO) {
    std::cout << "[" <<
      LogLevelNames[static_cast<std::underlying_type<LogLevel>::type>(logLevel)] << "]: " <<
      "LOGGER set to: " <<
      LogLevelNames[static_cast<std::underlying_type<LogLevel>::type>(logLevel)] << std::endl;
  }
}

Logger::~Logger()
{
  if (logLevel <= LogLevel::WARN) {
    std::cout << "[" <<
      LogLevelNames[static_cast<std::underlying_type<LogLevel>::type>(logLevel)] << "]:" <<
      " LOGGER destroyed" << std::endl;
  }
}

void Logger::log(LogLevel level, std::string message)
{
  if (logLevel <= level) {
    switch (logLevel) {
      case LogLevel::DEBUG:
        RCLCPP_DEBUG(rclcpp::get_logger("Logger"), message);

      case LogLevel::INFO:
        RCLCPP_INFO(rclcpp::get_logger("Logger"), message);

      case LogLevel::WARN:
        RCLCPP_WARN(rclcpp::get_logger("Logger"), message);

      case LogLevel::ERROR:
        RCLCPP_ERROR(rclcpp::get_logger("Logger"), message);

      case LogLevel::FATAL:
        RCLCPP_FATAL(rclcpp::get_logger("Logger"), message);
    }
  }
}
