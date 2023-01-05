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

#include <iostream>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "../Logger.hpp"

void Logger::SetLogLevel(LogLevel level)
{
  if ((m_logLevel <= LogLevel::INFO) || (static_cast<LogLevel>(level) <= LogLevel::INFO)) {
    RCLCPP_INFO_STREAM(
      rclcpp::get_logger(
        "Logger"), "Log level set to: " <<
        LogLevelNames[static_cast<std::underlying_type<LogLevel>::type>(m_logLevel)]);
  }
  m_logLevel = level;
}

void Logger::SetLogLevel(unsigned int level)
{
  if ((m_logLevel <= LogLevel::INFO) || (static_cast<LogLevel>(level) <= LogLevel::INFO)) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("Logger"), "Log level set to: " << LogLevelNames[level]);
  }

  m_logLevel = static_cast<LogLevel>(level);
}

Logger::~Logger()
{
  if (m_logLevel <= LogLevel::INFO) {
    RCLCPP_INFO_STREAM(
      rclcpp::get_logger("Logger"), "Logger destroyed");
  }
}

void Logger::log(LogLevel level, std::string message)
{
  const char * message_c_str = message.c_str();
  if (m_logLevel <= level) {
    switch (level) {
      case LogLevel::DEBUG:
        RCLCPP_DEBUG(rclcpp::get_logger("Logger"), message_c_str);
        break;

      case LogLevel::INFO:
        RCLCPP_INFO(rclcpp::get_logger("Logger"), message_c_str);
        break;

      case LogLevel::WARN:
        RCLCPP_WARN(rclcpp::get_logger("Logger"), message_c_str);
        break;

      case LogLevel::ERROR:
        RCLCPP_ERROR(rclcpp::get_logger("Logger"), message_c_str);
        break;

      case LogLevel::FATAL:
        RCLCPP_FATAL(rclcpp::get_logger("Logger"), message_c_str);
        break;
    }
  }
}
