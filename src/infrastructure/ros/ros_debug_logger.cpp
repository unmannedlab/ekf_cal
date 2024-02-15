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

#include <stddef.h>

#include <string>
#include <type_traits>

#include <rclcpp/rclcpp.hpp>

#include "../debug_logger.hpp"


DebugLogger * DebugLogger::m_instance_pointer = NULL;

void DebugLogger::SetLogLevel(LogLevel level)
{
  if ((m_log_level <= LogLevel::INFO) || (static_cast<LogLevel>(level) <= LogLevel::INFO)) {
    RCLCPP_INFO_STREAM(
      rclcpp::get_logger(
        "Logger"), "Log level set to: " <<
        m_log_level_names[static_cast<std::underlying_type<LogLevel>::type>(m_log_level)]);
  }
  m_log_level = level;
}

void DebugLogger::SetLogLevel(unsigned int level)
{
  if ((m_log_level <= LogLevel::INFO) || (static_cast<LogLevel>(level) <= LogLevel::INFO)) {
    RCLCPP_INFO_STREAM(
      rclcpp::get_logger("Logger"),
      "Log level set to: " << m_log_level_names[level]);
  }

  m_log_level = static_cast<LogLevel>(level);
}

void DebugLogger::Log(LogLevel level, std::string message)
{
  const char * message_c_str = message.c_str();
  if (m_log_level >= level) {
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
