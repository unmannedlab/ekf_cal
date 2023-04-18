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

#include <string>

#include <rclcpp/rclcpp.hpp>

#include "../DebugLogger.hpp"


DebugLogger * DebugLogger::m_instancePointer = NULL;

void DebugLogger::setLogLevel(LogLevel level)
{
  if ((m_logLevel <= LogLevel::INFO) || (static_cast<LogLevel>(level) <= LogLevel::INFO)) {
    RCLCPP_INFO_STREAM(
      rclcpp::get_logger(
        "Logger"), "Log level set to: " <<
        m_logLevelNames[static_cast<std::underlying_type<LogLevel>::type>(m_logLevel)]);
  }
  m_logLevel = level;
}

void DebugLogger::setLogLevel(unsigned int level)
{
  if ((m_logLevel <= LogLevel::INFO) || (static_cast<LogLevel>(level) <= LogLevel::INFO)) {
    RCLCPP_INFO_STREAM(
      rclcpp::get_logger("Logger"),
      "Log level set to: " << m_logLevelNames[level]);
  }

  m_logLevel = static_cast<LogLevel>(level);
}

DebugLogger::~DebugLogger()
{
  if (m_logLevel <= LogLevel::INFO) {
    RCLCPP_INFO_STREAM(
      rclcpp::get_logger("Logger"), "Logger destroyed");
  }
}

void DebugLogger::log(LogLevel level, std::string message)
{
  const char * message_c_str = message.c_str();
  if (m_logLevel >= level) {
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
