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

#include <iostream>
#include <string>
#include <type_traits>

#include "../debug_logger.hpp"


DebugLogger * DebugLogger::m_instance_pointer = NULL;

void DebugLogger::SetLogLevel(LogLevel level)
{
  if ((m_log_level <= LogLevel::INFO) ||
    (static_cast<LogLevel>(level) <= LogLevel::INFO))
  {
    std::cout << "[" <<
      m_log_level_names[static_cast<std::underlying_type<LogLevel>::type>(LogLevel::INFO)
      ] << "]: " <<
      "Log level set to: " <<
      m_log_level_names[static_cast<std::underlying_type<LogLevel>::type>(level)] <<
      std::endl;
  }

  m_log_level = level;
}

void DebugLogger::SetLogLevel(unsigned int level)
{
  if ((m_log_level <= LogLevel::INFO) ||
    (static_cast<LogLevel>(level) <= LogLevel::INFO))
  {
    std::cout << "[" <<
      m_log_level_names[static_cast<std::underlying_type<LogLevel>::type>(LogLevel::INFO)
      ] << "]: " <<
      "Log level set to: " <<
      m_log_level_names[static_cast<std::underlying_type<LogLevel>::type>(level)] <<
      std::endl;
  }

  m_log_level = static_cast<LogLevel>(level);
}

void DebugLogger::Log(LogLevel level, std::string message)
{
  if (m_log_level >= level) {
    switch (level) {
      case LogLevel::DEBUG:
      case LogLevel::INFO:
        std::cout << "[" <<
          m_log_level_names[static_cast<std::underlying_type<LogLevel>::type>(level)] <<
          "]: " <<
          message << std::endl;
        return;
      case LogLevel::WARN:
      case LogLevel::ERROR:
      case LogLevel::FATAL:
        std::cerr << "[" <<
          m_log_level_names[static_cast<std::underlying_type<LogLevel>::type>(level)] <<
          "]: " <<
          message << std::endl;
        return;
    }
  }
}
