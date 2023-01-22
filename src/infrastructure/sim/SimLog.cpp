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
#include <iostream>

#include "../Logger.hpp"


void Logger::setLogLevel(LogLevel level)
{
  if ((m_logLevel <= LogLevel::INFO) || (static_cast<LogLevel>(level) <= LogLevel::INFO)) {
    std::cout << "[" <<
      LogLevelNames[static_cast<std::underlying_type<LogLevel>::type>(LogLevel::INFO)] << "]: " <<
      "Log level set to: " <<
      LogLevelNames[static_cast<std::underlying_type<LogLevel>::type>(level)] << std::endl;
  }

  m_logLevel = level;
}

void Logger::setLogLevel(unsigned int level)
{
  if ((m_logLevel <= LogLevel::INFO) || (static_cast<LogLevel>(level) <= LogLevel::INFO)) {
    std::cout << "[" <<
      LogLevelNames[static_cast<std::underlying_type<LogLevel>::type>(LogLevel::INFO)] << "]: " <<
      "Log level set to: " <<
      LogLevelNames[static_cast<std::underlying_type<LogLevel>::type>(level)] << std::endl;
  }

  m_logLevel = static_cast<LogLevel>(level);
}

Logger::~Logger()
{
  if (m_logLevel <= LogLevel::INFO) {
    std::cout << "[" <<
      LogLevelNames[static_cast<std::underlying_type<LogLevel>::type>(LogLevel::INFO)] << "]: " <<
      "Logger destroyed" << std::endl;
  }
}

void Logger::log(LogLevel level, std::string message)
{
  if (m_logLevel <= level) {
    switch (level) {
      case LogLevel::DEBUG:
      case LogLevel::INFO:
        std::cout << "[" <<
          LogLevelNames[static_cast<std::underlying_type<LogLevel>::type>(level)] << "]: " <<
          message << std::endl;
        return;
      case LogLevel::WARN:
      case LogLevel::ERROR:
      case LogLevel::FATAL:
        std::cerr << "[" <<
          LogLevelNames[static_cast<std::underlying_type<LogLevel>::type>(level)] << "]: " <<
          message << std::endl;
        return;
    }
  }
}
