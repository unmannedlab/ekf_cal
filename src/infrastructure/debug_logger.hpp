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

#ifndef INFRASTRUCTURE__DEBUG_LOGGER_HPP_
#define INFRASTRUCTURE__DEBUG_LOGGER_HPP_


#include <stddef.h>

#include <string>

enum class LogLevel
{
  FATAL,
  ERROR,
  WARN,
  INFO,
  DEBUG
};

///
/// @brief DebugLogger class
///
class DebugLogger
{
public:
  ///
  /// @brief DebugLogger Constructor
  /// @param log_level Logging Level
  /// @param log_directory Output directory
  ///
  DebugLogger(LogLevel log_level, const std::string & log_directory);

  ///
  /// @brief DebugLogger Constructor
  /// @param log_level Logging Level
  /// @param log_directory Output directory
  ///
  DebugLogger(unsigned int log_level, const std::string & log_directory);

  ///
  /// @brief Log message
  /// @param level Level of log
  /// @param message Message contents of log
  ///
  void Log(LogLevel level, const std::string & message);

  ///
  /// @brief Function to set the log level
  /// @param level LogLevel enumeration
  ///
  void SetLogLevel(LogLevel level);

  ///
  /// @brief Function to set the log level
  /// @param level LogLevel integer
  ///
  void SetLogLevel(unsigned int level);

private:
  LogLevel m_log_level;
  std::string m_log_directory;
  std::string m_log_level_names[5] = {"FATAL", "ERROR", "WARN ", "INFO ", "DEBUG"};
};

#endif  // INFRASTRUCTURE__DEBUG_LOGGER_HPP_
