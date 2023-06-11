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


/// @todo: Create grabber function from list of available debug loggers
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
private:
  static DebugLogger * m_instance_pointer;
  DebugLogger() {}

public:
  /// @todo re-enable once not a singleton
  // ~DebugLogger();

  ///
  /// @brief Log message
  /// @param level Level of log
  /// @param message Message contents of log
  ///
  void Log(LogLevel level, std::string message);


  ///
  /// @brief Delete copy constructor
  ///
  DebugLogger(const DebugLogger & obj) = delete;

  ///
  /// @brief Getter for DebugLogger singleton
  /// @return Pointer to DebugLogger singleton
  /// @todo Remove singleton pattern to support multiple log files with mutex
  ///
  static DebugLogger * GetInstance()
  {
    // If there is no instance of class
    // then we can create an instance.
    if (m_instance_pointer == NULL) {
      // We can access private members
      // within the class.
      m_instance_pointer = new DebugLogger();

      // returning the instance pointer
      return m_instance_pointer;
    } else {
      // if m_instance_pointer != NULL that means
      // the class already have an instance.
      // So, we are returning that instance
      // and not creating new one.
      return m_instance_pointer;
    }
  }

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

  ///
  /// @brief Output directory setter
  /// @param directory Output directory string
  ///
  void SetOutputDirectory(std::string directory);

private:
  LogLevel m_log_level = LogLevel::FATAL;
  std::string m_log_level_names[5] = {"FATAL", "ERROR", "WARN ", "INFO ", "DEBUG"};
  std::string m_output_directory = "";
};

#endif  // INFRASTRUCTURE__DEBUG_LOGGER_HPP_
