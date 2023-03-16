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

#ifndef INFRASTRUCTURE__DEBUGLOGGER_HPP_
#define INFRASTRUCTURE__DEBUGLOGGER_HPP_


/// TODO: Create grabber function from list of available debug loggers get_Debuglogger("DebugLogger")->
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
  static DebugLogger * m_instancePointer;
  DebugLogger() {}

public:
  ~DebugLogger();

  ///
  /// @brief Log message
  /// @param level Level of log
  /// @param message Message contents of log
  ///
  void log(LogLevel level, std::string message);


  ///
  /// @brief Delete copy constructor
  ///
  DebugLogger(const DebugLogger & obj) = delete;

  ///
  /// @brief Getter for DebugLogger singleton
  /// @return Pointer to DebugLogger singleton
  /// @todo Remove singleton pattern to support multiple log files with mutex
  ///
  static DebugLogger * getInstance()
  {
    // If there is no instance of class
    // then we can create an instance.
    if (m_instancePointer == NULL) {
      // We can access private members
      // within the class.
      m_instancePointer = new DebugLogger();

      // returning the instance pointer
      return m_instancePointer;
    } else {
      // if m_instancePointer != NULL that means
      // the class already have an instance.
      // So, we are returning that instance
      // and not creating new one.
      return m_instancePointer;
    }
  }

  ///
  /// @brief Function to set the log level
  /// @param level LogLevel enumeration
  ///
  static void setLogLevel(LogLevel level);

  ///
  /// @brief Function to set the log level
  /// @param level LogLevel integer
  ///
  static void setLogLevel(unsigned int level);

  ///
  /// @brief Output directory setter
  /// @param directory Output directory string
  ///
  static void setOutputDirectory(std::string directory);

private:
  static std::string m_logLevelNames[5];
  static LogLevel m_logLevel;
  static std::string m_outputDirectory;
};

#endif  // INFRASTRUCTURE__DEBUGLOGGER_HPP_
