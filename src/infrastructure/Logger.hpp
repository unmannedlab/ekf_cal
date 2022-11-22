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

#ifndef INFRASTRUCTURE__LOGGER_HPP_
#define INFRASTRUCTURE__LOGGER_HPP_


/// TODO: Create generic logging class
/// TODO: Create grabber function from list of available loggers get_logger("Logger")->
#include <string>


enum class LogLevel
{
  DEBUG,
  INFO,
  WARN,
  ERROR,
  FATAL
};

/// @brief Logger class
/// @todo Make Logger class a singleton or set of singletons by logfile
class Logger
{
private:
  static Logger * instancePointer;
  Logger() {}
  // explicit Logger(LogLevel level);

public:
  ~Logger();
  void log(LogLevel level, std::string message);


  ///
  /// @brief Delete copy constructor
  ///
  Logger(const Logger & obj) = delete;

  static Logger * getInstance()
  {
    // If there is no instance of class
    // then we can create an instance.
    if (instancePointer == NULL) {
      // We can access private members
      // within the class.
      instancePointer = new Logger();

      // returning the instance pointer
      return instancePointer;
    } else {
      // if instancePointer != NULL that means
      // the class already have an instance.
      // So, we are returning that instance
      // and not creating new one.
      return instancePointer;
    }
  }

  void SetLogLevel(LogLevel level);

private:
  LogLevel logLevel = LogLevel::DEBUG;
  std::string LogLevelNames[5] = {"DEBUG", "INFO ", "WARN ", "ERROR", "FATAL"};
};

#endif  // INFRASTRUCTURE__LOGGER_HPP_
