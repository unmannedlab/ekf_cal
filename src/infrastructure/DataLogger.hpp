// Copyright 2023 Jacob Hartzer
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

#ifndef INFRASTRUCTURE__DATALOGGER_HPP_
#define INFRASTRUCTURE__DATALOGGER_HPP_


#include <string>
#include <fstream>


///
/// @brief DataLogger class
///
class DataLogger
{
public:
  ///
  /// @brief DataLogger constructor
  /// @param output_directory Output directory for creating data log file
  /// @param file_name Name of data log file
  ///
  DataLogger(std::string output_directory, std::string file_name);

  ///
  /// @brief Log message
  /// @param message Message contents of log
  ///
  void Log(std::string message);

  ///
  /// @brief Function to set the output file header
  /// @param header Header string for output file
  ///
  void DefineHeader(std::string header);

  ///
  /// @brief Function to switch the logger on/off
  /// @param value Logger on/off value
  ///
  void SetLogging(bool value);

  ///
  /// @brief Output directory setter
  /// @param directory Output directory string
  ///
  void SetOutputDirectory(std::string directory);

private:
  bool m_initialized{false};
  std::string m_log_header{"\n"};
  std::ofstream m_log_file;
  bool m_logging_on {false};
  std::string m_log_file_path;
};

#endif  // INFRASTRUCTURE__DATALOGGER_HPP_
