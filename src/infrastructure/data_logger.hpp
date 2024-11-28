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

#ifndef INFRASTRUCTURE__DATA_LOGGER_HPP_
#define INFRASTRUCTURE__DATA_LOGGER_HPP_

#include <string>
#include <fstream>

///
/// @brief DataLogger class
///
class DataLogger
{
public:
  DataLogger() {}

  ///
  /// @brief DataLogger constructor
  /// @param output_directory Output directory for creating data log file
  /// @param file_name Name of data log file
  ///
  DataLogger(std::string output_directory, std::string file_name);

  ///
  /// @brief DataLogger constructor
  /// @param output_directory Output directory for creating data log file
  /// @param file_name Name of data log file
  /// @param logging_rate Logging rate
  ///
  DataLogger(std::string output_directory, std::string file_name, double logging_rate);

  ///
  /// @brief Log message
  /// @param message Message contents of log
  ///
  void Log(std::string message);

  ///
  /// @brief Log rate-limited messages
  /// @param message Message contents of log
  /// @param time Message log time for rate-limited logging
  ///
  void RateLimitedLog(std::string message, double time);

  ///
  /// @brief Function to set the output file header
  /// @param header Header string for output file
  ///
  void DefineHeader(std::string header);

  ///
  /// @brief Function to enable logging
  ///
  void EnableLogging();

  ///
  /// @brief Output directory setter
  /// @param output_directory Output directory string
  ///
  void SetOutputDirectory(std::string output_directory);

  ///
  /// @brief Output file name setter
  /// @param file_name Output file name
  ///
  void SetOutputFileName(std::string file_name);

  ///
  /// @brief Data logging rate setter
  /// @param rate Data logging rate
  ///
  void SetLogRate(double rate);

private:
  bool m_initialized{false};
  std::string m_log_header{""};
  std::ofstream m_log_file;
  bool m_logging_on {false};
  std::string m_output_directory {""};
  std::string m_file_name {"default.log"};
  double m_rate{0.0};
  double m_time_init{0};
  unsigned int m_log_count{0};
};

#endif  // INFRASTRUCTURE__DATA_LOGGER_HPP_
