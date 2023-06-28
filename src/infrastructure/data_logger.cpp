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

#include "infrastructure/data_logger.hpp"

#include <iostream>
#include <fstream>


DataLogger::DataLogger(std::string output_directory, std::string file_name)
{
  m_output_directory = output_directory;
  m_file_name = file_name;
}


void DataLogger::Log(std::string message)
{
  if (m_logging_on) {
    if (!m_initialized) {
      /// @todo check if path exists
      m_log_file.open(m_output_directory + m_file_name);
      m_log_file << m_log_header;
      m_initialized = true;
    }
    /// @todo Flush is not always necessary. Add as option or remove when sim is faster
    m_log_file << message << std::flush;
  }
}


void DataLogger::SetLogging(bool value)
{
  m_logging_on = value;
}


void DataLogger::SetOutputDirectory(std::string output_directory)
{
  m_output_directory = output_directory;
}


void DataLogger::SetOutputFileName(std::string file_name)
{
  m_file_name = file_name;
}


void DataLogger::DefineHeader(std::string header)
{
  m_log_header = header;
}
