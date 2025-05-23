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

#include <fstream>


DataLogger::DataLogger(const std::string & log_directory, const std::string & file_name)
{
  m_log_directory = log_directory;
  m_file_name = file_name;
}

DataLogger::DataLogger(
  const std::string & log_directory, const std::string & file_name,
  double logging_rate)
{
  m_log_directory = log_directory;
  m_file_name = file_name;
  m_rate = logging_rate;
}

void DataLogger::Log(const std::string & message)
{
  if (m_logging_on) {
    if (!m_initialized) {
      m_log_file.open(m_log_directory + m_file_name);
      m_log_file << m_log_header << std::endl;
      m_initialized = true;
    }
    m_log_file << message << std::endl;
    ++m_log_count;
  }
}

void DataLogger::RateLimitedLog(const std::string & message, double time)
{
  if (m_logging_on) {
    if (m_time_init != 0.0) {
      double log_count = static_cast<double>(m_log_count);
      double max_count = m_rate * (time - m_time_init);
      if ((m_rate == 0.0) || (log_count < max_count)) {
        Log(message);
      }
    } else {
      m_time_init = time;
      Log(message);
    }
  }
}

void DataLogger::EnableLogging()
{
  m_logging_on = true;
}


void DataLogger::SetOutputDirectory(const std::string & log_directory)
{
  m_log_directory = log_directory;
}


void DataLogger::SetOutputFileName(const std::string & file_name)
{
  m_file_name = file_name;
}


void DataLogger::DefineHeader(const std::string & header)
{
  m_log_header = header;
}

void DataLogger::SetLogRate(double rate)
{
  m_rate = rate;
}
