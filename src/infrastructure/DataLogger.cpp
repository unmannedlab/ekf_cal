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

#include "infrastructure/DataLogger.hpp"

#include <iostream>
#include <fstream>


DataLogger::DataLogger(std::string path)
{
  /// @todo check if path exists
  m_logFile.open(path);
}


void DataLogger::log(std::string message)
{
  if (m_loggingOn) {
    if (!m_initialized) {
      m_logFile << m_logHeader;
      m_initialized = true;
    }
    m_logFile << message;
  }
}


void DataLogger::setLogging(bool value)
{
  m_loggingOn = value;
}


void DataLogger::defineHeader(std::string header)
{
  m_logHeader = header;
}
