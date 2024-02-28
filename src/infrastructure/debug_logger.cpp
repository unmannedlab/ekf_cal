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

#include "infrastructure/debug_logger.hpp"

#include <string>


DebugLogger::DebugLogger(LogLevel log_level, std::string output_directory)
: m_log_level(log_level),
  m_output_directory(output_directory) {}

DebugLogger::DebugLogger(unsigned int log_level, std::string output_directory)
: m_log_level(static_cast<LogLevel>(log_level)),
  m_output_directory(output_directory) {}
