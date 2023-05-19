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

#include <gtest/gtest.h>

#include <string>

#include "infrastructure/data_logger.hpp"

TEST(test_data_logger, log_data) {
  std::string directory {""};
  std::string file_name {"test.log"};
  DataLogger data_logger(directory, file_name);
  data_logger.SetLogging(true);
  data_logger.DefineHeader("Val_1,Val_2,Val_3\n");
  data_logger.Log("1,2,3\n");
  data_logger.Log("4,5,6\n");
  data_logger.Log("7,8,9\n");
}
