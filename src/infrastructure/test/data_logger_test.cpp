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

#include <gtest/gtest.h>

#include <string>

TEST(data_logger, data_logger_constructor_1) {
  DataLogger data_logger;

  data_logger.Log("a1,b1");
  data_logger.SetOutputDirectory("/temp/");
  data_logger.SetOutputFileName("data.csv");
  data_logger.DefineHeader("col1,col2");
  data_logger.EnableLogging();
  data_logger.Log("a1,b1");
}

TEST(data_logger, data_logger_constructor_2) {
  DataLogger data_logger("/temp/", "data.csv");

  data_logger.Log("a1,b1");
  data_logger.DefineHeader("col1,col2");
  data_logger.EnableLogging();
  data_logger.Log("a1,b1");
}

TEST(data_logger, data_logger_constructor_3) {
  DataLogger data_logger("/temp/", "data.csv", 1.0);

  data_logger.Log("a1,b1");
  data_logger.DefineHeader("col1,col2");
  data_logger.EnableLogging();
  data_logger.Log("a1,b1");
}
