// Copyright (C) 2009 by Sebastien Dalibard.
//
// This file is part of the hpp-wholebody-step-planner.
//
// hpp-wholebody-step-planner is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// hpp-wholebody-step-planner is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with hpp-wholebody-step-planner.  If not, see <http://www.gnu.org/licenses/>.


#ifndef TESTS_COMMON_HH
# define TESTS_COMMON_HH
# include <stdexcept>
# include <iostream>
# include "config.h"

static const int TEST_FAILED = 10;
static const int TEST_SUCCEED = 0;

# define GENERATE_TEST()                                \
  int                                                   \
  main (int argc, char** argv)                          \
  {                                                     \
    if (argc == 2                                       \
        && std::string (argv[1]) == "--version")        \
      {                                                 \
        std::cout << PACKAGE_STRING << std::endl;       \
        return 0;                                       \
      }                                                 \
                                                        \
    int status = 0;                                     \
    try                                                 \
      {                                                 \
        status = run_test ();                           \
      }                                                 \
    catch (std::runtime_error& e)                       \
      {                                                 \
        std::cerr << e.what () << std::endl;            \
        return 1;                                       \
      }                                                 \
    catch (...)                                         \
      {                                                 \
        std::cerr << "Unexpected error" << std::endl;   \
        return 2;                                       \
      }                                                 \
    return status;                                      \
  }

#define CHECK_FAILURE(EXCEPTION, CMD)           \
  {                                             \
    bool failed = true;                         \
    try                                         \
      {                                         \
        CMD;                                    \
      }                                         \
    catch (EXCEPTION&)                          \
      {                                         \
        failed = false;                         \
      }                                         \
    catch (...)                                 \
      {}                                        \
    if (failed)                                 \
      return TEST_FAILED;                       \
  }

#endif //! TESTS_COMMON_HH
