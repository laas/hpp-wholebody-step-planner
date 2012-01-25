# Copyright (c) 2011 CNRS
# Author: Florent Lamiraux
#
# This file is part of hpp-wholebody-step-planner.
# hpp-wholebody-step-planner is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp-wholebody-step-planner is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-wholebody-step-planner.  If not, see
# <http://www.gnu.org/licenses/>.

# Requires at least CMake 2.6 to configure the package.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(CXX_DISABLE_WERROR true)
SET(CUSTOM_HEADER_DIR "hpp/wholebody-step-planner")

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/cpack.cmake)

SET(PROJECT_NAME hpp-wholebody-step-planner)
SET(PROJECT_DESCRIPTION "Algorithm to plan whole-body and walk planning trajectories for humanoid robots by dichotomy")
SET(PROJECT_URL "http://github.com/laas/hpp-wholebody-step-planner")

SET(${PROJECT_NAME}_HEADERS
  include/hpp/wholebody-step-planner/planner.hh
  include/hpp/wholebody-step-planner/config-motion-constraint.hh
  include/hpp/wholebody-step-planner/position-motion-constraint.hh
  include/hpp/wholebody-step-planner/rotation-motion-constraint.hh
  include/hpp/wholebody-step-planner/path-optimizer.hh
  include/hpp/wholebody-step-planner/fwd.hh
)

SETUP_PROJECT()


# Search for dependencies.
ADD_REQUIRED_DEPENDENCY("hpp-roboptim >= 0.1")
ADD_REQUIRED_DEPENDENCY("hpp-constrained >= 0.1")
ADD_REQUIRED_DEPENDENCY("hpp-constrained-planner >= 0.1")
ADD_REQUIRED_DEPENDENCY("kwsPlus >= 1.8")
ADD_REQUIRED_DEPENDENCY("hpp-util >= 0.4")
PKG_CONFIG_APPEND_LIBS(${PROJECT_NAME})

ADD_SUBDIRECTORY(src)

SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()
