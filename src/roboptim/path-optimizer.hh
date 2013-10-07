// Copyright (C) 2011,2012 CNRS-LAAS
// Author: Florent Lamiraux
//
// This file is part of the hpp-wholebody-step-planner.
//
// hpp-wholebody-step-planner is free software: you can redistribute
// it and/or modify it under the terms of the GNU Lesser General
// Public License as published by the Free Software Foundation, either
// version 3 of the License, or (at your option) any later version.
//
// hpp-wholebody-step-planner is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with hpp-wholebody-step-planner.  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_WHOLEBODY_STEP_PLANNER_ROBOPTIM_PATH_OPTIMIZER_HH
# define HPP_WHOLEBODY_STEP_PLANNER_ROBOPTIM_PATH_OPTIMIZER_HH

# include <KineoWorks2/kwsPathOptimizer.h>

namespace hpp {
  namespace wholeBodyStepPlanner {
    namespace roboptim {
      HPP_KIT_PREDEF_CLASS (PathOptimizer);
      /// Path optimizer based on numerical optimization of B-splines
      class PathOptimizer : public CkwsPathOptimizer
      {
      public:
	static PathOptimizerShPtr create
	(const std::vector<CjrlGikStateConstraint*>& manifold);
	void setGoalConstraints
	(const std::vector<CjrlGikStateConstraint*>& goalConstraints);
      protected:
	PathOptimizer (const std::vector<CjrlGikStateConstraint*>& manifold);
	ktStatus init (PathOptimizerWkPtr weakPtr_);
	virtual ktStatus doOptimizePath (const CkwsPathShPtr& io_path);
      private:
	PathOptimizerWkPtr weakPtr_;
	std::vector<CjrlGikStateConstraint*> manifold_;
	std::vector<CjrlGikStateConstraint*> goalConstraints_;
      }; // class PathOptimizer
    } // namespace roboptim
  } // namespace wholeBodyStepPlanner
} // namespace hpp
#endif // HPP_WHOLEBODY_STEP_PLANNER_ROBOPTIM_PATH_OPTIMIZER_HH
