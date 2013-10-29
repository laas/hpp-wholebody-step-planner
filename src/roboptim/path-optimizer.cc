// Copyright (C) 2011,2012 CNRS-LAAS
// Author: Sebastien Dalibard.
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

#include <hpp/util/debug.hh>
#include <hpp/roboptim/spline-directpath.hh>
#include "../src/roboptim/path-optimizer.hh"

namespace hpp {
  namespace wholeBodyStepPlanner {
    namespace roboptim {
      PathOptimizerShPtr PathOptimizer::create
      (const std::vector<CjrlGikStateConstraint*>& manifold)
      {
	PathOptimizer* optimizer = new PathOptimizer (manifold);
	PathOptimizerShPtr shPtr (optimizer);
	PathOptimizerWkPtr wkPtr (shPtr);

	if (optimizer->init(wkPtr) != KD_OK) {
	  shPtr.reset ();
	}
	return shPtr;
      }

      PathOptimizer::PathOptimizer (const std::vector<CjrlGikStateConstraint*>&
				    manifold) :
	CkwsPathOptimizer (), manifold_ (manifold),
	goalConstraints_ ()
      {}

      void PathOptimizer::setGoalConstraints
      (const std::vector<CjrlGikStateConstraint*>& goalConstraints)
      {
	goalConstraints_ = goalConstraints;
      }

      ktStatus PathOptimizer::init (PathOptimizerWkPtr weakPtr)
      {
	if (CkwsPathOptimizer::init (weakPtr) != KD_OK) return KD_ERROR;
	weakPtr_ = weakPtr;
	return KD_OK;
      }

      ktStatus PathOptimizer::doOptimizePath (const CkwsPathShPtr& io_path)
      {
	hppDout (info, "Creating spline direct path.");
	hppDout (info, "Path length: " << io_path->length ());
	assert (io_path->length () > 0);
	CkwsDirectPathShPtr spline = hpp::roboptim::SplineDirectPath::create
	  (io_path, 10, manifold_, goalConstraints_);
	assert (spline);
	io_path->clear ();
	io_path->appendDirectPath (spline);
	return KD_OK;
      }
    } // namespace roboptim
  } // namespace wholeBodyStepPlanner
} // namespace hpp
