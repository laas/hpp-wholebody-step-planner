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

#ifndef C_HPP_WHOLEBODY_STEP_PLANNER_PATH_OPTIMIZER
#define C_HPP_WHOLEBODY_STEP_PLANNER_PATH_OPTIMIZER

# include <vector>

# include <KineoWorks2/kwsLoopOptimizer.h>


namespace hpp
{
  namespace wholeBodyStepPlanner
  {
    KIT_PREDEF_CLASS(PathOptimizer);

    class PathOptimizer : public CkwsLoopOptimizer
    {
    public:
      ~PathOptimizer();

      static PathOptimizerShPtr create();
      
      virtual ktStatus doOptimizeOneStep(const CkwsPathShPtr & io_path);

      void targetConfig (CkwsConfigShPtr targetCfg);

      ktStatus optimizeConfig (CkwsConfig & io_cfg);

      ktStatus setConfigMask(std::vector<bool> & i_mask);

    protected:

      PathOptimizer();

      ktStatus init( const PathOptimizerWkPtr i_weakPtr);

    private:

      PathOptimizerWkPtr wkPtr_;

      std::vector<bool> cfgMask_;

      CkwsConfigShPtr targetCfg_;

      double interpolationParam_;

    };
  }
}
# endif
