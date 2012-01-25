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

    /// Path optimizer that optimize configurations between direct paths

    /// Iteratively tries to make configurations linking successive direct paths
    /// tend toward a target configuration.

    /// See PathOptimizer::doOptimizeOneStep and PathOptimizer::optimizeConfig
    /// for more details.
    class PathOptimizer : public CkwsLoopOptimizer
    {
    public:
      ~PathOptimizer();

      static PathOptimizerShPtr create();

      /// One step of optimization

      /// Loop over the pairs of successive direct paths. For each pair of
      /// successive direct,
      /// \li optimize the configuration linking the direct paths,
      /// \li build and validate new direct paths going to and starting from
      /// the optimized configuration,
      /// \li replace both direct paths if both are valid.
      virtual ktStatus doOptimizeOneStep(const CkwsPathShPtr & io_path);

      /// Set target configuration
      void targetConfig (CkwsConfigShPtr targetCfg);

      /// Get first last valid config in the direction of target configuration.

      /// Build a direct path from input configuration to target configuration,
      /// validate discretized configurations (step = 0.01) along direct path
      /// and return the last valid configuration. Only degrees of freedom
      /// specified by mask are considered (see PathOptimizer::setConfigMask).
      /// \return KD_ERROR if target configuration is not set, fail to create
      /// direct path between input and target configuration, input
      /// configuration is not valid.
      ktStatus optimizeConfig (CkwsConfig & io_cfg);

      /// Define which degrees of freedom are active.
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
