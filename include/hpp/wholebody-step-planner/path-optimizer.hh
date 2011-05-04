// Copyright (C) 2011 by Sebastien Dalibard.
//

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
