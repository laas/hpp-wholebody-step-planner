// Copyright (C) 2011 by Antonio El Khoury.
//

#ifndef C_HPP_WHOLEBODY_STEP_PLANNER_DISTANCE
#define C_HPP_WHOLEBODY_STEP_PLANNER_DISTANCE

# include <vector>

# include <KineoWorks2/kwsLoopOptimizer.h>


namespace hpp
{
  namespace wholeBodyStepPlanner
  {
    KIT_PREDEF_CLASS(Distance);

    class Distance : public CkwsDistance
    {
    public:
      ~Distance();

      static DistanceShPtr create();
      
      void targetConfig (CkwsConfigShPtr targetCfg);

      ktStatus setConfigMask(std::vector<bool> & i_mask);

      virtual double distance (const CkwsConfig &i_cfg1,
			       const CkwsConfig &i_cfg2) const;

      virtual double cost (const CkwsDirectPathConstShPtr &i_directPath) const;

    protected:

      Distance();

      ktStatus init( const DistanceWkPtr i_weakPtr);

    private:

      DistanceWkPtr wkPtr_;

      std::vector<bool> cfgMask_;

      CkwsConfigShPtr targetCfg_;
    };
  }
}
# endif
