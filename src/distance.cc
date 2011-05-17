# include <iostream>

# include <KineoWorks2/kwsDevice.h>
#include <KineoWorks2/kwsConfig.h>

# include <hpp/wholebody-step-planner/distance.hh>

namespace hpp
{
  namespace wholeBodyStepPlanner
  {
    Distance::Distance()
      : CkwsDistance ()
    {
    }
    
    Distance::~Distance()
    {
    }

    DistanceShPtr
    Distance::create()
    {
      Distance * flatPtr = new Distance();
      DistanceShPtr shPtr ( flatPtr );
      DistanceWkPtr wkPtr ( shPtr );
      if ( flatPtr->init(wkPtr) != KD_OK ) shPtr.reset();
      return shPtr;
    }

    ktStatus
    Distance::init (const DistanceWkPtr i_weakPtr)
    {
      ktStatus success = this->CkwsDistance::init(i_weakPtr);
      if ( success == KD_OK ) wkPtr_ = i_weakPtr;
      return success;
    }

    void
    Distance::targetConfig (CkwsConfigShPtr targetCfg)
    {
      targetCfg_ = targetCfg;
    }

    ktStatus
    Distance::setConfigMask (std::vector<bool> &i_mask)
    {
      cfgMask_ = i_mask;
      return KD_OK;
    }

    double
    Distance::distance (const CkwsConfig &i_cfg1,
			const CkwsConfig &i_cfg2) const
    {
      if (!targetCfg_)
	return KD_ERROR;

      CkwsConfig targetCfg1 (i_cfg1);
      CkwsConfig targetCfg2 (i_cfg2);
      
      for (unsigned int i = 0; i < i_cfg1.device ()->countDofs (); i++)
	{
	  double dofValue1 = cfgMask_[i] ? targetCfg_->dofValue (i)
	    : i_cfg1.dofValue (i);
	  targetCfg1.dofValue (i, dofValue1);
	  double dofValue2 = cfgMask_[i] ? targetCfg_->dofValue (i)
	    : i_cfg2.dofValue (i);
	  targetCfg2.dofValue (i, dofValue2);
	}

      double distanceToHalfSitting
	= CkwsDistance::distance (i_cfg1, targetCfg1)
	+CkwsDistance::distance (i_cfg2, targetCfg2);

      double configDistance
	= (1 + distanceToHalfSitting) * CkwsDistance::distance (i_cfg1, i_cfg2);
      
      return configDistance;
    }

    double
    Distance::cost (const CkwsDirectPathConstShPtr &i_directPath) const
    {
      CkwsConfig startCfg (i_directPath->device ());
      CkwsConfig endCfg (i_directPath->device ());
      
      i_directPath->getConfigAtStart (startCfg);
      i_directPath->getConfigAtEnd (endCfg);

      return distance (startCfg, endCfg);
    }
  }
}
