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

# include <iostream>

# include <KineoWorks2/kwsDevice.h>
# include <KineoWorks2/kwsSteeringMethod.h>
# include <KineoWorks2/kwsValidatorDPCollision.h>

# include <hpp/util/debug.hh>
# include <hpp/kwsio/configuration.hh>

# include <hpp/wholebody-step-planner/path-optimizer.hh>

# define INTER_PARAM 0.01

namespace hpp
{
  namespace wholeBodyStepPlanner
  {
    PathOptimizer::PathOptimizer()
      :interpolationParam_(INTER_PARAM)
    {
    }

    PathOptimizer::~PathOptimizer()
    {
   }

    PathOptimizerShPtr
    PathOptimizer::create()
    {
      PathOptimizer * flatPtr = new PathOptimizer();
      PathOptimizerShPtr shPtr (flatPtr);
      PathOptimizerWkPtr wkPtr (shPtr);
      if (flatPtr->init(wkPtr) != KD_OK) shPtr.reset();
      return shPtr;
    }

    ktStatus
    PathOptimizer::init(const PathOptimizerWkPtr i_weakPtr)
    {
      ktStatus success = this->CkwsLoopOptimizer::init(i_weakPtr);
      if (success == KD_OK) wkPtr_ = i_weakPtr;
      return success;
    }

    void
    PathOptimizer::targetConfig (CkwsConfigShPtr targetCfg)
    {
      targetCfg_ = targetCfg;
    }

    ktStatus
    PathOptimizer::setConfigMask(std::vector<bool> &i_mask)
    {
      cfgMask_ = i_mask;
      return KD_OK;
    }

    ktStatus
    PathOptimizer::optimizeConfig (CkwsConfig & io_cfg)
    {
      hppDout (info, "input config = " << io_cfg);
      if (!targetCfg_)
	return KD_ERROR;

      // New target taking into account configuration mask.
      // Active dof are copied from targetCfg_,
      // non active dofs are copied from input configuration.
      CkwsConfig localTargetCfg(io_cfg);

      for (unsigned int i = 0; i<io_cfg.device()->countDofs(); i++)
	{
	  double dofValue = cfgMask_[i] ? targetCfg_->dofValue(i) :
	    io_cfg.dofValue(i);
	  localTargetCfg.dofValue(i,dofValue);
	}

      // Direct path between input configuration and target configuration.
      CkwsDirectPathShPtr dp = io_cfg.device ()->steeringMethod ()->
	makeDirectPath(io_cfg,localTargetCfg);

      if(!dp)
	return KD_ERROR;

      double param = interpolationParam_;
      CkwsConfig currentCfg = io_cfg;

      // Validate direct path for collision
      while ((param <= dp->length())
	      && currentCfg.isValid())
	{
	  dp->getConfigAtParam(param,currentCfg);
	  param += interpolationParam_;
	}

      // If collision, get latest valid config -> currentCfg.
      if (!currentCfg.isValid())
	dp->getConfigAtParam(param - interpolationParam_,currentCfg);

      if (!currentCfg.isValid())
	return KD_ERROR;

      io_cfg = currentCfg;
      hppDout (info, "output config = " << io_cfg);

      return KD_OK;
    }

    ktStatus
    PathOptimizer::doOptimizeOneStep(const CkwsPathShPtr & io_path)
    {
      std::cout << "Running in post-optimizer loop"<< std::endl;

      CkwsValidatorDPCollisionShPtr dpValidator =
	io_path->device ()->directPathValidators ()->
	retrieve<CkwsValidatorDPCollision> ();
      dpValidator->penetration (penetration ());

      CkwsDeviceShPtr device = io_path->device();
      CkwsSteeringMethodShPtr steeringMethod = device->steeringMethod();
      CkwsConfig cfgStart(device), cfgEnd(device), cfgMiddle(device),
	cfgMiddle_opt(device);

      io_path->getConfigAtStart(cfgStart);

      // Loop over direct paths starting at second one.
      hppDout (info, "nb direct path = " << io_path->countDirectPaths());
      for (unsigned int i = 1; i<io_path->countDirectPaths(); i++)
	{
	  bool hasBeenReplaced = false;

	  CkwsDirectPathConstShPtr dp = io_path->directPath(i);
	  dp->getConfigAtStart(cfgMiddle);
	  dp->getConfigAtEnd(cfgEnd);

	  if (optimizeConfig(cfgMiddle) == KD_OK)
	    {
	      CkwsDirectPathShPtr dp1 =
		steeringMethod->makeDirectPath(cfgStart,cfgMiddle);
	      CkwsDirectPathShPtr dp2 =
		steeringMethod->makeDirectPath(cfgMiddle,cfgEnd);

	      dpValidator->validate (*dp1);
	      dpValidator->validate (*dp2);

	      if ((dp1->isValid()) && (dp2->isValid()))
		{
		  CkwsPathShPtr replacingPath = CkwsPath::create(device);
		  replacingPath->appendDirectPath(dp1);
		  replacingPath->appendDirectPath(dp2);
		  if (io_path->replacePart(i-1,i+1,replacingPath) != KD_OK)
		    {
		    }
		  else
		    {
		      hasBeenReplaced = true;
		    }
		}
	    }
	  if (hasBeenReplaced)
	    cfgStart = cfgMiddle;
	  else
	    dp->getConfigAtStart(cfgStart);
	}
      return KD_OK;
    }

  }
}
