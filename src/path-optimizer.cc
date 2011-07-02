# include <iostream>

# include <KineoWorks2/kwsDevice.h>
# include <KineoWorks2/kwsSteeringMethod.h>
# include <KineoWorks2/kwsValidatorDPCollision.h>

# include "hpp/wholebody/step/planner/path-optimizer.hh"

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
      PathOptimizerShPtr shPtr ( flatPtr );
      PathOptimizerWkPtr wkPtr ( shPtr );
      if ( flatPtr->init(wkPtr) != KD_OK ) shPtr.reset();
      return shPtr;
    }

    ktStatus
    PathOptimizer::init( const PathOptimizerWkPtr i_weakPtr)
    {
      ktStatus success = this->CkwsLoopOptimizer::init(i_weakPtr);
      if ( success == KD_OK ) wkPtr_ = i_weakPtr;
      return success;
    }

    void
    PathOptimizer::targetConfig ( CkwsConfigShPtr targetCfg)
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
    PathOptimizer::optimizeConfig ( CkwsConfig & io_cfg)
    {
      if (!targetCfg_)
	return KD_ERROR;

      CkwsConfig localTargetCfg(io_cfg);
      
      for (unsigned int i = 0; i<io_cfg.device()->countDofs(); i++)
	{
	  double dofValue = cfgMask_[i] ? targetCfg_->dofValue(i) : io_cfg.dofValue(i);
	  localTargetCfg.dofValue(i,dofValue);
	}

      CkwsDirectPathShPtr dp = 
	io_cfg.device()->steeringMethod()->makeDirectPath(io_cfg,localTargetCfg);

      if(!dp)
	return KD_ERROR;

      double param = interpolationParam_;
      CkwsConfig currentCfg = io_cfg;

      while ( (param <= dp->length())
	      && currentCfg.isValid() ) 
	{
	  dp->getConfigAtParam(param,currentCfg);
	  param += interpolationParam_;
	}

      if (!currentCfg.isValid())
	dp->getConfigAtParam(param - interpolationParam_,currentCfg);

      if (!currentCfg.isValid() )
	return KD_ERROR;

      io_cfg = currentCfg;
      return KD_OK;
    }

    ktStatus
    PathOptimizer::doOptimizeOneStep(const CkwsPathShPtr & io_path)
    {
      std::cout << "Running in post-optimizer loop"<< std::endl;

      CkwsValidatorDPCollisionShPtr dpValidator = io_path->device ()->directPathValidators ()
	->retrieve<CkwsValidatorDPCollision> ();
      dpValidator->penetration (penetration ());
     
      CkwsDeviceShPtr device = io_path->device();
      CkwsSteeringMethodShPtr steeringMethod = device->steeringMethod();
      CkwsConfig cfgStart(device), cfgEnd(device), cfgMiddle(device), cfgMiddle_opt(device);
  
      io_path->getConfigAtStart(cfgStart);

      for ( unsigned int i = 1;i<io_path->countDirectPaths();i++)
	{
	  bool hasBeenReplaced = false;
    
	  CkwsDirectPathConstShPtr dp = io_path->directPath(i);
	  dp->getConfigAtStart(cfgMiddle);
	  dp->getConfigAtEnd(cfgEnd);
    

	  if ( optimizeConfig(cfgMiddle) == KD_OK)
	    {
	      CkwsDirectPathShPtr dp1 = steeringMethod->makeDirectPath(cfgStart,cfgMiddle);
	      CkwsDirectPathShPtr dp2 = steeringMethod->makeDirectPath(cfgMiddle,cfgEnd);
      
	      dpValidator->validate (*dp1);
	      dpValidator->validate (*dp2);

	      if ( (dp1->isValid()) && (dp2->isValid()))
		{
		  CkwsPathShPtr replacingPath = CkwsPath::create(device);
		  replacingPath->appendDirectPath(dp1);
		  replacingPath->appendDirectPath(dp2);
		  if ( io_path->replacePart(i-1,i+1,replacingPath) != KD_OK)
		    {
		    }
		  else
		    {
		      hasBeenReplaced = true;
		    }
		}
	    }
	  if ( hasBeenReplaced )
	    cfgStart = cfgMiddle;
	  else
	    dp->getConfigAtStart(cfgStart);
	}


      return KD_OK;
    }

  }
}
