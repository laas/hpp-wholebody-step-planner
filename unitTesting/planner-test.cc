/*
 *  Copyright
 */

#include <iostream>
#include <vector>
#include <fstream>

#include <hpp/corbaserver/openhrp.hh>

#include <kprParserXML/kprParserManager.h>
#include <KineoModuleManager/kppModuleManager.h>
#include <KineoGUI/kppOpenFileCommand.h>
#include <KineoController/kppSaveDocumentCommand.h>
#include <KineoModel/kppDofComponent.h>

#include "planner-test.hh"

namespace hpp
{
  namespace wholeBodyStepPlanner
  {
    PlannerTest::PlannerTest (char * i_envFile,
			      char * i_pathFile,
			      const double i_penetration,
			      const double i_samplingPeriod)
      : attPlanner_ ( NULL )
    {
      attEnvFile_ = i_envFile;
      attPathFile_ = i_pathFile;
      attPenetration_ = i_penetration;
      attPlanner_ = new Planner (i_samplingPeriod);
    }

    void PlannerTest::initScene()
    {
      hpp::corbaServer::impl::OpenHRP openHrp  (attPlanner_);
      
      if (attPlanner_->parseFile (attEnvFile_) != KD_OK)
	{
	  std::cerr << "Unable to load env file "<< std::endl;
	}
      
      if (openHrp.loadHrp2Model (attPenetration_) !=  KD_OK)
	{
	  std::cerr << "PlannerTest::initScene(): error in loading HRP2"<< std::endl;
	}

      attPlanner_->initializeProblem ();

      if (initAndGoalConfig () != KD_OK)
	{
	  std::cerr << "ERROR: init and goal configs could not be intialized" << std::endl;
	}

      std::cout << "initScene done" << std::endl;
    }
   
    ktStatus PlannerTest::initAndGoalConfig ()
    {
      unsigned int dofNb;
      std::fstream pathFile (attPathFile_);

      pathFile >> dofNb;
      if (dofNb != attPlanner_->robotIthProblem (0)->countDofComponents ())
	{
	  std::cerr
	    << "ERROR: dof number in path file does not math that of device."
	    << std::endl;
	  
	  return KD_ERROR;
	}

      CkwsConfigShPtr iCfg
	= CkwsConfig::create (attPlanner_->robotIthProblem (0)->kwsDevice ());
      CkwsConfigShPtr fCfg
	= CkwsConfig::create (attPlanner_->robotIthProblem (0)->kwsDevice ());
      
      double dofValue;
      for (unsigned int dof = 0; dof < dofNb; ++dof)
	{
	  pathFile >> dofValue;
	  iCfg->dofValue (dof, dofValue);
	}
      for (unsigned int dof = 0; dof < dofNb; ++dof)
	{
	  pathFile >> dofValue;
	  fCfg->dofValue (dof, dofValue);
	}
      
      attPlanner_->initConfIthProblem (0, iCfg);
      attPlanner_->goalConfIthProblem (0, fCfg);

      double dofMinValue, dofMaxValue;

      pathFile >> dofMinValue;
      pathFile >> dofMaxValue;
      attPlanner_->robotIthProblem (0)->dofComponent(0)->isBounded (true);
      attPlanner_->robotIthProblem (0)->dofComponent(0)
	->bounds (dofMinValue, dofMaxValue);

      pathFile >> dofMinValue;
      pathFile >> dofMaxValue;
      attPlanner_->robotIthProblem (0)->dofComponent(1)->isBounded (true);
      attPlanner_->robotIthProblem (0)->dofComponent(1)
	->bounds (dofMinValue, dofMaxValue);
      
      pathFile >> dofMinValue;
      pathFile >> dofMaxValue;
      attPlanner_->robotIthProblem (0)->dofComponent(5)->isBounded (true);
      attPlanner_->robotIthProblem (0)->dofComponent(5)
	->bounds (dofMinValue, dofMaxValue);

      pathFile.close ();

      return KD_OK;
    }
    
    ktStatus PlannerTest::solveProblem ()
    {
      if (attPlanner_->solve() == KD_OK)
	return KD_OK;
      else
	return KD_ERROR;
    }

    ktStatus PlannerTest::findDynamicPath ()
    {
      CkwsPathShPtr path = attPlanner_->hppProblem (0)
	->getIthPath (attPlanner_->hppProblem (0)->getNbPaths () -1);

      if (!path)
	{
	  std::cerr << "ERROR: could not retrieve path to animate" << std::endl;
	  return KD_ERROR;
	}
      else
	{
	  attPlanner_->humanoidRobot ()->userConstraints ()
	    ->remove (attPlanner_->wholeBodyConstraint ());

	  CkwsPathShPtr dynamicPath =  attPlanner_->findDynamicPath (path);
	  
	  if (!dynamicPath)
	    {
	      std::cerr << "ERROR: could not animate path" << std::endl;
	      return KD_ERROR;
	    }
	  else return KD_OK;
	}
    }
  }
}
