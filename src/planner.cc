#include <sstream>

# include <KineoWorks2/kwsRoadmap.h>
# include <KineoWorks2/kwsDiffusingRdmBuilder.h>
# include <KineoWorks2/kwsRandomOptimizer.h>

#include <KineoModel/kppConfigComponent.h>

# include <kwsPlus/roadmap/kwsPlusLTRdmBuilder.h>
# include <hpp/gik/task/generic-task.hh>

# include <hpp/util/debug.hh>

# include <hppModel/hppJoint.h>

# include <tlcWholeBodyPlanner/tlcGikCfgOptimizer.h>

# include <hpp/wholebody-step-planner/planner.hh>
# include <hpp/wholebody-step-planner/config-motion-constraint.hh>
# include <hpp/wholebody-step-planner/rotation-motion-constraint.hh>
# include <hpp/wholebody-step-planner/path-optimizer.hh>


# define PARAM_PRECISION 0.01
# define TASK_PRECISION 1e-4

/* Pattern Generator default parameters */
# define END_COEFF 0.5
# define START_SHIFT_TIME 0.1
# define END_SHIFT_TIME 0.1
# define FOOT_FILGHT_TIME 0.8
# define STEP_HEIGHT 0.05

namespace hpp
{
  namespace wholeBodyStepPlanner
  {

    Planner::Planner(double samplingPeriod )
      :humanoidRobot_(),
       gikStandingRobot_(NULL),
       samplingPeriod_(samplingPeriod),
       gikManager_(),
       wholeBodyConstraint_(),
       zmpEndCoeff_(END_COEFF),
       zmpStartShiftTime_(START_SHIFT_TIME),
       zmpEndShiftTime_(END_SHIFT_TIME),
       footFlightTime_(FOOT_FILGHT_TIME),
       stepHeight_(STEP_HEIGHT),
       waistPlaneConstraint_(NULL),
       waistParallelConstraint_(NULL),
       relativeRightFootTransformation_(),
       relativeLeftFootTransformation_(),
       waistZ_(1.0),
       minX_(-1),
       maxX_(1),
       minY_(-1),
       maxY_(1),
       minTheta_(-1),
       maxTheta_(1),
       paramPrecision_(PARAM_PRECISION),
       timestamp_ (0),
       posFile_
       (debug::getFilename ("trajectory.pos", "hpp-wholebody-step-planner").c_str ()),
       zmpFile_
       (debug::getFilename ("trajectory.zmp", "hpp-wholebody-step-planner").c_str ()),
       zmpMeasuredFile_
       (debug::getFilename ("trajectory.zmpmeas", "hpp-wholebody-step-planner").c_str ()),
       rpyFile_
       (debug::getFilename ("trajectory.rpy", "hpp-wholebody-step-planner").c_str ())
    {
      std::cout << "ParamPrecision: " 
		<<  paramPrecision_ 
		<< std::endl;
    }

    Planner::~Planner()
    {
      if (waistPlaneConstraint_) delete waistPlaneConstraint_ ;
      if (waistParallelConstraint_) delete waistParallelConstraint_ ;
      if (gikStandingRobot_) delete gikStandingRobot_;
      if (currentGikMotion_) delete currentGikMotion_;
      if (validGikMotion_.size () > 0)
	for (unsigned int i = 0; i < validGikMotion_.size (); ++i)
	  delete validGikMotion_[i];
    }

    ChppHumanoidRobotShPtr Planner::humanoidRobot ()
    {
      return humanoidRobot_;
    }

    ChppGikStandingRobot* Planner::robot ()
    {
      return gikStandingRobot_;
    }

    vector<ChppRobotMotion*> Planner::robotMotions ()
    {
      return validGikMotion_;
    }

    double Planner::zmpStartShiftTime ()
    {
      return zmpStartShiftTime_;
    }

    double Planner::zmpEndShiftTime ()
    {
      return zmpEndShiftTime_;
    }

    double Planner::footFlightTime ()
    {
      return footFlightTime_;
    }

    std::map<ChppGikFootprint*, double> Planner::stepFracOfFootprint ()
    {
      return stepFracOfFootprint_;
    }

    std::vector<Planner::footprintOfParam_t> Planner::resultFootprints ()
    {
      return resultFootprints_;
    }

    std::map<double,double> Planner::paramOfTime ()
    {
      return paramOfTime_;
    }

    wholeBodyConstraintShPtr Planner::wholeBodyConstraint ()
    {
      return wholeBodyConstraint_;
    }

    CtlcGraspBallGoalGeneratorShPtr Planner::getGoalTask()
    {
      return goalConfigGenerator_;
    }
    
    ktStatus
    Planner::initializeProblem()
    {

      std::cout <<  "Initialize problem(): paramPrecision: "
		<< paramPrecision_ << std::endl;

      unsigned nbRobots = getNbHppProblems ();
      if (nbRobots < 1)
	{
	  return KD_ERROR;
	}

      humanoidRobot_ = KIT_DYNAMIC_PTR_CAST
	(ChppHumanoidRobot, robotIthProblem (0));

      if (!humanoidRobot_)
	{
	  return KD_ERROR;
	}

      gikStandingRobot_ = new ChppGikStandingRobot(*humanoidRobot_);
      gikManager_ =  CtlcGikManager::create(humanoidRobot_);
   

      std::string property,value;
      property="ComputeZMP"; value="true";humanoidRobot_->setProperty ( property,value );
      property="TimeStep"; value="0.005";humanoidRobot_->setProperty ( property,value );
      property="ComputeAccelerationCoM"; value="true";humanoidRobot_->setProperty ( property,value );
      property="ComputeBackwardDynamics"; value="true";humanoidRobot_->setProperty ( property,value );
      property="ComputeMomentum"; value="true";humanoidRobot_->setProperty ( property,value );
      property="ComputeAcceleration"; value="true";humanoidRobot_->setProperty ( property,value );
      property="ComputeVelocity"; value="true";humanoidRobot_->setProperty ( property,value );
      property="ComputeSkewCom"; value="true";humanoidRobot_->setProperty ( property,value );
      property="ComputeCoM"; value="true";humanoidRobot_->setProperty ( property,value );


 

      /* Building the tasks */
      waistZ_ = 
	MAL_S4x4_MATRIX_ACCESS_I_J(humanoidRobot_->waist()->currentTransformation(),2,3);

      vector3d ZLocalAxis = 
	gikStandingRobot_->halfsittingLocalWaistVertical () ;
      vector3d ZWorldAxis ( 0, 0, 1 ) ;
      
      waistPlaneConstraint_ = 
	new ChppGikPlaneConstraint ( *humanoidRobot_,
				     *(humanoidRobot_->waist()),
				     vector3d ( 0,0,0 ),
				     vector3d ( 0, 0, waistZ_),
				     ZWorldAxis ) ;

      waistParallelConstraint_ =
	new ChppGikParallelConstraint ( *humanoidRobot_,
					*(humanoidRobot_->waist()), 
					ZLocalAxis, 
					ZWorldAxis );
      

      /* Computing the relative positions of the footprints in the waist frame */

      ChppGikSupportPolygon * supportPolygon = 
	gikStandingRobot_->supportPolygon();

      if(!supportPolygon) {
	return KD_ERROR;
      }
      
      if(!supportPolygon->isDoubleSupport()) {
	return KD_ERROR;
      }

      const ChppGikFootprint * leftFootprint = supportPolygon->leftFootprint();
      const ChppGikFootprint * rightFootprint = supportPolygon->rightFootprint();

      matrix4d leftFtT;
      MAL_S4x4_MATRIX_SET_IDENTITY (leftFtT);
      MAL_S4x4_MATRIX_ACCESS_I_J(leftFtT,0,0) = cos (leftFootprint->th());
      MAL_S4x4_MATRIX_ACCESS_I_J(leftFtT,0,1) = - sin (leftFootprint->th());
      MAL_S4x4_MATRIX_ACCESS_I_J(leftFtT,1,1) = cos (leftFootprint->th() );
      MAL_S4x4_MATRIX_ACCESS_I_J(leftFtT,1,0) = sin (leftFootprint->th());
      MAL_S4x4_MATRIX_ACCESS_I_J(leftFtT,0,3) = leftFootprint->x();
      MAL_S4x4_MATRIX_ACCESS_I_J(leftFtT,1,3) = leftFootprint->y();
      MAL_S4x4_MATRIX_ACCESS_I_J(leftFtT,2,3) = 0.;

      matrix4d rightFtT;
      MAL_S4x4_MATRIX_SET_IDENTITY (rightFtT);
      MAL_S4x4_MATRIX_ACCESS_I_J(rightFtT,0,0) = cos (rightFootprint->th());
      MAL_S4x4_MATRIX_ACCESS_I_J(rightFtT,0,1) = - sin (rightFootprint->th());
      MAL_S4x4_MATRIX_ACCESS_I_J(rightFtT,1,1) = cos (rightFootprint->th() );
      MAL_S4x4_MATRIX_ACCESS_I_J(rightFtT,1,0) = sin (rightFootprint->th());
      MAL_S4x4_MATRIX_ACCESS_I_J(rightFtT,0,3) = rightFootprint->x();
      MAL_S4x4_MATRIX_ACCESS_I_J(rightFtT,1,3) = rightFootprint->y();
      MAL_S4x4_MATRIX_ACCESS_I_J(rightFtT,2,3) = 0.;

      matrix4d currentWaistTransformation = 
	humanoidRobot_->waist()->currentTransformation();

      std::cout << "Waist Tranformation: \n" 
		<< currentWaistTransformation
		<< std::endl;


      matrix4d inverseWaistT;
      MAL_S4x4_INVERSE (currentWaistTransformation, inverseWaistT, double);

      MAL_S4x4_C_eq_A_by_B ( relativeLeftFootTransformation_,
			     inverseWaistT,
			     leftFtT);

      MAL_S4x4_C_eq_A_by_B ( relativeRightFootTransformation_,
			     inverseWaistT,
			     rightFtT);

      
      /* Creating kineo constraint */
      
      std::vector<CjrlGikStateConstraint*>  sot;
      sot.push_back(waistPlaneConstraint_);
      sot.push_back(waistParallelConstraint_);
      
      wholeBodyConstraint_ = 
	wholeBodyConstraint::create("WholeBody Constaint",
				    humanoidRobot_,
				    gikManager_);

      wholeBodyConstraint_->setConstraints(sot);

      /* Initializeing goal config generator */
     
      goalConfigGenerator_ =
	CtlcGraspBallGoalGenerator::create(gikManager_, humanoidRobot_);
      if(!goalConfigGenerator_) {
	std::cerr <<
	  (":initializeProblem: Creating the attWholeBodyConfigGenerator failed.") << std::endl;
	return KD_ERROR;
      }
      
      /* initializing the motion planning problem */

      CkwsRoadmapShPtr roadmap = CkwsRoadmap::create(humanoidRobot_);
      CkwsDiffusingRdmBuilderShPtr rdmBuilder =
	//CkwsDiffusingRdmBuilder::create(roadmap);
	CkwsPlusLTRdmBuilder< CkwsDiffusingRdmBuilder >::create (roadmap, hppProblem (0)->penetration ());
      rdmBuilder->diffuseFromProblemGoal (true);

      
      steeringMethodIthProblem(0, CkwsSMLinear::create ());
      roadmapBuilderIthProblem (0,rdmBuilder);
      
      CkwsLoopOptimizerShPtr optimizer = 
	CkwsRandomOptimizer::create();
      optimizer->penetration (hppProblem (0)->penetration ());

      PathOptimizerShPtr postOptimizer = 
	PathOptimizer::create();
      postOptimizer->penetration (hppProblem (0)->penetration () / 100);

      CkwsConfigShPtr halfSittingCfg;
      humanoidRobot_->getCurrentConfig(halfSittingCfg);

      /* Building wholebody mask, without the free flyer dofs */
      std::vector<bool> wbMask(humanoidRobot_->countDofs(),true);
      for(unsigned int i = 0; i<6; i++) wbMask[i] = false;
      
      postOptimizer->targetConfig(halfSittingCfg);
      postOptimizer->setConfigMask(wbMask);

      //optimizer->postOptimizer(postOptimizer);

      pathOptimizerIthProblem(0,optimizer);

      hppProblem (0)->alwaysOptimize (true);

      return KD_OK;
    }

    ktStatus Planner::goalWaistConfig (CkwsPathShPtr inPath)
    {
      assert (inPath->device() == humanoidRobot_);

      CkwsConfigShPtr fCfg = inPath->configAtEnd ();
      goalConfigGenerator_->setGoalBoxPosition (fCfg->dofValue (0),
						fCfg->dofValue (1),
						fCfg->dofValue (5));
      return KD_OK;
    }

    ktStatus
    Planner::initAndGoalConfig (CkwsPathShPtr inPath)
    {
      assert (inPath->device() == humanoidRobot_);
      
      CkwsConfigShPtr iCfg = inPath->configAtStart();
      CkwsConfigShPtr fCfg = inPath->configAtEnd();

      initConfIthProblem(0,iCfg);
      goalConfIthProblem(0,fCfg);
      
      return KD_OK;
    }

    ktStatus Planner::generateGoalConfig ()
    {
      std::vector<CjrlGikStateConstraint*> sot;
      sot.push_back (waistPlaneConstraint_);
      sot.push_back (waistParallelConstraint_);
      gikManager_->setTasks (sot);

      if (!goalConfigGenerator_->compute() == KD_OK)
        {
          std::cerr << (":ERROR generateGoalConfig::Generating goal config.") << std::endl;
          return KD_ERROR;
        }

      std::vector <CkwsConfigShPtr> goalWholeBodyConfigVector = 
	goalConfigGenerator_->getWholeBodyTargetConfig();
      
      if (goalWholeBodyConfigVector.size() == 0)
	{
	  std::cerr << (":generateGoalConfig::No goal config has been generated. You can try another time") << std::endl;
	  return KD_ERROR;
	}

      CtlcGikCfgOptimizerShPtr gikCfgOptimizer
	= CtlcGikCfgOptimizer::create (gikManager_);

      for (unsigned int i = 0; i < goalWholeBodyConfigVector.size (); ++i)
	{
	  /* Attach goal config to device */

	  std::stringstream ss (stringstream::in | stringstream::out);
	  ss << "goal config " << i+1;
	  humanoidRobot_
	    ->addChildComponent (CkppConfigComponent::create (goalWholeBodyConfigVector[i],
	  						      ss.str ()));

	  /* Optimize each configuration towards half-sitting */ 

	  std::cout << "Starting optimizing config " << i << std::endl;
	  gikManager_->setRobotBoxPos (goalWholeBodyConfigVector[i]);
	  CkwsPathShPtr optimizedPath
	    = gikCfgOptimizer->optimizeCfg (goalWholeBodyConfigVector[i]);
	  std::cout << "Finished optimizing config " << i << std::endl;

	  /* Attach goal config to device */

	  std::stringstream ssOpt (stringstream::in | stringstream::out);
	  ssOpt << "otpimized goal config " << i+1;
	  humanoidRobot_
	    ->addChildComponent (CkppConfigComponent::create (optimizedPath->configAtEnd (),
	  						      ssOpt.str ()));
	}

      std::cout << "Goal config found." << std::endl;
      std::vector<CjrlGikStateConstraint*> emptyTask;
      gikManager_->setTasks (emptyTask);
      return KD_OK;
    }

    CkwsPathShPtr 
    Planner::findDynamicPath( CkwsPathShPtr i_path)
    {

      CkwsPathShPtr resultPath = CkwsPath::create(humanoidRobot_);
      CkwsPathShPtr pathToAnimate = i_path;
      CkwsPathShPtr pathAtEnd = CkwsPath::create(humanoidRobot_);
      std::vector<double> configKineo;
      CkwsConfig currentCfg(humanoidRobot_);

      resultFootprints_.clear();

      MAL_VECTOR_DIM(zeros, double, humanoidRobot_->numberDof());
      for(unsigned int i=0; i<zeros.size(); i++)
	zeros[i] =0;
  
      while ( (pathToAnimate->length() > 0.0) 
	      || (pathAtEnd->length() != 0) )
	{

	  std::cout << "-------------------------------" << std::endl
		    << "Animating path of length: " 
		    << pathToAnimate->length() << std::endl;

      
	  CkwsPathShPtr animatedPath;
	  footprintOfParam_t ftprints;
	    
	  if (computeFootPrints(pathToAnimate,ftprints) != KD_OK)
	    {
	      return resultPath;
	    }

	  std::cout << "Computed " << ftprints.size() 
		    << " footprints" << std::endl;

	  /* Initializing the state of the robot before animating */
	  if (!resultPath->isEmpty()) {
	    resultPath->getConfigAtEnd(currentCfg);
	    humanoidRobot_->hppSetCurrentConfig(currentCfg);
	    humanoidRobot_->currentVelocity(zeros);
	    humanoidRobot_->currentAcceleration(zeros);
	    humanoidRobot_->computeForwardKinematics();
	    humanoidRobot_->computeForwardKinematics();
	  }
	  else  {
	    pathToAnimate->getConfigAtStart(currentCfg);
	    humanoidRobot_->hppSetCurrentConfig(currentCfg);
	    humanoidRobot_->currentVelocity(zeros);
	    humanoidRobot_->currentAcceleration(zeros);
	    humanoidRobot_->computeForwardKinematics();
	    humanoidRobot_->computeForwardKinematics();
	  }

	  animatedPath = animatePath(pathToAnimate,ftprints);

	  if (!animatedPath) 
	    {
	      break;
	    }

	  animatedPath->validateWithPenetration(0.2);

	  if ((animatedPath->isValid()) && (animatedPath->length())) 
	    {

	      resultFootprints_.push_back( ftprints );

	      //validGikMotion_.push_back (currentGikMotion_);


	      if (!resultPath->isEmpty()) {
		CkwsConfigShPtr endStartCfg = resultPath->configAtEnd();
		CkwsConfigShPtr startEndCfg = animatedPath->configAtStart();
	    
		if (!endStartCfg->isEquivalent(*startEndCfg))
		  {
		    std::cout << "End Config of start path is not equivalent to start config of end path." 
			      << std::endl;
		      
		    CkwsDirectPathShPtr newDP = 
		      humanoidRobot_->steeringMethod()->makeDirectPath(*endStartCfg,*startEndCfg);
	     
		    resultPath->appendDirectPath ( newDP );
		      
		  }
	      }
	      resultPath->appendPath ( animatedPath );
	  
	      pathToAnimate = pathAtEnd;
	      pathAtEnd =  CkwsPath::create(humanoidRobot_) ;
	    }
	 
	  else
	    {
      
	      std::cout << "Animated path is not valid"
			<< std::endl;
	
	      /* Finding first unvalid directPath */
	      unsigned int i = 0;
	      unsigned int n = animatedPath->countDirectPaths();
		
	      while ( ( i < n ) && 
		      (animatedPath->directPath(i)->isValid()) ) { 
		i++; 
	      }

	      std::cout << "First unvalid path: " 
			<< i << "/" << n 
			<< std::endl;

	      freeFootPrints(ftprints);
	      ftprints.clear();
	      animatedPath->clear();
	
	      double halfLength = 
		n ?  pathToAnimate->length() * i/ (double) n : 0.5 * pathToAnimate->length()  ;

	      /* Attaching the constraint to the device before interpolating, to avoid flying configs */

	      humanoidRobot_->userConstraints()->add(wholeBodyConstraint_);

	      CkwsPathShPtr startPath = CkwsPath::createByExtractingTo(pathToAnimate,halfLength);
	      CkwsPathShPtr endPath = CkwsPath::createByExtractingFrom(pathToAnimate,halfLength);

	      humanoidRobot_->userConstraints()->remove(wholeBodyConstraint_);


	      CkwsPathShPtr newPathAtEnd = CkwsPath::create(humanoidRobot_);
	      newPathAtEnd->appendPath(endPath);
	      if (!pathAtEnd->isEmpty()) {
		if (newPathAtEnd->appendPath (pathAtEnd) != KD_OK ){
		  std::cerr << "FindDynamicPath(): ERROR: failed to add second part of the path to the agregator."
			    << std::endl ;
		  return resultPath;
		}
	      }
	      pathToAnimate = startPath ;
	      pathAtEnd = newPathAtEnd ;
	    }
	}
		

      /* Reinitializing robot state before animating the whole path */
      i_path->getConfigAtStart(currentCfg);
      humanoidRobot_->hppSetCurrentConfig(currentCfg);
      humanoidRobot_->currentVelocity(zeros);
      humanoidRobot_->currentAcceleration(zeros);
      humanoidRobot_->computeForwardKinematics();
      humanoidRobot_->computeForwardKinematics();
      CkwsPathShPtr wholeAnimatedPath = animateWholePath( i_path );

      if (wholeAnimatedPath) hppProblem(0)->addPath ( wholeAnimatedPath) ;
	


      return resultPath;
    }


    ktStatus 
    Planner::computeFootPrints ( CkwsPathShPtr i_path, footprintOfParam_t & o_footPrintOfParam)
    {
      
      std::cout << "Computing footprints..." << std::endl;

      bool isRightFoot = true;
      double currentDist = 0.;
      const ChppGikFootprint * currentFootPrint;
      double length = i_path->length();

      std::vector<double> configKineo;
      MAL_VECTOR_DIM(configJRL,double,humanoidRobot_->numberDof());

      CkwsConfig currentCfg(humanoidRobot_);
      i_path->getConfigAtStart(currentCfg);
      currentCfg.getDofValues(configKineo);
      humanoidRobot_->kwsToJrlDynamicsDofValues(configKineo,configJRL);
      gikStandingRobot_->staticState(configJRL);

      ChppGikSupportPolygon * supportPol = gikStandingRobot_->supportPolygon();

      if(!supportPol)
	{
	  std::cerr << "No support Polygon found when entering Planner::computeFootPrints()"
		    << std::endl;
	  return KD_ERROR;
	}
      
      if (!supportPol->isDoubleSupport())
	{
	  std::cerr << "Robot is not in a double support configuration when entering Planner::computeFootPrints()"
		    << std::endl;
	  return KD_ERROR;
	}

      currentFootPrint = supportPol->leftFootprint();

      while ( currentDist < length )
	{
	  std::cout << "Computing footprint at length: " 
		    << currentDist << std::endl;

	  ChppGikFootprint * newFootPrint = 
	    findNextFootPrint(i_path,currentDist,currentFootPrint,isRightFoot);
	  assert (newFootPrint);

	  std::cout << "Found footprint: " 
		    << newFootPrint->x() << " , "
	    	    << newFootPrint->y() << " , "
		    << newFootPrint->th() << " , "
		    << std::endl;

	  double dist = currentDist;
	  if (currentDist>= length) {
	    dist = length - (paramPrecision_ /100);
	  }

	  o_footPrintOfParam[dist] = newFootPrint;
	  isRightFoot = !isRightFoot;
	  currentFootPrint = newFootPrint;
	}

      ChppGikFootprint * newFootPrint = addLastFootPrint(i_path,currentFootPrint,isRightFoot);

      o_footPrintOfParam[length] = newFootPrint;

      return KD_OK;
    }


    void 
    Planner::freeFootPrints(footprintOfParam_t & footPrintOfParam)
    {
      footprintOfParam_t::iterator it;
      for (it=footPrintOfParam.begin();it!=footPrintOfParam.end();it++)
	delete (*it).second ;
      footPrintOfParam.clear();
    }

    CkwsPathShPtr  
    Planner::animatePath ( CkwsPathShPtr i_path ,  footprintOfParam_t & i_footPrintOfParam )
    {
      CkwsPathShPtr newPath = CkwsPath::create(humanoidRobot_);

      /* Maps the times of steps to the distance along Kineo path */
      std::map<double,double> paramOfTime;
      paramOfTime[0.] = 0.;
      double startTime = 0.;
      double time = 1.6;
      paramOfTime[time] = 0.;
      footprintOfParam_t::iterator it;

      /* Footstep parameters */
      double zmpEndShiftTime = zmpEndShiftTime_;
      double zmpStartShiftTime = zmpStartShiftTime_;
      double footFlightTime = footFlightTime_;
      double stepHeight = stepHeight_;
      double samplingPeriod = samplingPeriod_;
 
      if ( i_footPrintOfParam.size() <= 2) //Smaller steps 
	{
	  ChppGikSupportPolygon * supportPolygon = 
	    gikStandingRobot_->supportPolygon();	  
	  const ChppGikFootprint * leftFootprint = 
	    supportPolygon->leftFootprint();

	  double startX = leftFootprint->x();
	  double startY = leftFootprint->y();
	  double startTh = leftFootprint->th();

	  it =  i_footPrintOfParam.end();
	  it--;
	  double endX = (*it).second->x();
	  double endY = (*it).second->y();
	  double endTh = (*it).second->th();
	  
	  double dTh = abs (endTh - startTh);
	  if (dTh > M_PI) 
	    dTh = abs(atan2( sin (endTh - startTh) , cos (endTh - startTh))) ;

	  double deltaX = abs ( (endX - startX) /  maxX_ );
	  double deltaY = abs ( (endY - startY) /  maxY_ );
	  deltaY = 0;
	  double deltaTh = dTh  /  maxTheta_ ;

	  double stepFrac = std::max ( std::max ( deltaX , deltaY ),
				       deltaTh );

	  std::cout << "  deltaX: " << deltaX
		    << "  deltaY: " << deltaY
		    << "  deltaTh: " << deltaTh
		    << "  stepFrac: " <<  stepFrac
		    << std::endl ;

	  stepFrac = sqrt(stepFrac);

	  zmpEndShiftTime *= stepFrac ;
	  zmpStartShiftTime *= stepFrac ;
	  footFlightTime *= stepFrac ;
	  stepHeight *= stepFrac ;
	}

      if ( footFlightTime < 1 )   samplingPeriod = 0.02;
      if ( footFlightTime < 0.4 ) samplingPeriod = 0.01;
      if ( footFlightTime < 0.2 ) samplingPeriod = 0.05;

      samplingPeriod = 0.005;

      footFlightTime = ((int) (footFlightTime / samplingPeriod) +1) * samplingPeriod ;
      zmpStartShiftTime =  ((int) (zmpStartShiftTime /  samplingPeriod) +1) * samplingPeriod ;
      zmpEndShiftTime =  ((int) (zmpEndShiftTime /  samplingPeriod) +1) * samplingPeriod ;


      std::cout << "Step parameters: " << std::endl
		<< "\t footFlightTime: " << footFlightTime << std::endl
		<< "\t stepHeight: " << stepHeight << std::endl
		<< "\t zmpStartShiftTime: "<< zmpStartShiftTime << std::endl
		<< "\t zmpEndShiftTime: " << zmpEndShiftTime << std::endl
		<< "\t sampling period: " << samplingPeriod << std::endl ;

      if ( footFlightTime < 0.1 ) {
	std::cerr << "ERROR: foot flight time too short." << std::endl;
	newPath.reset();
	return newPath;
      }


      /* Creating generic task */
      ChppGikGenericTask genericTask( gikStandingRobot_ , samplingPeriod );
   
      genericTask.bringBackZMP(false,0,0);

      /* Iterating over footsteps */
      bool isRightFoot = true;
      double lastParam = 0.;
      for(it=i_footPrintOfParam.begin();it!=i_footPrintOfParam.end();it++)
	{
	  ChppGikStepElement * stepElement = new ChppGikStepElement ( gikStandingRobot_,
								      time,
								      (*it).second,
								      isRightFoot,
								      samplingPeriod,
								      zmpEndCoeff_,
								      zmpEndShiftTime,
								      zmpStartShiftTime,
								      footFlightTime,
								      stepHeight);

	  stepFracOfFootprint_[(*it).second] = footFlightTime / footFlightTime_;

	  double stepDuration = stepElement->duration();
	  time+= stepDuration;

	  // When the foot reaches the ground, the configuration to approach 
	  // is the one halfway between the two footsteps
	  double currentParam = (lastParam + (*it).first) /2.;
	  paramOfTime[time] = currentParam;
	  lastParam =  (*it).first ;

	  isRightFoot = !isRightFoot;

	  genericTask.addElement( stepElement );
	}
      
      std::cout << "Total walking time: "  << time << std::endl;

      time += 2;

      paramOfTime[time] = i_path->length();
   
      //Constraint on the waist height
      ChppGikInterpolatedElement heightElem ( gikStandingRobot_->robot(),
					      waistPlaneConstraint_,
					      2,
					      startTime,
					      time,
					      samplingPeriod);
      genericTask.addElement( &heightElem );

      //Constraint on the waist orientation
      // ChppGikInterpolatedElement verticalElem ( gikStandingRobot_->robot(),
      // 						waistParallelConstraint_,
      // 						3,
      // 						startTime,
      // 						time,
      // 						samplingPeriod);
      // genericTask.addElement( &verticalElem );

      // Constraint on the waist orientation
      CkppJointComponentShPtr kppWaist
	= humanoidRobot_->hppWaist ()->kppJoint ();
      if (!kppWaist)
	std::cerr << "ERROR: null pointer to kppWaist" << std::endl;

      ChppGikRotationMotionConstraint waistRotationConstraint(humanoidRobot_,startTime,time,i_path,paramOfTime, kppWaist);
      ChppGikPrioritizedMotion waistRotationElement(&(*humanoidRobot_),3,&waistRotationConstraint,1e-6);
      genericTask.addElement( &waistRotationElement );

      //Config Constraint
      vectorN ubMaskVector = gikStandingRobot_->maskFactory()->upperBodyMask();
      vectorN wbMaskVector = gikStandingRobot_->maskFactory()->wholeBodyMask();

      for(unsigned int i =0;i<6;i++){
	ubMaskVector[i] = 0;
	wbMaskVector[i] = 0;
      }
      
      ChppGikConfigMotionConstraint cfgConstraint(humanoidRobot_,startTime,time,i_path,paramOfTime,ubMaskVector);
      ChppGikPrioritizedMotion cfgElement(&(*humanoidRobot_),4,&cfgConstraint,1e-6);
      cfgElement.workingJoints(ubMaskVector);
      genericTask.addElement( &cfgElement );

      double configTaskDuration = 3;
      CkwsConfigShPtr endCfg = i_path->configAtEnd();
      std::vector<double> kineoTargetCfg; 
      endCfg->getDofValues(kineoTargetCfg);
      MAL_VECTOR_DIM(jrlTargetCfg, double, humanoidRobot_->numberDof());
      humanoidRobot_->kwsToJrlDynamicsDofValues(kineoTargetCfg,jrlTargetCfg);
      ChppGikConfigurationConstraint configTask(*(gikStandingRobot_->robot()), jrlTargetCfg, wbMaskVector);
      ChppGikInterpolatedElement interpolatedCfgElement(gikStandingRobot_->robot(),
      							&configTask,
      							5,
      							time,
      							configTaskDuration,
      							samplingPeriod);

      genericTask.addElement( &interpolatedCfgElement );
      
  
      std::cout << "Solving the task" << std::endl;

      //solving the task
      bool isSolved = genericTask.solve();
      

      if (isSolved)
	{
	  ChppRobotMotion  motion =   genericTask.solutionMotion();
	  if (!motion.empty())
	    {
	      currentGikMotion_ = new ChppRobotMotion (motion);
	      convertGikRobotMotionToKineoPath(&motion,newPath);
	    }
	}
      else 
	{
	  std::cout << "Failed to solve generic task"
		    << std::endl;
	}
      return newPath;
    }


    ChppGikFootprint *
    Planner::footPrintFromConfig(CkwsConfig & cfg, bool isRightFoot)
    {
      double x,y,theta;
      humanoidRobot_->hppSetCurrentConfig(cfg);
      
      matrix4d absoluteFootTransform;

      if (isRightFoot)
	{
	  absoluteFootTransform = 
	    humanoidRobot_->rightAnkle()->currentTransformation();
	}
      else
	{
	  absoluteFootTransform = 
	    humanoidRobot_->leftAnkle()->currentTransformation();
	}

      x = MAL_S4x4_MATRIX_ACCESS_I_J(absoluteFootTransform,0,3);
      y = MAL_S4x4_MATRIX_ACCESS_I_J(absoluteFootTransform,1,3);
      theta = atan2( MAL_S4x4_MATRIX_ACCESS_I_J(absoluteFootTransform,1,0),
		     MAL_S4x4_MATRIX_ACCESS_I_J(absoluteFootTransform,0,0));

      ChppGikFootprint * footPrint = new ChppGikFootprint(x,y,theta);
      return footPrint;
    }

    ktStatus 
    Planner::convertGikRobotMotionToKineoPath(ChppRobotMotion * i_motion, 
					      CkwsPathShPtr o_path)
    {
      o_path->clear();
      const ChppRobotMotionSample * motionSample = i_motion->firstSample();

      if (!motionSample)
	{
	  std::cerr << "ERROR: convertGikRobotMotionToKineoPath: Empty motion sample" 
		    << std::endl;
	  return KD_ERROR;
	}

      std::vector<double> startKineoCfg(humanoidRobot_->countDofs());
      std::vector<double> endKineoCfg(humanoidRobot_->countDofs());
      CkwsConfigShPtr startCfg = CkwsConfig::create(humanoidRobot_);
      CkwsConfigShPtr endCfg = CkwsConfig::create(humanoidRobot_);

      humanoidRobot_->jrlDynamicsToKwsDofValues(motionSample->configuration, startKineoCfg);
      startCfg->setDofValues(startKineoCfg);
      motionSample = i_motion->nextSample() ;

      while ( motionSample )
	{
	  humanoidRobot_->jrlDynamicsToKwsDofValues(motionSample->configuration, endKineoCfg);
	  endCfg->setDofValues(endKineoCfg);

	  if (!startCfg->isEquivalent(*endCfg)) {
	    if ( o_path->appendDirectPath( startCfg , endCfg) == KD_OK)
	      {
		startCfg->setDofValues(endKineoCfg);
	      }
	  }
	  motionSample = i_motion->nextSample() ;
	}

      return KD_OK;
    }

    ktStatus Planner::writeSeqplayFiles ()
    {
      if (validGikMotion_.size () == 0)
	{
	  std::cerr << "ERROR: writeSeqplayFiles: Empty motion vector" 
		    << std::endl;
	  return KD_ERROR;
	}
      else std::cout << "validGikMotion_ size: " << validGikMotion_.size () << std::endl;

      std::vector<double> kineoCfg(humanoidRobot_->countDofs ());
      std::vector<double> openHrpCfg(humanoidRobot_->countDofs ());
      
      for (vector<ChppRobotMotion*>::iterator it = validGikMotion_.begin ();
	   it < validGikMotion_.end (); it++)
	{
	  std::cout << "start x: " << (*it)->firstSample ()->configuration[0] << " "
		    << "end x: " << (*it)->lastSample ()->configuration[0] << " "
		    << std::endl;

	  const ChppRobotMotionSample * motionSample = (*it)->firstSample();
	  
	  if (!motionSample)
	    {
	      std::cerr << "ERROR: writeSeqplayFiles: Empty motion sample" 
			<< std::endl;
	      return KD_ERROR;
	    }
	 
	  // Connect two motion files with a 5ms gap between end and start
	  // since speed and ZMP are (theoretically) null.
	  timestamp_ += 0.005; 
	  
	  while (motionSample)
	    {
	      humanoidRobot_->jrlDynamicsToKwsDofValues(motionSample->configuration, kineoCfg);
	      kwsToOpenHrpDofValues (kineoCfg, openHrpCfg);

	      // Write configuration in Seqplay files.
	      zmpFile_ << timestamp_ << " "
	      	       << motionSample->ZMPwstPla[0] << " "
	      	       << motionSample->ZMPwstPla[1] << " "
		       << motionSample->ZMPwstPla[2] << "\n";

	      zmpMeasuredFile_ << timestamp_ << " "
			       << motionSample->ZMPwstObs[0] << " "
			       << motionSample->ZMPwstObs[1] << " "
			       << motionSample->ZMPwstObs[2] << "\n";

	      rpyFile_ << timestamp_ << " "
		       << motionSample->configuration[3] << " "
		       << motionSample->configuration[4] << " "
		       << motionSample->configuration[5] << "\n";

	      posFile_ << timestamp_ << " ";
	      for (unsigned int dof = 6; dof < humanoidRobot_->countDofs (); dof++)
		posFile_ << openHrpCfg[dof] << " ";
	      posFile_ << "\n";
	      
	      motionSample = (*it)->nextSample() ;
	      timestamp_ += (*it)->samplingPeriod ();
	    }
	}
      
      posFile_.close ();
      zmpFile_.close ();
      zmpMeasuredFile_.close ();
      rpyFile_.close ();

      return KD_OK;
    }
    
    ktStatus
    Planner::kwsToOpenHrpDofValues (const std::vector<double>& inKwsDofVector,
				    std::vector<double>& outOpenHrpDofVector)
    {
      if (outOpenHrpDofVector.size () != inKwsDofVector.size ())
	return KD_ERROR;
      
      for (unsigned int i = 0; i < 29; i++)
	outOpenHrpDofVector[i] = inKwsDofVector[i];

      // Switch vector index for left arm and right hand joints.
      outOpenHrpDofVector[29] = inKwsDofVector[34];
      outOpenHrpDofVector[30] = inKwsDofVector[35];
      outOpenHrpDofVector[31] = inKwsDofVector[36];
      outOpenHrpDofVector[32] = inKwsDofVector[37];
      outOpenHrpDofVector[33] = inKwsDofVector[38];
      outOpenHrpDofVector[34] = inKwsDofVector[39];
      outOpenHrpDofVector[35] = inKwsDofVector[40];

      outOpenHrpDofVector[36] = inKwsDofVector[29];
      outOpenHrpDofVector[37] = inKwsDofVector[30];
      outOpenHrpDofVector[38] = inKwsDofVector[31];
      outOpenHrpDofVector[39] = inKwsDofVector[32];
      outOpenHrpDofVector[40] = inKwsDofVector[33];

      for (unsigned int i = 41 ; i < 46; i++) 
	outOpenHrpDofVector[i] = inKwsDofVector[i];

      return KD_OK;
    }

    ChppGikFootprint * 
    Planner::findNextFootPrint(CkwsPathShPtr i_path, 
			       double & param, 
			       const ChppGikFootprint * currentFootPrint,
			       bool isRightFoot)
    {
  

      CkwsConfig cfg(humanoidRobot_);
      double deltaParam = 1.;
      double precision = 1.;
      double newParam ;
      ChppGikFootprint * newFootPrint = NULL;
      double dmax = i_path->length();

      while ( (precision > paramPrecision_)
	      && (param + deltaParam - precision < dmax) ) 
	// Invariant: the footstep at param: (param + deltaParam - precision) is valid
	{
	  if (newFootPrint) { 
	    delete newFootPrint;
	    newFootPrint = NULL;
	  }
	  newParam = param + deltaParam;
	  i_path->getConfigAtDistance(newParam, cfg);
	  newFootPrint = footPrintFromConfig(cfg,isRightFoot);
     
	  if ( successiveFootPrints(currentFootPrint, newFootPrint,isRightFoot) )
	    {
	      deltaParam += precision;
	    }
	  else 
	    {
	      precision = precision /2. ;
	      deltaParam -= precision ;
	    }
	}
      assert ( deltaParam - precision > 0);
      if (newFootPrint) { 
	delete newFootPrint;
	newFootPrint = NULL;
      }
      deltaParam -= precision;
      param += deltaParam;
      i_path->getConfigAtDistance(param, cfg);
      newFootPrint = footPrintFromConfig(cfg,isRightFoot);
      return newFootPrint;
    }

    ChppGikFootprint * 
    Planner::addLastFootPrint(CkwsPathShPtr i_path, 
			      const ChppGikFootprint * currentFootPrint,
			      bool isRightFoot)
    {
      CkwsConfig cfg(humanoidRobot_);
      i_path->getConfigAtEnd(cfg);
      ChppGikFootprint * newFootPrint =  footPrintFromConfig(cfg,isRightFoot);
      return newFootPrint;
    }


    bool 
    Planner::successiveFootPrints(const ChppGikFootprint * ft1,
				  const ChppGikFootprint * ft2,
				  bool isRightFoot)
    {
      double minY = isRightFoot ? minY_ : -maxY_;
      double maxY = isRightFoot ? maxY_ : -minY_;
      double minTheta = isRightFoot ? minTheta_ : -maxTheta_;
      double maxTheta = isRightFoot ? maxTheta_ : -minTheta_;

      ChppGikFootprint * ft2Local = 
	new ChppGikFootprint(ft2->x(),ft2->y(),ft2->th());
      ChppGikFootprint::makeRelative(ft1,ft2Local);

      // Compute min foot rotation as a function of y to avoid big
      // rotations toward the oustide when feet are very
      // close. Minimum allowed rotation is then set to a small
      // negative value.
      double minAllowedTheta = 0.;
      double minThetaSlope = (minTheta - minAllowedTheta)
	/ (minY - maxY);
      double minThetaLocal = minThetaSlope * (ft2Local->y () - maxY)
	+ minAllowedTheta;

      bool res =     ( (ft2Local->x()  >= minX_)
		       && (ft2Local->x() <= maxX_)
		       && (ft2Local->y() >= minY)
		       && (ft2Local->y() <= maxY)
		       && (ft2Local->th() >= minThetaLocal)
		       && (ft2Local->th() <= maxTheta) );


      delete ft2Local;
      return res;
    }

    CkwsPathShPtr Planner::animateWholePath(CkwsPathShPtr i_path) {
      
      footprintOfParam_t allFootprints;
      footprintOfParam_t::iterator it;
      double length = 0;
      double extraLength = 0; //Used when we concatenate two footprints


      /* Building footprintofparam for the total path */
      for ( unsigned int i = 0; i < resultFootprints_.size() ; i++)
	{
	  bool isRightFoot = true;

	  footprintOfParam_t footPrintOfParam = resultFootprints_[i] ;

	  for (it=footPrintOfParam.begin();it!=footPrintOfParam.end();it++){
	    allFootprints[(*it).first + length] = (*it).second ;

	    stepFracOfFootprint_ [ (*it).second ] += extraLength;
	    extraLength = 0;

	    isRightFoot = !isRightFoot;
	  }
	  if ( (!isRightFoot)   //The last footprint is right foot, we erase it.
	       && (i < resultFootprints_.size() -1) )
	    {
	      it = allFootprints.end(); it --;
	      extraLength = stepFracOfFootprint_[ (*it).second ];
	      allFootprints.erase(it);
	    }
	  it = footPrintOfParam.end(); it --;
	  length += (*it).first;
	}

      // Load new footprints in result vector.
      resultFootprints_.clear ();
      resultFootprints_.push_back (allFootprints);

      std::cout << "---------------------" << std::endl
		<< " Animating Whole Path" << std::endl
		<< "Length: " << i_path->length() << std::endl;

      std::cout << "Footprints: "  << std::endl;

      for ( it = allFootprints.begin() ; it != allFootprints.end() ; it++) 
	{
	  std::cout << "\tat length " << (*it).first 
		    << " : "
		    << (*it).second->x() << ","
		    << (*it).second->y() << ","
		    << (*it).second->th() << ","
		    << std::endl ;
	}


      CkwsPathShPtr newPath = CkwsPath::create(humanoidRobot_);
      std::map<double,double> paramOfTime;
      paramOfTime[0.] = 0.;
      double startTime = 0.;
      //double time = 1.6;
      //double time = 3.2;
      double time = 4.8;
      paramOfTime[time] = 0.;
  
      /* Footstep parameters */
      double samplingPeriod = 5e-3;

      /* Creating generic task */
      ChppGikGenericTask genericTask( gikStandingRobot_ , samplingPeriod );
   
      /* Iterating over footsteps */
      bool isRightFoot = true;
      double lastParam = 0.;

      for(it=allFootprints.begin();it!=allFootprints.end();it++)
	{
	  double stepFrac = stepFracOfFootprint_[ (*it).second ];
	  double zmpEndShiftTime = stepFrac * zmpEndShiftTime_;
	  double zmpStartShiftTime = stepFrac * zmpStartShiftTime_;
	  double footFlightTime = stepFrac * footFlightTime_;
	  double stepHeight = stepFrac * stepHeight_;
	  footFlightTime = ((int) (footFlightTime / samplingPeriod) +1) * samplingPeriod ;
	  zmpStartShiftTime =  ((int) (zmpStartShiftTime /  samplingPeriod) +1) * samplingPeriod ;
	  zmpEndShiftTime =  ((int) (zmpEndShiftTime /  samplingPeriod) +1) * samplingPeriod ;

	  ChppGikStepElement * stepElement = new ChppGikStepElement ( gikStandingRobot_,
								      time,
								      (*it).second,
								      isRightFoot,
								      samplingPeriod,
								      zmpEndCoeff_,
								      zmpEndShiftTime,
								      zmpStartShiftTime,
								      footFlightTime,
								      stepHeight);

	  double stepDuration = stepElement->duration();
	  time+= stepDuration;

	  // When the foot reaches the ground, the configuration to approach 
	  // is the one halfway between the two footsteps
	  double currentParam = (lastParam + (*it).first) /2.;
	  paramOfTime[time] = currentParam;
	  lastParam =  (*it).first ;
	  isRightFoot = !isRightFoot;

	  genericTask.addElement( stepElement );
	}

      std::cout << "Total walking time: "  << time << std::endl;

      time += 2;
      paramOfTime[time] = i_path->length();
      paramOfTime_ = paramOfTime;
   
      //Constraint on the waist height
      ChppGikInterpolatedElement heightElem ( gikStandingRobot_->robot(),
					      waistPlaneConstraint_,
					      2,
					      startTime,
					      time,
					      samplingPeriod);
      genericTask.addElement( &heightElem );

      //Constraint on the waist orientation
      // ChppGikInterpolatedElement verticalElem ( gikStandingRobot_->robot(),
      // 						waistParallelConstraint_,
      // 						3,
      // 						startTime,
      // 						time,
      // 						samplingPeriod);
      // genericTask.addElement( &verticalElem );

      // Constraint on the waist orientation
      CkppJointComponentShPtr kppWaist
	= humanoidRobot_->hppWaist ()->kppJoint ();
      if (!kppWaist)
	std::cerr << "ERROR: null pointer to kppWaist" << std::endl;

      ChppGikRotationMotionConstraint waistRotationConstraint(humanoidRobot_,startTime,time,i_path,paramOfTime, kppWaist);
      ChppGikPrioritizedMotion waistRotationElement(&(*humanoidRobot_),3,&waistRotationConstraint,1e-6);
      genericTask.addElement( &waistRotationElement );

      //Config Constraint
      vectorN ubMaskVector = gikStandingRobot_->maskFactory()->upperBodyMask();
      vectorN wbMaskVector = gikStandingRobot_->maskFactory()->wholeBodyMask();

      for(unsigned int i =0;i<6;i++){
	ubMaskVector[i] = 0;       
	wbMaskVector[i] = 0;
      }

      ChppGikConfigMotionConstraint cfgConstraint(humanoidRobot_,startTime,time,i_path,paramOfTime,ubMaskVector);
      ChppGikPrioritizedMotion cfgElement(&(*humanoidRobot_),4,&cfgConstraint,1e-6);
      cfgElement.workingJoints(ubMaskVector);
      genericTask.addElement( &cfgElement );
      
      std::cout << "Solving the task" << std::endl;

      // Interpolated config task
      double configTaskDuration = 3;
      CkwsConfigShPtr endCfg = i_path->configAtEnd();
      std::vector<double> kineoTargetCfg; 
      endCfg->getDofValues(kineoTargetCfg);
      MAL_VECTOR_DIM(jrlTargetCfg, double, humanoidRobot_->numberDof());
      humanoidRobot_->kwsToJrlDynamicsDofValues(kineoTargetCfg,jrlTargetCfg);
      ChppGikConfigurationConstraint configTask(*(gikStandingRobot_->robot()), jrlTargetCfg, wbMaskVector);
      ChppGikInterpolatedElement interpolatedCfgElement(gikStandingRobot_->robot(), 
							&configTask,
							5,
							time,
							configTaskDuration,
							samplingPeriod);

      genericTask.addElement( &interpolatedCfgElement );
      


      //solving the task
      bool isSolved = genericTask.solve();
      

      if (isSolved)
	{
	  ChppRobotMotion  motion =   genericTask.solutionMotion();
	  if (!motion.empty())
	    {
	      // currentGikMotion_ = new ChppRobotMotion (motion);
	      // validGikMotion_.push_back (currentGikMotion_);
	      // if (writeSeqplayFiles() != KD_OK) {
	      // 	std::cerr << "ERROR in writing seqplay files." << std::endl;
	      // }
	      // convertGikRobotMotionToKineoPath(&motion,newPath);
	    }

	  bool filterWorked = genericTask.filterZmpError(1.0);

	  if (filterWorked) {
	    isSolved = genericTask.solve();

	    if (!isSolved) {
	      std::cerr << "ERROR: AnimateWholePath - Second pass zmp filtering failed."
	  		<< std::endl;
	      newPath.reset();
	      return newPath;
	    }
	  }

	  motion =   genericTask.solutionMotion();
	  if (!motion.empty())
	    {
	      // currentGikMotion_ = new ChppRobotMotion (motion);
	      // validGikMotion_.push_back (currentGikMotion_);
	      // if (writeSeqplayFiles() != KD_OK) {
	      // 	std::cerr << "ERROR in writing seqplay files." << std::endl;
	      // }
	      // convertGikRobotMotionToKineoPath(&motion,newPath);
	    }

	  filterWorked = false;
	  filterWorked = genericTask.filterZmpError(1.0);

	  if (filterWorked) {
	    isSolved = genericTask.solve();

	    if (!isSolved) {
	      std::cerr << "ERROR: AnimateWholePath - Second pass zmp filtering failed."
	  		<< std::endl;
	      newPath.reset();
	      return newPath;
	    }
	  }

	  motion =   genericTask.solutionMotion();
	  if (!motion.empty())
	    {
	      currentGikMotion_ = new ChppRobotMotion (motion);
	      validGikMotion_.push_back (currentGikMotion_);
	      if (writeSeqplayFiles() != KD_OK) {
		std::cerr << "ERROR in writing seqplay files." << std::endl;
	      }
	      convertGikRobotMotionToKineoPath(&motion,newPath);
	    }

	  genericTask.filterZmpError(1.0);

	}
      else 
	{
	  std::cout << "Failed to solve generic task"
		    << std::endl;

	  ChppRobotMotion  motion =   genericTask.solutionMotion();
	  if (!motion.empty())
	    {
	      convertGikRobotMotionToKineoPath(&motion,newPath);
	    }
	  hppProblem(0)->addPath (newPath);

	}
      return newPath;
	  
    }

    void Planner::setFootPrintLimits(double minX,
				     double maxX,
				     double minY,
				     double maxY,
				     double minTheta,
				     double maxTheta)
    {
      minX_ = minX;
      maxX_ = maxX;
      minY_ = minY;
      maxY_ = maxY;
      minTheta_ = minTheta;
      maxTheta_ = maxTheta;
    }


    ktStatus Planner::solve()
    {

      humanoidRobot_->userConstraints()->add(wholeBodyConstraint_);
      ktStatus res =  solveOneProblem(0);
      //humanoidRobot_->userConstraints()->remove(wholeBodyConstraint_);

      if ( res != KD_OK )
	{
	  return res;
	}
      

      /*
	CkwsPathShPtr resultPath = 
	getPath (0,getNbPaths (0) - 1);

	if (!resultPath)
	{
	return KD_ERROR;
	}
       
      
	CkwsPathShPtr animatedPath = 
	findDynamicPath ( resultPath );

	if (! animatedPath )
	return KD_ERROR;
      
	hppProblem(0)->addPath ( animatedPath);
      */
      return KD_OK;
    }

  }
}
