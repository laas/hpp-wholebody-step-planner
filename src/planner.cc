
# include <KineoWorks2/kwsRoadmap.h>
# include <KineoWorks2/kwsDiffusingRdmBuilder.h>
# include <KineoWorks2/kwsRandomOptimizer.h>

# include <hpp/gik/task/generic-task.hh>


# include <hpp/wholebody-step-planner/planner.hh>
# include <hpp/wholebody-step-planner/config-motion-constraint.hh>
# include <hpp/wholebody-step-planner/path-optimizer.hh>


# define PARAM_PRECISION 0.01

/* Pattern Generator default parameters */
# define END_COEFF 0.5
# define START_SHIFT_TIME 0.1
# define END_SHIFT_TIME 0.1
# define FOOT_FILGHT_TIME 1.0
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
       paramPrecision_(PARAM_PRECISION)
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

      std::string property,value;
      property="ComputeZMP"; value="false";humanoidRobot_->setProperty ( property,value );
      property="TimeStep"; value="0.005";humanoidRobot_->setProperty ( property,value );
      property="ComputeAccelerationCoM"; value="false";humanoidRobot_->setProperty ( property,value );
      property="ComputeBackwardDynamics"; value="false";humanoidRobot_->setProperty ( property,value );
      property="ComputeMomentum"; value="false";humanoidRobot_->setProperty ( property,value );
      property="ComputeAcceleration"; value="false";humanoidRobot_->setProperty ( property,value );
      property="ComputeVelocity"; value="false";humanoidRobot_->setProperty ( property,value );
      property="ComputeSkewCom"; value="false";humanoidRobot_->setProperty ( property,value );
      property="ComputeCoM"; value="true";humanoidRobot_->setProperty ( property,value );

  
      gikStandingRobot_ = new ChppGikStandingRobot(*humanoidRobot_);

      gikManager_ =  CtlcGikManager::create(humanoidRobot_);

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


      /* initializing the motion planning problem */

      CkwsRoadmapShPtr roadmap = CkwsRoadmap::create(humanoidRobot_);
      CkwsDiffusingRdmBuilderShPtr rdmBuilder = 
	CkwsDiffusingRdmBuilder::create ( roadmap );
      
      steeringMethodIthProblem(0, CkwsSMLinear::create ());
      roadmapBuilderIthProblem (0,rdmBuilder);
      
      CkwsLoopOptimizerShPtr optimizer = 
	CkwsRandomOptimizer::create();
      
      PathOptimizerShPtr postOptimizer = 
	PathOptimizer::create();

      CkwsConfigShPtr halfSittingCfg;
      humanoidRobot_->getCurrentConfig(halfSittingCfg);

      /* Building wholebody mask, without the free flyer dofs */
      std::vector<bool> wbMask(humanoidRobot_->countDofs(),true);
      for(unsigned int i = 0; i<6; i++) wbMask[i] = false;
      
      postOptimizer->targetConfig(halfSittingCfg);
      postOptimizer->setConfigMask(wbMask);

      optimizer->postOptimizer(postOptimizer);

      pathOptimizerIthProblem(0,optimizer);

      hppProblem (0)->alwaysOptimize (true);

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

    CkwsPathShPtr 
    Planner::findDynamicPath( CkwsPathShPtr i_path)
    {
      std::cout << "-------------------------------" << std::endl
		<< "Animating path of length: " 
		<< i_path->length() << std::endl;

      /* Initializing the state of the robot */
      std::vector<double> configKineo;
      CkwsConfigShPtr currentCfg = i_path->configAtStart();
      humanoidRobot_->hppSetCurrentConfig(*currentCfg);

      MAL_VECTOR_DIM(zeros, double, humanoidRobot_->numberDof());
      for(unsigned int i=0; i<zeros.size(); i++)
	zeros[i] =0;
      humanoidRobot_->currentVelocity(zeros);
      humanoidRobot_->currentAcceleration(zeros);
      humanoidRobot_->computeForwardKinematics();
      

      CkwsPathShPtr animatedPath;
      footprintOfParam_t ftprints;

      if (i_path->length() < 0.001 )
	{
	  animatedPath.reset();
	  return animatedPath;
	}

      if (computeFootPrints(i_path,ftprints) != KD_OK)
	{
	  animatedPath.reset();
	  return animatedPath;
	}

      std::cout << "Computed " << ftprints.size() 
		<< " footprints" << std::endl;

      animatedPath = animatePath(i_path,ftprints);


      animatedPath->validateWithPenetration(0.2);

      if ((animatedPath->length()==0) || (!animatedPath->isValid())) 
	{
	  
	  std::cout << "Animated path is not valid"
		    << std::endl;
	  
	  freeFootPrints(ftprints);
	  ftprints.clear();
	  animatedPath->clear();

	  double halfLength = 0.5 * i_path->length() ;

	  /* Attaching the constraint to the device before interpolating, to avoid flying configs */

	  humanoidRobot_->userConstraints()->add(wholeBodyConstraint_);

	  CkwsPathShPtr startPath = CkwsPath::createByExtractingTo(i_path,halfLength);
	  CkwsPathShPtr endPath  =  CkwsPath::createByExtractingFrom(i_path,halfLength);

	  humanoidRobot_->userConstraints()->remove(wholeBodyConstraint_);


   
	  CkwsPathShPtr startAnimatedPath = findDynamicPath ( startPath );

	  if (!startAnimatedPath)
	    {
	      animatedPath.reset();
	      return animatedPath;
	    }
	  CkwsPathShPtr endAnimatedPath = findDynamicPath ( endPath );

	  if (!endAnimatedPath)
	    {
	      animatedPath.reset();
	      return animatedPath;
	    }

	  CkwsConfigShPtr endStartCfg = startAnimatedPath->configAtEnd();
	  CkwsConfigShPtr startEndCfg = endAnimatedPath->configAtStart();

	  animatedPath->appendPath(startAnimatedPath);
	  
	  if (!endStartCfg->isEquivalent(*startEndCfg))
	    {
	      std::cout << "End Config of start path is not equivalent to start config of end path." 
			<< std::endl;

	      CkwsDirectPathShPtr newDP = 
		humanoidRobot_->steeringMethod()->makeDirectPath(*endStartCfg,*startEndCfg);
	     
	      animatedPath->appendDirectPath ( newDP );
	    }
	  animatedPath->appendPath(endAnimatedPath);
	}

      if (animatedPath)
	hppProblem(0)->addPath(animatedPath);

      return animatedPath;
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

	  o_footPrintOfParam[currentDist] = newFootPrint;
	  isRightFoot = !isRightFoot;
	  currentFootPrint = newFootPrint;
	}

      ChppGikFootprint * newFootPrint = addLastFootPrint(i_path,currentFootPrint,isRightFoot);

      o_footPrintOfParam[currentDist+1] = newFootPrint;

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

      /* Initializing the state of the robot */
      std::vector<double> configKineo;
      CkwsConfigShPtr currentCfg = i_path->configAtStart();

      humanoidRobot_->hppSetCurrentConfig(*currentCfg);

      MAL_VECTOR_DIM(zeros, double, humanoidRobot_->numberDof());
      for(unsigned int i=0; i<zeros.size(); i++)
	zeros[i] =0;
      humanoidRobot_->currentVelocity(zeros);
      humanoidRobot_->currentAcceleration(zeros);
      humanoidRobot_->computeForwardKinematics();

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

	  zmpEndShiftTime *= stepFrac ;
	  zmpStartShiftTime *= stepFrac ;
	  footFlightTime *= stepFrac ;
	  stepHeight *= stepFrac ;
	}

      if (( footFlightTime < 0.5 ) && ( samplingPeriod > 5e-3)) samplingPeriod = 5e-3;

      std::cout << "Step parameters: " << std::endl
		<< "\t footFlightTime: " << footFlightTime << std::endl
		<< "\t stepHeight: " << stepHeight << std::endl
		<< "\t zmpStartShiftTime: "<< zmpStartShiftTime << std::endl
		<< "\t zmpEndShiftTime: " << zmpEndShiftTime << std::endl
		<< "\t sampling period: " << samplingPeriod << std::endl ;

      /* Creating generic task */
      ChppGikGenericTask genericTask( gikStandingRobot_ , samplingPeriod );
   
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
      time += 3.;

      //Constraint on the waist height
      ChppGikInterpolatedElement heightElem ( gikStandingRobot_->robot(),
					      waistPlaneConstraint_,
					      1,
					      startTime,
					      time,
					      samplingPeriod);
      genericTask.addElement( &heightElem );

      //Constraint on the waist orientation
      ChppGikInterpolatedElement verticalElem ( gikStandingRobot_->robot(),
						waistParallelConstraint_,
						2,
						startTime,
						time,
						samplingPeriod);
      genericTask.addElement( &verticalElem );

      //Config Constraint
      vectorN ubMaskVector = gikStandingRobot_->maskFactory()->upperBodyMask();
      vectorN wbMaskVector = gikStandingRobot_->maskFactory()->wholeBodyMask();
      
      ChppGikConfigMotionConstraint cfgConstraint(humanoidRobot_,startTime,time,i_path,paramOfTime,ubMaskVector);
      ChppGikPrioritizedMotion cfgElement(&(*humanoidRobot_),3,&cfgConstraint,0.1);
      cfgElement.workingJoints(wbMaskVector);
      genericTask.addElement( &cfgElement );

      //Config interpolated element at end of locomotion
      CkwsConfigShPtr endCfg = i_path->configAtEnd();
      std::vector<double> kineoTargetCfg;
      endCfg->getDofValues(kineoTargetCfg);
      MAL_VECTOR_DIM(jrlTargetCfg, double, humanoidRobot_->numberDof());
      humanoidRobot_->kwsToJrlDynamicsDofValues(kineoTargetCfg,jrlTargetCfg);

      ChppGikConfigurationConstraint configTask(*humanoidRobot_,
						jrlTargetCfg,
						wbMaskVector);
      double cfgElementDuration = 2.;
      ChppGikInterpolatedElement interCfgElement(&(*humanoidRobot_),
						 &configTask,
						 0,
						 time,
						 cfgElementDuration,
						 samplingPeriod);
      
      genericTask.addElement( &interCfgElement );





      //solving the task
      bool isSolved = genericTask.solve();
      

      if (isSolved)
	{
	  ChppRobotMotion  motion =   genericTask.solutionMotion();
	  if (!motion.empty())
	    {
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

      bool res =     ( (ft2Local->x()  >= minX_)
		       && (ft2Local->x() <= maxX_)
		       && (ft2Local->y() >= minY)
		       && (ft2Local->y() <= maxY)
		       && (ft2Local->th() >= minTheta)
		       && (ft2Local->th() <= maxTheta) );


      delete ft2Local;
      return res;
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
      humanoidRobot_->userConstraints()->remove(wholeBodyConstraint_);

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
