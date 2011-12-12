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

#include <sstream>

#include <KineoWorks2/kwsRoadmap.h>
#include <KineoWorks2/kwsDiffusingRdmBuilder.h>
#include <KineoWorks2/kwsRandomOptimizer.h>
#include <KineoWorks2/kwsMultiplePlanner.h>

#include <KineoModel/kppConfigComponent.h>
#include <KineoModel/kppSMLinearComponent.h>

#ifdef HPP_DEBUG
# include <KineoWorks2/kwsShooterConfigList.h>
# include <KineoWorks2/kwsShooterConfigSpace.h>
# include <KineoWorks2/kwsShooterAdaptiveMulti.h>
# include <KineoWorks2/kwsShooterPath.h>
# include <KineoWorks2/kwsShooterRoadmapBox.h>
# include <KineoWorks2/kwsShooterRoadmapNodes.h>
#endif
#include <jrl/mal/matrixabstractlayer.hh>
#include <kwsPlus/roadmap/kwsPlusLTRdmBuilder.h>
#include <hpp/gik/task/generic-task.hh>

#include <hpp/util/debug.hh>
#include <hpp/model/joint.hh>

#include <hpp/gik/robot/foot-print-related.hh>
#include <hpp/gik/robot/robot-motion.hh>
#include <hpp/gik/robot/standing-robot.hh>
#include <hpp/gik/constraint/parallel-constraint.hh>
#include <hpp/gik/constraint/plane-constraint.hh>
#include <hpp/gik/constraint/position-constraint.hh>
#include <hpp/gik/motionplanner/element/step-element.hh>
#include <hpp/gik/motionplanner/element/interpolated-element.hh>

#include <hpp/constrained/config-extendor.hh>
#include <hpp/constrained/roadmap-builder.hh>
#include <hpp/constrained/config-optimizer.hh>
#include <hpp/constrained/goal-config-generator.hh>
#include <hpp/constrained/planner/planner.hh>

#include "hpp/wholebody-step-planner/planner.hh"
#include "hpp/wholebody-step-planner/config-motion-constraint.hh"
#include "hpp/wholebody-step-planner/path-optimizer.hh"
#include "hpp/wholebody-step-planner/shooter-humanoid.hh"
#include "hpp/wholebody-step-planner/config-shooter-reaching.hh"

#include "../src/roboptim/path-optimizer.hh"

#define PARAM_PRECISION 0.01
#define TASK_PRECISION 1e-4

/* Pattern Generator default parameters */
#define END_COEFF 0.5
#define START_SHIFT_TIME 0.1
#define END_SHIFT_TIME 0.1
#define FOOT_FILGHT_TIME 0.8
#define STEP_HEIGHT 0.05

namespace hpp
{
  namespace wholeBodyStepPlanner
  {
#ifdef HPP_DEBUG
    static void logDiffusionShooter (CkwsDiffusionShooterShPtr shooter);
#endif
    KIT_PREDEF_CLASS (ConfigShooterReaching)
    Planner::size_type Planner::robotId=0;

    Planner::Planner(double samplingPeriod )
      :humanoidRobot_(),
       gikStandingRobot_(NULL),
       samplingPeriod_(samplingPeriod),
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
      hppDout (info, "ParamPrecision: " <<  paramPrecision_);
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

    hpp::model::HumanoidRobotShPtr Planner::humanoidRobot ()
    {
      return humanoidRobot_;
    }

    ChppGikStandingRobot* Planner::robot ()
    {
      return gikStandingRobot_;
    }

    std::vector<ChppRobotMotion*> Planner::robotMotions ()
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

    hpp::constrained::KwsConstraintShPtr Planner::wholeBodyConstraint ()
    {
      return wholeBodyConstraint_;
    }

    // Perform several initializations
    //
    // - Initialize dynamic part of robot and set gikStandingRobot_.
    // - Get current height of waist and store in waistZ_.
    // - Return current configuration of robot.
    //    robot is supposed to be in half-sitting configuration. This
    //    configuration is then used as an "optimal" configuration for latter
    //    optimizations.
    // - get robot local z axis.

    static CkwsConfigShPtr
    initializeRobot (hpp::model::HumanoidRobotShPtr humanoidRobot,
		     ChppGikStandingRobot*& gikStandingRobot,
		     double& waistZ, vector3d& ZLocalAxis)
    {
      std::string property,value;
      property="ComputeZMP"; value="true";
      humanoidRobot->setProperty ( property,value );
      property="TimeStep"; value="0.005";
      humanoidRobot->setProperty ( property,value );
      property="ComputeAccelerationCoM";
      value="true";humanoidRobot->setProperty ( property,value );
      property="ComputeBackwardDynamics"; value="false";
      humanoidRobot->setProperty ( property,value );
      property="ComputeMomentum"; value="true";
      humanoidRobot->setProperty ( property,value );
      property="ComputeAcceleration"; value="true";
      humanoidRobot->setProperty ( property,value );
      property="ComputeVelocity"; value="true";
      humanoidRobot->setProperty ( property,value );
      property="ComputeSkewCom"; value="true";
      humanoidRobot->setProperty ( property,value );
      property="ComputeCoM"; value="true";
      humanoidRobot->setProperty ( property,value );

      gikStandingRobot = new ChppGikStandingRobot(*humanoidRobot);

      waistZ = MAL_S4x4_MATRIX_ACCESS_I_J
	(humanoidRobot->waist()->currentTransformation(),2,3);
      ZLocalAxis =
	gikStandingRobot->halfsittingLocalWaistVertical ();

      CkwsConfigShPtr halfSittingCfg;
      humanoidRobot->getCurrentConfig(halfSittingCfg);
      return halfSittingCfg;
    }

    // Define sliding quasi static stability constraints:
    //   - Right foot on the ground
    //   - left foot rigidly fixed to right foot,
    //   - projection the center of mass on the ground,
    //   - waist height, roll and pitch constant
    static void initializeStabilityTasks
    (ChppGikPlaneConstraint*& waistPlaneConstraint,
     ChppGikParallelConstraint*& waistParallelConstraint,
     model::HumanoidRobotShPtr& humanoidRobot,
     double waistZ, vector3d& ZLocalAxis, CkwsConfigShPtr& halfSittingCfg,
     std::vector<CjrlGikStateConstraint*>& slidingStabilityConstraints)
    {
      vector3d ZWorldAxis ( 0, 0, 1 );
      waistPlaneConstraint =
	new ChppGikPlaneConstraint ( *humanoidRobot,
				     *(humanoidRobot->waist()),
				     vector3d ( 0,0,0 ),
				     vector3d ( 0, 0, waistZ),
				     ZWorldAxis ) ;

      waistParallelConstraint =
	new ChppGikParallelConstraint ( *humanoidRobot,
					*(humanoidRobot->waist()),
					ZLocalAxis,
					ZWorldAxis );

      /* Creating kineo constraint */
      hpp::constrained::Planner::
	buildDoubleSupportSlidingStaticStabilityConstraints
	(halfSittingCfg, slidingStabilityConstraints);
      slidingStabilityConstraints.push_back (waistPlaneConstraint);
      slidingStabilityConstraints.push_back (waistParallelConstraint);

    }

    ktStatus Planner::initializeProblem ()
    {

      hppDout(info, "Initialize problem(): paramPrecision: "
	      << paramPrecision_);

      assert (getNbHppProblems () >= 1);
      assert (humanoidRobot_ = KIT_DYNAMIC_PTR_CAST
	      (hpp::model::HumanoidRobot, robotIthProblem (0)));

      // Initialize dynamic part of robot and get half-sitting configuration.
      vector3d ZLocalAxis;
      halfSittingCfg_ = initializeRobot (humanoidRobot_, gikStandingRobot_,
					 waistZ_, ZLocalAxis);

      // Define sliding quasi static stability constraints:
      //   - Right foot on the ground
      //   - left foot rigidly fixed to right foot,
      //   - projection the center of mass on the ground,
      //   - waist height, roll and pitch constant
      initializeStabilityTasks (waistPlaneConstraint_, waistParallelConstraint_,
				humanoidRobot_, waistZ_, ZLocalAxis,
				halfSittingCfg_, slidingStabilityConstraints_);

      hpp::constrained::ConfigExtendor* extendor =
	new hpp::constrained::ConfigExtendor (humanoidRobot_);
      extendor->setConstraints (slidingStabilityConstraints_);

      // Set weights to solver
      ChppGikMaskFactory maskFactory(&(*humanoidRobot_));
      vectorN weightVector = maskFactory.weightsDoubleSupport ();
      extendor->getGikSolver()->weights(weightVector);

      wholeBodyConstraint_ =
	hpp::constrained::KwsConstraint::create("Whole-Body Constraint",
						extendor);

      // Create roadmap builder
      CkwsRoadmapShPtr roadmap = CkwsRoadmap::create(humanoidRobot_);
      CkwsDiffusingRdmBuilderShPtr rdmBuilder =
	constrained::DiffusingRoadmapBuilder::create(roadmap,extendor);
      CkwsDiffusionShooterShPtr diffusionShooter =
	ShooterHumanoid::create (humanoidRobot_, 5.);
      rdmBuilder->diffusionShooter (diffusionShooter);
#ifdef HPP_DEBUG
      logDiffusionShooter (rdmBuilder->diffusionShooter ());
#endif
      rdmBuilder->diffuseFromProblemStart (true);
      rdmBuilder->diffuseFromProblemGoal (true);

      // Set steering method to linear
      steeringMethodIthProblem(0, CkppSMLinearComponent::create ());
      assert (roadmapBuilderIthProblem (0, rdmBuilder, true) == KD_OK);
      hppDout (info, "Set roadmap builder.");
      CkwsLoopOptimizerShPtr randomOptimizer = CkwsRandomOptimizer::create();
      randomOptimizer->penetration (hppProblem (0)->penetration ());

      PathOptimizerShPtr intermediateConfigOptimizer = PathOptimizer::create();
      intermediateConfigOptimizer->penetration
	(hppProblem (0)->penetration () / 100);

      /* Building wholebody mask, without the free flyer dofs */
      std::vector<bool> wbMask(humanoidRobot_->countDofs(),true);
      for(unsigned int i = 0; i<6; i++) wbMask[i] = false;

      intermediateConfigOptimizer->targetConfig(halfSittingCfg_);
      intermediateConfigOptimizer->setConfigMask(wbMask);

      // Initialize goal configuration optimizer
      goalExtendor_ = new hpp::constrained::ConfigExtendor (humanoidRobot_);
      constrained::ConfigOptimizerShPtr goalOptimizer =
	constrained::ConfigOptimizer::create (humanoidRobot_,
					      goalExtendor_,
					      halfSittingCfg_);
      numericOptimizer_ = roboptim::PathOptimizer::create
	(slidingStabilityConstraints_);
      assert (numericOptimizer_);
      std::vector<CkwsPathPlannerShPtr> optVector;
      optVector.push_back (goalOptimizer);
      optVector.push_back (randomOptimizer);
      optVector.push_back (numericOptimizer_);
      //optVector.push_back (intermediateConfigOptimizer);
      CkwsMultiplePlannerShPtr combinedOptimizer =
	CkwsMultiplePlanner::create (optVector);
      pathOptimizerIthProblem(0, combinedOptimizer);
      hppProblem (0)->alwaysOptimize (true);

      return KD_OK;
    }

    ktStatus
    Planner::initAndGoalConfig (CkwsPathConstShPtr inPath)
    {
      assert (inPath->device() == humanoidRobot_);

      CkwsConfigShPtr iCfg = inPath->configAtStart();
      CkwsConfigShPtr fCfg = inPath->configAtEnd();

      initConfIthProblem (0,iCfg);
      resetGoalConfIthProblem (0);
      addGoalConfIthProblem (0,fCfg);

      return KD_OK;
    }

    ktStatus Planner::generateGoalConfig (double xTarget,double yTarget,
					  double zTarget, unsigned int nbConfig)
    {
      vector3d handCenter;
      CjrlHand* hand = humanoidRobot_->rightHand();
      hand->getCenter (handCenter);
      CjrlJoint* reachingJoint = hand->associatedWrist ();
      rightHandConstraint_ =
	new ChppGikPositionConstraint (*humanoidRobot_, *reachingJoint,
				       handCenter,
				       vector3d(xTarget,yTarget,zTarget));

      // Initialize goal manifold stack of constraints
      goalConstraints_ = slidingStabilityConstraints_;
      goalConstraints_.push_back(rightHandConstraint_);
      constrained::GoalConfigGeneratorShPtr gcg = goalConfigGenerator (robotId);
      gcg->setConstraints(goalConstraints_);
      ConfigShooterReachingShPtr configShooter = ConfigShooterReaching::create
	(humanoidRobot_, model::Joint::fromJrlJoint (reachingJoint));
      configShooter->setTarget (xTarget, yTarget, zTarget);
      gcg->configShooter (configShooter);
      assert (numericOptimizer_);
      numericOptimizer_->setGoalConstraints (goalConstraints_);

      if (generateGoalConfigurations(0, nbConfig) != KD_OK) {
	hppDout (error, "Failed to generate a goal configuration");
	return KD_ERROR;
      }

      goalExtendor_->setConstraints (goalConstraints_);
      return KD_OK;
    }

    CkwsPathShPtr
    Planner::findDynamicPath( CkwsPathConstShPtr i_path)
    {

      CkwsPathShPtr resultPath = CkwsPath::create(humanoidRobot_);
      CkwsPathConstShPtr pathToAnimate = i_path;
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

	  hppDout (info, "Animating path of length: "
		   << pathToAnimate->length());


	  CkwsPathShPtr animatedPath;
	  footprintOfParam_t ftprints;

	  if (computeFootPrints(pathToAnimate,ftprints) != KD_OK)
	    {
	      return resultPath;
	    }

	  hppDout (info, "Computed " << ftprints.size() << " footprints");

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
		    hppDout (error, "End Config of start path is not equivalent"
			     " to start config of end path.");

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

	      hppDout (error, "Animated path is not valid");

	      /* Finding first unvalid directPath */
	      unsigned int i = 0;
	      unsigned int n = animatedPath->countDirectPaths();

	      while ( ( i < n ) &&
		      (animatedPath->directPath(i)->isValid()) ) {
		i++;
	      }

	      hppDout (info, "First unvalid path: " << i << "/" << n);

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
		  hppDout (error,
			   "Failed to add second part of the path to"
			   " the agregator.");
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

      if (wholeAnimatedPath) hppProblem(robotId)->addPath ( wholeAnimatedPath) ;



      return resultPath;
    }


    ktStatus
    Planner::computeFootPrints (CkwsPathConstShPtr i_path,
				footprintOfParam_t & o_footPrintOfParam)
    {

      hppDout (info, "Computing footprints...");

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
	  hppDout (error, "No support Polygon found.");
	  return KD_ERROR;
	}

      if (!supportPol->isDoubleSupport())
	{
	  hppDout (error, "Robot is not in a double support configuration.");
	  return KD_ERROR;
	}

      currentFootPrint = supportPol->leftFootprint();

      while ( currentDist < length )
	{
	  hppDout (info, "Computing footprint at length: " << currentDist);

	  ChppGikFootprint * newFootPrint =
	    findNextFootPrint(i_path,currentDist,currentFootPrint,isRightFoot);
	  assert (newFootPrint);

	  hppDout (info, "Found footprint: "
		    << newFootPrint->x() << " , "
	    	    << newFootPrint->y() << " , "
		   << newFootPrint->th() << " , ");

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
    Planner::animatePath (CkwsPathConstShPtr i_path ,
			  footprintOfParam_t & i_footPrintOfParam )
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

	  double dTh = fabs (endTh - startTh);
	  if (dTh > M_PI)
	    dTh = fabs(atan2( sin (endTh - startTh) , cos (endTh - startTh))) ;

	  double deltaX = fabs ( (endX - startX) /  maxX_ );
	  double deltaY = fabs ( (endY - startY) /  maxY_ );
	  deltaY = 0;
	  double deltaTh = dTh  /  maxTheta_ ;

	  double stepFrac = std::max ( std::max ( deltaX , deltaY ),
				       deltaTh );

	  hppDout (info, "  deltaX: " << deltaX
		   << "  deltaY: " << deltaY
		   << "  deltaTh: " << deltaTh
		   << "  stepFrac: " <<  stepFrac);

	  stepFrac = sqrt(stepFrac);

	  zmpEndShiftTime *= stepFrac ;
	  zmpStartShiftTime *= stepFrac ;
	  footFlightTime *= stepFrac ;
	  stepHeight *= stepFrac ;
	}

      if ( footFlightTime < 1 )   samplingPeriod = 0.02;
      if ( footFlightTime < 0.4 ) samplingPeriod = 0.01;
      if ( footFlightTime < 0.2 ) samplingPeriod = 0.05;

      footFlightTime = ((int) (footFlightTime / samplingPeriod) +1) * samplingPeriod ;
      zmpStartShiftTime =  ((int) (zmpStartShiftTime /  samplingPeriod) +1) * samplingPeriod ;
      zmpEndShiftTime =  ((int) (zmpEndShiftTime /  samplingPeriod) +1) * samplingPeriod ;


      hppDout (info, "Step parameters: " << std::endl
	       << "\t footFlightTime: " << footFlightTime << std::endl
	       << "\t stepHeight: " << stepHeight << std::endl
	       << "\t zmpStartShiftTime: "<< zmpStartShiftTime << std::endl
	       << "\t zmpEndShiftTime: " << zmpEndShiftTime << std::endl
	       << "\t sampling period: " << samplingPeriod);

      if ( footFlightTime < 0.1 ) {
	hppDout (error, "Foot flight time too short.");
	newPath.reset();
	return newPath;
      }


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

      hppDout (info, "Total walking time: "  << time);

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

      //Config Constraint
      vectorN ubMaskVector = gikStandingRobot_->maskFactory()->upperBodyMask();
      vectorN wbMaskVector = gikStandingRobot_->maskFactory()->wholeBodyMask();

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


      hppDout (info, "Solving the task");

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
	  hppDout (info, "Failed to solve generic task");
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
	  hppDout (error, "Empty motion sample.");
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
	  hppDout (error, "Empty motion vector.");
	  return KD_ERROR;
	}
      else hppDout (info, "validGikMotion_ size: " << validGikMotion_.size ());

      std::vector<double> kineoCfg(humanoidRobot_->countDofs ());
      std::vector<double> openHrpCfg(humanoidRobot_->countDofs ());

      for (std::vector<ChppRobotMotion*>::iterator it = validGikMotion_.begin ();
	   it < validGikMotion_.end (); it++)
	{
	  hppDout (info, "start x: " << (*it)->firstSample ()->configuration[0]
		   << " "
		   << "end x: " << (*it)->lastSample ()->configuration[0]
		   << " ");

	  const ChppRobotMotionSample * motionSample = (*it)->firstSample();

	  if (!motionSample)
	    {
	      hppDout (error, "Empty motion sample.");
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
    Planner::findNextFootPrint(CkwsPathConstShPtr i_path,
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
    Planner::addLastFootPrint(CkwsPathConstShPtr i_path,
			      const ChppGikFootprint*,
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

    CkwsPathShPtr Planner::animateWholePath(CkwsPathConstShPtr i_path) {

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

      hppDout (info, " Animating Whole Path" << std::endl
	       << "Length: " << i_path->length());

      hppDout (info, "Footprints: ");

      for ( it = allFootprints.begin() ; it != allFootprints.end() ; it++)
	{
	  hppDout (info, "\tat length " << (*it).first
		    << " : "
		    << (*it).second->x() << ","
		    << (*it).second->y() << ","
		   << (*it).second->th() << ",");
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

      hppDout (info, "Total walking time: "  << time);

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
      ChppGikInterpolatedElement verticalElem ( gikStandingRobot_->robot(),
						waistParallelConstraint_,
						3,
						startTime,
						time,
						samplingPeriod);
      genericTask.addElement( &verticalElem );

      //Config Constraint
      vectorN ubMaskVector = gikStandingRobot_->maskFactory()->upperBodyMask();
      vectorN wbMaskVector = gikStandingRobot_->maskFactory()->wholeBodyMask();

      //Removing ff dofs from upper body mask vector
      for(unsigned int i = 0;i<6;i++)
	ubMaskVector[i] = 0;


      ChppGikConfigMotionConstraint cfgConstraint
	(humanoidRobot_,startTime,time,i_path,paramOfTime,ubMaskVector);
      ChppGikPrioritizedMotion cfgElement(&(*humanoidRobot_),4,&cfgConstraint,1e-6);
      cfgElement.workingJoints(ubMaskVector);
      genericTask.addElement( &cfgElement );

      hppDout (info, "Solving the task");

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

	  bool filterWorked = genericTask.filterZmpError();

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
	  filterWorked = genericTask.filterZmpError();

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
		hppDout (error, "when writing seqplay files.");
	      }
	      convertGikRobotMotionToKineoPath(&motion,newPath);
	    }

	  genericTask.filterZmpError();

	}
      else
	{
	  hppDout (info, "Failed to solve generic task");

	  ChppRobotMotion  motion =   genericTask.solutionMotion();
	  if (!motion.empty())
	    {
	      convertGikRobotMotionToKineoPath(&motion,newPath);
	    }
	  hppProblem(robotId)->addPath (newPath);

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
      ktStatus res =  solveOneProblem (robotId);
      //humanoidRobot_->userConstraints()->remove(wholeBodyConstraint_);

      if ( res != KD_OK ) return res;
      hppDout (info, "solveOneProblem succeeded.");
      CkwsPathShPtr resultPath =
	getPath (robotId ,getNbPaths (robotId) - 1);
      if (!resultPath) return KD_ERROR;

#if 0
      CkwsPathShPtr animatedPath = findDynamicPath ( resultPath );

      if (! animatedPath )
	return KD_ERROR;

      hppProblem (robotId)->addPath ( animatedPath);
#endif
      return KD_OK;
    }

#ifdef HPP_DEBUG
    static void logDiffusionShooter (CkwsDiffusionShooterShPtr shooter)
    {
      if (KIT_DYNAMIC_PTR_CAST (CkwsShooterConfigList, shooter)) {
	hppDout (info, "CkwsShooterConfigList");
      } else if (KIT_DYNAMIC_PTR_CAST (CkwsShooterConfigSpace, shooter)) {
	hppDout (info, "CkwsShooterConfigSpace");
      } else if (KIT_DYNAMIC_PTR_CAST (CkwsShooterAdaptiveMulti, shooter)) {
	hppDout (info, "CkwsShooterAdaptiveMulti");
      } else if (KIT_DYNAMIC_PTR_CAST (CkwsShooterPath, shooter)) {
	hppDout (info, "CkwsShooterPath");
      } else if (KIT_DYNAMIC_PTR_CAST (CkwsShooterRoadmapBox, shooter)) {
	hppDout (info, "CkwsShooterRoadmapBox");
      } else if (KIT_DYNAMIC_PTR_CAST (CkwsShooterRoadmapNodes, shooter)) {
	hppDout (info, "CkwsShooterRoadmapNodes");
      } else if (KIT_DYNAMIC_PTR_CAST (ShooterHumanoid, shooter)) {
	hppDout (info, "ShooterHumanoid");
      } else {
	hppDout (info, "unknown shooter.");
      }
    }
#endif
  }
}
