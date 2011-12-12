// Copyright (C) 2011 by Sebastien Dalibard.
//

#ifndef CHPP_WHOLEBODY_STEP_PLANNER
#define CHPP_WHOLEBODY_STEP_PLANNER

# include <fstream>
# include <vector>
# include <map> 

# include <hpp/gik/robot/foot-print-related.hh>
# include <hpp/gik/robot/robot-motion.hh>
# include <hpp/gik/robot/standing-robot.hh>
# include <hpp/gik/constraint/parallel-constraint.hh>
# include <hpp/gik/constraint/plane-constraint.hh>
# include <hpp/gik/motionplanner/element/step-element.hh>
# include <hpp/gik/motionplanner/element/interpolated-element.hh>
# include <hpp/core/planner.hh>
# include <hppModel/hppHumanoidRobot.h>

# include <tlcWholeBodyPlanner/tlcGikManager.h>
# include <tlcWholeBodyPlanner/tlcGraspBallGoalGenerator.h>

# include <hpp/wholebody-step-planner/wholebody-constraint.hh>

namespace hpp
{
  namespace wholeBodyStepPlanner
  {
    class Planner : public ChppPlanner
    {
    public:

      typedef std::map<double,ChppGikFootprint *> footprintOfParam_t;

      Planner(double samplingPeriod);

      ~Planner();

      ChppHumanoidRobotShPtr humanoidRobot ();

      ChppGikStandingRobot* robot ();

      vector<ChppRobotMotion*> robotMotions ();

      double zmpStartShiftTime ();

      double zmpEndShiftTime ();

      double footFlightTime ();

      std::map<ChppGikFootprint*, double> stepFracOfFootprint ();

      std::vector<footprintOfParam_t> resultFootprints ();

      std::map<double,double> paramOfTime ();

      wholeBodyConstraintShPtr wholeBodyConstraint ();

      CtlcGraspBallGoalGeneratorShPtr getGoalTask();

      ktStatus goalWaistConfig (CkwsPathShPtr inPath);

      ktStatus initAndGoalConfig (CkwsPathShPtr inPath);

      ktStatus generateGoalConfig ();

      virtual ktStatus initializeProblem();

      virtual ktStatus solve();


      ktStatus computeFootPrints( CkwsPathShPtr i_path, 
				  footprintOfParam_t & o_footPrintOfParam);

      CkwsPathShPtr  animatePath ( CkwsPathShPtr i_path ,  
				   footprintOfParam_t & i_footPrintOfParam );

      CkwsPathShPtr animateWholePath (CkwsPathShPtr i_path);

  
      CkwsPathShPtr findDynamicPath ( CkwsPathShPtr i_path );

      void setFootPrintLimits(double minX,
			      double maxX,
			      double minY,
			      double maxY,
			      double minTheta,
			      double maxTheta);
      
      ktStatus writeSeqplayFiles ();

      ktStatus
      kwsToOpenHrpDofValues (const std::vector<double>& inKwsDofVector,
			     vector<double>& outOpenHrpDofVector);

    protected:

      ChppGikFootprint * findNextFootPrint(CkwsPathShPtr i_path, 
					   double & param, 
					   const ChppGikFootprint * currentFootPrint,
					   bool isRightFoot);

      ChppGikFootprint * addLastFootPrint(CkwsPathShPtr i_path, 
					  const ChppGikFootprint * currentFootPrint,
					  bool isRightFoot);

      void freeFootPrints(footprintOfParam_t & footPrintOfParam);

      ktStatus convertGikRobotMotionToKineoPath(ChppRobotMotion * i_motion, 
						CkwsPathShPtr o_path);
      
      ChppGikFootprint * footPrintFromConfig(CkwsConfig & cfg, 
					     bool isRightFoot);

      bool successiveFootPrints(const ChppGikFootprint * ft1,
				const ChppGikFootprint * ft2,
				bool isRightFoot);

      
 
    private:
      
      ChppHumanoidRobotShPtr humanoidRobot_;
      ChppGikStandingRobot * gikStandingRobot_;
      double samplingPeriod_;
      CtlcGikManagerShPtr  gikManager_;
      wholeBodyConstraintShPtr wholeBodyConstraint_;

      /* Attributes for goal configurations generation */
      CtlcGraspBallGoalGeneratorShPtr goalConfigGenerator_;

      /* Footstep parameters */
      double zmpEndCoeff_;
      double zmpStartShiftTime_;
      double zmpEndShiftTime_;
      double footFlightTime_;
      double stepHeight_;
      
      /* Gik constraints allocated only once */
      ChppGikPlaneConstraint * waistPlaneConstraint_;
      ChppGikParallelConstraint* waistParallelConstraint_;

      /* Humanoid robot specificities */
      matrix4d relativeRightFootTransformation_;
      matrix4d relativeLeftFootTransformation_;
      double waistZ_;

      /* Distance between two foot prints parameters */
      double minX_;
      double maxX_;
      double minY_;
      double maxY_;
      double minTheta_;
      double maxTheta_;

      /* Precision threshold for path exploration */
      double paramPrecision_;


      /* step parameters for each footprint */

      std::map < ChppGikFootprint *, double> stepFracOfFootprint_;
      std::vector  < footprintOfParam_t > resultFootprints_;
      std::map<double,double> paramOfTime_;

      /* Valid Gik motions to be outputted */
      ChppRobotMotion * currentGikMotion_;
      vector<ChppRobotMotion*> validGikMotion_;

      /* Seqplay files */
      double timestamp_;
      std::ofstream posFile_;
      std::ofstream rpyFile_;
      std::ofstream zmpFile_;
      std::ofstream zmpMeasuredFile_;

    };
  }
}

#endif
