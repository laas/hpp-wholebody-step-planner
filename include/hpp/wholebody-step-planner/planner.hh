// Copyright (C) 2011 by Sebastien Dalibard.
//

#ifndef CHPP_WHOLEBODY_STEP_PLANNER
#define CHPP_WHOLEBODY_STEP_PLANNER

# include <fstream>
# include <vector>
# include <map> 

# include <KineoUtility/kitDefine.h>

# include <hpp/core/planner.hh>
# include <hpp/model/humanoid-robot.hh>

# include <hpp/constrained/kws-constraint.hh>

class ChppGikFootprint;
class ChppGikStandingRobot;
class ChppGikPlaneConstraint;
class ChppGikParallelConstraint;

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

      hpp::model::HumanoidRobotShPtr humanoidRobot ();

      hpp::constrained::KwsConstraintShPtr wholeBodyConstraint ();

      /*
      CtlcGraspBallGoalGeneratorShPtr getGoalTask();

      ktStatus goalWaistConfig (CkwsPathShPtr inPath);
      */
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
      
      ktStatus
      kwsToOpenHrpDofValues (const std::vector<double>& inKwsDofVector,
			     std::vector<double>& outOpenHrpDofVector);

      ChppGikFootprint * footPrintFromConfig(CkwsConfig & cfg, 
					     bool isRightFoot);

      bool successiveFootPrints(const ChppGikFootprint * ft1,
				const ChppGikFootprint * ft2,
				bool isRightFoot);

      
 
    private:
      
      hpp::model::HumanoidRobotShPtr humanoidRobot_;
      ChppGikStandingRobot * gikStandingRobot_;
      double samplingPeriod_;
      hpp::constrained::KwsConstraintShPtr wholeBodyConstraint_;

      /* Attributes for goal configurations generation */
      //CtlcGraspBallGoalGeneratorShPtr goalConfigGenerator_;

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


      /* Valid Gik motions to be outputted */
      ChppRobotMotion * currentGikMotion_;
      std::vector<ChppRobotMotion*> validGikMotion_;

      /* Seqplay files */
      double timestamp_;
      std::ofstream posFile_;
      std::ofstream zmpFile_;
      std::ofstream rpyFile_;

    };
  }
}

#endif
