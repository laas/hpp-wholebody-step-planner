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

#ifndef CHPP_WHOLEBODY_STEP_PLANNER
#define CHPP_WHOLEBODY_STEP_PLANNER

# include <fstream>
# include <vector>
# include <map>

# include <KineoUtility/kitDefine.h>


# include <hpp/core/planner.hh>
# include <hpp/model/humanoid-robot.hh>

# include <hpp/constrained/kws-constraint.hh>
# include <hpp/constrained/planner/planner.hh>

class ChppGikFootprint;
class ChppGikStandingRobot;
class ChppGikPlaneConstraint;
class ChppGikParallelConstraint;
class ChppRobotMotion;
class ChppGikPositionConstraint;
class ChppGikGenericTask;

namespace hpp
{
  namespace wholeBodyStepPlanner
  {
    namespace roboptim {
      KIT_PREDEF_CLASS (PathOptimizer)
    } // namespace roboptim

    class Planner : public hpp::constrained::Planner
    {
    public:
      typedef unsigned int size_type;
      typedef std::map<double,ChppGikFootprint *> footprintOfParam_t;

      static size_type robotId;

      Planner(double samplingPeriod);

      ~Planner();

      hpp::model::HumanoidRobotShPtr humanoidRobot ();

      hpp::constrained::KwsConstraintShPtr wholeBodyConstraint ();

      /// Initialize initial and goal configurations from a path
      ktStatus initAndGoalConfig (CkwsPathConstShPtr inPath);

      /// Generate goal configurations for the right wrist
      ///
      /// \param xTarget, yTarget, zTarget Requested position for the right
      /// wrist.
      /// \param nbConfig Number of goal configurations to generate.
      ///
      /// Build a vector of constraints containing
      /// \li double support static stability constraints,
      /// \li the rightwrist position constraint,
      /// \li a constraint for the waist to stay in a plane,
      /// \li a constraint for the waist orientation to remain horizontal.
      /// Initialize hpp::constrained::Planner::goalConfigGenerator_ with these
      /// constraints. Call
      /// hpp::constrained::Planner::generateGoalConfigurations to generate
      /// one goal configuration, and optimize the goal configuration using
      /// distance to robot configuration at beginning of function call.
      ktStatus generateGoalConfig (double xTarget, double yTarget,
				   double zTarget, unsigned int nbConfig);

      ChppGikStandingRobot* robot ();

      std::vector<ChppRobotMotion*> robotMotions ();

      double zmpStartShiftTime ();

      double zmpEndShiftTime ();

      double footFlightTime ();

      std::map<ChppGikFootprint*, double> stepFracOfFootprint ();

      std::vector<footprintOfParam_t> resultFootprints ();

      std::map<double,double> paramOfTime ();

      ktStatus goalWaistConfig (CkwsPathShPtr inPath);

      ktStatus initAndGoalConfig (CkwsPathShPtr inPath);

      ktStatus generateGoalConfig ();

      virtual ktStatus initializeProblem();

      virtual ktStatus solve();


      ktStatus computeFootPrints( CkwsPathConstShPtr i_path,
				  footprintOfParam_t & o_footPrintOfParam);

      CkwsPathShPtr  animatePath ( CkwsPathConstShPtr i_path ,
				   footprintOfParam_t & i_footPrintOfParam );

      CkwsPathShPtr animateWholePath (CkwsPathConstShPtr i_path);


      CkwsPathShPtr findDynamicPath ( CkwsPathConstShPtr i_path );

      void setFootPrintLimits(double minX,
			      double maxX,
			      double minY,
			      double maxY,
			      double minTheta,
			      double maxTheta);

      ktStatus writeSeqplayFiles ();

      ktStatus
      kwsToOpenHrpDofValues (const std::vector<double>& inKwsDofVector,
			     std::vector<double>& outOpenHrpDofVector);

    protected:

      CkwsConfigShPtr
      initializeRobot (hpp::model::HumanoidRobotShPtr humanoidRobot,
		       ChppGikStandingRobot*& gikStandingRobot,
		       double& waistZ, vector3d& ZLocalAxis);

      ChppGikFootprint * findNextFootPrint(CkwsPathConstShPtr i_path,
					   double & param,
					   const ChppGikFootprint * currentFootPrint,
					   bool isRightFoot);

      ChppGikFootprint * addLastFootPrint(CkwsPathConstShPtr i_path,
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

      /// \brief Add whole-body constraints to generic task.
      void
      addWholeBodyConstraints (const double& startTime,
			       const double& time,
			       const double& samplingPeriod,
			       const CkwsPathConstShPtr& i_path,
			       const std::map<double,double>& paramOfTime,
			       ChppGikGenericTask& genericTask);

    private:

      hpp::model::HumanoidRobotShPtr humanoidRobot_;
      ChppGikStandingRobot* gikStandingRobot_;
      double samplingPeriod_;
      /// Set of constraints enforcing static stability by calling
      /// hpp::constrained::Planner::
      /// buildDoubleSupportSlidingStaticStabilityConstraints.
      hpp::constrained::KwsConstraintShPtr wholeBodyConstraint_;

      /// Numeric optimizer called before generating steps
      roboptim::PathOptimizerShPtr numericOptimizer_;
      /// Final configuration optimizer
      hpp::constrained::ConfigExtendor* goalExtendor_;
      /// Target configuration for optimization
      CkwsConfigShPtr halfSittingCfg_;

      /* Footstep parameters */
      double zmpEndCoeff_;
      double zmpStartShiftTime_;
      double zmpEndShiftTime_;
      double footFlightTime_;
      double stepHeight_;

      /* Gik constraints allocated only once */
      ChppGikPlaneConstraint* waistPlaneConstraint_;
      ChppGikParallelConstraint* waistParallelConstraint_;
      ChppGikPositionConstraint* rightHandConstraint_;
      std::vector<CjrlGikStateConstraint*> slidingStabilityConstraints_;
      std::vector<CjrlGikStateConstraint*> goalConstraints_;

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

      /* Valid Gik motions to be produced as output */
      ChppRobotMotion * currentGikMotion_;
      std::vector<ChppRobotMotion*> validGikMotion_;

      /* Seqplay files */
      double timestamp_;
      std::ofstream posFile_;
      std::ofstream zmpFile_;
      std::ofstream rpyFile_;
      std::ofstream zmpMeasuredFile_;

    };
  }
}

#endif
