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

# include <vector>
# include <string>
# include <iostream>

# include <KineoModel/kppConfigComponent.h>
# include <jrl/mal/matrixabstractlayer.hh>
# include <hpp/gik/constraint/configuration-constraint.hh>
# include <hpp/model/humanoid-robot.hh>

# include <hpp/wholebody-step-planner/config-motion-constraint.hh>

namespace hpp
{
  namespace wholeBodyStepPlanner
  {
    using hpp::model::HumanoidRobotShPtr;

    ChppGikConfigMotionConstraint::
    ChppGikConfigMotionConstraint(const HumanoidRobotShPtr humanoidRobot,
				  const double startTime,
				  const double endTime,
				  const CkwsPathConstShPtr inPath,
				  const std::map<double,double> & paramOfTime,
				  const vectorN maskVector):
      configConstraint_(NULL),
      startTime_(startTime),
      endTime_(endTime),
      wbPath_(inPath),
      paramOfTime_(paramOfTime),
      humanoidRobot_(humanoidRobot),
      maskVector_(maskVector)
    {
      MAL_VECTOR_DIM(jrlConfig, double, humanoidRobot_->numberDof());
      configConstraint_ = new ChppGikConfigurationConstraint
	(*humanoidRobot_,jrlConfig,maskVector);
    }

    ChppGikConfigMotionConstraint::~ChppGikConfigMotionConstraint()
    {
      if ( configConstraint_ ) {
	delete configConstraint_;
	configConstraint_ = NULL;
      }
    }

    CjrlDynamicRobot*
    ChppGikConfigMotionConstraint::robot()
    {
      return &(*humanoidRobot_);
    }

    CjrlGikMotionConstraint*
    ChppGikConfigMotionConstraint::clone() const
    {
      ChppGikConfigMotionConstraint * ret =
	new ChppGikConfigMotionConstraint(humanoidRobot_,
					  startTime_,
					  endTime_,
					  wbPath_,
					  paramOfTime_,
					  maskVector_);
      return ret;
    }

    CjrlGikStateConstraint*
    ChppGikConfigMotionConstraint::stateConstraintAtTime(double inTime)
    {
      if (inTime > endTime_)
	inTime = endTime_;

      double time_i,time_f;
      double dist_i,dist_f;
      double dist;

      std::map<double,double>::iterator it =  paramOfTime_.lower_bound(inTime);


      if (it == paramOfTime_.end() )
	{
	  it--;
	  dist = (*it).second;
	}
      else if (it == paramOfTime_.begin())
	{
	  dist = (*it).second;
	}
      else
	{
	  time_f = (*it).first;
	  dist_f = (*it).second;
	  it--;
	  time_i = (*it).first;
	  dist_i = (*it).second;
	  dist = dist_i + (dist_f - dist_i)*(inTime - time_i)/(time_f - time_i);
	}

      CkwsConfig kineoCfg(humanoidRobot_);
      wbPath_->getConfigAtDistance(dist,kineoCfg );
      std::vector<double> cfg;
      kineoCfg.getDofValues(cfg);

      MAL_VECTOR_DIM(jrlCfg, double, humanoidRobot_->numberDof());
      humanoidRobot_->kwsToJrlDynamicsDofValues(cfg,jrlCfg);

      configConstraint_->target(jrlCfg);

      return configConstraint_;
    }

    void
    ChppGikConfigMotionConstraint::startTime(double inStartTime)
    {
      startTime_ = inStartTime;
    }

    double
    ChppGikConfigMotionConstraint::startTime()
    {
      return startTime_;
    }

    double
    ChppGikConfigMotionConstraint::endTime()
    {
      return endTime_;
    }

  }
}
