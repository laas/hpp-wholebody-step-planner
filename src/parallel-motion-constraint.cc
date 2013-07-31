// Copyright (C) 2013 CNRS-LAAS
// Author: Antonio El Khoury.
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
# include <hpp/gik/constraint/parallel-constraint.hh>

# include <hpp/model/joint.hh>
# include <hpp/wholebody-step-planner/parallel-motion-constraint.hh>

namespace hpp
{
  namespace wholeBodyStepPlanner
  {
    ChppGikParallelMotionConstraint::
    ChppGikParallelMotionConstraint
    (const hpp::model::HumanoidRobotShPtr humanoidRobot,
     const double startTime,
     const double endTime,
     const CkwsPathConstShPtr inPath,
     const std::map<double,double>& paramOfTime,
     hpp::model::JointShPtr constrainedJoint):
      parallelConstraint_(NULL),
      startTime_(startTime),
      endTime_(endTime),
      wbPath_(inPath),
      paramOfTime_(paramOfTime),
      humanoidRobot_(humanoidRobot),
      joint_(constrainedJoint)
    {
      parallelConstraint_ = new ChppGikParallelConstraint
	(*humanoidRobot_,*(constrainedJoint->jrlJoint()),
	 vector3d(0,0,0), vector3d(0,0,0));
    }

    ChppGikParallelMotionConstraint::~ChppGikParallelMotionConstraint()
    {
      if ( parallelConstraint_ ) {
	delete parallelConstraint_;
	parallelConstraint_ = NULL;
      }
    }

    CjrlDynamicRobot*
    ChppGikParallelMotionConstraint::robot()
    {
      return &(*humanoidRobot_);
    }

    CjrlGikMotionConstraint*
    ChppGikParallelMotionConstraint::clone() const
    {
      ChppGikParallelMotionConstraint * ret =
	new ChppGikParallelMotionConstraint(humanoidRobot_,
					    startTime_,
					    endTime_,
					    wbPath_,
					    paramOfTime_,
					    joint_);
      return ret;
    }

    CjrlGikStateConstraint*
    ChppGikParallelMotionConstraint::stateConstraintAtTime(double inTime)
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

      humanoidRobot_->setCurrentConfig(kineoCfg);

      CkitMat4 jointT = joint_->kppJoint()->kwsJoint()->currentPosition();
      // FIXME local and target vectors set to the target z axis for now.
      vector3d localVector(0,0,1);
      vector3d targetVector(0,0,1);

      parallelConstraint_->localVector (localVector);
      parallelConstraint_->targetVector (targetVector);

      return parallelConstraint_;
    }

    void
    ChppGikParallelMotionConstraint::startTime(double inStartTime)
    {
      startTime_ = inStartTime;
    }

    double
    ChppGikParallelMotionConstraint::startTime()
    {
      return startTime_;
    }

    double
    ChppGikParallelMotionConstraint::endTime()
    {
      return endTime_;
    }
  }
}
