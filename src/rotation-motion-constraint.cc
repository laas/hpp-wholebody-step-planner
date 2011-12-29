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
# include <hpp/gik/constraint/rotation-constraint.hh>

# include <hpp/model/joint.hh>
# include <hpp/wholebody-step-planner/rotation-motion-constraint.hh>

namespace hpp
{
  namespace wholeBodyStepPlanner
  {
    ChppGikRotationMotionConstraint::ChppGikRotationMotionConstraint(const hpp::model::HumanoidRobotShPtr humanoidRobot,
								     const double startTime,
								     const double endTime,
								     const CkwsPathShPtr inPath,
								     const std::map<double,double> & paramOfTime,
								     hpp::model::JointShPtr constrainedJoint):
      rotationConstraint_(NULL),
      startTime_(startTime),
      endTime_(endTime),
      wbPath_(inPath),
      paramOfTime_(paramOfTime),
      humanoidRobot_(humanoidRobot),
      joint_(constrainedJoint)
    {
      matrix3d target;
      rotationConstraint_ = new ChppGikRotationConstraint(*humanoidRobot_,*(constrainedJoint->jrlJoint()),target);
    }

    ChppGikRotationMotionConstraint::~ChppGikRotationMotionConstraint()
    {
      if ( rotationConstraint_ ) {
	delete rotationConstraint_;
	rotationConstraint_ = NULL;
      }
    }

    CjrlDynamicRobot* 
    ChppGikRotationMotionConstraint::robot()
    {
      return &(*humanoidRobot_);
    }

    CjrlGikMotionConstraint* 
    ChppGikRotationMotionConstraint::clone() const 
    {
      ChppGikRotationMotionConstraint * ret = 
	new ChppGikRotationMotionConstraint(humanoidRobot_, 
					    startTime_, 
					    endTime_, 
					    wbPath_,
					    paramOfTime_,
					    joint_);
      return ret;
    }

    CjrlGikStateConstraint* 
    ChppGikRotationMotionConstraint::stateConstraintAtTime(double inTime)
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
	  dist = dist_i + (dist_f - dist_i) * (inTime - time_i) / (time_f - time_i);
	}

      CkwsConfig kineoCfg(humanoidRobot_);
      wbPath_->getConfigAtDistance(dist,kineoCfg );

      humanoidRobot_->setCurrentConfig(kineoCfg);

      CkitMat4 jointT = joint_->kppJoint()->kwsJoint()->currentPosition();

      matrix3d target(jointT(0,0),jointT(0,1),jointT(0,2),
		      jointT(1,0),jointT(1,1),jointT(1,2),
		      jointT(2,0),jointT(2,1),jointT(2,2));

      rotationConstraint_->targetOrientation(target);

      return rotationConstraint_;
    }

    void 
    ChppGikRotationMotionConstraint::startTime(double inStartTime)
    {
      startTime_ = inStartTime;
    }

    double 
    ChppGikRotationMotionConstraint::startTime()
    {
      return startTime_;
    }

    double 
    ChppGikRotationMotionConstraint::endTime()
    {
      return endTime_;
    }

  }
}
