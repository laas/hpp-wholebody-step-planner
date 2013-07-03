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
# include <hpp/gik/constraint/plane-constraint.hh>

# include <hpp/model/joint.hh>
# include <hpp/wholebody-step-planner/plane-motion-constraint.hh>

namespace hpp
{
  namespace wholeBodyStepPlanner
  {
    ChppGikPlaneMotionConstraint::
    ChppGikPlaneMotionConstraint
    (const hpp::model::HumanoidRobotShPtr humanoidRobot,
     const double startTime,
     const double endTime,
     const CkwsPathConstShPtr inPath,
     const std::map<double,double>& paramOfTime,
     hpp::model::JointShPtr constrainedJoint):
      planeConstraint_(NULL),
      startTime_(startTime),
      endTime_(endTime),
      wbPath_(inPath),
      paramOfTime_(paramOfTime),
      humanoidRobot_(humanoidRobot),
      joint_(constrainedJoint)
    {
      vector3d targetPoint(0,0,0);
      vector3d targetNormal(0,0,0);
      planeConstraint_ = new ChppGikPlaneConstraint
	(*humanoidRobot_,*(constrainedJoint->jrlJoint()),
	 vector3d(0,0,0),targetPoint, targetNormal);
    }

    ChppGikPlaneMotionConstraint::~ChppGikPlaneMotionConstraint()
    {
      if ( planeConstraint_ ) {
	delete planeConstraint_;
	planeConstraint_ = NULL;
      }
    }

    CjrlDynamicRobot*
    ChppGikPlaneMotionConstraint::robot()
    {
      return &(*humanoidRobot_);
    }

    CjrlGikMotionConstraint*
    ChppGikPlaneMotionConstraint::clone() const
    {
      ChppGikPlaneMotionConstraint * ret =
	new ChppGikPlaneMotionConstraint(humanoidRobot_,
					    startTime_,
					    endTime_,
					    wbPath_,
					    paramOfTime_,
					    joint_);
      return ret;
    }

    CjrlGikStateConstraint*
    ChppGikPlaneMotionConstraint::stateConstraintAtTime(double inTime)
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
      vector3d targetPoint(jointT(0,3),jointT(1,3),jointT(2,3));
      // FIXME target normal hard-coded for now.
      vector3d targetNormal(0,0,1);

      planeConstraint_->worldPlanePoint (targetPoint);
      planeConstraint_->worldPlaneNormal (targetNormal);

      return planeConstraint_;
    }

    void
    ChppGikPlaneMotionConstraint::startTime(double inStartTime)
    {
      startTime_ = inStartTime;
    }

    double
    ChppGikPlaneMotionConstraint::startTime()
    {
      return startTime_;
    }

    double
    ChppGikPlaneMotionConstraint::endTime()
    {
      return endTime_;
    }
  }
}
