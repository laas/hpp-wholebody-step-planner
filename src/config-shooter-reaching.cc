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

#include <KineoUtility/kitDefine.h>

#include <hpp/util/debug.hh>
#include <hpp/model/humanoid-robot.hh>
#include <hpp/model/joint.hh>

#include "hpp/wholebody-step-planner/config-shooter-reaching.hh"

#define DOF_X 0
#define DOF_Y 1
#define DOF_YAW 5
namespace hpp {
  namespace wholeBodyStepPlanner {
    void ConfigShooterReaching::shoot (CkwsConfig &io_config) const
    {
      // First, sample joint values, roll, pitch and elevation.
      io_config.randomStep (0.01);
      // Second, sample x, y, and yaw.
      double r = meanDistance_ +
	randomGenerator_->generateNormal (meanDistance_);
      double theta = static_cast <CkitPRNG*> (randomGenerator_.get ())->
	generate (-M_PI, M_PI);
      double yaw = theta + M_PI + randomGenerator_->generateNormal (0.1);
      io_config.dofValue (DOF_X, xTarget_ + r*cos (theta));
      io_config.dofValue (DOF_Y, yTarget_ + r*sin (theta));
      io_config.dofValue (DOF_YAW, yaw);
    }

    ConfigShooterReachingShPtr ConfigShooterReaching::
    create (hpp::model::HumanoidRobotShPtr robot,
	    model::JointShPtr reachingJoint)
    {
      ConfigShooterReaching* ptr =
	new ConfigShooterReaching (robot, reachingJoint);
      ConfigShooterReachingShPtr shPtr (ptr);
      ConfigShooterReachingWkPtr wkPtr (shPtr);
      ptr->init (wkPtr);
      return shPtr;
    }

    void ConfigShooterReaching::setTarget (double x, double y, double z)
    {
      xTarget_ = x;
      yTarget_ = y;
      zTarget_ = z;
    }

    ConfigShooterReaching::ConfigShooterReaching
    (hpp::model::HumanoidRobotShPtr robot, model::JointShPtr reachingJoint) :
      constrained::ConfigShooter (robot),
      robot_ (robot), reachingJoint_ (reachingJoint),
      randomGenerator_ (CkitPRNGGameRand::create ()), meanDistance_ (0),
      xTarget_ (0), yTarget_ (0), zTarget_ (0)
    {
      computeMeanDistance ();
    }

    void ConfigShooterReaching::init (ConfigShooterReachingWkPtr wkPtr)
    {
      ConfigShooter::init (wkPtr);
    }

    void ConfigShooterReaching::computeMeanDistance ()
    {
      CjrlJoint* waist = robot_->waist ();
      CjrlJoint* reachingJoint = reachingJoint_->jrlJoint ();
      
      std::vector<CjrlJoint*> jointVector =
	robot_->jointsBetween (*waist, *reachingJoint);

      double length = 0;
      const matrix4d & posWaist = waist->currentTransformation ();
      vector3d xPrev (posWaist (0,3), posWaist (1,3), posWaist (2,3));
      for (std::vector<CjrlJoint*>::iterator it = jointVector.begin ();
	   it != jointVector.end(); it++)
	{
	  const matrix4d& pos = (*it)->currentTransformation ();
	  vector3d x (pos (0,3), pos (1,3), pos (2,3));
	  length += (x - xPrev).norm ();
	  xPrev = x;
	}
      meanDistance_ = .5*length;
      hppDout (info, meanDistance_);
    }    
  } // namespace wholeBodyStepPlanner
} // namespace hpp
