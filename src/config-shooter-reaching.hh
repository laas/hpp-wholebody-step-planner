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

#ifndef HPP_WHOLEBODY_STEP_PLANNER_CONFIG_SHOOTER_REACHING_HH
# define HPP_WHOLEBODY_STEP_PLANNER_CONFIG_SHOOTER_REACHING_HH

# include <hpp/model/fwd.hh>
# include <hpp/constrained/config-shooter.hh>

namespace hpp {
  namespace wholeBodyStepPlanner {
    KIT_PREDEF_CLASS (ConfigShooterReaching)
    /// Configuration shooter tuned to generate reaching goal configurations.

    /// Shoot configuration around a target in workspace. the position of the
    /// root joint is sampled in polar coordinate. The radius follows a Gaussian
    /// law centered at a distance proportional to the distance between the
    /// waist and the reaching joint along the kinematic chain.
    class ConfigShooterReaching : public constrained::ConfigShooter
    {
    public:
      typedef unsigned int size_type;
      virtual void shoot (CkwsConfig &io_config) const;
      static ConfigShooterReachingShPtr
      create (hpp::model::HumanoidRobotShPtr robot,
	      model::JointShPtr reachingJoint);
      void setTarget (double x, double y, double z);

    protected:
      ConfigShooterReaching (hpp::model::HumanoidRobotShPtr robot,
			     model::JointShPtr reachingJoint);

      void init (ConfigShooterReachingWkPtr);
    private:
      void computeMeanDistance ();
      model::HumanoidRobotShPtr robot_;
      model::JointShPtr reachingJoint_;
      CkitPRNGGameRandShPtr randomGenerator_;
      double meanDistance_;
      // Target to reach.
      double xTarget_, yTarget_, zTarget_;
    }; // class ConfigShooterReaching
  } // namespace wholeBodyStepPlanner
} // namespace hpp

#endif // HPP_WHOLEBODY_STEP_PLANNER_CONFIG_SHOOTER_REACHING_HH
