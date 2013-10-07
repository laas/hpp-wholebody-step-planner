// Copyright (C) 2012 by Florent Lamiraux
//
// This file is part of the hpp-wholebody-step-planner.
//
// hpp-wholebody-step-planner is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either
// version 3 of the License, or (at your option) any later version.
//
// hpp-wholebody-step-planner is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with hpp-wholebody-step-planner.  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_WHOLEBODY_STEP_PLANNER_SHOOTER_HUMANOID_HH
# define HPP_WHOLEBODY_STEP_PLANNER_SHOOTER_HUMANOID_HH

# include <KineoWorks2/kwsDiffusionShooter.h>
# include <hpp/model/fwd.hh>

#include "hpp/wholebody-step-planner/config.hh"

namespace hpp {
  namespace wholeBodyStepPlanner {
    HPP_KIT_PREDEF_CLASS (ShooterHumanoid);
    /// Diffusion shooter taking into account bounds of degrees of freedom

    /// Derives from 
    /// Shoots a random configuration according to the following principle:
    /// \li bounded degrees of freedom are uniformly sampled in the bounds,
    /// \li unbounded degrees of freedom are sampled according to a gaussian
    /// law with standard deviation defined by
    /// CkwsDiffusionShooter::standardDeviation. Default value is 0.
    class HPP_WHOLEBODY_STEP_PLANNER_DLLAPI ShooterHumanoid :
      public CkwsDiffusionShooter
    {
    public:
      typedef unsigned int size_type;
      static ShooterHumanoidShPtr create (model::DeviceShPtr robot,
					    double standardDeviation);

      /// Shoot a random configuration

      /// \param node shared pointer to a diffusion node
      /// \param type type of the diffusion node (START, GOAL, WAYPOINT)
      /// \param standardDeviation standard deviation (where relevant)
      /// \param shootData data stack for shootWasUseful() or shootWasUseless()
      /// \param cfg random configuration produced
      virtual ktStatus shoot (const CkwsNodeShPtr& node,
			      CkwsDiffusingRdmBuilder::EDiffusionNodeType type,
			      double standardDeviation,
			      TShootData& shootData,
			      CkwsConfig& cfg);
    protected:
      ShooterHumanoid (model::DeviceShPtr robot, double standardDeviation);
      /// Initialization
      void init (ShooterHumanoidWkPtr wkPtr);
    private:
      model::DeviceShPtr robot_;
      double standardDeviation_;
      CkitPRNGShPtr randomGenerator_;
    }; // class ShooterHumanoid
  } // namespace wholeBodyStepPlanner
} //namespace hpp

#endif // HPP_WHOLEBODY_STEP_PLANNER_SHOOTER_HUMANOID_HH
