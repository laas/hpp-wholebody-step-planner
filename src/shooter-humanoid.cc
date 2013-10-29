// Copyright (C) 2012 CNRS
//
// Author: Florent Lamiraux
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

#include <KineoUtility/kitPRNGGameRand.h>
#include <hpp/util/debug.hh>
#include <hpp/util/exception.hh>
#include <hpp/util/assertion.hh>
#include <hpp/model/device.hh>

#include "hpp/wholebody-step-planner/shooter-humanoid.hh"

namespace hpp {
  namespace wholeBodyStepPlanner {
    ShooterHumanoidShPtr ShooterHumanoid::create (model::DeviceShPtr robot,
						      double standardDeviation)
    {
      ShooterHumanoid* ptr = new ShooterHumanoid (robot, standardDeviation);
      ShooterHumanoidShPtr shPtr (ptr);
      ShooterHumanoidWkPtr wkPtr (shPtr);

      ptr->init (wkPtr);
      return shPtr;
    }

    ShooterHumanoid::ShooterHumanoid (model::DeviceShPtr robot,
				      double standardDeviation) :
      robot_ (robot), standardDeviation_ (standardDeviation),
      randomGenerator_ (CkitPRNGGameRand::create ())

    {
    }

    void ShooterHumanoid::init (ShooterHumanoidWkPtr wkPtr)
    {
      if (CkwsDiffusionShooter::init (wkPtr) != KD_OK) {
	HPP_THROW_EXCEPTION_
	  ("failed to init CkwsShooterHumanoid parent class.");
      }
    }

    ktStatus
    ShooterHumanoid::shoot (const CkwsNodeShPtr& node,
			    CkwsDiffusingRdmBuilder::EDiffusionNodeType,
			    double standardDeviation,
			    CkwsDiffusionShooter::TShootData&,
			    CkwsConfig& cfg)
    {
      HPP_ASSERT (cfg.device () == robot_);
      HPP_ASSERT (node->config ().device () == robot_);
      const CkwsConfig& diffusionConfig (node->config ());
      CkwsDevice::TDofVector dofVector;
      robot_->kwsDevice ()->getDofVector (dofVector);
      std::vector <double> dofValues (dofVector.size ());
      for (size_type i = 0; i<dofVector.size (); i++) {
	CkwsDofShPtr dof = dofVector [i];
	if (dof->isBounded ()) {
	  // If dof is bounded use uniform distribution within bounds.
	  dofValues [i] = randomGenerator_->generate
	    (dof->vmin (), dof->vmax ());
	} else {
	  // If dof is not bounded use normal distribution about initial value.
	  hppDout (info, "standardDeviation: " << standardDeviation_);
	  dofValues [i] = diffusionConfig.dofValue (i) +
	    randomGenerator_->generateNormal (standardDeviation_);
	}
      }
      ktStatus result = cfg.setDofValues (dofValues);
      HPP_ASSERT (result == KD_OK);
      return KD_OK;
    }
  } // namespace wholeBodyStepPlanner
} //namespace hpp
