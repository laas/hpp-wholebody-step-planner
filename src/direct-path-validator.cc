// Copyright (C) 2013 CNRS-LAAS
// Author: Florent Lamiraux
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

#include <KineoUtility/kitParameterMap.h>
#include <KineoWorks2/kwsReportDPCollision.h>

#include <hpp/wholebody-step-planner/direct-path-validator.hh>

#include <hpp/util/debug.hh>
#include <hpp/kwsio/configuration.hh>

namespace hpp {
  namespace wholeBodyStepPlanner {
    bool DirectPathValidator::doValidateDirectPath
    (const CkwsDirectPath &directPath,
       const CkitParameterMapConstShPtr &params,
       const CkwsValidationReportConstShPtr &,
       CkwsValidationReportShPtr &outReport) const
    {
      double length = directPath.length ();
      double lPrev = 0;
      int nbSamples = (int) ceil (length/(2*penetration_)) + 1;
      double dl = length / (nbSamples - 1);
      for (int i=0; i<nbSamples; ++i) {
	double l = i*dl;
	CkwsConfigShPtr config = directPath.configAtDistance (l);
	CkwsDeviceShPtr device (config->device ());
	device->setCurrentConfig (*config);
	device->configValidators ()->validate (*config);
	if (!config->isValid ()) {
	  CkwsReportDPCollisionShPtr report (CkwsReportDPCollision::create ());
	  report->isValid (false);
	  report->length (lPrev);
	  outReport = report;
	  hppDout (info, "invalid config: " << *config);
	  return false;
	}
	lPrev = l;
      }
      outReport = CkwsValidationReport::create ();
      outReport->isValid (true);
      return true;
    }

    CkwsValidatorShPtr DirectPathValidator::clone() const
    {
      return DirectPathValidator::createCopy (wkPtr_.lock ());
    }

    DirectPathValidatorShPtr DirectPathValidator::create
    (const CkwsConfigSpaceShPtr& configSpace,
     const std::string& name,
     double penetration)
    {
      DirectPathValidator* ptr = new DirectPathValidator (configSpace, name,
							  penetration);
      DirectPathValidatorShPtr shPtr (ptr);
      DirectPathValidatorWkPtr wkPtr (shPtr);

      if (ptr->init (wkPtr) != KD_OK) {
	shPtr.reset ();
      }
      return shPtr;
    }
    
    DirectPathValidatorShPtr DirectPathValidator::createCopy
    (const DirectPathValidatorShPtr& validator)
    {
      DirectPathValidator* ptr = new DirectPathValidator (*validator);
      DirectPathValidatorShPtr shPtr (ptr);
      DirectPathValidatorWkPtr wkPtr (shPtr);

      if (ptr->init (wkPtr) != KD_OK) {
	shPtr.reset ();
      }
      return shPtr;
    }
    
    ktStatus DirectPathValidator::init (DirectPathValidatorWkPtr wkPtr)
    {
      if (CkwsValidator::init (wkPtr) != KD_OK) {
	hppDout (error, "CkwsValidator::init failed");
	return KD_ERROR;
      }
      wkPtr_ = wkPtr;
      return KD_OK;
    }

    DirectPathValidator::DirectPathValidator (const CkwsConfigSpaceShPtr&
					      configSpace,
					      const std::string& name,
					      double penetration) :
      CkwsValidator (configSpace, name), penetration_ (penetration)
    {
    }

    DirectPathValidator::DirectPathValidator (const DirectPathValidator&
					      validator) :
      CkwsValidator (validator), penetration_ (validator.penetration_)
    {
    }

  } // namespace wholeBodyStepPlanner
} // namespace hpp
