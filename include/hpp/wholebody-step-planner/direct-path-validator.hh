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

#ifndef HPP_WHOLEBODY_STEP_PLANNER_DIRECT_PATH_VALIDATOR_HH
# define HPP_WHOLEBODY_STEP_PLANNER_DIRECT_PATH_VALIDATOR_HH

# include <KineoWorks2/kwsValidatorDPCollision.h>
# include <hpp/util/kitelab.hh>

namespace hpp {
  namespace wholeBodyStepPlanner {
    HPP_KIT_PREDEF_CLASS (DirectPathValidator);
    class DirectPathValidator : public CkwsValidator {
    public:
      virtual CkwsValidatorShPtr clone() const;
      virtual ~DirectPathValidator () {}
      /// Applies the validator to a direct path
      virtual bool doValidateDirectPath
      (const CkwsDirectPath &directPath,
       const CkitParameterMapConstShPtr &params,
       const CkwsValidationReportConstShPtr &inReport,
       CkwsValidationReportShPtr &outReport) const;

      static DirectPathValidatorShPtr create (const CkwsConfigSpaceShPtr&
					      configSpace,
					      const std::string& name,
					      double penetration);
      static DirectPathValidatorShPtr createCopy
      (const DirectPathValidatorShPtr& validator);

    protected:
      DirectPathValidator (const CkwsConfigSpaceShPtr& configSpace,
			   const std::string& name,
			   double penetration);
      DirectPathValidator (const DirectPathValidator& validator);
      ktStatus init (DirectPathValidatorWkPtr);
    private:
      DirectPathValidatorWkPtr wkPtr_;
      double penetration_;
    }; // class DirectPathValidator
  } // namespace wholeBodyStepPlanner
} // namespace hpp
#endif // HPP_WHOLEBODY_STEP_PLANNER_DIRECT_PATH_VALIDATOR_HH
