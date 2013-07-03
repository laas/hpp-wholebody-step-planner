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

#ifndef CHPP_GIK_POSITION_MOTION_CONSTRAINT
#define CHPP_GIK_POSITION_MOTION_CONSTRAINT

# include <map>

# include <jrl/mal/matrixabstractlayer.hh>
# include <KineoUtility/kitDefine.h>
# include <KineoWorks2/kwsPath.h>
# include <gikTask/jrlGikMotionConstraint.h>
# include <hpp/model/humanoid-robot.hh>

class ChppGikPositionConstraint;

namespace hpp
{
  namespace wholeBodyStepPlanner
  {
    /// Time varying position constraint for a joint defined by a Kite path
    class ChppGikPositionMotionConstraint : public CjrlGikMotionConstraint
    {
    public:
      /// Constructor

      /// \param humanoidRobot: Robot,
      /// \param startTime, endTime Interval of definition
      /// \param inPath Path from Kitelab
      /// \param paramOfTime Mapping from time to parameter on Kite path,
      /// \param constrainedJoint Joint the position of which is constrained.
      ChppGikPositionMotionConstraint
      (const hpp::model::HumanoidRobotShPtr humanoidRobot,
       const double startTime,
       const double endTime,
       const CkwsPathConstShPtr inPath,
       const std::map<double,double> & paramOfTime,
       const hpp::model::JointShPtr constrainedJoint);

      ~ChppGikPositionMotionConstraint();

      virtual CjrlDynamicRobot* robot();

      virtual CjrlGikMotionConstraint* clone() const;

      /// hpp-gik position constraint at given time

      /// Return the position constraint corresponding to a given time
      /// along the input Kite path. Parameter along Kite path is obtained
      /// from a mapping from time to Kite parameter given at construction.
      /// \sa ChppGikPositionMotionConstraint::ChppGikPositionMotionConstraint.
      virtual CjrlGikStateConstraint* stateConstraintAtTime(double inTime);

      virtual void startTime(double inStartTime);

      virtual double startTime();

      virtual double endTime();

    private:
      ChppGikPositionConstraint * positionConstraint_;
      double startTime_;
      double endTime_;
      CkwsPathConstShPtr wbPath_;
      std::map<double,double> paramOfTime_;
      hpp::model::HumanoidRobotShPtr humanoidRobot_;
      hpp::model::JointShPtr joint_;
    };
  }
}

#endif
