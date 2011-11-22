// Copyright (C) 2011 by Sebastien Dalibard.
//

#ifndef CHPP_GIK_CONFIG_MOTION_CONSTRAINT
#define CHPP_GIK_CONFIG_MOTION_CONSTRAINT

# include <map> 

# include <jrl/mal/matrixabstractlayer.hh>
# include <KineoUtility/kitDefine.h>
# include <KineoWorks2/kwsPath.h>
# include <gikTask/jrlGikMotionConstraint.h>
# include <hpp/model/humanoid-robot.hh>

class ChppGikConfigurationConstraint;


namespace hpp
{
  namespace wholeBodyStepPlanner
  {
    class ChppGikConfigMotionConstraint : public CjrlGikMotionConstraint
    {
    public:
      ChppGikConfigMotionConstraint(const hpp::model::HumanoidRobotShPtr humanoidRobot,
				    const double startTime,
				    const double endTime,
				    const CkwsPathShPtr inPath,
				    const std::map<double,double> & paramOfTime,
				    const vectorN maskVector);
      
      ~ChppGikConfigMotionConstraint();

      virtual CjrlDynamicRobot* robot();

      virtual CjrlGikMotionConstraint* clone() const;

      virtual CjrlGikStateConstraint* stateConstraintAtTime(double inTime);
      
      virtual void startTime(double inStartTime);

      virtual double startTime();

      virtual double endTime();

    private:
      ChppGikConfigurationConstraint * configConstraint_;
      double startTime_;
      double endTime_;
      CkwsPathShPtr wbPath_;
      std::map<double,double> paramOfTime_;
      hpp::model::HumanoidRobotShPtr humanoidRobot_;
      vectorN maskVector_;
      
    };
  }
}

#endif
