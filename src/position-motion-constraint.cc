# include <vector>
# include <string>
# include <iostream>

# include <KineoModel/kppConfigComponent.h>
# include <jrl/mal/matrixabstractlayer.hh>
# include <hpp/gik/constraint/position-constraint.hh>

# include <hpp/wholebody-step-planner/position-motion-constraint.hh>

# include <hppModel/hppJoint.h>

namespace hpp
{
  namespace wholeBodyStepPlanner
  {
    ChppGikPositionMotionConstraint::ChppGikPositionMotionConstraint(const ChppHumanoidRobotShPtr humanoidRobot,
								     const double startTime,
								     const double endTime,
								     const CkwsPathShPtr inPath,
								     const std::map<double,double> & paramOfTime,
								     CkppJointComponentShPtr constrainedJoint):
      positionConstraint_(NULL),
      startTime_(startTime),
      endTime_(endTime),
      wbPath_(inPath),
      paramOfTime_(paramOfTime),
      humanoidRobot_(humanoidRobot),
      joint_(constrainedJoint)
    {
      vector3d target;
      ChppJoint* hppJoint = humanoidRobot_->kppToHppJoint (joint_);
      positionConstraint_ = new ChppGikPositionConstraint(*humanoidRobot_,*(hppJoint->jrlJoint()),vector3d(0,0,0),target);
    }

    ChppGikPositionMotionConstraint::~ChppGikPositionMotionConstraint()
    {
      if ( positionConstraint_ ) {
	delete positionConstraint_;
	positionConstraint_ = NULL;
      }
    }

    CjrlDynamicRobot* 
    ChppGikPositionMotionConstraint::robot()
    {
      return &(*humanoidRobot_);
    }

    CjrlGikMotionConstraint* 
    ChppGikPositionMotionConstraint::clone() const 
    {
      ChppGikPositionMotionConstraint * ret = 
	new ChppGikPositionMotionConstraint(humanoidRobot_, 
					    startTime_, 
					    endTime_, 
					    wbPath_,
					    paramOfTime_,
					    joint_);
      return ret;
    }

    CjrlGikStateConstraint* 
    ChppGikPositionMotionConstraint::stateConstraintAtTime(double inTime)
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

      // CkwsConfig kineoCfg(humanoidRobot_);
      // wbPath_->getConfigAtDistance(dist,kineoCfg );
      std::vector<CkitMat4> jointTVector;
      wbPath_->getMatVectorAtDistance(dist, jointTVector);

      // humanoidRobot_->setCurrentConfig(kineoCfg);
      unsigned int jointRank;
      humanoidRobot_->getRankOfJoint (joint_->kwsJoint (),
				      jointRank);

      // CkitMat4 jointT = joint_->kwsJoint->currentPosition();
      CkitMat4 jointT = jointTVector[jointRank];

      vector3d target(jointT(0,3),jointT(1,3),jointT(2,3));

      positionConstraint_->worldTarget(target);

      return positionConstraint_;
    }

    void 
    ChppGikPositionMotionConstraint::startTime(double inStartTime)
    {
      startTime_ = inStartTime;
    }

    double 
    ChppGikPositionMotionConstraint::startTime()
    {
      return startTime_;
    }

    double 
    ChppGikPositionMotionConstraint::endTime()
    {
      return endTime_;
    }

  }
}
