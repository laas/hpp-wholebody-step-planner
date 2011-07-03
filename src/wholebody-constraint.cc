# include <tlcWholeBodyPlanner/tlcGikManager.h>
# include <hpp/wholebody-step-planner/wholebody-constraint.hh>

namespace hpp
{
  namespace wholeBodyStepPlanner
  {
    wholeBodyConstraint::wholeBodyConstraint ( const std::string & i_name, 
					       const ChppHumanoidRobotShPtr &i_robot,
					       CtlcGikManagerShPtr i_gikManager )
      :CkwsConstraint(i_name,i_robot),
       gikManager_(i_gikManager),
       humanoidRobot_(i_robot)
    {
    }
    
    wholeBodyConstraint::~wholeBodyConstraint()
    {
    }

    wholeBodyConstraintShPtr
    wholeBodyConstraint::create ( const std::string & i_name, 
				  const ChppHumanoidRobotShPtr &i_robot,
				  CtlcGikManagerShPtr i_gikManager )
    {
      wholeBodyConstraint * flatPtr = new wholeBodyConstraint(i_name, i_robot, i_gikManager);
      wholeBodyConstraintShPtr shPtr ( flatPtr );
      wholeBodyConstraintWkPtr wkPtr ( shPtr );
      if ( flatPtr->init(wkPtr) != KD_OK ) shPtr.reset();
      return shPtr;
    }

    ktStatus 
    wholeBodyConstraint::init( const wholeBodyConstraintWkPtr &inWeakPtr )
    {
      ktStatus success = this->CkwsConstraint::init( inWeakPtr );
      if ( success == KD_OK ) attWeakPtr = inWeakPtr;
      return success;
    }

    KIT_SHARED_PTR(CkwsValidator< CkwsConfig >)
    wholeBodyConstraint::clone() const
    {
      return create (name(), getHumanoidRobot(), getGikManager());
    }

    CtlcGikManagerShPtr 
    wholeBodyConstraint::getGikManager() const
    {
      return gikManager_;
    }

    ChppHumanoidRobotShPtr
    wholeBodyConstraint::getHumanoidRobot() const 
    {
      return humanoidRobot_;
    }

    ktStatus
    wholeBodyConstraint::setConstraints(std::vector<CjrlGikStateConstraint*> & sot)
    {
      gikConstraints_ = sot;
      return KD_OK;
    }
    
    ktStatus
    wholeBodyConstraint::doApply( CkwsConfig & io_config) const
    {
      if (!gikManager_)
	{
	  return KD_ERROR;
	}

      std::vector<double> i_dofs,o_dofs;
      io_config.getDofValues(i_dofs);

      gikManager_->setUserConstraint(gikConstraints_);

      bool didProject = 
	gikManager_->configProjector(i_dofs,o_dofs);

      if(didProject)
	{
	  io_config.setDofValues(o_dofs);
	  return KD_OK;
	}
      else
	{
	  return KD_ERROR;
	}
    }

  }
}
