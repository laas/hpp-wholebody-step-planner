// Copyright (C) 2011 by Sebastien Dalibard.
//

#ifndef C_HPP_WHOLEBODY_STEP_PLANNER_CONSTRAINT
#define C_HPP_WHOLEBODY_STEP_PLANNER_CONSTRAINT

# include <vector>

# include <KineoUtility/kitDefine.h>
# include <KineoWorks2/kwsConstraint.h>

KIT_PREDEF_CLASS (CtlcGikManager);

namespace hpp
{
  namespace wholeBodyStepPlanner
  {
    KIT_PREDEF_CLASS(wholeBodyConstraint);
    
    class wholeBodyConstraint : public CkwsConstraint
    {
    public:
      
      ~wholeBodyConstraint();

      static wholeBodyConstraintShPtr create ( const std::string & i_name, 
					       const ChppHumanoidRobotShPtr &i_robot,
					       CtlcGikManagerShPtr i_gikManager );

      KIT_SHARED_PTR(CkwsValidator< CkwsConfig >) clone() const;

      ktStatus setConstraints(std::vector<CjrlGikStateConstraint*> & sot);

      ktStatus doApply (CkwsConfig & io_config) const ;
      
      CtlcGikManagerShPtr getGikManager() const;
      
      ChppHumanoidRobotShPtr getHumanoidRobot() const;


    protected:

      wholeBodyConstraint ( const std::string & i_name, 
			    const ChppHumanoidRobotShPtr &i_robot,
			    CtlcGikManagerShPtr i_gikManager );

      ktStatus init ( const wholeBodyConstraintWkPtr &inWeakPtr );

    private:
      wholeBodyConstraintWkPtr attWeakPtr;
      CtlcGikManagerShPtr gikManager_;
      std::vector< CjrlGikStateConstraint * > gikConstraints_;
      ChppHumanoidRobotShPtr humanoidRobot_;

    };
  }
}
# endif
