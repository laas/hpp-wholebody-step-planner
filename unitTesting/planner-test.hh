/*
 *  Copyright
 */

#include "hpp/wholebody-step-planner/planner.hh"

/**
 \brief UnitTesting class of class Planner
 */

namespace hpp
{
  namespace wholeBodyStepPlanner
  {
    class PlannerTest
    {
    public:
      PlannerTest (char * i_envFile,
		   char * i_pathFile,
		   const double i_penetration,
		   const double i_samplingPeriod);
      
      ~PlannerTest() {}

      void initScene();

      ktStatus initAndGoalConfig ();

      ktStatus solveProblem ();

      ktStatus findDynamicPath ();

    private:
      char * attEnvFile_;

      char * attPathFile_;

      double attPenetration_;

      Planner * attPlanner_;
    };
  }
}




