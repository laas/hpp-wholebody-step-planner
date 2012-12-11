#include <iostream>
#include <fstream>

#include <KineoModel/kppUserData.h>
#include <KineoWorks2/kwsUtility.h>
#include <KineoModel/kppLicense.h>

#include <kwsIO/kwsioPath.h>
#include <kwsIO/kwsioConfig.h>

# include <kwsPlus/roadmap/kwsPlusLTRdmBuilder.h>
# include <kwsPlus/roadmap/kwsPlusPCARdmBuilder.h>

#include "load-hrp2.cc"
#include "path-parser.cc"
#include <hpp/wholebody-step-planner/planner.hh>
#include <hpp/wholebody-step-planner/kwsPlusBenchRdmBuilder.h>


int main(int argc, char *argv[])
{
  using namespace hpp::wholeBodyStepPlanner;

  // Check arguments number. Should be 1 if only path is given, and 2 if environment and path (in that order) are given.
  if (argc != 2 && argc != 3)
    {
      std::cerr	<< "Wrong arguments given, expected 1 (path to path.kxml) or 2 (path to env.kxml and path.kxml)" << std::endl;
      return -1;
    }

  // Check license.
  if ( !CkppLicense::initialize() )
    {
      std::cerr << "Error: license for KPP SDK was not found" << std::endl;
      return -1;
    }
  else
    {
      std::cout << "license for KPP SDK was found" << std::endl;
    }

  CkppUserData::getInstance()->initializeParameters();

  const char benchFile[] = "bench.log";
  std::ofstream log(benchFile, std::ofstream::app);

      CkwsUtility::randomSeed (time (NULL));

      // Create planner instance and set footprint limits.
      Planner* planner = new Planner (0.1);
  
      // planner->setFootPrintLimits(-0.17,0.17,-0.25,-0.16,-M_PI /4,0.1);

      // Build HRP-2 robot and load it in planner.
      ChppHumanoidRobotShPtr robot;
      if (loadHrp2Model (robot) !=  KD_OK)
	std::cerr << "Planner::initScene(): error in loading HRP2"
		  << std::endl;
  
      planner->addHppProblem(robot,0.1);

      // Load environment from file given in first argument if two
      // arguments are given.
      if (argc == 3)
	if (KD_OK != planner->parseFile (argv[1]))
	  {
	    std::cerr << "Planner::parseFile: could not load environment."
		      << std::endl;
	    std::cerr << "Make sure that the environment path is correctly set."
		      << std::endl;
	    return -1;
	  }
  
      // Initialize planner.
      planner->initializeProblem ();

      // Set special bound on robot freeflyer depending on environment.
      if (argv[1] == "/home/aelkhour/virtuals/objects-cloud/env.kxml")
	{
	  planner->robotIthProblem (0)->kwsDevice ()->rootJoint ()->dof (0)
	    ->bounds (-2.5, 2.5);
	  planner->robotIthProblem (0)->kwsDevice ()->rootJoint ()->dof (1)
	    ->bounds (-1, 1);
	  planner->robotIthProblem (0)->kwsDevice ()->rootJoint ()->dof (5)
	    ->bounds (-M_PI / 4, M_PI / 4);
	}
      if (argv[1] == "/home/aelkhour/virtuals/ijrr-11/demo-11/env.kxml")
	{
	  planner->robotIthProblem (0)->kwsDevice ()->rootJoint ()->dof (0)
	    ->bounds (-2., 1.);
	  planner->robotIthProblem (0)->kwsDevice ()->rootJoint ()->dof (1)
	    ->bounds (-1., 2.);
	}
      if (argv[1] == "/home/aelkhour/virtuals/chairs-floor/env.kxml")
	{
	  planner->robotIthProblem (0)->kwsDevice ()->rootJoint ()->dof (0)
	    ->bounds (-2., 2.);
	  planner->robotIthProblem (0)->kwsDevice ()->rootJoint ()->dof (1)
	    ->bounds (-0.1, 0.1);
	  planner->robotIthProblem (0)->kwsDevice ()->rootJoint ()->dof (2)
	    ->bounds (-0.001, 0.001);
	}

      // Load path containing initial and final configurations from file
      // given in first or second argument.
      CkwsPathShPtr path = CkwsPath::create (planner->humanoidRobot ());
      getKineoPathFromFile (argv[argc - 1], path);

      // Set initial and goal config in planner.
      CkwsConfig initConfig (robot);
      CkwsConfig goalConfig (robot);

      path->getConfigAtStart (initConfig);
      path->getConfigAtEnd (goalConfig);

      // Initialize random path optimizer.
      CkwsLoopOptimizerShPtr optimizer = CkwsRandomOptimizer::create();
      optimizer->penetration (planner->hppProblem (0)->penetration ());
      Timer timer;

      // ------------- RRT + OPTIMIZATION BENCHMARKS ----------------------

      KIT_SHARED_PTR(CkwsPlusBenchRdmBuilder<CkwsDiffusingRdmBuilder>)
      	benchRdmBuilder
      	= KIT_DYNAMIC_PTR_CAST(CkwsPlusBenchRdmBuilder<CkwsDiffusingRdmBuilder>,
      			       planner->roadmapBuilderIthProblem (0));
      // KIT_SHARED_PTR(CkwsPlusBenchRdmBuilder<CkwsIPPRdmBuilder>)
      // 	benchRdmBuilder
      // 	= KIT_DYNAMIC_PTR_CAST(CkwsPlusBenchRdmBuilder<CkwsIPPRdmBuilder>,
      // 			       planner->roadmapBuilderIthProblem (0));
      // KIT_SHARED_PTR(CkwsPlusBenchRdmBuilder< CkwsPlusLTRdmBuilder
      // 		     < CkwsDiffusingRdmBuilder> >) benchRdmBuilder
      // 	= KIT_DYNAMIC_PTR_CAST(CkwsPlusBenchRdmBuilder< CkwsPlusLTRdmBuilder
      // 			       < CkwsDiffusingRdmBuilder> >,
      // 			       planner->roadmapBuilderIthProblem (0));
      // KIT_SHARED_PTR(CkwsPlusBenchRdmBuilder< CkwsPlusPCARdmBuilder
      // 		     < CkwsDiffusingRdmBuilder> >) benchRdmBuilder
      // 	= KIT_DYNAMIC_PTR_CAST(CkwsPlusBenchRdmBuilder< CkwsPlusPCARdmBuilder
      // 			       < CkwsDiffusingRdmBuilder> >,
      // 			       planner->roadmapBuilderIthProblem (0));
      // KIT_SHARED_PTR(CkwsPlusBenchRdmBuilder< CkwsPlusPCARdmBuilder
      // 		     < CkwsIPPRdmBuilder> >) benchRdmBuilder
      // 	= KIT_DYNAMIC_PTR_CAST(CkwsPlusBenchRdmBuilder< CkwsPlusPCARdmBuilder
      // 			       < CkwsIPPRdmBuilder> >,
      // 			       planner->roadmapBuilderIthProblem (0));
      // KIT_SHARED_PTR(CkwsPlusBenchRdmBuilder< CkwsPlusLTRdmBuilder
      // 		     < CkwsIPPRdmBuilder> >) benchRdmBuilder
      // 	= KIT_DYNAMIC_PTR_CAST(CkwsPlusBenchRdmBuilder< CkwsPlusLTRdmBuilder
      // 			       < CkwsIPPRdmBuilder> >,
      // 			       planner->roadmapBuilderIthProblem (0));
      // KIT_SHARED_PTR(CkwsPlusBenchRdmBuilder
      // 		     < CkwsPlusPCARdmBuilder
      // 		     < CkwsPlusLTRdmBuilder
      // 		     < CkwsDiffusingRdmBuilder> > >) benchRdmBuilder
      // 	= KIT_DYNAMIC_PTR_CAST(CkwsPlusBenchRdmBuilder
      // 			       < CkwsPlusPCARdmBuilder
      // 			       <CkwsPlusLTRdmBuilder
      // 			       <CkwsDiffusingRdmBuilder> > >,
      // 			       planner->roadmapBuilderIthProblem (0));
      // KIT_SHARED_PTR(CkwsPlusBenchRdmBuilder
      // 		     < CkwsPlusPCARdmBuilder
      // 		     < CkwsPlusLTRdmBuilder
      // 		     < CkwsIPPRdmBuilder> > >) benchRdmBuilder
      // 	= KIT_DYNAMIC_PTR_CAST(CkwsPlusBenchRdmBuilder
      // 			       < CkwsPlusPCARdmBuilder
      // 			       <CkwsPlusLTRdmBuilder
      // 			       <CkwsIPPRdmBuilder> > >,
      // 			       planner->roadmapBuilderIthProblem (0));

      log << "# test_id(0) "
	  << "nb_it(1) "
	  << "time(2) "
	  << "mean_time(3) "
	  << "std_time(4) "
	  << "min_time(5) "
	  << "max_time(6) "
	  << "nb_diffusion_it(7) "
	  << "diffusion_time(8) "
	  << "mean_diffusion_time(9) "
	  << "std_diffusion_time(10) "
	  << "min_diffusion_time(11) "
	  << "max_diffusion_time(12) "
	  << "nb_extension_it(13) "
	  << "extension_time(14) "
	  << "mean_extension_time(15) "
	  << "std_extension_time(16) "
	  << "min_extension_time(17) "
	  << "max_extension_time(18) "
	  << "nb_nodes(19) "
	  << "path_lenth(20) "
	  << "nb_path_config(21) "
	  << "optim_time(22) "
	  << "optim_path_length(23) "
	  << "nb_optim_path_config(24)"
	  << std::endl;

  unsigned i = 0;
  unsigned nbFail = 0;
  while (i < 500)
    {
      // Reset data.
      benchRdmBuilder->resetAccumulators ();
      planner->roadmapBuilderIthProblem (0)->roadmap ()->clear();
      planner->initConfIthProblem (0, CkwsConfig::create (initConfig));
      planner->goalConfIthProblem (0, CkwsConfig::create (goalConfig));

      // Solve path planning problem for sliding robot.
      if (KD_OK == planner->solve ())
	{
	  std::cout << "success " << i << std::endl;
	  ++i;
	}
      else
	{
	  std::cout << "failure " << std::endl;
	  ++nbFail;
	  continue;
	}

      // Log RRT data.
      log << i << " ";

      log << benchRdmBuilder->getStepsCount () << " ";
      log << benchRdmBuilder->getStepsTotalTime () << " ";
      log << benchRdmBuilder->getStepsMeanTime () << " ";
      log << sqrt (benchRdmBuilder->getStepsVarianceTime ()) << " ";
      log << benchRdmBuilder->getStepsMinTime () << " ";
      log << benchRdmBuilder->getStepsMaxTime () << " ";

      log << benchRdmBuilder->getDiffusionsCount () << " ";
      log << benchRdmBuilder->getDiffusionsTotalTime () << " ";
      log << benchRdmBuilder->getDiffusionsMeanTime () << " ";
      log << sqrt (benchRdmBuilder->getDiffusionsVarianceTime ()) << " ";
      log << benchRdmBuilder->getDiffusionsMinTime () << " ";
      log << benchRdmBuilder->getDiffusionsMaxTime () << " ";

      log << benchRdmBuilder->getExtensionsCount () << " ";
      log << benchRdmBuilder->getExtensionsTotalTime () << " ";
      log << benchRdmBuilder->getExtensionsMeanTime () << " ";
      log << sqrt (benchRdmBuilder->getExtensionsVarianceTime ()) << " ";
      log << benchRdmBuilder->getExtensionsMinTime () << " ";
      log << benchRdmBuilder->getExtensionsMaxTime () << " ";

      log << benchRdmBuilder->getNodeCount () << " ";

      log << planner->getPath (0, planner->getNbPaths (0) - 1)
	->length () << " ";
      log << planner->getPath (0, planner->getNbPaths (0) - 1)
	->countConfigurations () << " ";
      
      // Optimze path..
      CkwsPathShPtr optimizedPath = CkwsPath::createCopy
	(planner->getPath (0, planner->getNbPaths (0) - 1));
      timer.reinit ();
      timer.start ();
      optimizer->optimizePath (optimizedPath);
      timer.stop ();

      // Log optimization data.
      log << timer.get () / 1e3 << " ";
      log << optimizedPath->length () << " ";
      log << optimizedPath->countConfigurations () << std::endl;
    }
  
  std::cout << "failure ratio: " << (double) nbFail / double (i - 1)  << std::endl;
  log << "# failure ratio: " << (double) nbFail / double (i - 1)  << std::endl;

  // ------------- ANIMATION -------------------------------

  // // Remove constraint that constrains feet to the ground before
  // // starting animation.

  // planner->humanoidRobot ()->userConstraints ()
  //   ->remove (planner->wholeBodyConstraint ());

  // // Animate Path.

  // planner->findDynamicPath (path);

  return 0;
}
