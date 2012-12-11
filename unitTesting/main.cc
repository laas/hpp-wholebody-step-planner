
/*
 *  Copyright
 */

#include <iostream>
#include <fstream>

#include "KineoModel/kppLicense.h"
#include "KineoModel/kppUserData.h"

#include "planner-test.hh"

using namespace hpp::wholeBodyStepPlanner;

int main ( int argc, char* argv[])
{
  if ( !CkppLicense::initialize() )
    {
      std::cerr << "Error: license for KPP SDK was not found" << std::endl;
      return -1;
    }
  else
    {
      std::cout << "license for KPP SDK was found" << std::endl;
    }

  if (argc < 4)
    {
      std::cerr << "ERROR: missing arguments, make sure to give path to Kineo environment, path to Kineo path, and path to output file in that order." << std::endl;
    }
  else if (argc > 4)
    {
      std::cout << "Warning: too many arguments given. The first 3 will be taken into account, please make sure they are correct." << std::endl;
    }

  struct timeval *Tps, *Tpf;
  struct timezone *Tzp;

  Tps = (struct timeval*) malloc (sizeof (struct timeval));
  Tpf = (struct timeval*) malloc (sizeof (struct timeval));
  Tzp = 0;

  std::ofstream timeFile;
  timeFile.open (argv[3]);
  timeFile << "Computation times in ms using: "<< std::endl;
  timeFile << "Penetration: " << 0.1 << std::endl;
  timeFile << "Environment: " << argv[1] << std::endl;
  timeFile << "Path and bounds: " << argv[2] <<std::endl;
  timeFile << "---------------------------------------------------------------"
	   << std::endl;
  
  CkppUserData::getInstance()->initializeParameters();
  CkwsUtility::randomSeed (time (NULL));

  for (unsigned int i = 0; i < 1; i++)
    {
      PlannerTest plannerTest (argv[1], argv[2], 0.1, 0.05);
      plannerTest.initScene();

      std::cout << "Starting test " << i+1 << std::endl;
      gettimeofday (Tps, Tzp);
      if (plannerTest.solveProblem () == KD_OK)
	{
	  gettimeofday (Tpf, Tzp);
	  timeFile << (Tpf->tv_sec - Tps->tv_sec) * 1000
	    + (Tpf->tv_usec - Tps->tv_usec) / 1000 << "\n";
	}
      else
	{
	  std::cerr << "ERROR:test " << i << " not solved." << std::endl;
	  return -1;
	}
    }

  free (Tps);
  free (Tpf);
  timeFile.close ();

  return 0;
}
