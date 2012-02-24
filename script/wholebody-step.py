# This script is provided as an example.
# It is supposed to work only under some restrictive asumptions.

import sys, os
sys.path.append ('.')
import hpp.corbaserver
import hpp.corbaserver.wholebody_step
from hpp_corbaserver.hpp import Configuration
from tools import playPath, parseConfigInLog

cl = hpp.corbaserver.Client(['robot', 'obstacle', 'problem'])
clw = hpp.corbaserver.wholebody_step.client.Client()

model_dir = os.getenv('DEVEL_DIR')+'/model'

cl.robot.loadHrp2Model(.05)
cl.robot.setJointDisplayPath (0, 0, True)
half_sitting = cl.robot.getCurrentConfig(0)
cl.problem.initializeProblem()
cl.problem.parseFile(model_dir + "/appartment/appartment-toyota.kxml")

q = half_sitting[::]
q[0:2]=[-2., 1.]
cl.problem.setInitialConfig(0, q)

cl.robot.setCurrentConfig(0,q)
if clw.problem.generateGoalConfig (-0.29, -2.66, 1.2, 5) != 0:
    raise RuntimeError ("Failed to generate goal configuration.")
#cl.problem.setPathOptimizer (0, "none", 50)
cl.problem.solveOneProblem (0)

