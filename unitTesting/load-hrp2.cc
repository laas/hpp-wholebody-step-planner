// Copyright (C) 2011 by Antonio El Khoury, CNRS.
//
// This file is part of the hpp-wholebody-optimizer.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#include <iostream>
#include <string>

#include <KineoKCDModel/kppKCDPolyhedron.h>

#include <robotbuilder/robotbuilder.hh>

#include <hpp/core/collision-pair.hh>
#include <hppModel/hppBody.h>
#include <hppModel/hppJoint.h>
#include <hppModel/hppSpecificHumanoidRobot.h>

#include <hrp2-dynamics/hrp2OptHumanoidDynamicRobot.h>

#include <hrp2-14/hrp2_14.h>

namespace hpp
{
  namespace wholeBodyStepPlanner
  {
    void makeColPair(ChppColPair &cp);
    void setHRP2OuterLists(ChppHumanoidRobotShPtr i_hrp2);
    ktStatus
    loadHrp2Model
    (ChppHumanoidRobotShPtr& HRP2Device, const std::string& inModel = "hrp-2");

    enum {
      BODY = 0 ,

      RLEG_LINK0 = 1 ,
      RLEG_LINK1 ,
      RLEG_LINK2 ,
      RLEG_LINK3 ,
      RLEG_LINK4 ,
      RLEG_LINK5 ,

      LLEG_LINK0 = 7 ,
      LLEG_LINK1 ,
      LLEG_LINK2 ,
      LLEG_LINK3 ,
      LLEG_LINK4 ,
      LLEG_LINK5 ,

      CHEST_LINK0 = 13 ,
      CHEST_LINK1 ,

      HEAD_LINK0 = 15 ,
      HEAD_LINK1 ,

      RARM_LINK0 = 17 ,
      RARM_LINK1 ,
      RARM_LINK2 ,
      RARM_LINK3 ,
      RARM_LINK4 ,
      RARM_LINK5 ,
      RARM_LINK6 ,

      RHAND_LINK0 =	24 ,
      RHAND_LINK1 ,
      RHAND_LINK2 ,
      RHAND_LINK3 ,
      RHAND_LINK4 ,

      LARM_LINK0 = 29 ,
      LARM_LINK1 ,
      LARM_LINK2 ,
      LARM_LINK3 ,
      LARM_LINK4 ,
      LARM_LINK5 ,
      LARM_LINK6 ,

      LHAND_LINK0 =	36 ,
      LHAND_LINK1 ,
      LHAND_LINK2 ,
      LHAND_LINK3 ,
      LHAND_LINK4 ,

      NO_LINK = 41

    } ;

    void makeColPair (ChppColPair &cp)
    {
      // BODY:  +ARM2  +LEG2
      cp.addColPairRange (BODY, RARM_LINK2, RARM_LINK6);
      cp.addColPairRange (BODY, LARM_LINK2, LARM_LINK6);
      cp.addColPairRange (BODY, RHAND_LINK0, RHAND_LINK4);
      cp.addColPairRange (BODY, LHAND_LINK0, LHAND_LINK4);
      cp.addColPairRange (BODY, RLEG_LINK2, RLEG_LINK5);
      cp.addColPairRange (BODY, LLEG_LINK2, LLEG_LINK5);
      cp.addColPairRange (CHEST_LINK0, RARM_LINK3, RARM_LINK6);
      // BODY + CHEST_JOINT1
      cp.addColPair(BODY, CHEST_LINK1);

      // CHEST0  +ARM3  +LEG3
      cp.addColPairRange (CHEST_LINK0, RARM_LINK3, RARM_LINK6);
      cp.addColPairRange (CHEST_LINK0, LARM_LINK3, LARM_LINK6);
      cp.addColPairRange (CHEST_LINK0, RHAND_LINK0, RHAND_LINK4);
      cp.addColPairRange (CHEST_LINK0, LHAND_LINK0, LHAND_LINK4);
      cp.addColPairRange (CHEST_LINK0, RLEG_LINK3, RLEG_LINK5);
      cp.addColPairRange (CHEST_LINK0, LLEG_LINK3, LLEG_LINK5);

      // CHEST1  +ARM2  +LEG3
      cp.addColPairRange (CHEST_LINK1, RARM_LINK2, RARM_LINK6);
      cp.addColPairRange (CHEST_LINK1, LARM_LINK2, LARM_LINK6);
      cp.addColPairRange (CHEST_LINK1, RHAND_LINK0, RHAND_LINK4);
      cp.addColPairRange (CHEST_LINK1, LHAND_LINK0, LHAND_LINK4);
      cp.addColPairRange (CHEST_LINK1, RLEG_LINK3, RLEG_LINK5);
      cp.addColPairRange (CHEST_LINK1, LLEG_LINK3, LLEG_LINK5);

      // HEAD0  +ARM4
      cp.addColPairRange (HEAD_LINK0, RARM_LINK4, RARM_LINK6);
      cp.addColPairRange (HEAD_LINK0, LARM_LINK4, LARM_LINK6);
      cp.addColPairRange (HEAD_LINK0, RHAND_LINK0, RHAND_LINK4);
      cp.addColPairRange (HEAD_LINK0, LHAND_LINK0, LHAND_LINK4);

      // HEAD1  +ARM4
      cp.addColPairRange (HEAD_LINK1, RARM_LINK4, RARM_LINK6);
      cp.addColPairRange (HEAD_LINK1, LARM_LINK4, LARM_LINK6);
      cp.addColPairRange (HEAD_LINK1, RHAND_LINK0, RHAND_LINK4);
      cp.addColPairRange (HEAD_LINK1, LHAND_LINK0, LHAND_LINK4);

      // LEGS
      //        Same side leg       other side          others
      // LEG0    +LINK4             LINK0, LINK2-5      +ARM3
      // LEG1    Not exposed to outside. no collision check
      // LEG2     none              +LINK2              +ARM2 (same) +ARM3 (other)
      // LEG3    LINK5*             LINK0, LINK2-5      +ARM2 (same) +ARM3 (other)
      //          * can go very close.
      // LEG4     none              +LEG2               +ARM2 (same) +ARM3 (other)
      // LEG5     none              +LEG2               +ARM2 (same) +ARM3 (other)


      // Right legs
      // LEG0
      cp.addColPairRange (RLEG_LINK0, RLEG_LINK4, RLEG_LINK5);
      cp.addColPair(RLEG_LINK0, LLEG_LINK0);
      cp.addColPairRange (RLEG_LINK0, LLEG_LINK2, LLEG_LINK5);
      cp.addColPairRange (RLEG_LINK0, RARM_LINK3, RARM_LINK6);
      cp.addColPairRange (RLEG_LINK0, LARM_LINK3, LARM_LINK6);
      cp.addColPairRange (RLEG_LINK0, RHAND_LINK0, RHAND_LINK4);
      cp.addColPairRange (RLEG_LINK0, LHAND_LINK0, LHAND_LINK4);

      // LEG2
      cp.addColPairRange (RLEG_LINK2, LLEG_LINK2, LLEG_LINK5);

      // LEG3
      // pair leg3 and leg5 excluded.
      // addColPair(RLEG_LINK3, RLEG_LINK5, cp);
      cp.addColPair(RLEG_LINK3, LLEG_LINK0);
      cp.addColPairRange (RLEG_LINK3, LLEG_LINK2, LLEG_LINK3);
      // leg4 no need
      cp.addColPair(RLEG_LINK3, LLEG_LINK5);

      // LEG4 no need.
      // addColPair(RLEG_LINK4, LLEG_LINK0, cp);

      // LEG5
      cp.addColPair(RLEG_LINK5, LLEG_LINK0);
      // addColPairRange (RLEG_LINK5, LLEG_LINK2, LLEG_LINK5, cp);
      cp.addColPairRange (RLEG_LINK5, LLEG_LINK2, LLEG_LINK3);
      cp.addColPair(RLEG_LINK5, LLEG_LINK5);

      // for arm/hands
      for(unsigned int j=RLEG_LINK2; j<=RLEG_LINK5; j++){
	cp.addColPairRange (j, RARM_LINK2, RARM_LINK6);
	cp.addColPairRange (j, LARM_LINK3, LARM_LINK6);
	cp.addColPairRange (j, RHAND_LINK0, RHAND_LINK4);
	cp.addColPairRange (j, LHAND_LINK0, LHAND_LINK4);
      }


      // Left legs
      // LEG0
      cp.addColPairRange (LLEG_LINK0, LLEG_LINK4, LLEG_LINK5);
      cp.addColPair(LLEG_LINK0, RLEG_LINK0);
      cp.addColPairRange (LLEG_LINK0, RLEG_LINK2, RLEG_LINK5);
      cp.addColPairRange (LLEG_LINK0, RARM_LINK3, RARM_LINK6);
      cp.addColPairRange (LLEG_LINK0, LARM_LINK3, LARM_LINK6);
      cp.addColPairRange (LLEG_LINK0, RHAND_LINK0, RHAND_LINK4);
      cp.addColPairRange (LLEG_LINK0, LHAND_LINK0, LHAND_LINK4);

      // LEG2
      cp.addColPairRange (LLEG_LINK2, RLEG_LINK2, RLEG_LINK5);

      // LEG3
      // pair leg3 and leg5 excluded.
      // addColPair(LLEG_LINK3, LLEG_LINK5, cp);
      cp.addColPair(LLEG_LINK3, RLEG_LINK0);
      // addColPairRange (LLEG_LINK3, RLEG_LINK2, RLEG_LINK5, cp);
      cp.addColPairRange (LLEG_LINK3, RLEG_LINK2, RLEG_LINK3);
      cp.addColPair(LLEG_LINK3, RLEG_LINK5);

      // LEG4 no need.
      // addColPair(LLEG_LINK4, RLEG_LINK0, cp);

      // LEG5 leg4 exluded.
      cp.addColPair(LLEG_LINK5, RLEG_LINK0);
      // addColPairRange (LLEG_LINK5, RLEG_LINK2, RLEG_LINK5, cp);
      cp.addColPairRange (LLEG_LINK5, RLEG_LINK2, RLEG_LINK3);
      cp.addColPair(LLEG_LINK5, RLEG_LINK5);

      // for arm/hands
      for(unsigned int j=LLEG_LINK2; j<=LLEG_LINK5; j++){
	cp.addColPairRange (j, RARM_LINK2, RARM_LINK6);
	cp.addColPairRange (j, LARM_LINK2, LARM_LINK6);
	cp.addColPairRange (j, RHAND_LINK0, RHAND_LINK4);
	cp.addColPairRange (j, LHAND_LINK0, LHAND_LINK4);
      }

      // ARM/HANDS
      //        Same side arm/hand     other side arm/hand
      // ARM0      +ARM4                  +ARM4
      // ARM1      none
      // ARM2      none                   +ARM3
      // ARM3      none                   +ARM3
      // ARM4      none                   +ARM2
      // ARM5      none                   +ARM2
      // ARM6      none                   +ARM2
      // HANDS     none                   +ARM2

      // Right arm/hands

      // LINK0
      cp.addColPairRange (RARM_LINK0, RARM_LINK4, RARM_LINK6);
      cp.addColPairRange (RARM_LINK0, RHAND_LINK0, RHAND_LINK4);
      cp.addColPairRange (RARM_LINK0, LARM_LINK4, LARM_LINK6);
      cp.addColPairRange (RARM_LINK0, LHAND_LINK0, LHAND_LINK4);

      // LINK1
      cp.addColPairRange (RARM_LINK1, LARM_LINK3, LARM_LINK6);
      cp.addColPairRange (RARM_LINK1, LHAND_LINK0, LHAND_LINK4);

      // LINK2
      cp.addColPairRange (RARM_LINK2, LARM_LINK3, LARM_LINK6);
      cp.addColPairRange (RARM_LINK2, LHAND_LINK0, LHAND_LINK4);

      // LINK3 - LINK6, HAND
      for(unsigned int j=RARM_LINK3; j<=RARM_LINK6; j++){
	cp.addColPairRange (j, LARM_LINK2, LARM_LINK6);
	cp.addColPairRange (j, LHAND_LINK0, LHAND_LINK4);
      }
      for(unsigned int j=RHAND_LINK0; j<=RHAND_LINK4; j++){
	cp.addColPairRange (j, LARM_LINK2, LARM_LINK6);
	cp.addColPairRange (j, LHAND_LINK0, LHAND_LINK4);
      }

      // Left arm/hands

      // LINK0
      cp.addColPairRange (LARM_LINK0, LARM_LINK4, LARM_LINK6);
      cp.addColPairRange (LARM_LINK0, LHAND_LINK0, LHAND_LINK4);
      cp.addColPairRange (LARM_LINK0, RARM_LINK4, RARM_LINK6);
      cp.addColPairRange (LARM_LINK0, RHAND_LINK0, RHAND_LINK4);

      // LINK1
      cp.addColPairRange (LARM_LINK1, RARM_LINK3, RARM_LINK6);
      cp.addColPairRange (LARM_LINK1, RHAND_LINK0, RHAND_LINK4);

      // LINK2
      cp.addColPairRange (LARM_LINK2, RARM_LINK3, RARM_LINK6);
      cp.addColPairRange (LARM_LINK2, RHAND_LINK0, RHAND_LINK4);

      // LINK3 - LINK6, HAND
      for(unsigned int j=LARM_LINK3; j<=LARM_LINK6; j++){
	cp.addColPairRange (j, RARM_LINK2, RARM_LINK6);
	cp.addColPairRange (j, RHAND_LINK0, RHAND_LINK4);
      }
      for(unsigned int j=LHAND_LINK0; j<=LHAND_LINK4; j++){
	cp.addColPairRange (j, RARM_LINK2, RARM_LINK6);
	cp.addColPairRange (j, RHAND_LINK0, RHAND_LINK4);
      }
    }

    void setHRP2OuterLists (ChppHumanoidRobotShPtr i_hrp2)
    {
      ChppColPair hrpCP;
      makeColPair (hrpCP);

      std::vector<CkwsJointShPtr> jv;
      i_hrp2->getJointVector (jv);

      // adding outer objects
      for(unsigned int iJoint = 0; iJoint < jv.size (); ++iJoint)
	{
	  ChppBodyShPtr hppBody;
	  hppBody = KIT_DYNAMIC_PTR_CAST(ChppBody, jv[iJoint]->attachedBody());

	  std::vector<CkcdObjectShPtr> mergedList;

	  for (unsigned int jJoint=0; jJoint < jv.size (); ++jJoint)
	    {
	      ChppBodyShPtr hppOtherBody =
		KIT_DYNAMIC_PTR_CAST(ChppBody, jv[jJoint]->attachedBody());

	      if (jJoint != iJoint && hrpCP.existPairNarrow(iJoint, jJoint) )
		{
		  std::vector<CkcdObjectShPtr> clist;
		  clist = hppOtherBody->innerObjects();
		  for (unsigned int k = 0; k < clist.size (); ++k)
		    {
		      CkppKCDPolyhedronShPtr kppPolyhedron = KIT_DYNAMIC_PTR_CAST(CkppKCDPolyhedron, clist[k]);
		      mergedList.push_back(kppPolyhedron);
		    }
		}
	    }

	  // add to outerObject.
	  for (std::vector<CkcdObjectShPtr>::iterator it = mergedList.begin();
	       it < mergedList.end ();
	       ++it)
	    hppBody->addOuterObject(*it);
	}
    }

    ktStatus
    loadHrp2Model
    (ChppHumanoidRobotShPtr& HRP2Device, const std::string& inModel)
    {
      ChppHumanoidRobotShPtr humanoid;
      Robotbuilder::Robotbuilder robotBuilder;
      humanoid = robotBuilder.makeRobot();
      Chrp2OptHumanoidDynamicRobot *hrp2;
      hrp2 = dynamic_cast<Chrp2OptHumanoidDynamicRobot *>(humanoid.get());
      hrp2->isKineoOrder(true);
      HRP2Device = humanoid;
      if (!HRP2Device)
	{
	  std::cout << "failed to build Kineo HRP2 Model" << std::endl;
	  return KD_ERROR;
	}

      //
      // set HRP2 joints in invisible mode by default
      //
      HRP2Device->isVisible (false);

      // set Collision Check Pairs
      setHRP2OuterLists (humanoid);

      //
      // set HRP2 in a default configuration (HALFSITTING) ;
      //
      CkwsConfig halfSittingConfig (HRP2Device);
      double dInitPos[] = HALFSITTINGPOSITION_RAD_KINEO;
      std::vector<double>  halfSittingVector (dInitPos, dInitPos + sizeof(dInitPos) / sizeof(double));
      halfSittingConfig.setDofValues (halfSittingVector);
      HRP2Device->hppSetCurrentConfig (halfSittingConfig);

      return KD_OK;
    }
  } // end of namespace wholebody-optimizer.
} // end of namespace hpp.
