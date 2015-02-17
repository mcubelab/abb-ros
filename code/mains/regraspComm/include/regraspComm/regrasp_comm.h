#ifndef REGRASP_COMM_H
#define REGRASP_COMM_H

#include <ros/ros.h>
#include <matVec/matVec.h>
#include <geometry_msgs/Pose.h>
#include <robot_comm/robot_comm.h>
#include <hand_comm/hand_comm.h>
#include <vector>

#include "regraspComm/SetHand.h"
#include "regraspComm/ThrowToPalm.h"
#include "regraspComm/PushInEnveloping.h"
#include "regraspComm/Pick.h"
#include "regraspComm/Place.h"
#include "regraspComm/Vibrate.h"
#include "regraspComm/RollOnGround.h"
#include "regraspComm/PushInFingers.h"
#include "regraspComm/ThrowAndFlip.h"
#include "regraspComm/RollToFingertip.h"
#include "regraspComm/RollToPalm.h"
#include "regraspComm/RollToGround.h"
#include "regraspComm/DroopInFingers.h"
#include "regraspComm/ThrowToFingertip.h"
#include "regraspComm/StandToLie.h"
#include "regraspComm/LieToStand.h"
#include "regraspComm/Move.h"
#include "regraspComm/LongEdgeDroop.h"
#include "regraspComm/ShortEdgeDroop.h"
#include "regraspComm/Topple.h"
#include "regraspComm/Regrasp.h"
#include "regraspComm/regrasp_Execute.h"
#include "regraspComm/Droop.h"

using namespace std;

  typedef enum Regrasps {
    ERROR,
    PICK,
    PLACE,
    MOVE,
    DROOP_IN_FINGERS,
    PUSH_IN_ENVELOPING,
    PUSH_IN_FINGERS,
    ROLL_ON_GROUND,
    ROLL_TO_FINGERTIP,
    ROLL_TO_GROUND,
    ROLL_TO_PALM,
    SQUEEZE,
    VIBRATE,
    THROW_AND_FLIP,
    THROW_TO_PALM,
    THROW_TO_FINGERTIP,
    LIE_TO_STAND,
    STAND_TO_LIE,
    LONG_EDGE_DROOP,
    SHORT_EDGE_DROOP,
    TOPPLE,
    DROOP
  };

class RegraspComm
{
 public: 
  RegraspComm();
  RegraspComm(ros::NodeHandle * np);
  ~RegraspComm();

  ros::NodeHandle *node;
  void subscribe();
  void shutdown();
  bool Execute(vector<regraspComm::Regrasp> regrasps);
  bool Execute(vector<int> regrasps);
  bool Execute(regraspComm::Regrasp regrasp);
  bool Execute(int regrasp);
  regraspComm::Regrasp getDefaults(int action);

 private:
  ros::ServiceClient handle_regrasp_Execute;
  regraspComm::regrasp_Execute regrasp_Execute_srv;
  
};


#endif
