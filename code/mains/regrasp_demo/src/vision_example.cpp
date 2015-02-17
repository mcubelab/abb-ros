

#include <ros/ros.h>
#include <robot_comm/robot_comm.h>
#include <hand_comm/hand_comm.h>
#include <matVec/matVec.h>
#include <regrasp_comm/regrasp_comm.h>
#include <handRec_comm/handRec_comm.h>


// Speed of the robot
#define MOVE_TCP 100.0   // Slow speed mm / s
#define MOVE_ORI 35.0   // degrees / s

#define HAND_FORCE 0.90
#define HAND_SPEED 0.90

#define HAND_OPEN_ANGLE 0.0   // Max opening angle
#define HAND_CLOSE_ANGLE 100.0  // Close Angle

static const double HOME_J[NUM_JOINTS] = {0.0, 0.0, 0.0, 0.0, 90.0, 0.0};

// Location to grasp object from user
static const HomogTransf GRASP_POSE(Quaternion("0.0 0.7071 0.7071 0.0").getRotMat(), Vec("600 200 400",3));

// Location to drop object on ground
static const HomogTransf DROP_POSE(Quaternion("0.0 0.7071 0.7071 0.0").getRotMat(), Vec("600 200 200",3));

// Guess of where the object is in the hand
static const HomogTransf OBJECT_GUESS(Quaternion("-0.3827 0.0 0.0 0.9239").getRotMat(), Vec("0 40 0",3));

// Guess of where object is in 3D space (TODO: HACK. We should at least make this with respect to the robot hand, not the world frame)
//static const HomogTransf OBJECT_GUESS(Quaternion("-0.0009 0.9916 -0.1294 -0.0055").getRotMat(), Vec("0.4812291  0.3341413  0.4637221",3));



int main(int argc, char** argv)
{
  ros::init(argc, argv, "vision_example");
  ros::NodeHandle node;

  // Initialize Services
  RobotComm robot(&node);
  HandComm hand(&node);
  HandRecComm handRec(&node);

  // Initial Setup
  hand.SetForce(HAND_FORCE);
  hand.SetSpeed(HAND_SPEED);
  robot.SetDefaults();
  robot.SetSpeed(MOVE_TCP, MOVE_ORI);

  // Open Hand and move to home position
  robot.SetJoints(HOME_J);
  hand.Calibrate();
  hand.SetAngle(HAND_OPEN_ANGLE);
  hand.WaitRest(0.25);

  handRec.SetObject(RecObj::BIG_TRIANGLE);
  handRec.SetPrefOrient(Quaternion("1.0 0.0 0.0 0.0"));

  do
  {
    // Move hand to position to grasp object
    robot.SetCartesian(GRASP_POSE);

    // Close fingers once a finger has been pushed
    double mot_ang;
    Vec ini_fing_angs(NUM_FINGERS);
    Vec cur_fing_angs(NUM_FINGERS);

    hand.GetAngles(mot_ang, ini_fing_angs);

    std::cout << "Please push one of my fingers to grasp an object!" << std::endl;


    ros::Rate loop_rate(20);
    while(ros::ok())
    {
      // Get the current finger angles, and if they're significantly 
      // different, a finger has been moved!
      hand.GetAngles(mot_ang, cur_fing_angs);
      if ((ini_fing_angs - cur_fing_angs).norm() > 2.0)
        break;
      loop_rate.sleep();
    }

    // Close the hand, and wait until we're done
    hand.SetAngle(HAND_CLOSE_ANGLE);
    //  hand.SetAngle(30.0);
    hand.WaitRest(0.25);

    // Compute object pose
    std::cout << "About to move. Waiting for a second so you get out of the way!" << std::endl;
    ros::Duration(1.0).sleep();

    int obj = -1;
    HomogTransf objPose;
    //bool ret = true;

    //objRec.SetGuess(OBJECT_GUESS);
    //bool ret = util.getObjMulti(obj, objPose);
    //  bool ret = util.getObjMultiWithGuess(obj, objPose, OBJECT_GUESS);
    //bool ret = util.getObjMulti(obj, objPose);
    bool ret = handRec.GetPose(obj, objPose);

    std::cout << "getPose returned: " << ret << std::endl;
    std::cout << "Object Pose: " << objPose << std::endl;
    std::cout << "Quaternion: " << objPose.getQuaternion() << std::endl;

    // Drop object
    robot.SetSpeed(MOVE_TCP, MOVE_ORI);
    robot.SetCartesian(DROP_POSE);
    hand.SetAngle(HAND_OPEN_ANGLE);
    hand.WaitRest(0.25);

  }while(ros::ok());

  return 0;
}
