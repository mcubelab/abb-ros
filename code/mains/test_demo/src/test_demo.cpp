#include <ros/ros.h>
#include <matVec/matVec.h>
#include <grasp_comm/grasp_comm.h>
#include <place_comm/place_comm.h>
#include <util_comm/util_comm.h>
#include <matlab_comm/matlab_comm.h>
#include <hand_comm/hand_comm.h>

#define HAND_CAL_CNT 1
#define MIN_FINGER_SUM 4650

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_demo");
  ros::NodeHandle node;
  GraspComm grasper(&node);
  UtilComm util(&node);  
  PlaceComm placer(&node);  
  MatlabComm matlab(&node);  
  HandComm hand(&node);

  GM_InputParams grasperInParams;
  GM_OutputParams grasperOutParams;

  grasperInParams.type = GM_TYPE_OPENLOOP;
  grasperInParams.z_lim = 150.0;
  grasperInParams.up = 36.0;
  grasperInParams.down = 36.0;
  grasperInParams.nFlips = 4;
  grasperInParams.oscillationAmplitude = 0.8683;
  grasperInParams.handSpeed = 0.5;

  PM_InputParams placerInParams;
  PM_OutputParams placerOutParams;

  placerInParams.type = PM_TYPE_OPENLOOP;
  placerInParams.v1_dist = 70.0;
  placerInParams.v2_dist = 60.0;
  placerInParams.test = true;

  util.go_test_home();
  while (!hand.Ping()) ;
  hand.Calibrate(0);

  //matlab.sendCommand("Singulation_Init();");
  matlab.sendCommand("DataPlace_Init();");

  int hand_cnt = 0;
  int i = 1;
  int singulated = 0;
  int placed = 0;
  double runningProbability = 0;

  int mot_enc;
  Vec encVals(3);

  while (ros::ok())
  {
    if (hand_cnt >= HAND_CAL_CNT)
    {
      while (!hand.Ping()) ;
      hand.Calibrate(1);
    }

    grasperInParams.id = i;
    util.get_random_bin_pos(grasperInParams.x, grasperInParams.y);

    ROS_INFO("Sensing command to grasp node...");
    grasperOutParams = grasper.grasp(grasperInParams);

    if (grasperOutParams.error != GM_ERROR_NONE)
    {
      ROS_INFO("Grasping Error!");
      if (grasperOutParams.error == GM_ERROR_COLLISION)
      {
        ROS_INFO("Collision occured while grasping. Dropping this marker, recalibrating, and continuing...");
        util.drop();
        util.calibrate_hand();
        hand_cnt = 0;
        continue;
      }
      else
      {
        ROS_INFO("Unexpected error detected!");
        break;
      }
    }

    // Use an SVM to check if we have a single marker
    ROS_INFO("Checking if we have a single marker...");
    hand.GetEncoders(mot_enc, encVals);

    //matlab.sendVec("encoderVals", encVals);
    //matlab.sendCommand("singulated = Singulation_Test(encoderVals);");
    int sum = 0;
    for (int k=0; k<NUM_FINGERS; k++)
      sum += encVals[k];

    //if (matlab.getValue("singulated") == 1)
    if (sum >= MIN_FINGER_SUM)
    {
      singulated++;

      ///////////////////////////////////////////
      // Compute best plan from encoder values
      ///////////////////////////////////////////
      //hand.GetEncoders(mot_enc, encVals);
      matlab.sendVec("encoderVals", encVals);
      matlab.sendCommand("[bestParams, mapParams, probSuccess] = DataPlace_ComputeParams(encoderVals);");
      Vec bestParams = matlab.getVec("bestParams");
      Vec mapParams = matlab.getVec("mapParams");
      double probSuccess = matlab.getValue("probSuccess");

      placerInParams.angle = bestParams[1];
      placerInParams.h_dist = bestParams[0];

      ROS_INFO("bestParams: (%f, %f). mapParams: (%f, %f). Success Probability: %f", 
          bestParams[0], bestParams[1], mapParams[0], mapParams[1], probSuccess);

      util.go_test_home();
      util.go_test_action();

      // Place our marker with our best parameters
      placerOutParams = placer.place(placerInParams);
      if (placerOutParams.success)
      {
        ROS_INFO("Successfully placed marker!");
        placed++;
      }
      else
      {
        ROS_INFO("Placing Unsuccessful.");
      }

      // Compute the running total of predicted probabilities 
      runningProbability = (runningProbability * (singulated - 1) + probSuccess) / singulated;

      // Done, so go home and try again
      util.go_home_from_test();  
    }
    /*
    else
    {
      // Get the current position of the 3 fingers
      hand.GetEncoders(mot_enc, encVals);

      int sum = 0;
      for (int k=0; k<NUM_FINGERS; k++)
        sum += encVals[k];

      if (sum >= MIN_FINGER_SUM)
      {
        util.random_drop();
      }
    }
    */

    ROS_INFO("DATA_PLACE: (S = %d, N = %d, %f%%)", singulated, i-singulated, (100.0*singulated)/i);
    ROS_INFO("DATA_PLACE: (Ppred = %f, Preal= %f)", runningProbability, ((double)placed)/singulated);

    hand_cnt++;
    i++;
  }

  return 0;
}
