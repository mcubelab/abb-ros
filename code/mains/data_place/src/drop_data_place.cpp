#include <ros/ros.h>
#include <matVec/matVec.h>
#include <grasp_comm/grasp_comm.h>
#include <sense_comm/sense_comm.h>
#include <place_comm/place_comm.h>
#include <util_comm/util_comm.h>
#include <logger_comm/logger_comm.h>
#include <matlab_comm/matlab_comm.h>
#include <hand_comm/hand_comm.h>
#include <cstdlib>

#define VIS_CAL_CNT 20
#define HAND_CAL_CNT 1

#define GRASP_EXPERIMENT_NAME "data_drop_5_30_2012_grasp"
#define PLACE_EXPERIMENT_NAME "data_drop_5_30_2102_drop"

#define MIN_FINGER_SUM 4650

int main(int argc, char** argv)
{
  ros::init(argc, argv, "data_drop");
  ros::NodeHandle node;
  GraspComm grasper(&node);
  SenseComm sensor(&node);  
  UtilComm util(&node);  
  PlaceComm dropper(&node);  
  LoggerComm logger(&node);  
  MatlabComm matlab(&node);  
  HandComm hand(&node);

  srand(time(NULL));

  GM_InputParams grasperInParams;
  GM_OutputParams grasperOutParams;

  grasperInParams.type = GM_TYPE_OPENLOOP;
  grasperInParams.z_lim = 150.0;
  grasperInParams.up = 36.0;
  grasperInParams.down = 36.0;
  grasperInParams.nFlips = 4;
  grasperInParams.oscillationAmplitude = 0.8683;
  grasperInParams.handSpeed = 0.5;

  SM_InputParams sensorInParams;
  SM_OutputParams sensorOutParams;

  sensorInParams.type = SM_TYPE_VISION;

  PM_InputParams dropperInParams;
  PM_OutputParams dropperOutParams;

  dropperInParams.type = PM_TYPE_DROP;

  std::string graspFolder = GRASP_EXPERIMENT_NAME;
  std::string placeFolder = PLACE_EXPERIMENT_NAME;

  util.go_home();
  util.calibrate_hand();
  util.calibrate_vision();

  matlab.sendCommand("DataDrop_Init();");

  int hand_cnt = 0;
  int vis_cnt = 0;
  int i = 1;
  int singulated = 0;
  int dropped = 0;
  double runningProbability = 0;

  char buffer[1024];

  int motor_enc;
  int finger_enc[NUM_FINGERS];
  int sum;

  while (ros::ok())
  {
    if (vis_cnt >= VIS_CAL_CNT)
    {
      util.calibrate_vision();
      vis_cnt = 0;
    }
    if (hand_cnt >= HAND_CAL_CNT)
    {
      util.calibrate_hand(1);
      hand_cnt = 0;
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

    // Get the current position of the 3 fingers
    hand.GetEncoders(motor_enc, &finger_enc[0]);

    sum = 0;
    for (int k=0; k<NUM_FINGERS; k++)
      sum += finger_enc[k];

    if (sum < MIN_FINGER_SUM)
    {
      ROS_INFO("Encoder sum = %d. We are not holding any markers.", sum);
      sprintf(buffer, "#S,99,0,0.0,0,0.0,0.0,0.0,0.0,0.0,%d",
          motor_enc);
      for (int i=0; i<NUM_FINGERS; i++)
        sprintf(buffer,"%s,%d",buffer,finger_enc[i]);
      sprintf(buffer,"%s,no_picture",buffer);
      std::string str = buffer;
      logger.Append(grasperOutParams.logFileName, str);
      logger.Copy(grasperOutParams.logFileName, graspFolder);
    }
    else
    {
      // Now that we have a marker, figure out exactly where it is for 
      //  validation purposes later. 
      //  (Note that we will NOT use these values to place the marker)
      ROS_INFO("Going to camera to validate marker pose...");
      sensorInParams.logFileName = grasperOutParams.logFileName;
      sensorOutParams = sensor.sense(sensorInParams);

      // Save logFile to folder
      logger.Copy(sensorOutParams.logFileName, graspFolder);

      if (sensorOutParams.singulated)
      {
        singulated++;

        ///////////////////////////////////////////
        // Compute best plan from encoder values
        ///////////////////////////////////////////
        int mot_enc;
        Vec encVals(3);
        hand.GetEncoders(mot_enc, encVals);
        matlab.sendVec("encoderVals", encVals);
        matlab.sendCommand("[bestParams, mapParams, probSuccess] = DataDrop_ComputeParams(encoderVals);");
        Vec bestParams = matlab.getVec("bestParams");
        Vec mapParams = matlab.getVec("mapParams");
        double probSuccess = matlab.getValue("probSuccess");

        dropperInParams.angle = bestParams[1];
        dropperInParams.h_dist = bestParams[0];

        ROS_INFO("bestParams: (%f, %f). mapParams: (%f, %f). Success Probability: %f", 
            bestParams[0], bestParams[1], mapParams[0], mapParams[1], probSuccess);

        // Place our marker with our best parameters
        dropperOutParams = dropper.place(dropperInParams);
        if (dropperOutParams.success)
        {
          ROS_INFO("Successfully dropped marker!");
          dropped++;
          util.go_home();  
          util.random_drop();  
        }
        else
        {
          ROS_INFO("Dropping Unsuccessful.");
          util.go_home();  
        }

        // Print out a long list detailing how our data place went
        sprintf(buffer, "#D,%d,%f,%f,%f,%f,%f,%f,%f,%d,%d,%d,%d,%s,%s,%s", 
            dropperOutParams.success, bestParams[0], bestParams[1], 
            mapParams[0], mapParams[1], probSuccess, 
            sensorOutParams.alpha, sensorOutParams.distance,
            mot_enc, (int)encVals[0], (int)encVals[1], (int)encVals[2],
            sensorInParams.logFileName.c_str(), 
            dropperOutParams.picFileName.c_str(), 
            sensorOutParams.picFileName.c_str());
        std::string str = buffer;
        logger.Append(dropperOutParams.logFileName, str);
        logger.Copy(dropperOutParams.logFileName, placeFolder);

        // Compute the running total of predicted probabilities 
        runningProbability = (runningProbability * (singulated - 1) + probSuccess) / singulated;

        // Done, so go home and try again
      }
      else
      {
        util.go_home_from_vision();
        ROS_INFO("num markers: %d", sensorOutParams.nMarkers);
        if (sensorOutParams.nMarkers > 0)
        {
          util.random_drop();
        }
      }
    }

    ROS_INFO("DATA_DROP: (S = %d, N = %d, %2.3f%%)", singulated, i-singulated, (100.0*singulated)/i);
    ROS_INFO("DATA_DROP: (Ppred = %f, Preal= %f)", runningProbability, ((double)dropped)/singulated);

    vis_cnt++;
    hand_cnt++;
    i++;
  }

  return 0;
}
