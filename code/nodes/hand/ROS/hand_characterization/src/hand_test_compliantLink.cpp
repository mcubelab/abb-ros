//map between motor position/angle motor encoder number and finger angle is nonlinear but almost linear. map from motor position tofinger encoder. then map from finger encoder position to finger angle on the arduino.
//given an encoder value as input, return angle value. given an angle value, return encoder value.

#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <unistd.h>
#include <sys/stat.h>

#include <ros/ros.h>
#include <matVec/matVec.h>
#include <hand_comm/hand_comm.h>
#include <matlab_comm/matlab_comm.h>
#include <robot_comm/robot_comm.h>

//ATI Force Sensor
#include <geometry_msgs/WrenchStamped.h>


//Experiment
#define N_DATA 100
#define N_AVERAGES 10
#define RESULTS_FOLDER_NAME "/home/simplehands/Documents/hands/code/nodes/hand/ROS/hand_characterization/results"

//Hand
#define HAND_FORCE 0.8
#define HAND_SPEED 0.5
//#define COMPRESSION_STROKE 12  //degrees
//#define EXTENSION_STROKE   35  //degrees
#define COMPRESSION_STROKE 17  //degrees
#define EXTENSION_STROKE   38  //degrees

//Robot configuration
#define TCP_FAST 100.0
#define ORI_FAST 25.0
#define TCP_SLOW 5.0
#define ORI_SLOW 2.0
#define ZONE 0

double netFT_force[3];
double netFT_torque[3];

pthread_mutex_t netFTMutex;

static void forceDataCallback( const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
  pthread_mutex_lock(&netFTMutex);
  netFT_force[0] = msg->wrench.force.x;
  netFT_force[1] = msg->wrench.force.y;
  netFT_force[2] = msg->wrench.force.z;
  netFT_torque[0] = msg->wrench.torque.x;
  netFT_torque[1] = msg->wrench.torque.y;
  netFT_torque[2] = msg->wrench.torque.z;
  pthread_mutex_unlock(&netFTMutex);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hand_test_compliantLink");
  ros::NodeHandle node;
  
  //////////////////////
  // ROS initialization
  //////////////////////
  pthread_mutex_init(&netFTMutex, NULL);
  ROS_INFO("HAND_TEST_COMPLIANT_LINK");
  ROS_INFO("Routine to characterize the stiffness profile of the transmission.");
  ROS_INFO("The robot is going to move to the vicinity of the force sensor!!!");
  HandComm hand(&node);
  RobotComm robot(&node);
  ros::Subscriber forceDataSub = node.subscribe("/netft_data", 100, forceDataCallback );
  ros::AsyncSpinner spinner(2); 
  spinner.start();

  ROS_INFO("Waiting for robot...");
  while(!robot.Ping()) ;
  ROS_INFO("Waiting for hand...");
  while(!hand.Ping()) ;

  /////////////////////////
  // Set up experiment
  /////////////////////////

  // Open file where to save data
  char fileName[1024];
  sprintf(fileName, "stiffnessProfile.txt");
  FILE* pFile = fopen(fileName, "w");
  if (!pFile)
    { 
      ROS_INFO("Could not open the file to save results.");
      exit(-1);
    }

  // Split data points between compression and extension
  double combinedStroke = COMPRESSION_STROKE + EXTENSION_STROKE;
  double degPerStep = combinedStroke/(double)(N_DATA);
  // In compression
  int stepsComp = round((double)COMPRESSION_STROKE/degPerStep);
  Vec anglesComp(stepsComp + 1);
  Mat forcesComp(stepsComp + 1,3);
  for (int i=0; i<=stepsComp; i++)
    anglesComp[i] = -degPerStep*i;
  //In extension
  int stepsExt = N_DATA - stepsComp;
  Vec anglesExt(stepsExt + 1);
  Mat forcesExt(stepsExt + 1,3);
  for (int i=0; i<=stepsExt; i++)
    anglesExt[i] = degPerStep*i;
  

  ///////////////////////////////////
  // Capture data in compression
  ///////////////////////////////////
  
  // Set up robot
  robot.SetComm(BLOCKING);
  robot.SetZone(ZONE);
  robot.SetSpeed(TCP_FAST, ORI_FAST);
  robot.SetTool(0.0, 0.0, 101.31, 1.0, 0.0, 0.0, 0.0);
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, 90.0, 0.0);  //Go to a safe start location in joint coordinates
  
  // Approach force sensor and calibrate hand
  robot.SetJoints(-90.0, 0.0, 0.0, 0.0, 90.0, 0.0);  
  //HomogTransf iniPose = HomogTransf(Quaternion("0.0 1.0 0.0 0.0").getRotMat(),Vec("105.5 780 178.5",3));
  HomogTransf iniPose = HomogTransf(Quaternion("0.0 1.0 0.0 0.0").getRotMat(),Vec("105.5 747 182",3));
  HomogTransf safePose = HomogTransf(iniPose.getRotation(), iniPose.getTranslation() + Vec("0.0 0.0 150.0",3));
  HomogTransf approachPose = HomogTransf(iniPose.getRotation(), iniPose.getTranslation() + Vec("0.0 0.0 25.0",3));
  robot.SetCartesian(safePose);
    
  // Set up hand
  hand.Calibrate(0);
  hand.SetForce(HAND_FORCE);
  hand.SetSpeed(HAND_SPEED);
  hand.SetAngle(0.0);
  hand.WaitRest(0.500);
   
  // Go down to reach contact
  robot.SetCartesian(approachPose);
  robot.SetSpeed(TCP_SLOW, ORI_SLOW);
  robot.SetCartesian(iniPose);

  // Engage compliance and read force sensor 
  ROS_INFO("Mapping stiffness in compression...");
  HomogTransf iniToFingerAxis = HomogTransf(Quaternion("1.0 0.0 0.0 0.0").getRotMat(), Vec("0.0 -44.45 -13.359",3));
  HomogTransf fingerRot;
  Vec rotAxis = Vec("-1.0 0.0 0.0", 3);

  // Direct direction
  int i,j,k;
  double motAng;
  double fingerAngles[NUM_FINGERS];
  double force[3];
  double torque[3];
  RotMat rot;
    
  // hand.SetAngle(55-(k*2*5)); //increment finger angles 5k degrees beyond 55
  //if(k==0)
  //   {
	 hand.SetAngle(0); // for the first loop, we will gather data for the most open finger positions
	 //   }
	 //   if(k==1)
	 // {
	 //	 hand.SetAngle(80); // for the second loop, we will gather data for a more closed finger position
	 //}
     for (i=0; i<anglesComp.nn; i++)
    {

      //      rot.setAxisAngle(rotAxis, -(anglesComp[i]+(k*5))*PI/180.0); //rotate hand 5k degrees beyond ini pose         
      rot.setAxisAngle(rotAxis, -(anglesComp[i]+0)*PI/180.0); //rotate hand 5k degrees beyond ini pose         
      fingerRot = HomogTransf(rot,Vec("0.0 0.0 0.0",3));

      robot.SetCartesian(iniPose*iniToFingerAxis.inv()*fingerRot*iniToFingerAxis);
      usleep(500000);
      hand.GetAngles(motAng, fingerAngles);
      fprintf(pFile,"%d,%lf,%lf,%lf,%lf,",k,motAng,fingerAngles[0],fingerAngles[1],fingerAngles[2]); 
      force[0] = 0;
      force[1] = 0;
      force[2] = 0;
      torque[0] = 0;
      torque[1] = 0;
      torque[2] = 0;
      for (j=0; j<N_AVERAGES; j++)
      {
        pthread_mutex_lock(&netFTMutex);
        force[0] += netFT_force[0];
        force[1] += netFT_force[1];
        force[2] += netFT_force[2];
        torque[0] += netFT_torque[0];
        torque[1] += netFT_torque[1];
        torque[2] += netFT_torque[2];
        pthread_mutex_unlock(&netFTMutex);
        usleep(50000);
       }      
      fprintf(pFile,"%lf,%lf,%lf,%lf,%lf,%lf\n",force[0],
						force[1],
						force[2],
						torque[0],
						torque[1],
						torque[2]); 
      ROS_INFO("%d out of %d: %5.1lf %5.2lf %2.5lf %5.2lf %5.2lf %5.2lf %5.2lf",
						i+1,
						anglesComp.nn*2,
					    	fingerAngles[1],
						force[0],
						force[1],
						force[2],
						torque[0],
						torque[1],
						torque[2]);
    }
  // Reverse direction
  for (i=anglesComp.nn-1; i>=0; i--)
    //      hand.SetAngle(55+(k*5)); //increment finger angles 5k degrees beyond 55          
    {
      rot.setAxisAngle(rotAxis, -(anglesComp[i]+(0))*PI/180.0);

      fingerRot = HomogTransf(rot,Vec("0.0 0.0 0.0",3));
      //start loop
      //run motor a little
      //move fingers to touch force sensor
      robot.SetCartesian(iniPose*iniToFingerAxis.inv()*fingerRot*iniToFingerAxis);
      usleep(500000);
      hand.GetAngles(motAng, fingerAngles);
      //get encoder values instead
      fprintf(pFile,"%d,%lf,%lf,%lf,%lf,",k,motAng,fingerAngles[0],fingerAngles[1],fingerAngles[2]); 
      force[0] = 0;
      force[1] = 0;
      force[2] = 0;
      torque[0] = 0;
      torque[1] = 0;
      torque[2] = 0;
      for (j=0; j<N_AVERAGES; j++)
      {
        pthread_mutex_lock(&netFTMutex);
        force[0] += netFT_force[0];
        force[1] += netFT_force[1];
        force[2] += netFT_force[2];
        torque[0] += netFT_torque[0];
        torque[1] += netFT_torque[1];
        torque[2] += netFT_torque[2];
        pthread_mutex_unlock(&netFTMutex);
        usleep(50000);
       }      
      fprintf(pFile,"%lf,%lf,%lf,%lf,%lf,%lf\n",force[0],
						force[1],
						force[2],
						torque[0],
						torque[1],
						torque[2]); 
      ROS_INFO("%d out of %d: %5.1lf %5.2lf %2.5lf %5.2lf %5.2lf %5.2lf %5.2lf",
						anglesComp.nn*2 -i,
						anglesComp.nn*2,
					    	fingerAngles[1],
						force[0],
						force[1],
						force[2],
						torque[0],
						torque[1],
						torque[2]);
    }
    
  //end loop
  robot.SetCartesian(approachPose);
  robot.SetSpeed(TCP_FAST, ORI_FAST);
  robot.SetCartesian(safePose);

  
  //////////////////////////////
  // Transition
  /////////////////////////////
  // Intermediate configuration to simplify the transition between both experiments
  robot.SetCartesian(HomogTransf(Quaternion("0.0 0.7071 0.7071 0.0").getRotMat(),safePose.getTranslation()));
  robot.SetCartesian(HomogTransf(Quaternion("0.0 0.0 1.0 0.0").getRotMat(),safePose.getTranslation()));


  ////////////////////////////////
  // Capture data in extension
  ////////////////////////////////
  
  // Approach force sensor
  //HomogTransf iniPose = HomogTransf(Quaternion("0.0 1.0 0.0 0.0").getRotMat(),Vec("105.5 780 178.5",3)); /first ini pose
  iniPose = HomogTransf(Quaternion("0.0, 0.0, -0.7071, 0.7071").getRotMat(),Vec("106.11, 840.0, 107.8",3));
  safePose = HomogTransf(iniPose.getRotation(), iniPose.getTranslation() + Vec("0.0 0.0 150.0",3));
  approachPose = HomogTransf(iniPose.getRotation(), iniPose.getTranslation() + Vec("0.0 0.0 25.0",3));
  robot.SetCartesian(safePose);
  hand.SetAngle(0.0);
  hand.WaitRest(0.50);
  robot.SetCartesian(approachPose);
 
  //Go down to reach contact
  robot.SetSpeed(TCP_SLOW, ORI_SLOW);
  robot.SetCartesian(iniPose);

  //Engage compliance and read force sensor 
  ROS_INFO("Mapping stiffness in tension...");
  //for(int k=0;k<4;k++)
  //{
  //	  hand.SetAngle(55-(k*5)); //increment finger angles 5k degrees beyond 55
  for (i=0; i<anglesExt.nn; i++)
    {
      RotMat rot;
	  rot.setAxisAngle(rotAxis, (anglesExt[i])*PI/180.0);          
      
      fingerRot = HomogTransf(rot,Vec("0.0 0.0 0.0",3));
      robot.SetCartesian(iniPose*iniToFingerAxis*fingerRot*iniToFingerAxis.inv());
      usleep(500000);
      hand.GetAngles(motAng, fingerAngles);
      fprintf(pFile,"%d, %lf,%lf,%lf,%lf,",k,motAng,fingerAngles[0],fingerAngles[1],fingerAngles[2]); 
      force[0] = 0;
      force[1] = 0;
      force[2] = 0;
      torque[0] = 0;
      torque[1] = 0;
      torque[2] = 0;
      for (j=0; j<N_AVERAGES; j++)
      {
        pthread_mutex_lock(&netFTMutex);
        force[0] += netFT_force[0];
        force[1] += netFT_force[1];
        force[2] += netFT_force[2];
        torque[0] += netFT_torque[0];
        torque[1] += netFT_torque[1];
        torque[2] += netFT_torque[2];
        pthread_mutex_unlock(&netFTMutex);
        usleep(50000);
       }      
      fprintf(pFile,"%lf,%lf,%lf,%lf,%lf,%lf\n",force[0],
						force[1],
						force[2],
						torque[0],
						torque[1],
						torque[2]); 
      ROS_INFO("%d out of %d: %5.1lf %5.2lf %2.5lf %5.2lf %5.2lf %5.2lf %5.2lf",
						i+1,
						anglesExt.nn*2,
					    	fingerAngles[1],
						force[0],
						force[1],
						force[2],
						torque[0],
						torque[1],
						torque[2]);
    }

  //Coming back
  //     hand.SetAngle(55+(k*5)); //increment finger angles 5k degrees beyond 55
  for (i=anglesExt.nn-1; i>=0; i--)
    {
      RotMat rot;
      rot.setAxisAngle(rotAxis, (anglesExt[i])*PI/180.0);          

      fingerRot = HomogTransf(rot,Vec("0.0 0.0 0.0",3));
      robot.SetCartesian(iniPose*iniToFingerAxis*fingerRot*iniToFingerAxis.inv());
      usleep(500000);
      hand.GetAngles(motAng, fingerAngles);
      fprintf(pFile,"%d, %lf,%lf,%lf,%lf,",k,motAng,fingerAngles[0],fingerAngles[1],fingerAngles[2]); 
      force[0] = 0;
      force[1] = 0;
      force[2] = 0;
      torque[0] = 0;
      torque[1] = 0;
      torque[2] = 0;
      for (j=0; j<N_AVERAGES; j++)
      {
        pthread_mutex_lock(&netFTMutex);
        force[0] += netFT_force[0];
        force[1] += netFT_force[1];
        force[2] += netFT_force[2];
        torque[0] += netFT_torque[0];
        torque[1] += netFT_torque[1];
        torque[2] += netFT_torque[2];
        pthread_mutex_unlock(&netFTMutex);
        usleep(50000);
       }      
      fprintf(pFile,"%lf,%lf,%lf,%lf,%lf,%lf\n",force[0],
						force[1],
						force[2],
						torque[0],
						torque[1],
						torque[2]); 
      ROS_INFO("%d out of %d: %5.1lf %5.2lf %2.5lf %5.2lf %5.2lf %5.2lf %5.2lf",
						anglesExt.nn*2 - i,
						anglesExt.nn*2,
					    	fingerAngles[1],
						force[0],
						force[1],
						force[2],
						torque[0],
						torque[1],
						torque[2]);
    }
	
  robot.SetCartesian(approachPose);
  robot.SetSpeed(TCP_FAST, ORI_FAST);
  robot.SetCartesian(safePose);
   
  ////////////////////////////////
  // End of experiment
  ////////////////////////////////
  fclose(pFile);
  spinner.stop();
  pthread_mutex_destroy(&netFTMutex);
  hand.SetRest();
  robot.SetJoints(0.0,0.0,0.0,0.0,90.0,0.0); //bring robot back to safe joint configuration

  //Back to default robot configuration
  double defTx,defTy,defTz,defTq0,defTqx,defTqy,defTqz;
  int zone;
  double speedTCP, speedORI;
  node.getParam("/robot/toolX",defTx);
  node.getParam("/robot/toolY",defTy);
  node.getParam("/robot/toolZ",defTz);
  node.getParam("/robot/toolQ0",defTq0);
  node.getParam("/robot/toolQX",defTqx);
  node.getParam("/robot/toolQY",defTqy);
  node.getParam("/robot/toolQZ",defTqz);
  node.getParam("/robot/speedTCP",speedTCP);
  node.getParam("/robot/speedORI",speedORI);
  node.getParam("/robot/zone",zone);
  robot.SetTool(defTx,defTy,defTz,defTq0,defTqx,defTqy,defTqz);
  robot.SetZone(zone);
  robot.SetSpeed(speedTCP, speedORI);

  return 0;
}
