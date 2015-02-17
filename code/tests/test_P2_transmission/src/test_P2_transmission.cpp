#include <stdio.h>
#include <unistd.h>

#include <ros/ros.h>
#include <matVec/matVec.h>
#include <handP2_comm/handP2_comm.h>

#define N_ITERATIONS 150
#define N_STEPS 50
#define HAND_FORCE 1.0
#define HAND_SPEED 0.5
#define HAND_CLOSE_ANGLE 130.0
#define HAND_OPEN_ANGLE -20.0
#define FILE_NAME "/home/simplehands/Documents/hands/code/tests/test_P2_transmission/P2_test_transmission.txt"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "demo");
  ros::NodeHandle node;

  HandComm hand(&node);

  ROS_INFO("Routine to test the transmission of P2");
  ROS_INFO("For best accuracy, the palm should be horizontal and looking down.");
  
  ROS_INFO("Waiting for hand...");
  while(!hand.Ping()) ;

  int i = 0;
  int j = 0;

  // We start test with a calibrated hand
  hand.Calibrate();
  hand.WaitRest(1.0);
  hand.SetForce(HAND_FORCE);
  hand.SetSpeed(HAND_SPEED);

  // For N_ITER times we go through the process:
  // - Opening and closing the hand 10 times.
  // - Mapping input-output transmission  
  FILE * pFile;
  pFile = fopen(FILE_NAME,"w");
  if(pFile ==NULL)
   exit(-1);
  while(i < N_ITERATIONS && ros::ok())
    {
      ROS_INFO("Iteration: %d out of %d",i+1, N_ITERATIONS);
      //Maneuvre to reduce histeresis
      hand.SetAngle(75);
      hand.WaitRest(0.500);
      usleep(500*1000);
   
      double angle;
      int motorEncoder;
      int fingerEncoders[3];
      for(j=0; j<=N_STEPS; j++)
	{
          angle = HAND_OPEN_ANGLE + (HAND_CLOSE_ANGLE-HAND_OPEN_ANGLE)*(double)j/(double)N_STEPS;
          hand.SetAngle(angle);
          hand.WaitRest(0.500);
	  usleep(250*1000);
          hand.GetEncoders(motorEncoder, fingerEncoders);
          fprintf(pFile,"%d,%lf,%d,%d,%d,%d\n", i, angle, motorEncoder, fingerEncoders[0], fingerEncoders[1], fingerEncoders[2]);
	}
     i++;
     if(i < N_ITERATIONS)
       {
         for(j=0; j<10; j++)
	   {
	     hand.SetAngle(HAND_OPEN_ANGLE);
             hand.WaitRest(0.500);
	     usleep(500*1000);
	     hand.SetAngle(HAND_CLOSE_ANGLE);
             hand.WaitRest(0.250);
	     usleep(500*1000);
	   }
       }
 
     //Sleep for a minute to give a rest to the motor of the hand
     hand.SetAngle((HAND_CLOSE_ANGLE + HAND_OPEN_ANGLE)/2.0);
     hand.WaitRest(0.500);
     usleep(500*1000);
     hand.SetRest();
     sleep(30);
   }
  ROS_INFO("Data saved to %s",FILE_NAME);
  fclose(pFile);
}
