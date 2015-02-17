#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <unistd.h>
#include <sys/stat.h>

#include <ros/ros.h>
#include <matVec/matVec.h>
#include <hand_comm/hand_comm.h>
#include <matlab_comm/matlab_comm.h>

#define N_ITERATIONS 2
#define N_STEPS 50
#define TRANS_HAND_FORCE 0.8
#define TRANS_HAND_SPEED 0.5
#define CAL_HAND_FORCE 0.5
#define CAL_HAND_SPEED 0.3
#define HAND_CLOSE_ANGLE 110.0
#define HAND_OPEN_ANGLE -10.0

#define NUM_NOISE 4
#define NOISE_DATA 100
#define MAX_HAND_FORCE 1.0
#define MAX_HAND_SPEED 1.0

#define NUM_SLACK 3
#define SLACK_WAIT 50000
#define SLACK_DATA 100

#define NUM_HYSTERESIS 3
#define HYSTERESIS_DATA 30
#define HYST_ANG_CHG  15.0

#define NUM_CALIBRATION 10

#define MIN_CAL_DIST 1.0

#define MAX_CAL_CHG 10.0

#define RESULTS_FOLDER_NAME "/home/simplehands/Documents/hands/code/nodes/hand/ROS/hand_characterization/results"

const double noise_angles[NUM_NOISE] = {0.0, 35.0, 70.0, 105.0};
const double slack_angles[NUM_SLACK] = {20.0, 40.0, 60.0};
const double hysteresis_angles[NUM_HYSTERESIS] = {20.0, 40.0, 60.0};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hand_characterization");
  ros::NodeHandle node;

  srand ( time(NULL) );

  HandComm hand(&node);
  MatlabComm matlab(&node);

  printf("***********************************************************\n");
  printf("Routine to characterize hand.\n");
  printf("For best accuracy, the palm should be horizontal and looking down.\n");




  // Figure out the folder to save all of our data to.
  char input[1024];
  char folderName[1024];
  char testFile[1024];
  bool dir_exists;

  do
  {

  printf("Please enter the name of the folder in the 'results' folder to save the data to: ");
  int aux = scanf("%s", input);

  // Create the full folder name
  sprintf(folderName, "%s/%s", RESULTS_FOLDER_NAME, input);

  // Attempt to write a file inside of the folder
  sprintf(testFile, "%s/test.txt", folderName);
  FILE* tempFile = fopen(testFile, "w");

  // If it was successful, the folder exists, so remove the folder and try again
  if ((aux != EOF && aux > 0) && tempFile != NULL)
  {
    dir_exists = true;
    fclose(tempFile);
    remove(testFile);
  }
  else
  {
    dir_exists = false;
  }

  }while(dir_exists);

  // If the user has chosen a directory that does not exist, 
  //  create it, and send the folder name to matlab
  printf("Creating folder: %s\n", folderName);
  mkdir(folderName, 0777);
  matlab.sendString("folderName", folderName);

  printf("Waiting for hand...\n");
  while(!hand.Ping()) ;

  //*************************************************************************
  // Analyze the noise in the sensors
  //*************************************************************************
  printf("***********************************************************\n");
  printf("STEP 1: SENSOR NOISE ANALYSIS\n");
  printf("***********************************************************\n");

  // We start test with a calibrated hand
  hand.Calibrate();
  hand.SetForce(MAX_HAND_FORCE);
  hand.SetSpeed(MAX_HAND_SPEED);

  double fingerAngles[NOISE_DATA][NUM_FINGERS];
  double palmForces[NOISE_DATA][NUM_HAND_FORCES];

  double motAng;

  for (int i = 0; i < NUM_NOISE && ros::ok(); i++)
  {
    hand.SetAngle(noise_angles[i]);
    hand.WaitRest(2000);
    for (int j=0; j < NOISE_DATA && ros::ok(); j++)
    {
      hand.GetAngles(motAng, fingerAngles[j]); 
      hand.GetForces(palmForces[j]); 
      usleep(50000);
    }

    matlab.sendMat("enc_noise", NOISE_DATA, NUM_FINGERS, &fingerAngles[0][0]);
    matlab.sendMat("palm_noise", NOISE_DATA, NUM_HAND_FORCES, &palmForces[0][0]);

    matlab.sendCommand("[range, stdev] = HandCharacterization_sensorNoise([enc_noise palm_noise], folderName)");

    Vec ranges;
    Vec stdevs;
    matlab.getVec("range", ranges);
    matlab.getVec("stdev", stdevs);

    printf("Angle: %f\n", noise_angles[i]);

    char buf1[1024];
    char buf2[1024];

    sprintf(buf1, "ranges: ");
    sprintf(buf2, "stdevs: ");
    for(int j = 0; j < NUM_FINGERS + NUM_HAND_FORCES; j++)
    {
      sprintf(buf1, "%s %3.3f", buf1, ranges[j]);
      sprintf(buf2, "%s %3.3f", buf2, stdevs[j]);
    }
    printf("%s\n",buf1);
    printf("%s\n",buf2);
    printf("*********************************************************\n");
  }


  //************************************************************************
  // Analyze the slack
  //************************************************************************
  printf("***********************************************************\n");
  printf("STEP 2: SLACK ANALYSIS\n");
  printf("***********************************************************\n");

  double slackFingers[SLACK_DATA][NUM_FINGERS];

  for (int i = 0; i < NUM_SLACK && ros::ok(); i++)
  {
    hand.SetAngle(slack_angles[i]);
    hand.WaitRest(2000);
    printf("Recording slack data for %3.1f seconds.\n", ((double)SLACK_WAIT*SLACK_DATA) / 1000000);

    for (int j=0; j < SLACK_DATA && ros::ok(); j++)
    {
      hand.GetAngles(motAng, slackFingers[j]); 
      usleep(SLACK_WAIT);
    }

    matlab.sendMat("enc_slack", SLACK_DATA, NUM_FINGERS, &slackFingers[0][0]);

    matlab.sendCommand("range = HandCharacterization_slack(enc_slack, folderName)");

    Vec ranges;
    matlab.getVec("range", ranges);

    printf("Angle: %f\n", slack_angles[i]);

    char buf[1024];

    sprintf(buf, "ranges: ");
    for(int j = 0; j < NUM_FINGERS; j++)
    {
      sprintf(buf, "%s %3.3f", buf, ranges[j]);
    }
    printf("%s\n",buf);
    printf("*********************************************************\n");
  }



  //************************************************************************
  // Investigate Hysteresis
  //************************************************************************

  printf("*********************************************************\n");
  printf("STEP 3: HYSTERESIS ANALYSIS\n");
  printf("*********************************************************\n");

  hand.Calibrate(1);

  double finit[HYSTERESIS_DATA][NUM_FINGERS];
  double ffinal[HYSTERESIS_DATA][NUM_FINGERS];
  double binit[HYSTERESIS_DATA][NUM_FINGERS];
  double bfinal[HYSTERESIS_DATA][NUM_FINGERS];

  for (int i = 0; i < NUM_HYSTERESIS && ros::ok(); i++)
  {
    // Go to the specified angle
    hand.SetAngle(hysteresis_angles[i]);
    hand.WaitRest(250);
    usleep(100*1000);

    for (int j=0; j < HYSTERESIS_DATA && ros::ok(); j++)
    {
      // Save this pose
      hand.GetAngles(motAng, finit[j]); 

      // Increase our angle a bit, then once we get there, come back
      hand.SetAngle(hysteresis_angles[i] + HYST_ANG_CHG);
      hand.WaitRest(250);
      hand.SetAngle(hysteresis_angles[i]);
      hand.WaitRest(250);
      usleep(100*1000);

      // Store this position. the difference will be the slack
      hand.GetAngles(motAng, ffinal[j]); 

      // Note that we will use this pose as the starting pose 
      //  when checking hysteresis in the opposite direction
      for (int k=0; k < NUM_FINGERS; k++)
        binit[j][k] = ffinal[j][k];

      // Decrease our angle a bit, and once we get there, come back
      hand.SetAngle(hysteresis_angles[i] - HYST_ANG_CHG);
      hand.WaitRest(250);
      hand.SetAngle(hysteresis_angles[i]);
      hand.WaitRest(250);
      usleep(100*1000);
      hand.GetAngles(motAng, bfinal[j]); 
    }

    matlab.sendMat("finit", HYSTERESIS_DATA, NUM_FINGERS, &finit[0][0]);
    matlab.sendMat("ffinal", HYSTERESIS_DATA, NUM_FINGERS, &ffinal[0][0]);
    matlab.sendMat("binit", HYSTERESIS_DATA, NUM_FINGERS, &binit[0][0]);
    matlab.sendMat("bfinal", HYSTERESIS_DATA, NUM_FINGERS, &bfinal[0][0]);

    matlab.sendCommand("[frange, fstdev, brange, bstdev] = HandCharacterization_hysteresis(finit, ffinal, binit, bfinal, folderName)");

    Vec franges;
    Vec fstdevs;
    Vec branges;
    Vec bstdevs;
    matlab.getVec("frange", franges);
    matlab.getVec("fstdev", fstdevs);
    matlab.getVec("brange", branges);
    matlab.getVec("bstdev", bstdevs);

    printf("Angle: %f\n", hysteresis_angles[i]);

    char buf1[1024];
    char buf2[1024];
    char buf3[1024];
    char buf4[1024];

    sprintf(buf1, "franges: ");
    sprintf(buf2, "fstdevs: ");
    sprintf(buf3, "branges: ");
    sprintf(buf4, "bstdevs: ");
    for(int j = 0; j < NUM_FINGERS; j++)
    {
      sprintf(buf1, "%s %3.3f", buf1, franges[j]);
      sprintf(buf2, "%s %3.3f", buf2, fstdevs[j]);
      sprintf(buf3, "%s %3.3f", buf3, branges[j]);
      sprintf(buf4, "%s %3.3f", buf4, bstdevs[j]);
    }
    printf("%s\n",buf1);
    printf("%s\n",buf2);
    printf("%s\n",buf3);
    printf("%s\n",buf4);
    printf("*********************************************************\n");
  }


  //************************************************************************
  // Analyze the transmission, and check calibration
  //************************************************************************

  printf("*********************************************************\n");
  printf("STEP 4: TRANSMISSION ANALYSIS\n");
  printf("*********************************************************\n");

  // We start test with a calibrated hand
  hand.Calibrate(1);
  hand.SetForce(TRANS_HAND_FORCE);
  hand.SetSpeed(TRANS_HAND_SPEED);

  // For N_ITER times we go through the process:
  // - Opening and closing the hand 10 times.
  // - Mapping input-output transmission
  double angle[N_STEPS][2*N_ITERATIONS];
  double motorEncoder[N_STEPS][2*N_ITERATIONS];
  double fingerEncoders[N_STEPS][NUM_FINGERS * 2*N_ITERATIONS];

  int tempFing[NUM_FINGERS];
  int tempMot;

  for(int i=0; i < N_ITERATIONS && ros::ok(); i++)
  {
    //Maneuvre to reduce histeresis
    //hand.SetAngle(75);
    //hand.WaitRest(500);
    //usleep(500*1000);

    for(int j=0; j < N_STEPS; j++)
    {
      angle[j][2*i] = HAND_OPEN_ANGLE + (HAND_CLOSE_ANGLE-HAND_OPEN_ANGLE)*(double)j/(double)N_STEPS;
      hand.SetAngle(angle[j][2*i]);
      hand.WaitRest(500);
      usleep(250*1000);
      hand.GetEncoders(tempMot, tempFing);
      motorEncoder[j][i*2] = (double)tempMot;
      for(int k=0; k < NUM_FINGERS; k++)
        fingerEncoders[j][i*2*NUM_FINGERS+k] = (double)tempFing[k];
    }

    for(int j=0; j < N_STEPS; j++)
    {
      angle[j][2*i+1] = HAND_CLOSE_ANGLE + (HAND_OPEN_ANGLE-HAND_CLOSE_ANGLE)*(double)j/(double)N_STEPS;
      hand.SetAngle(angle[j][2*i+1]);
      hand.WaitRest(500);
      usleep(250*1000);
      hand.GetEncoders(tempMot, tempFing);
      motorEncoder[j][i*2+1] = (double)tempMot;
      for(int k=0; k < NUM_FINGERS; k++)
        fingerEncoders[j][(i*2+1)*NUM_FINGERS+k] = (double)tempFing[k];
    }
  }

  matlab.sendMat("angle", N_STEPS, 2*N_ITERATIONS, &angle[0][0]);
  matlab.sendMat("motorEncoder", N_STEPS, 2*N_ITERATIONS, &motorEncoder[0][0]);
  matlab.sendMat("fingerEncoders", N_STEPS, NUM_FINGERS*2*N_ITERATIONS, &fingerEncoders[0][0]);

  matlab.sendCommand("HandCharacterization_plotTransmission(angle, motorEncoder, fingerEncoders, folderName)");


  //************************************************************************
  // Check how consistent our calibration is
  //************************************************************************

  printf("*********************************************************\n");
  printf("STEP 5: CALIBRATION ANALYSIS\n");
  printf("*********************************************************\n");

  // We start test with a calibrated hand
  hand.Calibrate(1);

  double calibrationVals[NUM_CALIBRATION][2];
  double temp[NUM_FINGERS];
  double max_cal_angle, min_cal_angle;

  printf("Checking limits...\n");

  hand.SetAngle(80);
  hand.SetForce(MAX_HAND_FORCE);
  hand.SetSpeed(MAX_HAND_SPEED);
  hand.WaitRest(250);
  hand.SetForce(CAL_HAND_FORCE);
  hand.SetSpeed(CAL_HAND_SPEED);
  hand.SetAngle(HAND_CLOSE_ANGLE);
  hand.WaitRest(250);
  usleep(100*1000);

  // Record the maximum motor value
  hand.GetAngles(max_cal_angle, temp);

  hand.SetAngle(10);
  hand.SetForce(MAX_HAND_FORCE);
  hand.SetSpeed(MAX_HAND_SPEED);
  hand.WaitRest(250);
  hand.SetForce(CAL_HAND_FORCE);
  hand.SetSpeed(CAL_HAND_SPEED);
  hand.SetAngle(HAND_OPEN_ANGLE);
  hand.WaitRest(250);
  usleep(100*1000);

  // Record the maximum motor value
  hand.GetAngles(min_cal_angle, temp);

  printf("max limit: %3.2f, min limit: %3.2f\n", max_cal_angle, min_cal_angle);


  for (int i = 0; i < NUM_CALIBRATION && ros::ok(); i++)
  {
    // Go to a random angle close to the maximum angle at full speed
    double rand_ang = (((double)rand())/RAND_MAX) * MAX_CAL_CHG + MIN_CAL_DIST;
    hand.SetAngle(max_cal_angle - rand_ang);
    hand.SetForce(MAX_HAND_FORCE);
    hand.SetSpeed(MAX_HAND_SPEED);
    hand.WaitRest(250);

    // Once we're there, go the rest of the way at the calibration force and speed
    hand.SetForce(CAL_HAND_FORCE);
    hand.SetSpeed(CAL_HAND_SPEED);
    hand.SetAngle(HAND_CLOSE_ANGLE);
    hand.WaitRest(250);
    usleep(100*1000);

    // Record the motor value
    hand.GetAngles(calibrationVals[i][0], temp);


    // Go to a random angle close to the minimum angle at full speed
    rand_ang = (((double)rand())/RAND_MAX) * MAX_CAL_CHG + MIN_CAL_DIST;
    hand.SetAngle(min_cal_angle + rand_ang);
    hand.SetForce(MAX_HAND_FORCE);
    hand.SetSpeed(MAX_HAND_SPEED);
    hand.WaitRest(250);

    // Once we're there, go the rest of the way at the calibration force and speed
    hand.SetForce(CAL_HAND_FORCE);
    hand.SetSpeed(CAL_HAND_SPEED);
    hand.SetAngle(HAND_OPEN_ANGLE);
    hand.WaitRest(250);
    usleep(100*1000);

    // Record the motor value
    hand.GetAngles(calibrationVals[i][1], temp);
  }


  hand.SetAngle((HAND_OPEN_ANGLE + HAND_CLOSE_ANGLE)/2.0);
  hand.SetForce(MAX_HAND_FORCE);
  hand.SetSpeed(MAX_HAND_SPEED);

  // Send all of the calibration points to matlab, and analyze the data
  matlab.sendMat("calVals", NUM_CALIBRATION, 2, &calibrationVals[0][0]);

  matlab.sendCommand("[range, stdev] = HandCharacterization_calibration(calVals, folderName)");
  Vec ranges;
  Vec stdevs;
  matlab.getVec("range", ranges);
  matlab.getVec("stdev", stdevs);

  char buf1[1024];
  char buf2[1024];

  sprintf(buf1, "ranges: ");
  sprintf(buf2, "stdevs: ");
  for(int j = 0; j < 2; j++)
  {
    sprintf(buf1, "%s %3.3f", buf1, ranges[j]);
    sprintf(buf2, "%s %3.3f", buf2, stdevs[j]);
  }
  printf("%s\n",buf1);
  printf("%s\n",buf2);

  hand.WaitRest(250);

  return 0;
}
