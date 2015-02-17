#include "handP2_node.h"

#define MAX_BUFFER 256

HandController::HandController(ros::NodeHandle *n) 
{
  node = n;
  calibrated = false;
  node->getParam("/hand/stepsPerDegreeMotor",stepsPerDegreeMotor);
  node->getParam("/hand/stepsPerDegreeFingers",stepsPerDegreeFingers);
}

HandController::~HandController() {
  /// Shutting down services.
  handle_hand_Ping.shutdown();
  handle_hand_Calibrate.shutdown();
  handle_hand_GetEncoders.shutdown();
  handle_hand_GetAngles.shutdown();
  handle_hand_SetEncoder.shutdown();
  handle_hand_SetAngle.shutdown();
  handle_hand_IsMoving.shutdown();
  handle_hand_SetRest.shutdown();
  handle_hand_WaitRest.shutdown();
  handle_hand_SetSpeed.shutdown();
  handle_hand_SetForce.shutdown();

  // Shutting down topics.
  handle_hand_AnglesLog.shutdown();
  handle_hand_EncodersLog.shutdown();

  // Closing connection with hand
  hand.disconnectMotor();
  hand.disconnectArduino();
}


bool HandController::defaultHandConfiguration()
{
  //Set default speed
  int maxSpeed,minSpeed;
  double defaultSpeed;
  node->getParam("/hand/maxMotorSpeed",maxSpeed);
  node->getParam("/hand/minMotorSpeed",minSpeed);
  node->getParam("/hand/motorSpeed",defaultSpeed);

  double ratio = defaultSpeed;
  if (ratio > 1.0)
    ratio = 1.0;
  if (ratio < 0.0)
    ratio = 0.0;
  int speed = (int)(minSpeed + ratio*(double)(maxSpeed - minSpeed));
  if(!hand.setMotorSpeed(speed))
  {
    ROS_INFO("HAND_CONTROLLER: Could not set the motor speed.");
    return false;
  }

  //Set default intensity
  int maxIntensity,minIntensity;
  double defaultForce;
  node->getParam("/hand/maxMotorIntensity",maxIntensity);
  node->getParam("/hand/minMotorIntensity",minIntensity);
  node->getParam("/hand/motorForce",defaultForce);
  ratio = defaultForce;
  if (ratio > 1.0)
    ratio = 1.0;
  if (ratio < 0.0)
    ratio = 0.0;

  int intensity = (int)(minIntensity + ratio*(double)(maxIntensity - minIntensity));
  if(!hand.setMotorIntensity(intensity))
  {
    ROS_INFO("HAND_CONTROLLER: Could not set the motor intensity.");
    return false;
  }
  return true;
}

void HandController::advertiseTopics()
{
  handle_hand_EncodersLog = node->advertise<handP2_comm::hand_EncodersLog>("hand_EncodersLog", 100);
  handle_hand_AnglesLog = node->advertise<handP2_comm::hand_AnglesLog>("hand_AnglesLog", 100);
}

void HandController::advertiseServices()
{
  handle_hand_Ping = node->advertiseService("hand_Ping", &HandController::hand_Ping, this);
  handle_hand_Calibrate = node->advertiseService("hand_Calibrate", &HandController::hand_Calibrate, this);
  handle_hand_GetEncoders = node->advertiseService("hand_GetEncoders", &HandController::hand_GetEncoders, this);
  handle_hand_GetAngles = node->advertiseService("hand_GetAngles", &HandController::hand_GetAngles, this);
  handle_hand_SetEncoder = node->advertiseService("hand_SetEncoder", &HandController::hand_SetEncoder, this);
  handle_hand_SetAngle = node->advertiseService("hand_SetAngle", &HandController::hand_SetAngle, this);
  handle_hand_IsMoving = node->advertiseService("hand_IsMoving", &HandController::hand_IsMoving, this);
  handle_hand_SetRest = node->advertiseService("hand_SetRest", &HandController::hand_SetRest, this);
  handle_hand_WaitRest = node->advertiseService("hand_WaitRest", &HandController::hand_WaitRest, this);
  handle_hand_SetSpeed = node->advertiseService("hand_SetSpeed", &HandController::hand_SetSpeed, this);
  handle_hand_SetForce = node->advertiseService("hand_SetForce", &HandController::hand_SetForce, this);
}

bool HandController::hand_Ping(handP2_comm::hand_Ping::Request& req, handP2_comm::hand_Ping::Response& res)
{
  if(hand.connectedMotor)
  {
    int motorPos;
    if (hand.getMotor(&motorPos))
    {
      res.ret = 1;
      return true;
    }
    res.ret = 0;
    res.msg = "HAND_CONTROLLER: Could not read the motor position.";
    return false;
  }
  res.ret = 0;
  res.msg = "HAND_CONTROLLER: Hand not connected to motor controller.";
  return false;
}


bool HandController::hand_Calibrate(handP2_comm::hand_Calibrate::Request& req, handP2_comm::hand_Calibrate::Response& res)
{
  //Check that Arduino and motor are connected)
  if(!hand.connectedArduino || !hand.connectedMotor)
  {
    res.msg = "HAND_CONTROLLER: Cannot calibrate. Not connected to the hand.";
    res.ret = 0;
    ROS_INFO("%s",res.msg.c_str());
    return false;
  }

  // Tell the P2Interpreter we're calibrating, so it doesn't 
  //  do any fancy wrapping around with encoders
  hand.calibrating = true;

  int savedMotorSpeed = hand.motorSpeed;
  int savedMotorIntensity = hand.motorIntensity;

  // If we want a fast calibration, we're assuming our calibration is almost 
  // right, so go most of the way at a faster speed before moving at a 
  // slower speed at the end
  if(req.fast)
  {
    hand.setMotorIntensity(300);
    hand.setMotorSpeed(150);
    int zeroMotor;
    node->getParam("/hand/zeroMotorPos",zeroMotor);
    hand.setMotor(zeroMotor-1000);
    hand.waitRest(500.0); 
  }

  //Set low intensity and speed
  hand.setMotorIntensity(150);
  hand.setMotorSpeed(25);

  //Open the hand as much as possible and call that the zero position
  int initialMotorPose;
  hand.getMotor(&initialMotorPose);
  hand.setMotor(initialMotorPose + 30000); 
  hand.waitRest(500.0);

  //Check if the motor has actually moved
  int newMotorPose;
  hand.getMotor(&newMotorPose);
  if(newMotorPose==initialMotorPose)
  {
    res.msg = "HAND_CONTROLLER: Cannot calibrate. Cannot move the hand.";
    res.ret = 0;
    ROS_INFO("%s",res.msg.c_str());
    hand.calibrating = false;
    return false;
  }

  //Set that as the homing position.
  hand.setHome();

  //Restore saved motor intensity and speed
  hand.setMotorIntensity(savedMotorIntensity);
  hand.setMotorSpeed(savedMotorSpeed);

  //Move to origin position
  int zeroMotor;
  node->getParam("/hand/zeroMotorPos",zeroMotor);
  hand.setMotor(zeroMotor);
  hand.waitRest(250.0);  

  //Get calibration
  hand.getMotor(&encMotorZero);
  for (int i=0; i < NUM_FINGERS; i++)
  {
    hand.getFinger(&encFingersZero[i],i);
  }
  calibrated = true;

  res.encMotor = encMotorZero;
  res.enc.resize(NUM_FINGERS);
  for (int i=0; i < NUM_FINGERS; i++)
  {
    res.enc[i] = encFingersZero[i];
  }
  res.ret = 1;

  // We're done moving, so we can resume worrying 
  //  about encoders wrapping around
  hand.calibrating = false;

  
  // Check that our new calibrated finger values 
  // are close to the previously calibrated values
  FILE *fCalib;
  fCalib = fopen("P2Calibration.txt", "r");
  if (fCalib)
  {
    int motZ;
    int fileEnc[NUM_FINGERS];

    if (fscanf(fCalib, "%d", &motZ) == 1)
    {
      for (int i=0; i<NUM_FINGERS; i++)
      {
        if (fscanf(fCalib, "%d", &fileEnc[i]) != 1)
          break;
        if (fabs(fileEnc[i] - encFingersZero[i]) > MAX_CAL_DIFF)
        {
          ROS_WARN("******************************************************");
          ROS_WARN("HAND_CONTROLLER: Calibrated finger encoder position is "
              "not close to the previously calibrated value."
              " Has an encoder moved?");
          ROS_WARN("Finger %d: New: %d, Old: %d", 
              i+1, encFingersZero[i], fileEnc[i]);
          ROS_WARN("******************************************************");
        }
      }
    }
    fclose(fCalib);
  }

  //Save last calibration to a file
  ROS_INFO("HAND_CONTROLLER: Saving calibration to file.");
  fCalib = fopen("P2Calibration.txt","w");
  fprintf(fCalib,"%d\n",encMotorZero);
  for (int i=0; i < NUM_FINGERS; i++)
  {
    fprintf(fCalib,"%d\n",encFingersZero[i]);
  }
  fclose(fCalib);
  return true;
}


bool HandController::hand_GetEncoders(handP2_comm::hand_GetEncoders::Request& req, handP2_comm::hand_GetEncoders::Response& res)
{
  bool ok=true;
  int aux;
  if(!hand.getMotor(&aux))
    ok = false;
  else
    res.encMotor = aux;

  res.encFinger.resize(NUM_FINGERS);
  for (int i=0; i < NUM_FINGERS; i++)
  {
    if(!hand.getFinger(&aux,i))
      ok = false;
    else
      res.encFinger[i] = aux;
  }

  if(ok)
  {
    res.ret = 1;
    return true;
  }
  else
  {
    res.ret = 0;
    res.msg = "HAND_CONTROLLER: Could not read hand's pose.";
    ROS_INFO("%s",res.msg.c_str());
    return false;
  }
}

bool HandController::hand_GetAngles(handP2_comm::hand_GetAngles::Request& req, handP2_comm::hand_GetAngles::Response& res)
{
  bool ok=true;
  int aux;
  if(calibrated)
  {
    if(!hand.getMotor(&aux))
      ok = false;
    else
      res.angleMotor = (aux - encMotorZero)/(double)stepsPerDegreeMotor;

    res.angle.resize(NUM_FINGERS);
    for (int i=0; i < NUM_FINGERS; i++)
    {
      if(!hand.getFinger(&aux,i))
        ok = false;
      else
      {
        res.angle[i] = -(aux - encFingersZero[i])/(double)stepsPerDegreeFingers;
        // Make sure our angles are between -180 and 180
        if (res.angle[i] < -180.0)
          res.angle[i] += 360.0;
        else if (res.angle[i] > 180.0)
          res.angle[i] -= 360.0;
      }
    }

    if(ok)
    {
      res.ret = 1;
      return true;
    }
    else
    {
      res.ret = 0;
      res.msg = "HAND_CONTROLLER: Could not read hand's pose.";
      ROS_INFO("%s",res.msg.c_str());
      return false;
    }
  }
  else
  {
    res.ret = 0;
    res.msg = "HAND_CONTROLLER: Hand not calibrated.";
    return false;
  }
}


bool HandController::hand_SetEncoder(handP2_comm::hand_SetEncoder::Request& req, handP2_comm::hand_SetEncoder::Response& res)
{
  if(hand.setMotor(req.enc))
  {
    res.ret = 1;
    return true;
  }
  res.ret = 0;
  res.msg = "HAND_CONTROLLER: Not able to move the hand.";
  return false;
}

bool HandController::hand_SetAngle(handP2_comm::hand_SetAngle::Request& req, handP2_comm::hand_SetAngle::Response& res)
{
  if(calibrated)
  {
    int pose = req.angle*stepsPerDegreeMotor + encMotorZero;
    if(hand.setMotor(pose))
    {
      res.ret = 1;
      return true;
    }
    res.ret = 0;
    res.msg = "HAND_CONTROLLER: Not able to move the hand.";
    return false;
  }
  res.ret = 0;
  res.msg = "HAND_CONTROLLER: Hand not calibrated.";
  return false;
}

bool HandController::hand_SetRest(handP2_comm::hand_SetRest::Request& req, handP2_comm::hand_SetRest::Response& res)
{
  if(hand.setRest())
  {
    res.ret = 1;
    return true;
  }
  res.ret = 0;
  res.msg = "HAND_CONTROLLER: Problem setting the hand to rest.";
  return false;
} 

bool HandController::hand_WaitRest(handP2_comm::hand_WaitRest::Request& req, handP2_comm::hand_WaitRest::Response& res)
{
  hand.waitRest(req.delay);
  res.ret = 1;
  return true;
}

bool HandController::hand_SetSpeed(handP2_comm::hand_SetSpeed::Request& req, handP2_comm::hand_SetSpeed::Response& res)
{
  if (req.speed > 1.0)
  {
    res.ret = 0;
    res.msg = "HAND_CONTROLLER: Speed above allowed range (0-1).";
    return false;
  }
  if(req.speed < 0.0)
  {
    res.ret = 0;
    res.msg = "HAND_CONTROLLER: Speed below allowed range (0-1).";
    return false;
  }
  int maxSpeed,minSpeed;
  node->getParam("/hand/maxMotorSpeed",maxSpeed);
  node->getParam("/hand/minMotorSpeed",minSpeed);

  int speed = minSpeed + req.speed *(maxSpeed - minSpeed);
  if(!hand.setMotorSpeed(speed))
  {
    res.ret = 0;
    res.msg = "HAND_CONTROLLER: Could not set the motor speed.";
    ROS_INFO("%s",res.msg.c_str());
    return false;
  }
  //We save it to the system parameters
  node->setParam("/hand/motorSpeed",req.speed);
  res.ret = 1;
  return true;
}

bool HandController::hand_SetForce(handP2_comm::hand_SetForce::Request& req, handP2_comm::hand_SetForce::Response& res)
{
  if (req.force > 1.0)
  {
    res.ret = 0;
    res.msg = "HAND_CONTROLLER: Force above allowed range (0-1).";
    return false;
  }
  if(req.force < 0.0)
  {
    res.ret = 0;
    res.msg = "HAND_CONTROLLER: Force below allowed range (0-1).";
    return false;
  }
  int maxIntensity,minIntensity;
  node->getParam("/hand/maxMotorIntensity",maxIntensity);
  node->getParam("/hand/minMotorIntensity",minIntensity);

  int intensity = minIntensity + req.force *(maxIntensity - minIntensity);
  if(!hand.setMotorIntensity(intensity))
  {
    res.ret = 0;
    res.msg = "HAND_CONTROLLER: Could not set the motor intensity.";
    ROS_INFO("%s",res.msg.c_str());
    return false;
  }
  //We save it to the system parameters
  node->setParam("/hand/motorForce",req.force);
  res.ret = 1;
  return true;
}

bool HandController::hand_IsMoving(handP2_comm::hand_IsMoving::Request& req, handP2_comm::hand_IsMoving::Response& res)
{
  bool mov;
  if(hand.isMoving(&mov))
  {
    res.moving = mov;
    res.ret = 1;
    return true;
  }
  else
  {
    res.moving = false;
    res.ret = 0;
    res.msg = "HAND_CONTROLLER: Hand not connected to motor controller.";
    return false;
  }

  /*
  if(hand.connectedMotor)
  {
    int motorPos1, motorPos2;
    bool aux1 = hand.getMotor(&motorPos1);
    struct timespec req;
    req.tv_sec = 0;
    req.tv_nsec = (int)(1000000L * (200));//Wait 200 ms
    nanosleep(&req, (timespec *)NULL);
    bool aux2 = hand.getMotor(&motorPos2);
    if(aux1 && aux2)
    {
      if(fabs(motorPos1-motorPos2) < 10)
      {
        res.moving = false;
        res.ret = 1;
        return true;
      }
      else
      {
        res.moving = true;
        res.ret = 1;
        return true;
      }
    }
    else
    {
      res.ret = 0;
      res.msg = "HAND_CONTROLLER: Could not read the motor position.";
      return false;
    }
  }
  res.ret = 0;
  res.msg = "HAND_CONTROLLER: Hand not connected to motor controller.";
  return false;
  */
}

void HandController::logCallback(const ros::TimerEvent& event)
{
  handP2_comm::hand_EncodersLog msgEncoders;
  handP2_comm::hand_AnglesLog msgAngles;

  msgEncoders.encFinger.resize(NUM_FINGERS);
  msgAngles.angle.resize(NUM_FINGERS);

  int aux;
  if(hand.connectedMotor && hand.connectedArduino)
  {
    msgEncoders.timeStamp = event.current_real.toSec();
    msgAngles.timeStamp = event.current_real.toSec();
    if(hand.getMotor(&aux))
    {
      msgEncoders.encMotor = aux;
      if (calibrated)
        msgAngles.angleMotor = (aux - encMotorZero)/(double)stepsPerDegreeMotor;
    }
    for (int i=0; i < NUM_FINGERS; i++)
    {
      if(hand.getFinger(&aux,i))
      {
        msgEncoders.encFinger[i] = aux;
        if (calibrated)
        {
          msgAngles.angle[i] = -(aux - encFingersZero[i])/(double)stepsPerDegreeFingers;
          if (msgAngles.angle[i] < -180.0)
            msgAngles.angle[i] += 360.0;
          else if (msgAngles.angle[i] > 180.0)
            msgAngles.angle[i] -= 360.0;
        }
      }
    }
    handle_hand_EncodersLog.publish(msgEncoders);
    if(calibrated)
      handle_hand_AnglesLog.publish(msgAngles);
  }
}

void *loggerMain(void *args)
{
  //Recover the pointer to the main node
  HandController* P2hand;
  P2hand = (HandController*) args;

  ros::Timer loggerTimer;
  loggerTimer = P2hand->node->createTimer(ros::Duration(0.025), &HandController::logCallback, P2hand);
  ros::waitForShutdown();
  loggerTimer.stop();
  return NULL;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hand_controller");
  ros::NodeHandle node;
  HandController P2hand(&node);

  //Connection to Arduino
  ROS_INFO("HAND_CONTROLLER: Connecting to the Arduino...");
  std::string arduinoId;
  int arduinoBaud;
  node.getParam("/hand/arduinoSerialId",arduinoId);
  node.getParam("/hand/arduinoSerialBaud",arduinoBaud);
  ROS_INFO("Searching for arduino in %s with baud rate %d.",arduinoId.c_str(),arduinoBaud);
  if(!P2hand.hand.connectArduino(arduinoId.c_str(), arduinoBaud))
  {
    ROS_WARN("HAND_CONTROLLER: Impossible to connect to Arduino.");	
    ROS_WARN("HAND_CONTROLLER: Exit.");
    return 1;
  }

  //Connection to Motor Controller
  ROS_INFO("HAND_CONTROLLER: Connecting to the Motor controller...");
  std::string motorId;
  int motorBaud;
  node.getParam("/hand/motorSerialId",motorId);
  node.getParam("/hand/motorSerialBaud",motorBaud);
  if(!P2hand.hand.connectMotor(motorId.c_str(), motorBaud))
  {
    ROS_WARN("HAND_CONTROLLER: Impossible to connect to Motor controller.");
    ROS_WARN("HAND_CONTROLLER: Exit.");
    return 1;
  }

  //Default robot Configuration
  ROS_INFO("HAND_CONTROLLER: Setting default hand configuration...");
  if(!P2hand.defaultHandConfiguration())
  {
    ROS_INFO("HAND_CONTROLLER: Not able to set the robot to default configuration.");
    exit(-1);
  }

  //Look for last calibration
  FILE *fCalib;
  fCalib = fopen("P2Calibration.txt","r");
  if(fCalib)
  {
    int aux = fscanf(fCalib,"%d\n",&(P2hand.encMotorZero));
    if (aux != 0)
    {
      ROS_WARN("Couldn't read from file. Skipping file calibration");
    }
    else
    {
      for (int i=0; i < NUM_FINGERS; i++)
      {
        aux = fscanf(fCalib,"%d\n",&(P2hand.encFingersZero[i]));
        if (aux != 1)
        {
          ROS_WARN("Improper file format. Skipping rest of file calibration");
          P2hand.calibrated = false;
          break;
        }
      }
      fclose(fCalib);
      
      if (aux == 1)
      {
        ROS_INFO("HAND_CONTROLLER: ___________________WARNING___________________");
        ROS_INFO("HAND_CONTROLLER: Loading hand calibration from last execution.");
        P2hand.calibrated = true;
      }
    }
  }

  //Advertising ROS services
  ROS_INFO("HAND_CONTROLLER: Advertising ROS services...");
  P2hand.advertiseServices();

  //Advertising ROS topics
  ROS_INFO("HAND_CONTROLLER: Advertising ROS topics...");
  P2hand.advertiseTopics();

  //Topic broadcast performed in a dedicated thread
  pthread_t loggerThread;
  pthread_attr_t attr;
  pthread_attr_init(&attr);
  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
  if(pthread_create(&loggerThread,  &attr, loggerMain, (void*)&P2hand) != 0)
    ROS_INFO("HAND_CONTROLLER: Unable to create logger thread. Error number: %d.",errno);

  //Main ROS loop
  ROS_INFO("HAND_CONTROLLER: Running node /hand_controller...");
  //Multithreaded spinner so that callbacks can be handled on separate threads.
  ros::MultiThreadedSpinner spinner(2); // Use 2 threads 
  spinner.spin();
  ROS_INFO("HAND_CONTROLLER: Shutting down node /hand_controller...");

  //End log dedicated thread
  void *status;
  pthread_attr_destroy(&attr);
  pthread_join(loggerThread, &status);

  ROS_INFO("HAND_CONTROLLER: Done.");
  return 0;
}
