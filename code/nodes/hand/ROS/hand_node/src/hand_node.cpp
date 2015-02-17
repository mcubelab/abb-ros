//
// Name: Robbie Paolini
//
// File Name: hand_node.cpp
//
// Last Modified: 8/31/2012
// Last Modified: 10/03/2013 Nikhil: Added new service hand.DADA
//
// This file implements the hand node, which controls P3, a simple hand 
// designed in the Manipulation Lab. P3 is controlled by an arduino. This
// program interacts with the arduino over a USB serial port. It records
// and publishes finger, motor, and palm sensor data. It allows the user
// to change the speed and current limit of the motor and then command it
// to move to a certain pose. Since the motor encoder is relative, it also
// implements a calibration routine so we can get consistent motor poses 
// across runs. Please look at the arduino code on P3 for more details with
// communication

#include "hand_node.h"

//////////////////////////////////////////////////////////////////////////
// Constructor. Saves the node, and configures some status variables
HandController::HandController(ros::NodeHandle *n) 
{
  node = n;

  // Read from the parameter file what the relationship between encoder
  // counts and angles are
  node->getParam("/hand/stepsPerDegreeMotor",stepsPerDegreeMotor);
  node->getParam("/hand/stepsPerDegreeFingers",stepsPerDegreeFingers);

  // We just initialized, so the hand is not yet calibrated
  calibrated = false;

  // We have not yet connected to the arduino
  connectedArduino = false;

  // Hopefully we're not moving
  moving = false;
  
  non_block_action = HNB_NO_ACTION;

  // Our motor buffer, used to check if the hand is moving or not,
  // should start off as empty
  bufCnt = 0;
  bufFull = false;
  for (int i=0; i < MOT_BUF; i++)
    motBuf[i] = 0;

  // Our buffer to keep track of not yet processed serial characters read
  // in from the hand should initially be empty and null terminated
  curSerial = 0;
  serialBuffer[0] = '\0';
}

//////////////////////////////////////////////////////////////////////////
// Destructor. Shutdown all services, messages, and disconnect the arduino
HandController::~HandController() {
  /// Shutting down services.
  handle_hand_Ping.shutdown();
  handle_hand_Calibrate.shutdown();
  handle_hand_GetEncoders.shutdown();
  handle_hand_GetRawForces.shutdown();
  handle_hand_GetForces.shutdown();
  handle_hand_GetAngles.shutdown();
  handle_hand_SetEncoder.shutdown();
  handle_hand_SetAngle.shutdown();
  handle_hand_IsMoving.shutdown();
  handle_hand_SetRest.shutdown();
  handle_hand_WaitRest.shutdown();
  handle_hand_SetSpeed.shutdown();
  handle_hand_SetForce.shutdown();
  handle_hand_DADA.shutdown();

  // Shutting down topics.
  handle_hand_AnglesLog.shutdown();
  handle_hand_EncodersLog.shutdown();
  handle_hand_RawForcesLog.shutdown();
  handle_hand_ForcesLog.shutdown();

  // Closing connection with hand
  disconnectArduino();
}

//////////////////////////////////////////////////////////////////////////
// This function looks at the parameter file and sets the default 
// speed and force for this hand
bool HandController::defaultHandConfiguration()
{
  //Set default speed
  double defaultSpeed;
  node->getParam("/hand/motorSpeed",defaultSpeed);

  if(!setMotorSpeed(defaultSpeed))
  {
    ROS_INFO("HAND_CONTROLLER: Could not set the motor speed.");
    return false;
  }

  //Set default intensity
  double defaultForce;
  node->getParam("/hand/motorForce",defaultForce);

  if(!setMotorIntensity(defaultForce))
  {
    ROS_INFO("HAND_CONTROLLER: Could not set the motor intensity.");
    return false;
  }
  return true;
}

//////////////////////////////////////////////////////////////////////////
// This function advertises all of the topics that we will be publishing 
// to with this node
void HandController::advertiseTopics()
{
  handle_hand_EncodersLog = 
    node->advertise<hand_comm::hand_EncodersLog>("hand_EncodersLog", 100);
  handle_hand_AnglesLog = 
    node->advertise<hand_comm::hand_AnglesLog>("hand_AnglesLog", 100);
  handle_hand_RawForcesLog = 
    node->advertise<hand_comm::hand_RawForcesLog>("hand_RawForcesLog", 100);
  handle_hand_ForcesLog = 
    node->advertise<hand_comm::hand_ForcesLog>("hand_ForcesLog", 100);
}

//////////////////////////////////////////////////////////////////////////
// This function sets up to advertise all services offered by this node
void HandController::advertiseServices()
{
  handle_hand_Ping = 
    node->advertiseService("hand_Ping", &HandController::hand_Ping, this);
  handle_hand_Calibrate = 
    node->advertiseService("hand_Calibrate", &HandController::hand_Calibrate, this);
  handle_hand_GetEncoders = 
    node->advertiseService("hand_GetEncoders", &HandController::hand_GetEncoders, this);
  handle_hand_GetRawForces = 
    node->advertiseService("hand_GetRawForces", &HandController::hand_GetRawForces, this);
  handle_hand_GetForces = 
    node->advertiseService("hand_GetForces", &HandController::hand_GetForces, this);
  handle_hand_GetAngles = 
    node->advertiseService("hand_GetAngles", &HandController::hand_GetAngles, this);
  handle_hand_SetEncoder = 
    node->advertiseService("hand_SetEncoder", &HandController::hand_SetEncoder, this);
  handle_hand_SetAngle = 
    node->advertiseService("hand_SetAngle", &HandController::hand_SetAngle, this);
  handle_hand_IsMoving = 
    node->advertiseService("hand_IsMoving", &HandController::hand_IsMoving, this);
  handle_hand_SetRest = 
    node->advertiseService("hand_SetRest", &HandController::hand_SetRest, this);
  handle_hand_WaitRest = 
    node->advertiseService("hand_WaitRest", &HandController::hand_WaitRest, this);
  handle_hand_SetSpeed = 
    node->advertiseService("hand_SetSpeed", &HandController::hand_SetSpeed, this);
  handle_hand_SetForce = 
    node->advertiseService("hand_SetForce", &HandController::hand_SetForce, this);
  handle_hand_DADA = 
    node->advertiseService("hand_DADA", &HandController::hand_DADA, this);
}

//////////////////////////////////////////////////////////////////////////
// Check if the hand node is working
bool HandController::hand_Ping(hand_comm::hand_Ping::Request& req, 
    hand_comm::hand_Ping::Response& res)
{
  // Return true if we are connected to the arduino
  if (connectedArduino)
  {
    res.ret = 1;
    return true;
  }
  res.ret = 0;
  res.msg = "HAND_CONTROLLER: Not connected to the arduino.";
  return false;
}

//////////////////////////////////////////////////////////////////////////
// Service to calibrate the hand
bool HandController::hand_Calibrate(hand_comm::hand_Calibrate::Request& req, 
    hand_comm::hand_Calibrate::Response& res)
{
  // Make sure we're actually connected to the hand
  if(!connectedArduino)
  {
    res.msg = "HAND_CONTROLLER: Cannot calibrate. Not connected to the hand.";
    res.ret = 0;
    ROS_WARN("%s",res.msg.c_str());
    return false;
  }

  // Save the current motor speed and current limit, so we can restore them
  // when we're done.
  double savedMotorSpeed = motorSpeed;
  double savedMotorIntensity = motorIntensity;

  // If we want a fast calibration, we're assuming our calibration is almost 
  // right, so go most of the way at a faster speed before moving at a 
  // slower speed at the end
  if(req.fast)
  {
    setMotorIntensity(CAL_FAST_FORCE);
    setMotorSpeed(CAL_FAST_SPEED);
    int zeroMotor;
    node->getParam("/hand/zeroMotorPos",zeroMotor);
    setMotorPos(zeroMotor + CAL_FAST_OFFSET);
    waitRest(0.250); 
  }

  //Set low intensity and speed
  setMotorIntensity(CAL_SLOW_FORCE);
  setMotorSpeed(CAL_SLOW_SPEED);

  //Open the hand as much as possible and call that the zero position
  pthread_mutex_lock(&encMotorMutex);
  int initialMotorPose = encMotor;
  pthread_mutex_unlock(&encMotorMutex);

#ifdef P3_5
  setMotorPos(MAX_MOTOR_POS);
#else
  setMotorPos(MIN_MOTOR_POS); 
#endif

  //waitRest(0.250);
  waitRest(1);
  
  //Check if the motor has actually moved
  pthread_mutex_lock(&encMotorMutex);
  int newMotorPose = encMotor;
  pthread_mutex_unlock(&encMotorMutex);
  if(newMotorPose==initialMotorPose)
  {
    res.msg = "HAND_CONTROLLER: Cannot calibrate. Cannot move the hand.";
    res.ret = 0;
    ROS_WARN("%s",res.msg.c_str());

    //Restore saved motor intensity and speed
    setMotorIntensity(savedMotorIntensity);
    setMotorSpeed(savedMotorSpeed);

    return false;
  }

  // If we did move, and got stuck, set that as the homing position.
  setHome();

  // Now that we know we're safe, increase the speed
  setMotorIntensity(CAL_FAST_FORCE);
  setMotorSpeed(CAL_FAST_SPEED);

  //Move to origin position
  int zeroMotor;
  node->getParam("/hand/zeroMotorPos",zeroMotor);
  setMotorPos(zeroMotor);
  //waitRest(0.250);  
  waitRest(1); 
  
  // Look at the current values of the motor and the finger. This is the
  // origin to use when calculating angles
  pthread_mutex_lock(&encMotorMutex);
  node->getParam("/hand/encMotorAtZeroAngle",encMotorAtZeroAngle);
  //encMotorZero = encMotor;
  pthread_mutex_unlock(&encMotorMutex);

  double stepsPerDegreeFingers;
  double stepsPerDegreeMotor;
  node->getParam("/hand/stepsPerDegreeMotor",stepsPerDegreeMotor);
  node->getParam("/hand/stepsPerDegreeFingers",stepsPerDegreeFingers);
  pthread_mutex_lock(&encFingersMutex);
  for (int i=0; i < NUM_FINGERS; i++)
  {
    encFingersAtZeroAngle[i] = encFingers[i];// + (int)(((double)encMotorAtZeroAngle/stepsPerDegreeMotor)*stepsPerDegreeFingers);
  }
  pthread_mutex_unlock(&encFingersMutex);
  calibrated = true;

  // Set up our reply to the user
  res.encMotor = encMotorAtZeroAngle;
  res.enc.resize(NUM_FINGERS);
  for (int i=0; i < NUM_FINGERS; i++)
  {
    res.enc[i] = encFingersAtZeroAngle[i];
  }
  res.ret = 1;

  //Restore saved motor intensity and speed
  setMotorIntensity(savedMotorIntensity);
  setMotorSpeed(savedMotorSpeed);

  // Finally, check that our new calibrated finger values 
  // are close to the previously calibrated values
  FILE *fCalib;
  fCalib = fopen(CAL_FILE_NAME, "r");
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
        if (fabs(fileEnc[i] - encFingersAtZeroAngle[i]) > MAX_CAL_DIFF)
        {
          ROS_WARN("******************************************************");
          ROS_WARN("HAND_CONTROLLER: Calibrated finger encoder position is "
              "not close to the previously calibrated value."
              " Has an encoder moved?");
          ROS_WARN("Finger %d: New: %d, Old: %d", 
              i+1, encFingersAtZeroAngle[i], fileEnc[i]);
          ROS_WARN("******************************************************");
        }
      }
    }
    fclose(fCalib);
  }

  //Save last calibration to a file
  ROS_INFO("HAND_CONTROLLER: Saving calibration to file.");
  fCalib = fopen(CAL_FILE_NAME,"w");
  fprintf(fCalib,"%d\n",encMotorAtZeroAngle);
  for (int i=0; i < NUM_FINGERS; i++)
  {
    fprintf(fCalib,"%d\n",encFingersAtZeroAngle[i]);
  }
  fclose(fCalib);
  return true;
}


//////////////////////////////////////////////////////////////////////////
// Access function for motor and finger encoders
bool HandController::hand_GetEncoders(hand_comm::hand_GetEncoders::Request& req, 
    hand_comm::hand_GetEncoders::Response& res)
{
  // Save the motor position
  pthread_mutex_lock(&encMotorMutex);
  res.encMotor = encMotor;
  pthread_mutex_unlock(&encMotorMutex);

  // Allocate space for the finger encoders
  res.encFinger.resize(NUM_FINGERS);

  pthread_mutex_lock(&encFingersMutex);
  for (int i=0; i < NUM_FINGERS; i++)
  {
    res.encFinger[i] = encFingers[i];
  }
  pthread_mutex_unlock(&encFingersMutex);

  res.ret = 1;
  return true;
}


//////////////////////////////////////////////////////////////////////////
// Access function for finger and motor angles for hand
bool HandController::hand_GetAngles(hand_comm::hand_GetAngles::Request& req, 
    hand_comm::hand_GetAngles::Response& res)
{
  // Only return values if the hand is calibrated. If not, these values mean
  //  nothing
  if(calibrated)
  {
    pthread_mutex_lock(&encMotorMutex);
    res.angleMotor = (encMotor - encMotorAtZeroAngle)/(double)stepsPerDegreeMotor;
    pthread_mutex_unlock(&encMotorMutex);

    // Allocate space for the angles message
    res.angle.resize(NUM_FINGERS);

    // Now compute the finger angles and save those as well
    double angles[NUM_FINGERS];
    getAngles(angles);
    for (int i=0; i < NUM_FINGERS; i++)
    {
      res.angle[i] = angles[i];
    }

    res.ret = 1;
    return true;
  }
  else
  {
    res.ret = 0;
    res.msg = "HAND_CONTROLLER: Hand not calibrated.";
    return false;
  }
}


//////////////////////////////////////////////////////////////////////////
// Access function for raw forces on hand
bool HandController::hand_GetRawForces(hand_comm::hand_GetRawForces::Request& req, 
    hand_comm::hand_GetRawForces::Response& res)
{
  // Allocate space for the raw forces
  res.forces.resize(NUM_RAW_HAND_FORCES);

  // Simply return the current raw forces experienced by the hand
  pthread_mutex_lock(&rawForcesMutex);
  for (int i=0; i < NUM_RAW_HAND_FORCES; i++)
  {
    res.forces[i] = rawForces[i];
  }
  pthread_mutex_unlock(&rawForcesMutex);
  res.ret = 1;
  return true;
}

//////////////////////////////////////////////////////////////////////////
// Access function for calibrated forces on hand
bool HandController::hand_GetForces(hand_comm::hand_GetForces::Request& req, 
    hand_comm::hand_GetForces::Response& res)
{
  // Allocate space in our message for the forces
  res.forces.resize(NUM_HAND_FORCES);

  // Compute the calibrated forces from the raw forces and save them
  double forces[NUM_HAND_FORCES];
  getForces(forces);
  for (int i=0; i < NUM_HAND_FORCES; i++)
  {
    res.forces[i] = forces[i];
  }

  res.ret = 1;
  return true;
}


//////////////////////////////////////////////////////////////////////////
// This function moves the motor to a certain encoder pose
bool HandController::hand_SetEncoder(hand_comm::hand_SetEncoder::Request& req, 
    hand_comm::hand_SetEncoder::Response& res)
{
  // Use our helper function to move the encoder to a certain encoder value
  if(setMotorPos(req.enc))
  {
    res.ret = 1;
    return true;
  }
  res.ret = 0;
  res.msg = "HAND_CONTROLLER: Not able to move the hand.";
  return false;
}

//////////////////////////////////////////////////////////////////////////
// This function moves the motor to a certain angle. If the hand is calibrated,
// 0 degrees is the fingers perpendicular to the hand, and 90 is the fingers
// touching the hand
bool HandController::hand_SetAngle(hand_comm::hand_SetAngle::Request& req, 
    hand_comm::hand_SetAngle::Response& res)
{
  // We can only use this service if the hand is calibrated, since if not,
  // "angle" wouldn't make sense.
  if(calibrated)
  {
    // Convert the requested angle into encoder counts
    int pose = req.angle*stepsPerDegreeMotor + encMotorAtZeroAngle;

    // Use our helper function to move the motor
    if(setMotorPos(pose))
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

//////////////////////////////////////////////////////////////////////////
// This service stops the hand
bool HandController::hand_SetRest(hand_comm::hand_SetRest::Request& req, 
    hand_comm::hand_SetRest::Response& res)
{
  // Use our helper function to stop the hand
  if(setRest())
  {
    res.ret = 1;
    return true;
  }
  res.ret = 0;
  res.msg = "HAND_CONTROLLER: Problem setting the hand to rest.";
  return false;
} 

//////////////////////////////////////////////////////////////////////////
// Blocking service waits for the hand to stop moving, and then returns
bool HandController::hand_WaitRest(hand_comm::hand_WaitRest::Request& req, 
    hand_comm::hand_WaitRest::Response& res)
{
  // Use our helper function with the requested delay to wait for the hand
  // to stop moving
  waitRest(req.delay);
  res.ret = 1;
  return true;
}

//////////////////////////////////////////////////////////////////////////
// Service to set the speed of the hand
bool HandController::hand_SetSpeed(hand_comm::hand_SetSpeed::Request& req, 
    hand_comm::hand_SetSpeed::Response& res)
{
  // If we are trying to set a speed outside of the accepted range, do
  // nothing and alert the user
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

  // Otherwise, use our helpfer function to set the speed of the hand
  if(!setMotorSpeed(req.speed))
  {
    res.ret = 0;
    res.msg = "HAND_CONTROLLER: Could not set the motor speed.";
    ROS_INFO("%s",res.msg.c_str());
    return false;
  }

  // If successful, save the cureent speed to ros params
  node->setParam("/hand/motorSpeed",req.speed);
  res.ret = 1;
  return true;
}

//////////////////////////////////////////////////////////////////////////
// Service to set the current limit of the hand
bool HandController::hand_SetForce(hand_comm::hand_SetForce::Request& req,
    hand_comm::hand_SetForce::Response& res)
{
  // If the force is outside the allowed range, ignore the command and alert
  // the user
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

  // Set the current limit using a helper function
  if(!setMotorIntensity(req.force))
  {
    res.ret = 0;
    res.msg = "HAND_CONTROLLER: Could not set the motor intensity.";
    ROS_INFO("%s",res.msg.c_str());
    return false;
  }
  
  // If successful, save the current force to the system parameters
  node->setParam("/hand/motorForce",req.force);
  res.ret = 1;
  return true;
}

//////////////////////////////////////////////////////////////////////////
// Service checks whether hand is moving or not
bool HandController::hand_IsMoving(hand_comm::hand_IsMoving::Request& req,
    hand_comm::hand_IsMoving::Response& res)
{
  if(connectedArduino)
  {
    // If we are connected to the arduino, simply return the "moving" variable,
    //  which is updated elsewhere
    res.moving = moving;
    res.ret = 1;
    return true;
  }
  else
  {
    // If we are not connected, we know nothing
    res.moving = false;
    res.ret = 0;
    res.msg = "HAND_CONTROLLER: Hand not connected to motor controller.";
    return false;
  }
}

//////////////////////////////////////////////////////////////////////////
// This function waits for delay1, then moves the motor to a angle,
// then waits for delay2 and finally moves the moves the motor to angle2

bool HandController::hand_DADA(hand_comm::hand_DADA::Request& req, 
    hand_comm::hand_DADA::Response& res)
{
  ros::Rate loop_rate(30.0);
  while (ros::ok() && non_block_action != HNB_NO_ACTION)
  {
    loop_rate.sleep();
  }
  non_block_data.angle1 = req.angle1;
  non_block_data.angle2 = req.angle2;
  non_block_data.delay1 = req.delay1;
  non_block_data.delay2 = req.delay2;
  non_block_action = HNB_DADA;
  res.ret = 1;
  return true;
}


//////////////////////////////////////////////////////////////////////////
// Load the previous calibration from file, if it exists
bool HandController::loadCalibration()
{
  FILE *fCalib;
  fCalib = fopen(CAL_FILE_NAME,"r");
  if(fCalib)
  {
    int aux = fscanf(fCalib,"%d\n",&(encMotorAtZeroAngle));
    if (aux != 1)
    {
      fclose(fCalib);
      return false;
    }
    else
    {
      for (int i=0; i < NUM_FINGERS; i++)
      {
        aux = fscanf(fCalib,"%d\n",&(encFingersAtZeroAngle[i]));
        if (aux != 1)
          break;
      }
      fclose(fCalib);
      ROS_INFO("HAND_CONTROLLER: ___________________WARNING___________________");
      ROS_INFO("HAND_CONTROLLER: Loading hand calibration from last execution.");
      calibrated = true;
      return true;
    }
  }
  return false;
}


//////////////////////////////////////////////////////////////////////////
// Helper function that connects to the arduino
bool HandController::connectArduino()
{
  // Get the arduino address and baud rate from the parameter file
  std::string arduinoId;
  int arduinoBaud;
  node->getParam("/hand/arduinoSerialId",arduinoId);
  node->getParam("/hand/arduinoSerialBaud",arduinoBaud);

  // If we were conected to the arduino, disconnect it and try again
  if(connectedArduino)	
    disconnectArduino();

  // Now connect to the arduino we want at the chosen baud rate
  ROS_INFO("Searching for arduino in %s with baud rate %d.",arduinoId.c_str(),arduinoBaud);
  spArduino = SerialPort(arduinoId.c_str(), arduinoBaud);
  spArduino.setBlocking(false);

  // According to the arduino documentation, after opening a serial port 
  //  in linux, the bootloader automatically runs. The documentation 
  //  suggests waiting 1 second before continuing, so you don't happen 
  //  to miss any start characters. Since we just opened the port, 
  //  we do this now.
  ros::Duration(2.0).sleep();

  // Change the mode of the hand to broadcasting. This will also let us 
  // confirm that we are in fact connected to the hand.
  char buffer[MAX_BUFFER];

  // Once the Arduino initializes, it sends a start character '!'.
  // If we can find this value, then we know that the hand has
  // just initialized. If we cannot find it, it means 1 of 2 things
  // either a) the hand was already initialized a long time ago, or
  // b) nothing is being outputed from the hand

  ROS_INFO("Looking for the start character on the serial port...");
  if(spArduino.readData(buffer,MAX_BUFFER,"!",2000)>0)
  {
    if (strchr(buffer, '!') != NULL)
    {
      ROS_INFO("Start character found.");
    }
    else
    {
      ROS_INFO("No start character found. The hand may have already "
          "been initialized before hand_node started.");  
    }
  }
  else
  {
    ROS_INFO("Couldn't read any data from the serial port. The hand "
        "may currently not be publishing any data.");
  }

  // Just because we didn't see the start character doesn't mean 
  // the hand is not connected. Let's try to set the hand to start
  // streaming data, and then we can be sure.
  sprintf(buffer,"mode 4#");
  pthread_mutex_lock(&serialOutMutex);
  spArduino.writeData(buffer);
  pthread_mutex_unlock(&serialOutMutex);
  ROS_INFO("Setting up Arduino to stream data...");
  
  // When a new mode is set, the following string is returned:
  // "set mode to : 4$" This is the only time a '$' character
  //  will be sent from the arduino. So, if after sending the
  //  mode change, if we find a '$' character, it means we're
  //  communicating with the hand correctly
  if(spArduino.readData(buffer,MAX_BUFFER,"$",2000)>0)
  {
    if (strchr(buffer, '$') != NULL)
    {
      ROS_INFO("Mode change acknowledged. We are ready to use the hand.");
      connectedArduino = true;
    }
    else
    {
      ROS_WARN("Mode change not acknowledged. "
          "Our hand is not functioning properly.");  
      connectedArduino = false;
    }
  }
  else
  {
    ROS_WARN("Couldn't read any data from the serial port. "
        "Our hand is not functioning properly. Current buffer: %s", buffer);
    connectedArduino = false;
  }
  return connectedArduino;
}


//////////////////////////////////////////////////////////////////////////
// Disconnect from the serial port associated with the arduino
bool HandController::disconnectArduino()
{
  if(connectedArduino)
    spArduino.closePort();
  connectedArduino = spArduino.isConnected();
  return (!connectedArduino);
}



//////////////////////////////////////////////////////////////////////////
// This function is called at a certain frequency. We read the serial port connection
//  from the arduino, update internal variables about the position of the fingers, motor,
//  and forces on the palm, and publish those updates to the ros network
//
// The messages are in the following form:
//  Motor message: #M1234&, where 1234 is the motor position. This number can be from 0 to 65535, and is not fixed width
//  Encoder message: #E123456789012& There are 3 4-digit numbers representing the 3 finger encoders. These numbers are fixed width, so this message will always be this length
//  Force message: #F123456789012& There are again 3 4-digit numbers representing the 3 raw palm forces. These numbers are fixed width, so this message will always be this length
//
// Our serial parsing strategy is as follows:
// 
// Step 1: All messages start with #. If there is anything in the buffer before that, skip past it.
// Step 2: If there is at least 1 more character after #, see what kind of message it is.
// Step 3: (Do this for each kind of message) Check if there is an '&' symbol. If so, parse the message. If not, we're done.
// Step 4: (If we parsed a message) Skip past the '&' character in our buffer. Go back to step 1.
// Step 5: Once we cannot find anymore complete messages, save what's left to a buffer that will be added on at the beginning, the next time this function is called
void HandController::logCallback(const ros::TimerEvent& event)
{
  if(connectedArduino)
  {
    int n;
    char buffer[MAX_BUFFER];
    if((n=spArduino.readData(buffer, MAX_BUFFER-1)) > 0)
    {
      // Add an end character to form our string
      buffer[n] = '\0';

      // If we are reading too much data, and we are going to overflow our
      // buffer, drop the old messages and use the newest messages.
      int offset = 0;
      if (curSerial + n + 1 > MAX_BUFFER)
      {
        offset = curSerial + n + 1 - MAX_BUFFER;
        ROS_WARN("Hand node is receiving too much data on the serial port"
            " Dropping older data. Please consider increasing the rate at"
            " which this function is called, or increase the buffer size.");
      }
      
      // First combine our old buffer with the most recent buffer. Make sure
      //  that if our buffer is going to overflow, we only look at some of 
      //  the old buffer.
      sprintf(serialBuffer, "%s%s", serialBuffer + offset, buffer);

      // Keep track of how many characters are in our serialBuffer. Note that
      //  this does not include the '\0' character, so at most our serialBuffer
      //  can hold MAX_BUFFER - 1 characters.
      curSerial += n - offset;

      // This buffer keeps track of where we are in our string
      char* partialBuffer = serialBuffer;

      // As long as there are more messages to be read, 
      //  this variable stays true
      bool more_messages = true;

      while(more_messages)
      {
        // If we cannot find a '#' character, the beginning of a message, we're done.
        if (strchr(partialBuffer, '#') == NULL)
        {
          more_messages = false;
        }
        else if(strchr(partialBuffer, '#') != partialBuffer)
        {
          // If the first character in our string is not a '#' character, skip up to it.

          // find the location of the start of the message
          char *lastChar = strchr(partialBuffer, '#');
          // remember how much less we need to keep for next time
          curSerial -= (lastChar - partialBuffer);
          // Move the pointer to the '#' character
          partialBuffer = lastChar;
        }
        else if (curSerial <= 1 || strchr(partialBuffer, '&') == NULL)
        {
          // If we have exactly 1 character, which is the '#' symbol, we're done.
          // If we did not find the '&' character, the message is not complete, so we're done.
          more_messages = false;
        }
        else 
        {
          // If we made it here, we are certain that we have a full message.
          //  This message starts with #, then an identifying character, 
          //  then numbers, and finally, an '&' symbol.
          
          char msg_type = partialBuffer[1];
          switch (msg_type)
          {
            // Motor Message
            case 'M':
              {
                // Save this value
                pthread_mutex_lock(&encMotorMutex);
                sscanf(partialBuffer,"#M%d&", &encMotor);
                pthread_mutex_unlock(&encMotorMutex);

                // Check to see if we're still moving by looking at 
                //  a history of motor values
                checkMoving();

                break;
              }

            case 'E':
              {
                // #E123456789012&
                // encBuf is exactly the size of the fixed number stream 
                //  (#fingers * width of each number)
                char encBuf[NUM_FINGERS * ENC_DATA_WIDTH];
                
                // Copy all of the numbers into encBuf. Note that we start 
                // reading at partialBuffer+2 because we don't want to read 
                // the '#' or 'E' character. 
                strncpy(encBuf, partialBuffer+2, ENC_DATA_WIDTH*NUM_FINGERS);

                // Update our private finger encoder variables
                pthread_mutex_lock(&encFingersMutex);
                for (int i=0; i < NUM_FINGERS; i++)
                {
                  // Copy ENC_DATA_WIDTH digits into encString
                  char encString[ENC_DATA_WIDTH+1];
                  strncpy(encString, encBuf+(i*ENC_DATA_WIDTH), ENC_DATA_WIDTH);
                  encString[ENC_DATA_WIDTH] = '\0';

                  // Convert the string into a number and save it
                  sscanf(encString, "%d", &encFingers[i]);
                }
                pthread_mutex_unlock(&encFingersMutex);

                // Now that we have gotten an updated encoder message, publish it
                hand_comm::hand_EncodersLog msgEncoders;
                hand_comm::hand_AnglesLog msgAngles;

                msgEncoders.encFinger.resize(NUM_FINGERS);
                msgAngles.angle.resize(NUM_FINGERS);

                msgEncoders.timeStamp = event.current_real.toSec();
                msgAngles.timeStamp = event.current_real.toSec();

                // First save the motor position
                pthread_mutex_lock(&encMotorMutex);
                msgEncoders.encMotor = encMotor;
                if (calibrated)
                  msgAngles.angleMotor = (encMotor - encMotorAtZeroAngle)/(double)stepsPerDegreeMotor;
                pthread_mutex_unlock(&encMotorMutex);


                // Now the finger encoder position
                pthread_mutex_lock(&encFingersMutex);
                for (int i=0; i < NUM_FINGERS; i++)
                {
                  msgEncoders.encFinger[i] = encFingers[i];
                }
                pthread_mutex_unlock(&encFingersMutex);

                // Now compute the finger angles and save those as well
                double angles[NUM_FINGERS];
                getAngles(angles);
                for (int i=0; i < NUM_FINGERS; i++)
                {
                  msgAngles.angle[i] = angles[i];
                }

                // Finally, actually publish the message
                handle_hand_EncodersLog.publish(msgEncoders);
                if(calibrated)
                  handle_hand_AnglesLog.publish(msgAngles);
                break;
              }

            case 'F':
              {
                // #F123456789012&
                // forceBuf is exactly the size of the fixed number stream 
                //  (#force sensors * width of each number)
                char forceBuf[NUM_RAW_HAND_FORCES * FORCE_DATA_WIDTH];

                // Copy all of the numbers into forceBuf. Note that we start 
                // reading at partialBuffer+2 because we don't want to read 
                // the '#' or 'F' character. 
                strncpy(forceBuf, partialBuffer+2, FORCE_DATA_WIDTH*NUM_RAW_HAND_FORCES);

                // Update our private raw force values
                pthread_mutex_lock(&rawForcesMutex);
                for (int i=0; i < NUM_RAW_HAND_FORCES; i++)
                {
                  // Copy FORCE_DATA_WIDTH digits into forceString
                  char forceString[FORCE_DATA_WIDTH+1];
                  strncpy(forceString, forceBuf+(i*FORCE_DATA_WIDTH), FORCE_DATA_WIDTH);
                  forceString[FORCE_DATA_WIDTH] = '\0';

                  // Convert the string into a number and save it
                  sscanf(forceString, "%d", &rawForces[i]);
                }
                pthread_mutex_unlock(&rawForcesMutex);

                // Now that we have gotten an updated force message, publish it
                hand_comm::hand_RawForcesLog msgRawForces;
                hand_comm::hand_ForcesLog msgForces;

                msgRawForces.forces.resize(NUM_RAW_HAND_FORCES);
                msgForces.forces.resize(NUM_HAND_FORCES);

                msgRawForces.timeStamp = event.current_real.toSec();
                msgForces.timeStamp = event.current_real.toSec();

                // First save the raw forces
                pthread_mutex_lock(&rawForcesMutex);
                for (int i=0; i < NUM_RAW_HAND_FORCES; i++)
                {
                  msgRawForces.forces[i] = rawForces[i];
                }
                pthread_mutex_unlock(&rawForcesMutex);

                // Now compute and save the calibrated forces as well
                double forces[NUM_HAND_FORCES];
                getForces(forces);
                for (int i=0; i < NUM_HAND_FORCES; i++)
                {
                  msgForces.forces[i] = forces[i];
                }
   
                // Finally, actually publish this message
                handle_hand_RawForcesLog.publish(msgRawForces);
                handle_hand_ForcesLog.publish(msgForces);
                break;
              }
            default:
              ROS_WARN("HAND_NODE: Unexpected character after #: %c", msg_type);
              more_messages = false;
          }     

          // Move our pointer to just past the ending '&' character
          if (more_messages)
          {
            char *lastChar = strchr(partialBuffer, '&') + 1;
            curSerial -= (lastChar - partialBuffer);
            partialBuffer = lastChar;
          }
        }
      }

      // Save any characters we haven't used yet for the next message
      strncpy(serialBuffer, partialBuffer, curSerial);
      serialBuffer[curSerial] = '\0';
    }
  }
}

//////////////////////////////////////////////////////////////////////////
// Main loop that reads in messages from the hand. Create a timer event
// that calls a function to read the serial port at a given rate
void *loggerMain(void *args)
{
  //Recover the pointer to the main node
  HandController* P2hand;
  P2hand = (HandController*) args;

  // Setup the timer event and connect it to our callback function
  ros::Timer loggerTimer;
  loggerTimer = P2hand->node->createTimer(ros::Duration(LOGGER_THREAD_RATE), &HandController::logCallback, P2hand);

  // Now wait until this node has been killed
  ros::waitForShutdown();
  loggerTimer.stop();
  return NULL;
}

///////////////////////////////////////////////////////////////////////////
// Main loop that executes non-blocking functions
// In this separate thread, we constantly check to see if any non-blocking 
// actions have been requested, and if so, we execute them, and then mark 
// the action as complete.
void *nonBlockMain(void *args)
{
  // Recover the pointer to the main node
  HandController* P2hand;
  P2hand = (HandController*) args;
  
  ros::Rate loop_rate(30.0);
  
  // Loop forever
  while (ros::ok())
  {
    // Check to see if we have any non-blocking actions to execute
    switch(P2hand->non_block_action)
    {
      case HNB_DADA:
        // delay-angle-delay-angle action.
        P2hand->dada(P2hand->non_block_data.delay1, P2hand->non_block_data.angle1,
          P2hand->non_block_data.delay2, P2hand->non_block_data.angle2);
        // Once complete, we are no longer executing a non-blocking action
        P2hand->non_block_action = HNB_NO_ACTION;
        break;
      case HNB_NO_ACTION:
      default:
        break;
    }
    loop_rate.sleep();
  }
  
  return NULL;
}

//////////////////////////////////////////////////////////////////////////
// Main loop. Initializes the hand node, forks a process to read in sensor
// data, advertises services and messages, and waits for shutdown
int main(int argc, char** argv)
{
  ros::init(argc, argv, "hand_controller");
  ros::NodeHandle node;
  HandController P2hand(&node);

  //Connect to Arduino
  ROS_INFO("HAND_CONTROLLER: Connecting to the Arduino...");
  if(!P2hand.connectArduino())
  {
    ROS_WARN("HAND_CONTROLLER: Impossible to connect to Arduino.");	
    ROS_WARN("HAND_CONTROLLER: Exit.");
    return 1;
  }

  //Default hand configuration
  ROS_INFO("HAND_CONTROLLER: Setting default hand configuration...");
  if(!P2hand.defaultHandConfiguration())
  {
    ROS_INFO("HAND_CONTROLLER: Not able to set the robot to default configuration.");
    exit(-1);
  }
  
  // Load previous calibration from file, if it exists
  P2hand.loadCalibration();

  //Advertise ROS services
  ROS_INFO("HAND_CONTROLLER: Advertising ROS services...");
  P2hand.advertiseServices();

  //Advertise ROS topics
  ROS_INFO("HAND_CONTROLLER: Advertising ROS topics...");
  P2hand.advertiseTopics();

  // Initialize Mutexes
  pthread_mutex_init(&encMotorMutex, NULL);
  pthread_mutex_init(&encFingersMutex, NULL);
  pthread_mutex_init(&rawForcesMutex, NULL);
  pthread_mutex_init(&serialOutMutex, NULL);

  // Topic broadcast performed in a dedicated thread
  pthread_t loggerThread;
  pthread_attr_t attr;
  pthread_attr_init(&attr);
  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
  if(pthread_create(&loggerThread,  &attr, loggerMain, (void*)&P2hand) != 0)
    ROS_INFO("HAND_CONTROLLER: Unable to create logger thread. Error number: %d.",errno);
    
  // Thread used to handle non-blocking commands
  pthread_t nonBlockThread;
  pthread_attr_t attr_nb;
  pthread_attr_init(&attr_nb);
  pthread_attr_setdetachstate(&attr_nb, PTHREAD_CREATE_JOINABLE);
  if(pthread_create(&nonBlockThread,  &attr_nb, nonBlockMain, (void*)&P2hand) != 0)
    ROS_INFO("HAND_CONTROLLER: Unable to create non blocking thread. Error number: %d.",errno);


  //Main ROS loop
  ROS_INFO("HAND_CONTROLLER: Running node /hand_controller...");
  //Multithreaded spinner so that callbacks can be handled on separate threads.
  ros::MultiThreadedSpinner spinner(2); // Use 2 threads 
  spinner.spin();
  ROS_INFO("HAND_CONTROLLER: Shutting down node /hand_controller...");

  //End log dedicated thread
  void *status;
  pthread_attr_destroy(&attr);

  pthread_mutex_destroy(&encMotorMutex);
  pthread_mutex_destroy(&encFingersMutex);
  pthread_mutex_destroy(&rawForcesMutex);
  pthread_mutex_destroy(&serialOutMutex);

  pthread_join(loggerThread, &status);
  pthread_join(nonBlockThread, &status);

  ROS_INFO("HAND_CONTROLLER: Done.");
  return 0;
}



//////////////////////////////////////////////////////////////////////////
// Set the speed of the motor on the arduino
//
// The speed is set by sending the command: "speed 98#", where 98 is a number 
// between 0 and 100, 0 is slowest and 100 is fastest. 
//
// The input for this function is a number between 0.0 and 1.0
//
bool HandController::setMotorSpeed(double mSpeed)
{
  // Only change the speed if it's different from our current speed
  if(mSpeed != motorSpeed)
  {
    // Only change the speed if we're connected to the hand
    if(connectedArduino)
    {
      // Make sure our speed is between 0 and 1
      if (mSpeed > 1.0)
      {
        ROS_WARN("Commanded speed of %1.2f is out of range. Truncating", 
            mSpeed);
        mSpeed = 1.0;
      }
      else if (mSpeed < 0.0)
      {
        ROS_WARN("Commanded speed of %1.2f is out of range. Truncating", 
            mSpeed);
        mSpeed = 0.0;
      }

      // Actually send the command to the hand
      char buffer[MAX_BUFFER];
      sprintf(buffer,"speed %d#",(int)(mSpeed*100));
      pthread_mutex_lock(&serialOutMutex);
      spArduino.writeData(buffer);
      pthread_mutex_unlock(&serialOutMutex);
      motorSpeed = mSpeed;
      return true;
    }
    return false;
  }
  return true;
}

//////////////////////////////////////////////////////////////////////////
// Set the current limit of the hand
//
// The current limit is set by sending the command: "current 98#", 
// where 98 is a number between 0 and 100, 0 is slowest and 100 is fastest. 
//
// The input for this function is a number between 0.0 and 1.0
//

bool HandController::setMotorIntensity(double mIntensity)
{
  // If the current limit we want to set is already the current setting,
  // do nothing
  if(mIntensity != motorIntensity)
  {
    // Only set the current if we are connected to the arduino
    if(connectedArduino)
    {
      // Make sure we send a valid speed command.
      if (mIntensity > 1.0)
      {
        ROS_WARN("Commanded current limit of %1.2f is out of range. Truncating", 
            mIntensity);
        mIntensity = 1.0;
      }
      else if (mIntensity < 0.0)
      {
        ROS_WARN("Commanded current limit of %1.2f is out of range. Truncating", 
            mIntensity);
        mIntensity = 0.0;
      }

      // Set the current limit
      char buffer[MAX_BUFFER];
      sprintf(buffer,"current %d#",(int)(mIntensity*100));
      pthread_mutex_lock(&serialOutMutex);
      spArduino.writeData(buffer);
      pthread_mutex_unlock(&serialOutMutex);
      motorIntensity = mIntensity;
      return true;
    }
    return false;
  }
  return true;
}

//////////////////////////////////////////////////////////////////////////
// Move the motor to a specific encoder count
//
// We move the motor to a specific encoder count by sending the command: 
// "tar 1234#", where 1234 is the desired encoder count
//
// The input for this function is a number between -32768 and 32767
//
bool HandController::setMotorPos(int motorPos)
{
  // Only send a command if we're connected to the motor
  if(connectedArduino)
  {
    // Make sure we send a valid position
   
    if (motorPos > MAX_MOTOR_POS)
    {
      ROS_WARN("Commanded hand pose of %d is out of range. Truncating", 
          motorPos);
      motorPos = MAX_MOTOR_POS;
    }
    else if (motorPos < MIN_MOTOR_POS)
    {
      ROS_WARN("Commanded hand pose of %d is out of range. Truncating", 
          motorPos);
      motorPos = MIN_MOTOR_POS;
    }
      
    printf("commanded position: %d\n", motorPos);

    // Actually send the command to the arduino
    char buffer[MAX_BUFFER];
    sprintf(buffer, "tar %d#", motorPos);
    pthread_mutex_lock(&serialOutMutex);
    spArduino.writeData(buffer);
    pthread_mutex_unlock(&serialOutMutex);
    return true;
  }      
  return false;
}

//////////////////////////////////////////////////////////////////////////
// This function stops the hand.
// 
// This is done by sending the command "stop#"
//
bool HandController::setRest()
{
  // Only send the command if we are connected
  if(connectedArduino)
  {
    // Tell the robot to stop
    char buffer[MAX_BUFFER];
    sprintf(buffer,"float#");
    pthread_mutex_lock(&serialOutMutex);
    spArduino.writeData(buffer);
    pthread_mutex_unlock(&serialOutMutex);
    return true;
  }
  return false;
}

//////////////////////////////////////////////////////////////////////////
// Reset the motor encoder count to 0. Useful for calibration
//
// This is done by sending the command "home #"
//
bool HandController::setHome()
{
  // Only send the command if we are connected
  if(connectedArduino)
  {
    // Reset the motor encoder count
    char buffer[MAX_BUFFER];
    sprintf(buffer,"home#");
    pthread_mutex_lock(&serialOutMutex);
    spArduino.writeData(buffer);
    pthread_mutex_unlock(&serialOutMutex);
    return true;
  }
  return false;
}

//////////////////////////////////////////////////////////////////////////
// This is a blocking function that waits until the hand has stopped moving. 
// This will occur either if the hand completes a move, or if it gets 
// stuck by grasping hard on an object or calibrating.
//
// The function takes as input a delay time in seconds. The function will 
// wait for that time before checking if the hand is moving. This is very 
// useful if you want to check if the hand is done immediately after 
// commanding a move. It take a few cycles for the hand to react, and so
// the variable "moving" will not instantly be set to true, and so glitching
// behavior could occur. 
// 
bool HandController::waitRest(double delay)
{
  // Sleep for the requested time
  ros::Duration(delay).sleep();

  ros::Rate loop_rate(1.0 / MIN_DELAY);

  // Stay in this loop until the hand has stopped moving
  while (ros::ok() && moving)
  {
    loop_rate.sleep();
  }
  return true;
}

//////////////////////////////////////////////////////////////////////////
// This function updates the value of the "moving" variable
//
// It is a helper function, and is generally called whenever a new
// motor encoder message is received from the hand. We determine if
// the motor is moving or not by looking at the standard deviation
// of the last MOT_BUF motor encoder samples. If it is above some 
// threshold, which is related to the current speed of the motor,
// then the hand is moving. If it is below, then most of the samples
// are similar values, so it is presumably not moving
void HandController::checkMoving()
{
  // Read the current motor position and save it to our ring buffer
  pthread_mutex_lock(&encMotorMutex);
  motBuf[bufCnt] = encMotor;
  pthread_mutex_unlock(&encMotorMutex);

  // Update the index of our ring buffer
  bufCnt++;
  if (bufCnt >= MOT_BUF)
  {
    // If we get to the end of the array, go to the beginning
    bufFull = true;
    bufCnt = 0;
  }

  // Compute the average over the last MOT_BUF samples
  int m=0;
  for (int j=0; j<MOT_BUF; j++)
    m+=motBuf[j];
  m/=(double)MOT_BUF;

  // Now compute the variance over the last MOT_BUF samples
  int v=0;
  for (int j=0; j<MOT_BUF; j++)
    v+=(motBuf[j] - m) * (motBuf[j] - m);
  v/=(double)(MOT_BUF-1);

  // Based on the current speed, compute the expected encoder counts
  // per second if the motor was moving
  double cnts_per_sec = CNTS_PER_SEC_M * motorSpeed + CNTS_PER_SEC_B;
  
  // We know what rate the arduino is publishing the motor encoder value,
  // so now, compute the change in encoder counts we expect between readings
  double cnts_per_rdg = cnts_per_sec * DATA_RATE;

  // If the variance is small enough, the motor is not moving
  if(bufFull && sqrt(v) < MOVING_FACTOR * cnts_per_rdg)
    moving = false;
  else
    moving = true;
}

//////////////////////////////////////////////////////////////////////////
// Compute the calibrated forces from raw forces
void HandController::getForces(double forces[NUM_HAND_FORCES])
{
  // Compute the calibrated forces by doing a matrix multiplication
  // with raw forces as input
  pthread_mutex_lock(&rawForcesMutex);
  for (int i=0; i < NUM_HAND_FORCES; i++)
  {
    double sum = 0;
    for (int j=0; j < NUM_RAW_HAND_FORCES; j++)
    {
      sum += force_cal[i][j] * rawForces[j];
    }
    forces[i] = sum;
  }
  pthread_mutex_unlock(&rawForcesMutex);
}

//////////////////////////////////////////////////////////////////////////
// Compute the angles of each finger
void HandController::getAngles(double angles[NUM_FINGERS])
{
  pthread_mutex_lock(&encFingersMutex);
  for (int i=0; i < NUM_FINGERS; i++)
  {
    angles[i] = (encFingers[i] - encFingersAtZeroAngle[i])/(double)stepsPerDegreeFingers;
    // Make sure our angles are between -180 and 180
    if (angles[i] < -180.0)
      angles[i] += 360.0;
    else if (angles[i] > 180.0)
      angles[i] -= 360.0;
  }
  pthread_mutex_unlock(&encFingersMutex);
}


bool HandController::dada(double delay1, double angle1, double delay2, double angle2)
{
  if(calibrated)
  {
    // Convert the requested angles into encoder counts
    int pose1 = angle1*stepsPerDegreeMotor + encMotorAtZeroAngle;
    int pose2 = angle2*stepsPerDegreeMotor + encMotorAtZeroAngle;

    // Use our helper function to move the motor
    
    ros::Duration(delay1).sleep();
    
    if(setMotorPos(pose1))
    {
      ros::Duration(delay2).sleep();
      if(setMotorPos(pose2))
      {
        return true;
      }
    }
    return false;
  }
  return false;
}


