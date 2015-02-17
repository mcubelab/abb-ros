#include "P2Interpreter.h"

P2Interpreter::P2Interpreter()
{
  connectedArduino = false;
  connectedMotor = false;
  calibrating = false;
  moving = false;
  bufFull = false;
  bufCnt = 0;
  for (int i=0; i<NSAMPLES; i++)
    motBuf[i] = 0;
  motorSpeed = 100;
  motorIntensity = 600; 

  for (int i=0; i < NUM_FINGERS; i++)
    encFingers[i] = (MAX_WINDOW + MIN_WINDOW)/2;

  //Thread to capture the motor encoder
  pthread_attr_init(&motorLoggerThreadAttr);
  pthread_create(&motorLoggerThread,  &motorLoggerThreadAttr, motorLoggerMain, (void*)this);

  //Thread to capture the finger encoders
  pthread_attr_init(&fingersLoggerThreadAttr);
  pthread_create(&fingersLoggerThread,  &fingersLoggerThreadAttr, fingersLoggerMain, (void*)this);  
}

bool P2Interpreter::connectArduino(const char * serialAddressArduino, const int baudRate)
{
  char buffer[MAX_BUFFER];
  if(connectedArduino)	
    disconnectArduino();
  spArduino = SerialPort(serialAddressArduino,baudRate);
  printf("Just opened the serial port\n");
  sleep(2.0);
  spArduino.setBlocking(false);

  //Check wether the serial port is connected to the Arduino
  sprintf(buffer,"001#");
  spArduino.flushPort();
  spArduino.writeData(buffer);
  printf("Send initial string to Arduino.\n");
  if(spArduino.readData(buffer,MAX_BUFFER,'#',1000)>0)
  {
    printf("--%s--\n",buffer);
    int aux;
    if(sscanf(buffer,"%d",&aux))
      connectedArduino = true;
    else
    {
      connectedArduino = false; 
      printf("YESSSS\n");  
     }
  }
  else
  {
    connectedArduino = false;
    printf("NOOOOOO %s\n",buffer);
  }
  return connectedArduino;
}

bool P2Interpreter::connectMotor(const char * serialAddressMotor, const int baudRate)
{
  char buffer[MAX_BUFFER];

  if(connectedMotor)
    disconnectMotor();
  spMotor = SerialPort(serialAddressMotor,baudRate);
  //Before sending commands, we need to wait for the Motor controller to end up configuring the serial port.
  sleep(2.0);
  spMotor.setBlocking(false);

  //Check wether the serial port is connected to the Motor
  sprintf(buffer,"\nANSW0\n");
  spMotor.writeData(buffer);
  sprintf(buffer,"\nPOS\n");
  spMotor.flushPort();
  spMotor.writeData(buffer);
  if(spMotor.readData(buffer,MAX_BUFFER,'\n',1000)>0)
  {
    int aux;
    if(sscanf(buffer,"%d",&aux))
      connectedMotor = true;
    else
      connectedMotor = false;
  }
  else
    connectedMotor = false;				      

  //Basic configuration of the motor
  if(connectedMotor)
  {
    char buffer[MAX_BUFFER];
    //No asynchronous responses, neither confirmation commands
    sprintf(buffer,"\nANSW0\n");
    spMotor.writeData(buffer);
    //Set motor Speed and Intensity
    setMotorIntensity(motorIntensity);
    setMotorSpeed(motorSpeed);
  }
  return connectedMotor;
}

bool P2Interpreter::disconnectArduino()
{
  if(connectedArduino)
    spArduino.closePort();
  connectedArduino = spArduino.isConnected();
  return (!connectedArduino);
}

bool P2Interpreter::disconnectMotor()
{
  if(connectedMotor)
  {
    char toSend[MAX_BUFFER];
    sprintf(toSend,"\nDI\n");
    spMotor.writeData(toSend);
    sleep(1.0);
    spMotor.closePort();
  }
  connectedMotor = spMotor.isConnected();
  return (!connectedMotor);
}

bool P2Interpreter::setMotorSpeed(const int mSpeed)
{
  if(mSpeed != motorSpeed)
  {
    char buffer[MAX_BUFFER];
    if(connectedMotor)
    {
      sprintf(buffer,"\nSP%d\n",mSpeed);
      spMotor.writeData(buffer);
      motorSpeed = mSpeed;
      bufCnt = 0;
      bufFull = false;
      return true;
    }
    return false;
  }
  return true;
}

bool P2Interpreter::setMotorIntensity(const int mIntensity)
{
  if(mIntensity != motorIntensity)
  {
    char buffer[MAX_BUFFER];
    if(connectedMotor)
    {
      //Set the  max peack intensity
      sprintf(buffer,"\nLPC%d\n",(int)(1.5*(double)mIntensity));
      spMotor.writeData(buffer);
      //Set the mac continous intensity
      sprintf(buffer,"\nLCC%d\n",mIntensity);
      spMotor.writeData(buffer);
      motorIntensity = mIntensity;

      return true;
    }
    return false;
  }
  return true;
}

bool P2Interpreter::setMotor(int motorPos)
{
  char buffer[MAX_BUFFER];
  if(connectedMotor)
  {
    //Enable motor
    sprintf(buffer,"\nEN\n");
    spMotor.writeData(buffer);

    sprintf(buffer,"\nSP%d\n",motorSpeed);
    spMotor.writeData(buffer);

    //Set new position
    sprintf(buffer,"\nLA%d\n",motorPos);
    spMotor.writeData(buffer);

    //Move
    sprintf(buffer,"\nM\n");
    spMotor.writeData(buffer);
    return true;
  }      
  return false;
}


bool P2Interpreter::getMotor(int *encoder)
{
  if(connectedMotor)
  {
    *encoder = encMotor;
    return true;
  }
  return false;
}

bool P2Interpreter::getFinger(int *encoder, int fingerId)
{
  if(connectedArduino)
  {
    if((fingerId >= 0) && (fingerId < NUM_FINGERS))
    {
      *encoder = encFingers[fingerId];
      return true;
    }
  }
  return false;
}

bool P2Interpreter::isMoving(bool *mov)
{
  if(connectedMotor)
  {
    *mov = moving;
    return true;
  }
  return false;
}

bool P2Interpreter::setRest()
{
  char buffer[MAX_BUFFER];
  if(connectedMotor)
  {
    sprintf(buffer,"\nDI\n");
    spMotor.writeData(buffer);
    return true;
  }
  return false;
}

bool P2Interpreter::setHome()
{
  char buffer[MAX_BUFFER];
  if(connectedMotor)
  {
    sprintf(buffer,"\nHO\n");
    spMotor.writeData(buffer);
    return true;
  }
  return false;
}


P2Interpreter::~P2Interpreter(void)
{
  //Close communication
  disconnectArduino();
  disconnectMotor();

  //End log dedicated threads
  pthread_attr_destroy(&motorLoggerThreadAttr);
  pthread_attr_destroy(&fingersLoggerThreadAttr);
  pthread_kill(motorLoggerThread, SIGTERM);
  pthread_kill(fingersLoggerThread, SIGTERM);
}

void* fingersLoggerMain(void *args)
{
  //Recover the pointer to the hand
  P2Interpreter* hand;
  hand = (P2Interpreter*) args;

  //Log the motorPose at 1/SAMPLING_TIME Hz
  clock_t iniTime;
  char buffer[MAX_BUFFER];
  char *partialBuffer;
  int aux;
  //bool cal;
  int n;
  int encoderId;

  while(1)
  {
    iniTime = clock();
    if(hand->connectedArduino)
    {
      //cal = hand->calibrating;
      // Process all messages pending in the serial port
      if ((n=hand->spArduino.readData(buffer, MAX_BUFFER-1)) > 0) 
      {
        // Add an end character to form our string
        buffer[n] = '\0';
        partialBuffer = buffer;
        //printf("New: %s\n",buffer);
        // Each message ends with a '#' character. 
        // Read messages one at a time until that # (not including it).
        while((partialBuffer = strchr(partialBuffer,'@'))!=NULL)		
        {
          if((partialBuffer != buffer)&&(strchr(partialBuffer,'#')!=NULL))
          {
            partialBuffer--;
            // The number after the start character is the type of message
            int nParams = sscanf(partialBuffer,"%d@%d#", &encoderId,&aux);
            //printf("nParams = %d\n",nParams);
            if(nParams == 2)
            {
              //printf("%s\n",partialBuffer);
              if (aux != NO_ENC_VAL)
              {
                // Only change this encoder count if it's in an expected range
                if ((aux >= enc_limits[encoderId][0]) && (aux <= enc_limits[encoderId][1]))
                {
                  hand->encFingers[encoderId] = aux;
                }
                // If we wrap around, we make sure the 
                // encoder value doesn't change direction
                // (ignore this if we're calibrating)
                /*
                   if (cal)
                   {
                // If we're calibrating, we would like the encoder values to not differ
                // wildly from one time to the next. So, out of the 3 possible encoder
                // values (below 1, between 1 and 4096, above 4096), we choose the one
                // that is closest to the approximate calibration location
                double d_aux_neg = fabs(aux - MAX_PULSE - calibration_reference[encoderId]);
                double d_aux_pos = fabs(aux + MAX_PULSE - calibration_reference[encoderId]);
                double d_aux = fabs(aux - calibration_reference[encoderId]);

                if (d_aux <= d_aux_neg && d_aux <= d_aux_pos)
                hand->encFingers[encoderId] = aux;
                else if (d_aux_pos <= d_aux && d_aux_pos <= d_aux_neg)
                hand->encFingers[encoderId] = aux + MAX_PULSE;
                else
                hand->encFingers[encoderId] = aux - MAX_PULSE;
                }
                else if (hand->encFingers[encoderId] <= 0 && aux >= MIN_WINDOW)
                hand->encFingers[encoderId] = aux - MAX_PULSE;
                else if (hand->encFingers[encoderId] > MAX_PULSE && aux <= MAX_WINDOW)
                hand->encFingers[encoderId] = aux + MAX_PULSE;
                else if (hand->encFingers[encoderId] < MIN_WINDOW && aux > MAX_WINDOW)
                hand->encFingers[encoderId] = aux - MAX_PULSE;
                else if (hand->encFingers[encoderId] > MAX_WINDOW && aux < MIN_WINDOW)
                hand->encFingers[encoderId] = aux + MAX_PULSE;
                else
                hand->encFingers[encoderId] = aux;
                 */
              }    

            }
            // Increment partialBuffer, so we don't look at the same message again
            partialBuffer+=2;
          }
          else
            partialBuffer++;
        }
      }
    } 
    /*for(int i=0; i<NUM_FINGERS; i++)
      {
      sprintf(toSend,"00%d#",i+1);
      hand->spArduino.flushPort();
      hand->spArduino.writeData(toSend);
      if(hand->spArduino.readData(received,MAX_BUFFER,'#',SAMPLING_TIME*1000)>0)
      {
      if(sscanf(received,"%d",&aux))
      {
    // Only save this value if there wasn't an error in reading the encoder
    if (aux != NO_ENC_VAL)
    {
    // Make sure that if we wrap around, we make sure the 
    //  encoder value doesn't change direction (ignore this if we're calibrating)
    if (cal)
    {
    // If we're calibrating, we would like the encoder values to not differ
    // wildly from one time to the next. So, out of the 3 possible encoder
    // values (below 1, between 1 and 4096, above 4096), we choose the one
    // that is closest to the approximate calibration location
    double d_aux_neg = fabs(aux - MAX_PULSE - calibration_reference[i]);
    double d_aux_pos = fabs(aux + MAX_PULSE - calibration_reference[i]);
    double d_aux = fabs(aux - calibration_reference[i]);

    if (d_aux <= d_aux_neg && d_aux <= d_aux_pos)
    hand->encFingers[i] = aux;
    else if (d_aux_pos <= d_aux && d_aux_pos <= d_aux_neg)
    hand->encFingers[i] = aux + MAX_PULSE;
    else
    hand->encFingers[i] = aux - MAX_PULSE;
    }
    else if (hand->encFingers[i] <= 0 && aux >= MIN_WINDOW)
    hand->encFingers[i] = aux - MAX_PULSE;
    else if (hand->encFingers[i] > MAX_PULSE && aux <= MAX_WINDOW)
    hand->encFingers[i] = aux + MAX_PULSE;
    else if (hand->encFingers[i] < MIN_WINDOW && aux > MAX_WINDOW)
    hand->encFingers[i] = aux - MAX_PULSE;
    else if (hand->encFingers[i] > MAX_WINDOW && aux < MIN_WINDOW)
    hand->encFingers[i] = aux + MAX_PULSE;
    else
    hand->encFingers[i] = aux;
    }
    }
    }*/

    //Small sleep
    double duration = (double)(clock() - iniTime)/(double)CLOCKS_PER_SEC;
    if(duration<SAMPLING_TIME)
    {
      struct timespec req = {0,0};
      req.tv_sec = 0;
      req.tv_nsec = (int)(1000000000L * (SAMPLING_TIME - duration));
      nanosleep(&req, (timespec *)NULL);
    }
  }
  return NULL;
  }


  void* motorLoggerMain(void *args)
  {
    //Recover the pointer to the hand
    P2Interpreter* hand;
    hand = (P2Interpreter*) args;

    //Log the motorPose at 1/SAMPLING_TIME Hz
    clock_t iniTime;
    char toSend[MAX_BUFFER];
    char received[MAX_BUFFER];
    int aux;
    while(1)
    {
      iniTime = clock();
      if(hand->connectedMotor)
      {
        sprintf(toSend,"\nPOS\n");
        hand->spMotor.flushPort();
        hand->spMotor.writeData(toSend);
        if(hand->spMotor.readData(received,MAX_BUFFER,'\n',SAMPLING_TIME*1000)>0)
        {
          if(sscanf(received,"%d",&aux))
          {
            hand->encMotor = aux;
            hand->checkMoving();
          }
        }
      }
      double duration = (double)(clock() - iniTime)/(double)CLOCKS_PER_SEC;
      if(duration<SAMPLING_TIME)
      {
        struct timespec req = {0,0};
        req.tv_sec = 0;
        req.tv_nsec = (int)(1000000000L * (SAMPLING_TIME - duration));
        nanosleep(&req, (timespec *)NULL);
      }
    }
    return NULL;
  }

  //Helper function that wait for hand to rest.
  void P2Interpreter::waitRest(double delay)
  {
    int poses[NSAMPLES];
    int i,j;
    for (i=0; i<NSAMPLES; i++)
      poses[i] = i*1000000;
    double m = 5000000;
    double v = 1000000;
    i=0;

    //Check periodically for end of motion
    struct timespec req = {0,0};
    req.tv_sec = 0;
    req.tv_nsec = (int)(1000000000L * (delay));
    nanosleep(&req, (timespec *)NULL);

    while(connectedMotor)
    {
      poses[i%NSAMPLES] = encMotor;
      i++;
      //We compute the average value over the last NSAMPLES
      m=0;
      for (j=0; j<NSAMPLES; j++)
        m+=poses[j];
      m/=(double)NSAMPLES;
      //We compute the variance over the last NSAMPLES
      v=0;
      for (j=0; j<NSAMPLES; j++)
        v+=(poses[j] - m) * (poses[j] - m);
      v/=(double)(NSAMPLES-1);

      if((i > NSAMPLES) && (m < MAX_MOTOR_POS) && (m > MIN_MOTOR_POS))
      {
        if(sqrt(v)<30.0)
          return;
      }
      req.tv_sec = 0;
      req.tv_nsec = (int)(1000000000L * (SAMPLING_TIME));
      nanosleep(&req, (timespec *)NULL);
    }
    setRest();
  }

  void P2Interpreter::checkMoving()
  {
    motBuf[bufCnt] = encMotor;
    bufCnt++;

    if (bufCnt >= NUM_SAMPS)
    {
      bufFull = true;
      bufCnt = 0;
    }

    // Compute the average over the last NSAMPLES
    int m=0;
    for (int j=0; j<NUM_SAMPS; j++)
      m+=motBuf[j];
    m/=(double)NUM_SAMPS;

    // Now compute the variance over the last NSAMPLES
    int v=0;
    for (int j=0; j<NUM_SAMPS; j++)
      v+=(motBuf[j] - m) * (motBuf[j] - m);
    v/=(double)(NUM_SAMPS-1);

    // If the variance is small enough, the motor is not moving
    if(bufFull && (m < MAX_MOTOR_POS) && (m > MIN_MOTOR_POS) && sqrt(v) < (VAR_FACT*motorSpeed))
      moving = false;
    else
      moving = true;

  }


  /*bool P2Interpreter::calibrate()
    {
    char buffer[MAX_BUFFER];
    if(connectedMotor && connectedArduino)
    {
    sprintf(buffer,"\nEN\n");
    spMotor.writeData(buffer);

  //Set low intensity and speed
  int savedMotorSpeed = motorSpeed;
  int savedMotorIntensity = motorIntensity;
  setMotorIntensity(150);
  setMotorSpeed(25);

  //Open the hand as much as possible and call that the zero position
  int initialMotorPose;
  getMotorPose(&initialMotorPose);
  calibrated = true; // If not the hand won't move.
  setMotorPose(initialMotorPose + 30000); 
  calibrated = false;

  //Sleep to let the motor begin to move at low power and speed
  struct timespec req = {0};
  req.tv_sec = 2.0;
  req.tv_nsec = 0.0;
  nanosleep(&req, (timespec *)NULL);

  //Wait until the end of calibration motion
  waitRest();

  //Check if the calibration makes sense?
  int newMotorPose;
  getMotorPose(&newMotorPose);
  if(newMotorPose==initialMotorPose)
  return false;

  //Set that as the homing position.
  sprintf(buffer,"\nHO\n");
  spMotor.writeData(buffer);
  calibrated = true;

  //Restore saved motor intensity and speed
  setMotorIntensity(savedMotorIntensity);
  setMotorSpeed(savedMotorSpeed);

  //Move to origin position
  setMotorPose(-1500);
  req.tv_sec = 3.0;
  req.tv_nsec = 0.0;
  nanosleep(&req, (timespec *)NULL);
  waitRest();
  return true;
  }	    
  return false;
  }*/


