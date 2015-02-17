
#include <stdio.h>
#include <stdlib.h>

#include <Servo.h> 

#define NUM_KINECTS 6

const int servoPins[NUM_KINECTS] = {3,5,6,9,10,11};
const int ranges[NUM_KINECTS][2] = 
{
  {750, 2300},
  {580, 2150},
  {670, 2200},
  {650, 2100},
  {660, 2010},
  {730, 2130}
};

const int poses[NUM_KINECTS][2] = 
{
  {25,70},
  {32,80},
  {20,65},
  {25,90},
  {10,170},
  {10,170}
};


static char char_buf[30];
static int cnt;

static Servo k_servo[NUM_KINECTS];

void setup()
{
  for (int i=0; i < NUM_KINECTS; ++i)
  {
    k_servo[i].attach(servoPins[i], ranges[i][0], ranges[i][1]);
  }
  
  Serial.begin(9600);
  Serial.flush();
  cnt = 0;
}


void loop()
{
  if (Serial.available() > 0)
  {
    char ch = Serial.read();
    if (ch == '\n')
    {
      char_buf[cnt] = '\0';
      int val = atoi(char_buf);
      Serial.println(val, BIN);
      for (int i=0; i < NUM_KINECTS; ++i)
      {
        
        k_servo[i].write(poses[i][val & 0x1]);
        val = val >> 1;
        //Serial.println(k_servo[i].readMicroseconds());
      }
      
      cnt = 0;
    }
    else
    {
      char_buf[cnt] = ch;
      cnt++;
    }
  }
}
