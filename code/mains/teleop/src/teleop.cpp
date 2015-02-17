// Name: Robbie Pa

#include "teleop.hpp"

void updateCartData(const robot_comm::robot_CartesianLogConstPtr& msg)
{
  if (!readingC)
  {
    newC[0] = msg->x;
    newC[1] = msg->y;
    newC[2] = msg->z;
    newC[3] = msg->q0;
    newC[4] = msg->qx;
    newC[5] = msg->qy;
    newC[6] = msg->qz;
    updateC = true;
  }
}

void updateJointData(const robot_comm::robot_JointsLogConstPtr& msg)
{
  if (!readingJ)
  {
    newJ[0] = msg->j1;
    newJ[1] = msg->j2;
    newJ[2] = msg->j3;
    newJ[3] = msg->j4;
    newJ[4] = msg->j5;
    newJ[5] = msg->j6;
    updateJ = true;
  }
}

void updateHandData(const hand_comm::hand_AnglesLogConstPtr& msg)
{
  if (!readingH)
  {
    newH[0] = msg->angleMotor;
    for (int i=0; i<NUM_FINGERS; i++)
      newH[i+1] = msg->angle[i];
    updateH = true;
  }
}

bool initialize()
{
  robot.subscribe(nodePtr);
  logger.subscribe(nodePtr);
  hand.subscribe(nodePtr);

  if (!robot.Ping())
  {
    ROS_WARN("Robot not ready!");
    return false;
  }
  
  //if (!hand.Ping())
  //{
    //ROS_WARN("Hand not ready!");
    //return false;
  //}

  if (!robot.SetComm(NON_BLOCKING))
  {
    ROS_WARN("Unable to set communication for robot!");
    return false;
  }

  if (!robot.SetZone(T_ZONE))
  {
    ROS_WARN("Unable to set zone of robot!");
    return false;
  }

  workObj[0] = DEF_WORK_X;
  workObj[1] = DEF_WORK_Y;
  workObj[2] = DEF_WORK_Z;
  workObj[3] = DEF_WORK_Q0;
  workObj[4] = DEF_WORK_QX;
  workObj[5] = DEF_WORK_QY;
  workObj[6] = DEF_WORK_QZ;

  if (!robot.SetWorkObject(workObj[0], workObj[1], workObj[2], workObj[3],
        workObj[4], workObj[5], workObj[6]))
  {
    ROS_WARN("Unable to set work object for robot!");
    return false;
  }

  toolObj[0] = DEF_TOOL_X;
  toolObj[1] = DEF_TOOL_Y;
  toolObj[2] = DEF_TOOL_Z;
  toolObj[3] = DEF_TOOL_Q0;
  toolObj[4] = DEF_TOOL_QX;
  toolObj[5] = DEF_TOOL_QY;
  toolObj[6] = DEF_TOOL_QZ;

  if (!robot.SetTool(toolObj[0], toolObj[1], toolObj[2], toolObj[3],
        toolObj[4], toolObj[5], toolObj[6]))
  {
    ROS_WARN("Unable to set tool for robot!");
    return false;
  }

  curSpd[0] = DEF_TCP;
  curSpd[1] = DEF_ORI;

  if (!robot.SetSpeed(curSpd[0], curSpd[1]))
  {
    ROS_WARN("Unable to set speed of robot!");
    return false;
  }

  homeJ[0] = DEF_HOME_J1;
  homeJ[1] = DEF_HOME_J2;
  homeJ[2] = DEF_HOME_J3;
  homeJ[3] = DEF_HOME_J4;
  homeJ[4] = DEF_HOME_J5;
  homeJ[5] = DEF_HOME_J6;


  // Setup our cartesian rotation quaternions so we don't have to
  // recalculate them every time
  RotMat r;
  r.rotX(R_STEP);
  x_pos_rot = r.getQuaternion();
  r.rotX(-R_STEP);
  x_neg_rot = r.getQuaternion();
  r.rotY(R_STEP);
  y_pos_rot = r.getQuaternion();
  r.rotY(-R_STEP);
  y_neg_rot = r.getQuaternion();
  r.rotZ(R_STEP);
  z_pos_rot = r.getQuaternion();
  r.rotZ(-R_STEP);
  z_neg_rot = r.getQuaternion();


  updateC = false;
  updateJ = false;
  updateH = false;
  readingC = false;
  readingJ = false;
  readingH = false;

  robot.subscribeCartesian(nodePtr, 100, &updateCartData);
  robot.subscribeJoints(nodePtr, 100, &updateJointData);
  hand.subscribeAngles(nodePtr, 100, &updateHandData);

  enabled = false;

  console_init();
  console_update_workObj(workObj);
  console_update_tool(toolObj);
  console_update_home(homeJ);
  console_update_speed(curSpd);

  last_key = NO_KEY;

  return true;
}

void updateData()
{
  //ros::spinOnce();

  if (updateC)
  {
    readingC = true;
    memcpy(curC, newC, 7*sizeof(double));
    updateC = false;
    readingC = false;
    console_update_cart(curC);
  }
  if (updateJ)
  {
    readingJ = true;
    memcpy(curJ, newJ, 6*sizeof(double));
    updateJ = false;
    readingJ = false;
    console_update_joints(curJ);
  }
  if (updateH)
  {
    readingH = true;
    memcpy(curH, newH, 5*sizeof(double));
    updateH = false;
    readingH = false;
    console_update_hand(curH);
  }
}

void getCommand()
{
  KEY_CMDS cur_key = console_get_key();

  // If we are no longer pressing the same key, make sure we 
  // reflect this in the console
  if (cur_key != last_key)
    console_clear_key(last_key);

  // If this is an enable key, setup communication,
  // stop any movement, and we're done. 
  if (cur_key == ENABLE)
  {
    if (!enabled)
    {
      enabled = true;
      console_update_enabled(enabled);
      robot.SetComm(NON_BLOCKING);
      robot.SetZone(T_ZONE);
      robot.Stop();
      hand.SetRest();
    }
    last_key = ENABLE;
    return;
  }

  // If this is a disable key, stop movement, and we're done
  if (cur_key == DISABLE)
  {
    if (enabled)
    {
      enabled = false;
      console_update_enabled(enabled);
      robot.Stop();
      hand.SetRest();
    }
    last_key = DISABLE;
    return;
  }

  // If we're not enabled, and the user pressed something that's not quit, 
  // alert them that they're not doing anything useful
  if (!enabled)
  {
    if (cur_key == NO_KEY)
    {
      last_key = NO_KEY;
      return;
    }
    if (cur_key != QUIT)
    {
      console_flash();
      last_key = cur_key;
      return;
    }
  }

  updateData();

  switch (cur_key)
  {
    // Cartesian Moves
    case DEC_X:
        robot.SetCartesian(curC[0] - CART_STEP, curC[1], curC[2], curC[3],
            curC[4], curC[5], curC[6]);
        break;
    case DEC_Y:
        robot.SetCartesian(curC[0], curC[1] - CART_STEP, curC[2], curC[3],
            curC[4], curC[5], curC[6]);
        break;
    case DEC_Z:
      robot.SetCartesian(curC[0], curC[1], curC[2] - CART_STEP, curC[3],
                          curC[4], curC[5], curC[6]);
      break;
    case INC_X:
      robot.SetCartesian(curC[0] + CART_STEP, curC[1], curC[2], curC[3],
                          curC[4], curC[5], curC[6]);
      break;
    case INC_Y:
      robot.SetCartesian(curC[0], curC[1] + CART_STEP, curC[2], curC[3],
                          curC[4], curC[5], curC[6]);
      break;
    case INC_Z:
      robot.SetCartesian(curC[0], curC[1], curC[2] + CART_STEP, curC[3],
                          curC[4], curC[5], curC[6]);
      break;

    // Cartesian Rotations
    case DEC_RX:
      {
        Quaternion orient;
        orient[0] = curC[3];orient[1] = curC[4];
        orient[2] = curC[5];orient[3] = curC[6];

        orient = x_neg_rot ^ orient;
        robot.SetCartesian(curC[0], curC[1], curC[2], orient[0],
            orient[1], orient[2], orient[3]);
      
        break;
      }
    case DEC_RY:
      {
        Quaternion orient;
        orient[0] = curC[3];orient[1] = curC[4];
        orient[2] = curC[5];orient[3] = curC[6];

        orient = y_neg_rot ^ orient;
        robot.SetCartesian(curC[0], curC[1], curC[2], orient[0],
            orient[1], orient[2], orient[3]);
      
        break;
      }
    case DEC_RZ:
      {
        Quaternion orient;
        orient[0] = curC[3];orient[1] = curC[4];
        orient[2] = curC[5];orient[3] = curC[6];

        orient = z_neg_rot ^ orient;
        robot.SetCartesian(curC[0], curC[1], curC[2], orient[0],
            orient[1], orient[2], orient[3]);
      
        break;
      }
    case INC_RX:
      {
        Quaternion orient;
        orient[0] = curC[3];orient[1] = curC[4];
        orient[2] = curC[5];orient[3] = curC[6];

        orient = x_pos_rot ^ orient;
        robot.SetCartesian(curC[0], curC[1], curC[2], orient[0],
            orient[1], orient[2], orient[3]);
      
        break;
      }
    case INC_RY:
      {
        Quaternion orient;
        orient[0] = curC[3];orient[1] = curC[4];
        orient[2] = curC[5];orient[3] = curC[6];

        orient = y_pos_rot ^ orient;
        robot.SetCartesian(curC[0], curC[1], curC[2], orient[0],
            orient[1], orient[2], orient[3]);
      
        break;
      }
    case INC_RZ:
      {
        Quaternion orient;
        orient[0] = curC[3];orient[1] = curC[4];
        orient[2] = curC[5];orient[3] = curC[6];

        orient = z_pos_rot ^ orient;
        robot.SetCartesian(curC[0], curC[1], curC[2], orient[0],
            orient[1], orient[2], orient[3]);
      
        break;
      }      

    // Joint Moves
    case DEC_J1:
      robot.SetJoints(curJ[0] - J_STEP, curJ[1], curJ[2], 
                      curJ[3], curJ[4], curJ[5]);
      break;
    case DEC_J2:
      robot.SetJoints(curJ[0], curJ[1] - J_STEP, curJ[2], 
                      curJ[3], curJ[4], curJ[5]);
      break;
    case DEC_J3:
      robot.SetJoints(curJ[0], curJ[1], curJ[2] - J_STEP, 
                      curJ[3], curJ[4], curJ[5]);
      break;
    case DEC_J4:
      robot.SetJoints(curJ[0], curJ[1], curJ[2], 
                      curJ[3] - J_STEP, curJ[4], curJ[5]);
      break;
    case DEC_J5:
      robot.SetJoints(curJ[0], curJ[1], curJ[2], 
                      curJ[3], curJ[4] - J_STEP, curJ[5]);
      break;
    case DEC_J6:
      robot.SetJoints(curJ[0], curJ[1], curJ[2], 
                      curJ[3], curJ[4], curJ[5] - J_STEP);
      break;
    case INC_J1:
      robot.SetJoints(curJ[0] + J_STEP, curJ[1], curJ[2], 
                      curJ[3], curJ[4], curJ[5]);
      break;
    case INC_J2:
      robot.SetJoints(curJ[0], curJ[1] + J_STEP, curJ[2], 
                      curJ[3], curJ[4], curJ[5]);
      break;
    case INC_J3:
      robot.SetJoints(curJ[0], curJ[1], curJ[2] + J_STEP, 
                      curJ[3], curJ[4], curJ[5]);
      break;
    case INC_J4:
      robot.SetJoints(curJ[0], curJ[1], curJ[2], 
                      curJ[3] + J_STEP, curJ[4], curJ[5]);
      break;
    case INC_J5:
      robot.SetJoints(curJ[0], curJ[1], curJ[2], 
                      curJ[3], curJ[4] + J_STEP, curJ[5]);
      break;
    case INC_J6:
      robot.SetJoints(curJ[0], curJ[1], curJ[2], 
                      curJ[3], curJ[4], curJ[5] + J_STEP);
      break;

    // Hand Moves
    case CL_HAND:
      hand.SetAngle(curH[0] + H_STEP);
      break;
    case OP_HAND:
      hand.SetAngle(curH[0] - H_STEP);
      break;
    case CAL_HAND:
      hand.Calibrate();
      break;

    // Logging
    case STOP_LOG:
      logger.Stop();
      logger.Append(log_file, teleop_log_str);
      break;
    case START_LOG:
      logger.Start(log_file, TELEOP_ID);
      break;

    // Align hand to nearest cartesian axis
    case ALIGN:
      {
        FILE *pfile;
        pfile = fopen ("myfile.txt","w");

        robot.Stop();
        robot.SetComm(BLOCKING);

        Quaternion cur_q(curC+3);

        // First, find the direction our tool is currently pointing
        Vec dir = (cur_q^Quaternion("0 0 0 1")
                            ^(cur_q.conjugate())).getVector();

        fprintf(pfile, "dir: %f, %f, %f\n", dir[0], dir[1], dir[2]);

        // If it's pointing along the z axis already, we're done
        if ((dir[0]*dir[0] + dir[1]*dir[1]) > 0.00001)
        {
          // Now, find the quaternion that rotates the tool to where it's 
          // currently pointing (with no rotation)
          Vec temp = Vec("0 0 1", 3)^dir;
          Vec n_hat = temp / temp.norm();
          fprintf(pfile, "temp: %f, %f, %f\n", temp[0], temp[1], temp[2]);
          fprintf(pfile, "n_hat: %f, %f, %f\n", n_hat[0], n_hat[1], n_hat[2]);

          double sin_th = temp.norm();
          double cos_th = sqrt(1 - sin_th*sin_th);
          if (temp[2] < 0)
            cos_th *= -1;
          double cos_th2 = sqrt(0.5*(1 + cos_th));
          double sin_th2 = sqrt(0.5*(1 - cos_th));

          fprintf(pfile, "s: %f, c: %f, s2: %f, c2: %f\n", sin_th, cos_th, sin_th2, cos_th2);

          Quaternion q_z;
          q_z[0] = cos_th2;
          q_z[1] = sin_th2 * n_hat[0]; 
          q_z[2] = sin_th2 * n_hat[1]; 
          q_z[3] = sin_th2 * n_hat[2]; 

          fprintf(pfile, "qz: %f, %f, %f, %f\n", q_z[0], q_z[1], q_z[2], q_z[3]);

          // Now, express the rotation about the z-axis of the tool 
          // as a quaternion
          Quaternion q_r = (q_z.conjugate()) ^ cur_q;

          fprintf(pfile, "qr: %f, %f, %f, %f\n", q_r[0], q_r[1], q_r[2], q_r[3]);

          // Now, find the closest axis to rotate our tool towards
          // (This is done by finding the largest dot product)
          double max_dot_prod = -1;
          int closest_idx = -1;
          for (int i=0; i<NUM_ALIGN_DIRS; i++)
          {
            fprintf(pfile, "av[%d]: %f, %f, %f\n",
                i, align_vecs[i][0], align_vecs[i][1], align_vecs[i][2]);
            double dp = align_vecs[i] * dir;
            fprintf(pfile, "dp: %f\n", dp);
            if (dp > max_dot_prod)
            {
              max_dot_prod = dp;
              closest_idx = i;
              fprintf(pfile, "max_dp = %f, c_i = %d\n", dp, i);
            }
          }

          // Now, our goal orientation is the closest aligned location, 
          // which is then rotated by the current tool rotation
          Quaternion new_q = align_quats[closest_idx] ^ q_r;

          fprintf(pfile, "new_q: %f, %f, %f, %f\n", new_q[0], new_q[1], new_q[2], new_q[3]);
          fclose (pfile);

          // Set this new position
          robot.SetCartesian(curC[0], curC[1], curC[2], 
              new_q[0], new_q[1], new_q[2], new_q[3]);
        }

        robot.SetComm(NON_BLOCKING);
        break;
      }
    // Going and saving the home position
    case GO_HOME:
      robot.Stop();
      robot.SetComm(BLOCKING);
      robot.SetJoints(homeJ[0], homeJ[1], homeJ[2], 
                      homeJ[3], homeJ[4], homeJ[5]);
      robot.SetComm(NON_BLOCKING);
      break;
    case SV_HOME:
      memcpy(homeJ, curJ, 6*sizeof(double));
      console_update_home(homeJ);
      break;

    // Sets the current global position as the work object plus the tool
    // offset. After this command, the current position should be the origin
    case SV_WOBJ:
      {
        robot.Stop();

        // Convert our work object and current position 
        // into vectors and quaternions
        Vec workV(workObj, 3);
        Vec posV(curC, 3);
        Quaternion workQ(workObj+3);
        Quaternion posQ(curC+3);

        // Now, using these, create homogeneous matrices representing the
        // work object and current position transformations
        HomogTransf workH(workQ.getRotMat(), workV);
        HomogTransf posH(posQ.getRotMat(), posV);

        // Then, compose the matrices and extract our new work object
        HomogTransf globalH = workH * posH;
        Vec globalV = globalH.getTranslation();
        Quaternion globalQ = globalH.getRotation().getQuaternion();

        // Save our new work object
        workObj[0] = globalV[0];
        workObj[1] = globalV[1];
        workObj[2] = globalV[2];
        workObj[3] = globalQ[0];
        workObj[4] = globalQ[1];
        workObj[5] = globalQ[2];
        workObj[6] = globalQ[3];

        robot.SetWorkObject(workObj[0], workObj[1], workObj[2], workObj[3],
            workObj[4], workObj[5], workObj[6]);

        console_update_workObj(workObj);
        break;
      }

    // The tool is updated so that the current position is the origin
    case SV_TOOL:
      {
        robot.Stop();

        // Convert the tool object and current position 
        // into homogenous matrices
        Vec toolV(toolObj, 3);
        Vec posV(curC, 3);
        Quaternion toolQ(toolObj+3);
        Quaternion posQ(curC+3);

        HomogTransf toolH(toolQ.getRotMat(), toolV);
        HomogTransf posH(posQ.getRotMat(), posV);

        // Then, compose the matrices and extract our new tool object
        HomogTransf globalH = toolH * posH.inv();
        Vec globalV = globalH.getTranslation();
        Quaternion globalQ = globalH.getRotation().getQuaternion();

        // Finally, save our new tool object
        toolObj[0] = globalV[0];
        toolObj[1] = globalV[1];
        toolObj[2] = globalV[2];
        toolObj[3] = globalQ[0];
        toolObj[4] = globalQ[1];
        toolObj[5] = globalQ[2];
        toolObj[6] = globalQ[3];

        robot.SetTool(toolObj[0], toolObj[1], toolObj[2], toolObj[3],
            toolObj[4], toolObj[5], toolObj[6]);
        console_update_tool(toolObj);
        break;
      }

    case NUM_KEY_CMDS:
    case DISABLE:
    case ENABLE:
      ROS_WARN("Error, shouldn't be here!");
      ros::shutdown();
      break;

    // Speed Adjustments
    case DEC_TCP:
      curSpd[0] -= TCP_STEP;

      if (curSpd[0] < 0)
        curSpd[0] = 0;
      
      robot.SetSpeed(curSpd[0], curSpd[1]);
      console_update_speed(curSpd);
      break;
    case INC_TCP:
      curSpd[0] += TCP_STEP;

      if (curSpd[0] < 0)
        curSpd[0] = 0;
      
      robot.SetSpeed(curSpd[0], curSpd[1]);
      console_update_speed(curSpd);
      break;
    case DEC_ORI:
      curSpd[1] -= ORI_STEP;

      if (curSpd[1] < 0)
        curSpd[1] = 0;
      
      robot.SetSpeed(curSpd[0], curSpd[1]);
      console_update_speed(curSpd);
      break;
    case INC_ORI:
      curSpd[1] += ORI_STEP;

      if (curSpd[1] < 0)
        curSpd[1] = 0;
      
      robot.SetSpeed(curSpd[0], curSpd[1]);
      console_update_speed(curSpd);
      break;

    // Quitting the console. Simply shuts down ros, 
    // which will kick us out of our loop and end the program
    case QUIT:
      ros::shutdown();
      break;

    // If the user just lifted off of a key, make sure we 
    // immediately stop the robot and hand
    // Otherwise, do nothing.
    case NO_KEY:
      if (last_key != NO_KEY)
      {
        robot.Stop();
        hand.SetRest();
      }

      break;
    
    case UNKNOWN_KEY:
      // If the user pressed a random key, let them know
      console_flash();
      break;
  }

  // Remember the last key, so we can erase it when it's no longer pressed.
  last_key = cur_key;
}


///////////////
// MAIN LOOP //
///////////////
int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "teleop");
  ros::NodeHandle node;
  ros::Rate loop_rate(TELEOP_RATE);
  nodePtr = &node;

  // Initialize the robot, subscribe to services, and create the console
  if (!initialize())
  {
    ROS_WARN("Unable to initialize teleop. Exiting...");
    ros::shutdown();
    exit(0);
  }

  // Now simply keep track of position updates and execute any
  // user commands

  ros::AsyncSpinner spinner(1);
  spinner.start();

  while (ros::ok())
  {
    updateData();
    getCommand();
    loop_rate.sleep();
  }

  console_close();

  return 0;
}

