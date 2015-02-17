#include <ros/ros.h>
#include <robotiq_comm/robotiq_comm.h>
#include <cstdio>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "robotiq_demo");
  ros::NodeHandle node;
  
  RobotiqComm robotiq(&node);

  bool quit = false;
  char input[32];
  int val;

  do{
    printf("Simple Robotiq Hand Controller\n\n");
    printf("Available Commands:\n");
    printf("d: Disable\n");
    printf("e: Enable\n");
    printf("o: Open\n");
    printf("c: Close\n");
    printf("p <num>: Go to that position\n");
    printf("s <num>: Set speed\n");
    printf("f <num>: Set force\n");
    printf("a: Auto release\n");
    printf("t: Stop\n");
    printf("w: Wait until rest\n");
    printf("g: Get Status\n");
    printf("y: Get Pose\n");
    printf("r: Get Current\n");
    printf("q: Quit\n");
    printf("\nCommand: ");

    if (fgets(input, 32, stdin) == NULL)
    {
      printf("fgets error. quitting...\n");
      quit = true;
    }

    switch (input[0])
    {
      case 'd':
        robotiq.Disable();
        break;
      case 'e':
        robotiq.Enable();
        break;
      case 'o':
        robotiq.Open();
        break;
      case 'c':
        robotiq.Close();
        break;
      case 'p':
          val = atoi(input+1);
          robotiq.SetPose(val);
          break;
      case 's':
          val = atoi(input+1);
          robotiq.SetSpeed(val);
          break;
      case 'f':
          val = atoi(input+1);
          robotiq.SetForce(val);
          break;
      case 'a':
          robotiq.AutoRelease();
          break;
      case 't':
          robotiq.Stop();
          break;
      case 'w':
          robotiq.WaitRest(0.0);
          break;
      case 'g':
          {
            bool active, position_req;
            char activation_status, motion_status, fault;
            int req_pos, pos, current;
            robotiq.GetStatus(active, position_req, activation_status, motion_status, fault, req_pos, pos, current);
            printf("active = %d, position_req = %d\n", active, position_req);
            printf("activation_status = 0x%02X, motion_status = 0x%02X, fault = 0x%02X\n", activation_status, motion_status, fault);
            printf("req_pos = %d, pos = %d, current = %d\n", req_pos, pos, current);
            break;
          }
      case 'y':
          {
            int pos;
            robotiq.GetPose(pos);
            printf("pose = %d\n", pos);
            break;
          }
      case 'r':
          {
            int current;
            robotiq.GetCurrent(current);
            printf("current = %d\n", current);
            break;
          }
      case 'q':
          quit = true;
          break;
      default:
          printf("Unrecognized command.\n");
    }

  }while(!quit && ros::ok());
  
  return 0;
}
