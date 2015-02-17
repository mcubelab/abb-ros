#include "ros/ros.h"
#include "regraspComm/regrasp_comm.h"
#include "regraspComm/Regrasp.h"

using namespace std;

void setup(HandComm hand)
{
    // Close fingers once a finger has been pushed
    double mot_ang;
    Vec ini_fing_angs(NUM_FINGERS);
    Vec cur_fing_angs(NUM_FINGERS);
    hand.GetAngles(mot_ang, ini_fing_angs);

    cout << "Please push one of my fingers to grasp an object!" << endl;


    ros::Rate loop_rate(20);
    while(ros::ok())
    {
      // Get the current finger angles, and if they're significantly 
      // different, a finger has been moved!
      hand.GetAngles(mot_ang, cur_fing_angs);
      if ((ini_fing_angs - cur_fing_angs).norm() > 2.0)
        break;
      loop_rate.sleep();
    }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "regrasp_hand");
  ros::NodeHandle nh;
  ROS_INFO("Starting Regrasp Client...");

  RobotComm robot(&nh);
  HandComm hand(&nh);
  RegraspComm regrasp(&nh);

  regraspComm::Regrasp pick = regrasp.getDefaults(PICK); // DONE
  regraspComm::Regrasp place = regrasp.getDefaults(PLACE);
  regraspComm::Regrasp rtp = regrasp.getDefaults(ROLL_TO_PALM); // DONE
  regraspComm::Regrasp rtg = regrasp.getDefaults(ROLL_TO_GROUND); // DONE
  regraspComm::Regrasp pie = regrasp.getDefaults(PUSH_IN_ENVELOPING); // DONE
  regraspComm::Regrasp rtf = regrasp.getDefaults(ROLL_TO_FINGERTIP); // DONE
  regraspComm::Regrasp taf = regrasp.getDefaults(THROW_AND_FLIP);
  regraspComm::Regrasp ttf = regrasp.getDefaults(THROW_TO_FINGERTIP); 
  regraspComm::Regrasp pif = regrasp.getDefaults(PUSH_IN_FINGERS); 
  regraspComm::Regrasp v = regrasp.getDefaults(VIBRATE); 
  regraspComm::Regrasp s = regrasp.getDefaults(SQUEEZE); 
  regraspComm::Regrasp ttp = regrasp.getDefaults(THROW_TO_PALM); 
  regraspComm::Regrasp dif = regrasp.getDefaults(DROOP_IN_FINGERS); 
  regraspComm::Regrasp rog = regrasp.getDefaults(ROLL_ON_GROUND);

  while(true)
    {
      string input = "";
      cout << "Enter a regrasp: \n";
      getline(cin, input);
      cout << "Executing: " << input << endl << endl;
      
      if (input == "exit")
	{
	  break;
	}
      else if (input == "RESET")
	{
	  //setup(hand);
	  // Joints to zero, hand open
	  vector<regraspComm::Regrasp> seq(1);
	  seq.at(0) = place;
	  robot.SetJoints(0,0,0,0,0,0);
	  hand.SetAngle(10);
	}
      else if (input == "PICK") // DONE
	{
	  //setup(hand);
	  vector<regraspComm::Regrasp> seq(1);
	  seq.at(0) = pick;
	  regrasp.Execute(seq);
	}
      else if (input == "ROLL_TO_PALM") // DONE
	{
	  setup(hand);
	  vector<regraspComm::Regrasp> seq(2);
	  seq.at(0) = pick;
	  seq.at(1) = rtp;
	  regrasp.Execute(seq);

	}
      else if (input == "ROLL_TO_GROUND") // DONE
	{
	  setup(hand);
	  vector<regraspComm::Regrasp> seq(2);
	  seq.at(0) = pick;
	  seq.at(1) = rtg;
	  regrasp.Execute(seq);

	}
      else if (input == "PUSH_IN_ENVELOPING") // DONE
	{
	  setup(hand);
	  vector<regraspComm::Regrasp> seq(1);
	  robot.SetJoints(0,0,0,0,0,0);
	  hand.SetAngle(60);
	  hand.WaitRest(1);
	  seq.at(0) = pie;
	  regrasp.Execute(seq);

	} 
      else if (input == "ROLL_TO_FINGERTIP") // DONE
	{
	  setup(hand);
	  vector<regraspComm::Regrasp> seq(1);
	  robot.SetJoints(0,0,0,0,0,0);
	  hand.SetAngle(60);
	  seq.at(0) = rtf;
	  regrasp.Execute(seq);

	}
      else if (input == "DROOP_IN_FINGERS") // DONE (TODO param opt)
	{
	  setup(hand);
	  vector<regraspComm::Regrasp> seq1(1);
	  vector<regraspComm::Regrasp> seq2(1);
	  seq1.at(0) = pick;
	  //regrasp.Execute(seq1);
	  //robot.SetJoints(0,0,0,0,0,0);
	  //hand.openHand();
	  //hand.WaitRest(1);
	  //hand.closeHand();
	  seq2.at(0) = dif;
	  regrasp.Execute(seq2);
	}
      else if (input == "THROW_AND_FLIP") // DONE
	{
	  setup(hand);
	  vector<regraspComm::Regrasp> seq1(1);
	  vector<regraspComm::Regrasp> seq2(1);
	  robot.SetJoints(0,0,0,0,0,0);
	  hand.openHand();
	  hand.closeHand(); 
	  robot.SetJoints(0,0,0,0,-90,0);
	  hand.SetAngle(35);
	  hand.WaitRest(1);
	  seq1.at(0) = v;
	  regrasp.Execute(seq1);
	  hand.closeHand();
	  seq2.at(0) = taf;
	  regrasp.Execute(seq2);
	}
      else if (input == "VIBRATE") // DONE
	{
	  setup(hand);
	  vector<regraspComm::Regrasp> seq(1);
	  seq.at(0) = v;
	  regrasp.Execute(seq);
	}
      else if (input == "SQUEEZE")
	{
	  setup(hand);
	  vector<regraspComm::Regrasp> seq(1);
	  seq.at(0) = s;
	  regrasp.Execute(seq);
	}
      else if (input == "THROW_TO_FINGERTIP") // TODO opt params (sleep)
	{
	  setup(hand);
	  vector<regraspComm::Regrasp> seq1(1);
	  vector<regraspComm::Regrasp> seq2(1);
	  robot.SetJoints(0,0,0,0,0,0);
	  hand.openHand();
	  hand.WaitRest(1);
	  hand.closeHand();
	  hand.WaitRest(1);
	  robot.SetJoints(0,0,0,0,-90,0);
	  hand.SetAngle(35);
	  hand.WaitRest(1);
	  seq1.at(0) = v;
	  regrasp.Execute(seq1);
	  hand.closeHand();
	  seq2.at(0) = ttf;
	  regrasp.Execute(seq2);
	}
      else if (input == "PUSH_IN_FINGERS") // DONE
	{
	  setup(hand);
	  vector<regraspComm::Regrasp> seq(2);
	  seq.at(0) = pick;
	  seq.at(1) = pif;
	  regrasp.Execute(seq);
	}
      else if (input == "THROW_TO_PALM") // DONE
	{
	  setup(hand);
	  vector<regraspComm::Regrasp> seq1(1);
	  vector<regraspComm::Regrasp> seq2(1);
	  seq1.at(0) = pick;
	  seq2.at(0) = ttp;
	  regrasp.Execute(seq1);
	  robot.SetJoints(0,0,0,0,0,0);
	  regrasp.Execute(seq2);
	}
      else if (input == "ROLL_ON_GROUND")
        {
          vector<regraspComm::Regrasp> seq(2);
          seq.at(0) = pick;
          seq.at(1) = rog;
          regrasp.Execute(seq);
        }
      else
	{
	  cout << "Invalid regrasp. " << endl;
	}
  
    }

  // char str[256];
  // std::cout << "Enter a regrasp: ";
  // std::cin.get(str,256);
  // std::ifstream is(str);
  
  // while(is.good())
  //   {
  //     char c = is.get();
  //     if (is.good())
  // 	std::cout << c;
  //   }

  // is.close();

  // return 0;

      // if (strcmp(arg[0], "exit") == 0)
      // 	exit(0);      
      // else if (strcmp(arg[0], "PICK") == 0)
      // 	{
      // 	  seq.at(0) = pick;
      // 	  regrasp.Execute(seq);
      // 	}


  // }
  //vector<regraspComm::Regrasp> seq(2);
  //seq.at(0) = rmsg_pick;
  
  //  vector<regraspComm::Regrasp> seq = rollToPalm(regrasp);
  //vector<regraspComm::Regrasp> seq = rollToGround(regrasp);
  //vector<regraspComm::Regrasp> seq = pushInEnveloping(regrasp);
  //vector<regraspComm::Regrasp> seq = rollToFingertip(regrasp);
  //regrasp.Execute(seq);
}
