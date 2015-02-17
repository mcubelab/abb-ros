#ifndef ROBOTIQ_COMM_H
#define ROBOTIQ_COMM_H

#include <ros/ros.h>

#include <robotiq_c_model_control/CModel_robot_input.h>
#include <robotiq_c_model_control/CModel_robot_output.h>

#include <pthread.h>

#define ROBOTIQ_DEFAULT_SPD 100
#define ROBOTIQ_DEFAULT_FORCE 150
#define ROBOTIQ_DEFAULT_POS 0

pthread_mutex_t inputMutex;

class RobotiqComm
{
  public:
    RobotiqComm();
    RobotiqComm(ros::NodeHandle* np);
    ~RobotiqComm();

    // Subscribe Function
    void subscribe(ros::NodeHandle* np);

    // Subscribe to Topics
    void subscribeInput(ros::NodeHandle* np, int q_len, 
        void (*funcPtr)(const robotiq_c_model_control::CModel_robot_inputConstPtr&));

    void interpretStatus(const robotiq_c_model_control::CModel_robot_input msg, bool &active, bool &position_req, char &activation_status, char &motion_status, char &fault, int &req_pos, int &pos, int &current);

    // Shutdown service clients
    void shutdown();

    bool GetStatus(bool &active, bool &position_req, char &activation_status, char &motion_status, char &fault, int &req_pos, int &pos, int &current);

    bool GetPose(int &pos);
    bool GetCurrent(int &current);

    bool Disable();
    bool Enable();

    bool AutoRelease();

    bool Open();
    bool Close();
    bool Stop();
    bool SetPose(int pose);
    bool SetForce(int force);
    bool SetSpeed(int speed);

    bool WaitRest(double delay);

    bool Ping();


  private:
    // Function that is called when we get an input message
    void saveMessage(const robotiq_c_model_control::CModel_robot_inputConstPtr& msg);

    // Set to true once we receive our first message
    bool got_message;

    robotiq_c_model_control::CModel_robot_output curOutput;

    // Last received message from robotiq
    robotiq_c_model_control::CModel_robot_input last_message;

    // Shorthand for our subscriber (input message), and publisher (output_message)
    ros::Subscriber robotiq_input_sub;
    ros::Publisher robotiq_output_pub;

};
#endif //ROBOTIQ_COMM_H
