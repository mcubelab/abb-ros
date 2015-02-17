#include <stdio.h>
//#include <conio.h>
#include <string>
#include <vector>

using namespace std;


#include <ros/ros.h>
#include <matVec/matVec.h>
#include <matlab_comm/matlab_comm.h>
#include <robot_comm/robot_comm.h>

void Set_Up_Optimization();
bool Set_Up_Robot(int assembly_selection);
bool Attempt_Assembly(Vec params);
void Attempt_BB_Cover(Vec params);
void Attempt_ShieldCan(Vec params);

#define FLY_SPEED_TCP   200.0
#define FLY_SPEED_ORI   150.0
#define ZONE 1
// BB_Cover

#define BB_WObj_x 540
#define BB_WObj_y 295	
#define BB_WObj_z 300
#define BB_WObj_q0 0
#define BB_WObj_qx 0
#define BB_WObj_qy 1
#define BB_WObj_qz 0

#define BB_Tool_x -60
#define BB_Tool_y 0.85
#define BB_Tool_z 129.99
#define BB_Tool_q0 1
#define BB_Tool_qx 0
#define BB_Tool_qy 0
#define BB_Tool_qz 0

#define BB_Home_x 15
#define BB_Home_y -15
#define BB_Home_z -20
#define BB_Home_q0 1
#define BB_Home_qx 0
#define BB_Home_qy 0
#define BB_Home_qz 0

//ShieldCan


#define ShieldCan_WObj_x 678.2
#define ShieldCan_WObj_y 213.1
#define ShieldCan_WObj_z 192.5
#define ShieldCan_WObj_q0 0
#define ShieldCan_WObj_qx 0
#define ShieldCan_WObj_qy 1.0
#define ShieldCan_WObj_qz 0

#define ShieldCan_Tool_x -75.0
#define ShieldCan_Tool_y 5
#define ShieldCan_Tool_z 205.5
#define ShieldCan_Tool_q0 1
#define ShieldCan_Tool_qx 0
#define ShieldCan_Tool_qy 0
#define ShieldCan_Tool_qz 0

#define ShieldCan_Home_x 15
#define ShieldCan_Home_y -15
#define ShieldCan_Home_z -15
#define ShieldCan_Home_q0 1
#define ShieldCan_Home_qx 0
#define ShieldCan_Home_qy 0
#define ShieldCan_Home_qz 0

// Matlab Optimization Parameters
#define nParams 2

#define nMin 30
#define nMax 200

#define MinA1 2
#define MinA2 2
#define MinA3 2
#define MaxA1 20
#define MaxA2 10
#define MaxA3 10

#define grid 40
