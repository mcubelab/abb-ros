#include <iostream>
#include <string>
#include <fstream>
#include <math.h>
#include <string.h>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <pthread.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>

#include <matVec/matVec.h>

using namespace std;

int main(int argc, char** argv){

	RotMat rotation ("0.9997 0.0239 0.0029 0.0240 -0.9989 -0.0398 0.0020 0.03998 -0.9992");

	Quaternion q = rotation.getQuaternion();

	printf("Quaternion is %f, %f, %f, %f.", q[0], q[1], q[2], q[3]);

}
