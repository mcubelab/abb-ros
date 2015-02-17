/**
 * This sample node tests the vision_interface ROS node, asking it to capture
 * an image and then save it, then asking for the information about that
 * image.  Communication will be done via sockets so that this node can
 * be run on any computer.
 *
 * @author Alex Zirbel
 * @author Robbie Paolini
*/

#include <ros/ros.h>
#include <vision_comm/vision_comm.h>
#include <cstdlib>
#include <std_msgs/String.h>
#include <string.h>

using namespace std;

void calibrate(int cameraNum, string filename, bool newImg);
string capture(int cameraNum);
void process(string filename);
void captureAndProcess(int cameraNum);
void detectPlace(string filename);

void getOptions(int *optNum, int *cameraNum, string *filename, bool *newImg);
void die(string errorMsg);

/**
 * Runs the process in a loop
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_vision");
	if(argc != 1)
	{
		ROS_INFO("usage: test_vision");
		return 1;
	}

    int optNum, cameraNum;
    bool newImg;
    string filename;

    while(1)
    {
        getOptions(&optNum, &cameraNum, &filename, &newImg);

        switch(optNum)
        {
            case 1: calibrate(cameraNum, filename, newImg); break;
            case 2: filename = capture(cameraNum); break;
            case 3: process(filename); break;
            case 4: captureAndProcess(cameraNum); break;
            case 5: detectPlace(filename); break;
            default: cout << "Invalid option." << endl;
        }
    }
}

void calibrate(int cameraNum, string filename, bool newImg)
{
	// Send a calibration request.
	ros::NodeHandle _node;
	ros::ServiceClient client = _node.serviceClient
		<vision_comm::CalibrateVision>("calibrate_vision");
	vision_comm::CalibrateVision calibrate_srv;
    calibrate_srv.request.captureNew = newImg;
    calibrate_srv.request.cameraNum = cameraNum;
    calibrate_srv.request.calibrationFilename = filename;

	if(client.call(calibrate_srv))
	{
		ROS_INFO("Calibrated: %s", calibrate_srv.response.response.c_str());
	}
	else
	{
		ROS_ERROR("Failed to call service calibrate_vision");
		return;
	}
}

string capture(int cameraNum)
{
   // Ask the vision interface to capture an image.
    ros::NodeHandle _node;
    ros::ServiceClient client = _node.serviceClient
    <vision_comm::CaptureImage>("capture_image");
    vision_comm::CaptureImage img_srv;
    img_srv.request.cameraNum = cameraNum;

    // imagename saves the return information of the ROS service, so that
    // this program can ask about it later.
    string imagename;
    if(client.call(img_srv))
    {
        imagename = img_srv.response.filename;
        ROS_INFO("Filename: %s", img_srv.response.filename.c_str());
    }
    else
    {
        ROS_ERROR("Failed to call service capture_image");
        return NULL;
    }

    return imagename;
}

void process(string filename)
{
    // Call getInfo about the image we just got.
	ros::NodeHandle _node;
    ros::ServiceClient client = _node.serviceClient
        <vision_comm::GetInfo>("get_info");
    vision_comm::GetInfo info_srv;
    info_srv.request.filename = filename;

    if(client.call(info_srv))
    {
        ROS_INFO("Num markers: %d, Distance: %lf\n",
            info_srv.response.num_markers,
            info_srv.response.distance);
        ROS_INFO("Marker 1 Certainty: %f, PosX: %lf, PosY: %lf, Theta: %lf\n",
            (info_srv.response.certainty)[0],
            (info_srv.response.posx)[0],
            (info_srv.response.posy)[0],
            (info_srv.response.theta)[0]);
    }
    else
    {
        ROS_ERROR("Failed to call service get_info");
        return;
    }
}

void detectPlace(string filename)
{
    // Call getInfo about the image we just got.
	ros::NodeHandle _node;
    ros::ServiceClient client = _node.serviceClient
        <vision_comm::PlaceDetect>("place_detect");
    vision_comm::PlaceDetect pd_srv;
    pd_srv.request.filename = filename;

    if(client.call(pd_srv))
    {
        if (pd_srv.response.success)
            cout << "Marker Detected!" << endl;
        else
            cout << "No Marker Detected." << endl;
    }
    else
    {
        ROS_ERROR("Failed to call service place_detect");
        return;
    }
}

void captureAndProcess(int cameraNum)
{
    string filename = capture(cameraNum);

    process(filename);
}


/**
 * Prompts the user to choose which test to run.
 *
 * Options include calibrating the system, capturing and processing a single
 * image, starting video, and stopping video.  When appropriate, prompts for
 * camera number and filename.
 */
void getOptions(int *optNum, int *cameraNum, string *filename, bool *newImg)
{
    string optNumString, cameraNumString;

    char ans;

    cout << endl;
    cout << "********************************************************" << endl;
    cout << "*   Test the vision system:                            *" << endl;
    cout << "*     1: Calibrate vision                              *" << endl;
    cout << "*     2: Capture image                                 *" << endl;
    cout << "*     3: Process image                                 *" << endl;
    cout << "*     4: Capture and process image                     *" << endl;
    cout << "*     5: Detect Place                                  *" << endl;
    cout << "********************************************************" << endl;
    cout << "Choose an option: ";

    cin >> optNumString;
    optNumString = optNumString.substr(0,1);

    /* Bad option number. Note that we cannot use 0 as an option number because
     * this would also invalidate the check. */
    if(!(*optNum = atoi(optNumString.c_str())))
        die("Could not read option number.");

    /* Unless this is purely processing, we need to know which camera to use */
    if(*optNum != 3 && *optNum != 5)
    {
        cout << "Choose a camera (0 or 1): ";

        cin >> cameraNumString;
        cameraNumString = cameraNumString.substr(0,1);
        if(!sscanf(cameraNumString.c_str(),"%d",cameraNum))
            die("Could not read camera number.");
       //if(!(*cameraNum = atoi(cameraNumString.c_str())))
        //    die("Could not read camera number.");

        //(*cameraNum)--;
    }

    if (*optNum == 1)
    {
      cout << "Use existing image? (y/n) ";
      cin >> ans;
      *newImg = (ans == 'n');
    }

    if (*optNum == 1 || *optNum == 3 || *optNum == 5)
    {
        cout << "Specify filename: ";
        cin >> *filename;
    }
}

/**
 * Prints the error message and exits the program.
 */
void die(string errorMsg)
{
    cout << errorMsg << endl;
    exit(EXIT_FAILURE);
}
