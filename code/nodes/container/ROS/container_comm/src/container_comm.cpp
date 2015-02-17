#include "container_comm.h"

ContainerComm::ContainerComm()
{
}

ContainerComm::ContainerComm(ros::NodeHandle * np)
{
  subscribe(np);
}

ContainerComm::~ContainerComm()
{
  shutdown();
}

void ContainerComm::subscribe(ros::NodeHandle * np)
{
  handle_container_PingContainer = 
    np->serviceClient<container_comm::container_Ping>("container_Ping");
  handle_container_CaptureImage = 
    np->serviceClient<container_comm::container_capture_image>("container_CaptureImage");
  handle_container_GetAverageHeight = 
    np->serviceClient<container_comm::container_get_average_height>("container_GetAverageHeight");
  handle_container_TransformPoints = 
    np->serviceClient<container_comm::container_transform_points>("container_TransformPoints");
}

void ContainerComm::shutdown()
{
  handle_container_PingContainer.shutdown();
  handle_container_CaptureImage.shutdown();
  handle_container_GetAverageHeight.shutdown();
}

bool ContainerComm::Ping()
{
  return handle_container_PingContainer.call(container_PingContainer_srv);
}

bool ContainerComm::CaptureImage(string &filename)
{
  if (handle_container_CaptureImage.call(container_CaptureImage_srv))
  {
    return true;
  }
  return false;
}

bool ContainerComm::GetAverageHeight(double x, double y, double radius)
{
  return true;
}

bool ContainerComm::GetAverageHeight(double x, double y, double radius, double &surfaceHeight)
{
  container_GetAverageHeight_srv.request.x = x;
  container_GetAverageHeight_srv.request.y = y;
  container_GetAverageHeight_srv.request.radius = radius;

	if (handle_container_GetAverageHeight.call(container_GetAverageHeight_srv))
	{
		surfaceHeight = container_GetAverageHeight_srv.response.avgHeight;
		return true;
	}
	else
	  return false;
}

bool ContainerComm::TransformPoints()
{
  return true;
}
