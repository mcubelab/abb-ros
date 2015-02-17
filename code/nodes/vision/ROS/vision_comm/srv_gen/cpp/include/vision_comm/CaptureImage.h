/* Auto-generated by genmsg_cpp for file /home/mcube/hands_MLab/code/nodes/vision/ROS/vision_comm/srv/CaptureImage.srv */
#ifndef VISION_COMM_SERVICE_CAPTUREIMAGE_H
#define VISION_COMM_SERVICE_CAPTUREIMAGE_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "ros/service_traits.h"

#include "std_msgs/Header.h"



namespace vision_comm
{
template <class ContainerAllocator>
struct CaptureImageRequest_ {
  typedef CaptureImageRequest_<ContainerAllocator> Type;

  CaptureImageRequest_()
  : header()
  , cameraNum(0)
  {
  }

  CaptureImageRequest_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , cameraNum(0)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef int32_t _cameraNum_type;
  int32_t cameraNum;


  typedef boost::shared_ptr< ::vision_comm::CaptureImageRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vision_comm::CaptureImageRequest_<ContainerAllocator>  const> ConstPtr;
}; // struct CaptureImageRequest
typedef  ::vision_comm::CaptureImageRequest_<std::allocator<void> > CaptureImageRequest;

typedef boost::shared_ptr< ::vision_comm::CaptureImageRequest> CaptureImageRequestPtr;
typedef boost::shared_ptr< ::vision_comm::CaptureImageRequest const> CaptureImageRequestConstPtr;



template <class ContainerAllocator>
struct CaptureImageResponse_ {
  typedef CaptureImageResponse_<ContainerAllocator> Type;

  CaptureImageResponse_()
  : filename()
  {
  }

  CaptureImageResponse_(const ContainerAllocator& _alloc)
  : filename(_alloc)
  {
  }

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _filename_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  filename;


  typedef boost::shared_ptr< ::vision_comm::CaptureImageResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vision_comm::CaptureImageResponse_<ContainerAllocator>  const> ConstPtr;
}; // struct CaptureImageResponse
typedef  ::vision_comm::CaptureImageResponse_<std::allocator<void> > CaptureImageResponse;

typedef boost::shared_ptr< ::vision_comm::CaptureImageResponse> CaptureImageResponsePtr;
typedef boost::shared_ptr< ::vision_comm::CaptureImageResponse const> CaptureImageResponseConstPtr;


struct CaptureImage
{

typedef CaptureImageRequest Request;
typedef CaptureImageResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct CaptureImage
} // namespace vision_comm

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::vision_comm::CaptureImageRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::vision_comm::CaptureImageRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::vision_comm::CaptureImageRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "5d7827666f1c2a3808b4923a005fb963";
  }

  static const char* value(const  ::vision_comm::CaptureImageRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x5d7827666f1c2a38ULL;
  static const uint64_t static_value2 = 0x08b4923a005fb963ULL;
};

template<class ContainerAllocator>
struct DataType< ::vision_comm::CaptureImageRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vision_comm/CaptureImageRequest";
  }

  static const char* value(const  ::vision_comm::CaptureImageRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::vision_comm::CaptureImageRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
\n\
int32 cameraNum\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
";
  }

  static const char* value(const  ::vision_comm::CaptureImageRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::vision_comm::CaptureImageRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::vision_comm::CaptureImageRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::vision_comm::CaptureImageResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::vision_comm::CaptureImageResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::vision_comm::CaptureImageResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "030824f52a0628ead956fb9d67e66ae9";
  }

  static const char* value(const  ::vision_comm::CaptureImageResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x030824f52a0628eaULL;
  static const uint64_t static_value2 = 0xd956fb9d67e66ae9ULL;
};

template<class ContainerAllocator>
struct DataType< ::vision_comm::CaptureImageResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vision_comm/CaptureImageResponse";
  }

  static const char* value(const  ::vision_comm::CaptureImageResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::vision_comm::CaptureImageResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "string filename\n\
\n\
\n\
";
  }

  static const char* value(const  ::vision_comm::CaptureImageResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::vision_comm::CaptureImageRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.cameraNum);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct CaptureImageRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::vision_comm::CaptureImageResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.filename);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct CaptureImageResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<vision_comm::CaptureImage> {
  static const char* value() 
  {
    return "a0af4dcbda7838a3057789313ac665ad";
  }

  static const char* value(const vision_comm::CaptureImage&) { return value(); } 
};

template<>
struct DataType<vision_comm::CaptureImage> {
  static const char* value() 
  {
    return "vision_comm/CaptureImage";
  }

  static const char* value(const vision_comm::CaptureImage&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<vision_comm::CaptureImageRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "a0af4dcbda7838a3057789313ac665ad";
  }

  static const char* value(const vision_comm::CaptureImageRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<vision_comm::CaptureImageRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vision_comm/CaptureImage";
  }

  static const char* value(const vision_comm::CaptureImageRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<vision_comm::CaptureImageResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "a0af4dcbda7838a3057789313ac665ad";
  }

  static const char* value(const vision_comm::CaptureImageResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<vision_comm::CaptureImageResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "vision_comm/CaptureImage";
  }

  static const char* value(const vision_comm::CaptureImageResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // VISION_COMM_SERVICE_CAPTUREIMAGE_H

