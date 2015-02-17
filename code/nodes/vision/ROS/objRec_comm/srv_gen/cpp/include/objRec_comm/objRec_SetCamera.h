/* Auto-generated by genmsg_cpp for file /home/mcube/hands_MLab/code/nodes/vision/ROS/objRec_comm/srv/objRec_SetCamera.srv */
#ifndef OBJREC_COMM_SERVICE_OBJREC_SETCAMERA_H
#define OBJREC_COMM_SERVICE_OBJREC_SETCAMERA_H
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




namespace objRec_comm
{
template <class ContainerAllocator>
struct objRec_SetCameraRequest_ {
  typedef objRec_SetCameraRequest_<ContainerAllocator> Type;

  objRec_SetCameraRequest_()
  : trans()
  , quat()
  , cameraName()
  {
    trans.assign(0.0);
    quat.assign(0.0);
  }

  objRec_SetCameraRequest_(const ContainerAllocator& _alloc)
  : trans()
  , quat()
  , cameraName(_alloc)
  {
    trans.assign(0.0);
    quat.assign(0.0);
  }

  typedef boost::array<double, 3>  _trans_type;
  boost::array<double, 3>  trans;

  typedef boost::array<double, 4>  _quat_type;
  boost::array<double, 4>  quat;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _cameraName_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  cameraName;


  typedef boost::shared_ptr< ::objRec_comm::objRec_SetCameraRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::objRec_comm::objRec_SetCameraRequest_<ContainerAllocator>  const> ConstPtr;
}; // struct objRec_SetCameraRequest
typedef  ::objRec_comm::objRec_SetCameraRequest_<std::allocator<void> > objRec_SetCameraRequest;

typedef boost::shared_ptr< ::objRec_comm::objRec_SetCameraRequest> objRec_SetCameraRequestPtr;
typedef boost::shared_ptr< ::objRec_comm::objRec_SetCameraRequest const> objRec_SetCameraRequestConstPtr;



template <class ContainerAllocator>
struct objRec_SetCameraResponse_ {
  typedef objRec_SetCameraResponse_<ContainerAllocator> Type;

  objRec_SetCameraResponse_()
  : ret(0)
  , msg()
  {
  }

  objRec_SetCameraResponse_(const ContainerAllocator& _alloc)
  : ret(0)
  , msg(_alloc)
  {
  }

  typedef int64_t _ret_type;
  int64_t ret;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _msg_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  msg;


  typedef boost::shared_ptr< ::objRec_comm::objRec_SetCameraResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::objRec_comm::objRec_SetCameraResponse_<ContainerAllocator>  const> ConstPtr;
}; // struct objRec_SetCameraResponse
typedef  ::objRec_comm::objRec_SetCameraResponse_<std::allocator<void> > objRec_SetCameraResponse;

typedef boost::shared_ptr< ::objRec_comm::objRec_SetCameraResponse> objRec_SetCameraResponsePtr;
typedef boost::shared_ptr< ::objRec_comm::objRec_SetCameraResponse const> objRec_SetCameraResponseConstPtr;


struct objRec_SetCamera
{

typedef objRec_SetCameraRequest Request;
typedef objRec_SetCameraResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct objRec_SetCamera
} // namespace objRec_comm

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::objRec_comm::objRec_SetCameraRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::objRec_comm::objRec_SetCameraRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::objRec_comm::objRec_SetCameraRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "20d778bef8acdbbf7cd2ec043149d916";
  }

  static const char* value(const  ::objRec_comm::objRec_SetCameraRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x20d778bef8acdbbfULL;
  static const uint64_t static_value2 = 0x7cd2ec043149d916ULL;
};

template<class ContainerAllocator>
struct DataType< ::objRec_comm::objRec_SetCameraRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "objRec_comm/objRec_SetCameraRequest";
  }

  static const char* value(const  ::objRec_comm::objRec_SetCameraRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::objRec_comm::objRec_SetCameraRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
\n\
\n\
float64[3] trans\n\
float64[4] quat\n\
\n\
string cameraName\n\
\n\
\n\
";
  }

  static const char* value(const  ::objRec_comm::objRec_SetCameraRequest_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::objRec_comm::objRec_SetCameraResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::objRec_comm::objRec_SetCameraResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::objRec_comm::objRec_SetCameraResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "1e32786be6359fbbb6259aee4f579d10";
  }

  static const char* value(const  ::objRec_comm::objRec_SetCameraResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x1e32786be6359fbbULL;
  static const uint64_t static_value2 = 0xb6259aee4f579d10ULL;
};

template<class ContainerAllocator>
struct DataType< ::objRec_comm::objRec_SetCameraResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "objRec_comm/objRec_SetCameraResponse";
  }

  static const char* value(const  ::objRec_comm::objRec_SetCameraResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::objRec_comm::objRec_SetCameraResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
int64 ret\n\
string msg\n\
\n\
\n\
";
  }

  static const char* value(const  ::objRec_comm::objRec_SetCameraResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::objRec_comm::objRec_SetCameraRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.trans);
    stream.next(m.quat);
    stream.next(m.cameraName);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct objRec_SetCameraRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::objRec_comm::objRec_SetCameraResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.ret);
    stream.next(m.msg);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct objRec_SetCameraResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<objRec_comm::objRec_SetCamera> {
  static const char* value() 
  {
    return "f1d8276e89a5b46b314ccef3039988a4";
  }

  static const char* value(const objRec_comm::objRec_SetCamera&) { return value(); } 
};

template<>
struct DataType<objRec_comm::objRec_SetCamera> {
  static const char* value() 
  {
    return "objRec_comm/objRec_SetCamera";
  }

  static const char* value(const objRec_comm::objRec_SetCamera&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<objRec_comm::objRec_SetCameraRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "f1d8276e89a5b46b314ccef3039988a4";
  }

  static const char* value(const objRec_comm::objRec_SetCameraRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<objRec_comm::objRec_SetCameraRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "objRec_comm/objRec_SetCamera";
  }

  static const char* value(const objRec_comm::objRec_SetCameraRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<objRec_comm::objRec_SetCameraResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "f1d8276e89a5b46b314ccef3039988a4";
  }

  static const char* value(const objRec_comm::objRec_SetCameraResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<objRec_comm::objRec_SetCameraResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "objRec_comm/objRec_SetCamera";
  }

  static const char* value(const objRec_comm::objRec_SetCameraResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // OBJREC_COMM_SERVICE_OBJREC_SETCAMERA_H

