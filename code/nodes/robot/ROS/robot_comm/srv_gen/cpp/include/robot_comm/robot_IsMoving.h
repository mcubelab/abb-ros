/* Auto-generated by genmsg_cpp for file /home/mcube/hands_MLab/code/nodes/robot/ROS/robot_comm/srv/robot_IsMoving.srv */
#ifndef ROBOT_COMM_SERVICE_ROBOT_ISMOVING_H
#define ROBOT_COMM_SERVICE_ROBOT_ISMOVING_H
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




namespace robot_comm
{
template <class ContainerAllocator>
struct robot_IsMovingRequest_ {
  typedef robot_IsMovingRequest_<ContainerAllocator> Type;

  robot_IsMovingRequest_()
  {
  }

  robot_IsMovingRequest_(const ContainerAllocator& _alloc)
  {
  }


  typedef boost::shared_ptr< ::robot_comm::robot_IsMovingRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::robot_comm::robot_IsMovingRequest_<ContainerAllocator>  const> ConstPtr;
}; // struct robot_IsMovingRequest
typedef  ::robot_comm::robot_IsMovingRequest_<std::allocator<void> > robot_IsMovingRequest;

typedef boost::shared_ptr< ::robot_comm::robot_IsMovingRequest> robot_IsMovingRequestPtr;
typedef boost::shared_ptr< ::robot_comm::robot_IsMovingRequest const> robot_IsMovingRequestConstPtr;



template <class ContainerAllocator>
struct robot_IsMovingResponse_ {
  typedef robot_IsMovingResponse_<ContainerAllocator> Type;

  robot_IsMovingResponse_()
  : moving(false)
  , ret(0)
  , msg()
  {
  }

  robot_IsMovingResponse_(const ContainerAllocator& _alloc)
  : moving(false)
  , ret(0)
  , msg(_alloc)
  {
  }

  typedef uint8_t _moving_type;
  uint8_t moving;

  typedef int64_t _ret_type;
  int64_t ret;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _msg_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  msg;


  typedef boost::shared_ptr< ::robot_comm::robot_IsMovingResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::robot_comm::robot_IsMovingResponse_<ContainerAllocator>  const> ConstPtr;
}; // struct robot_IsMovingResponse
typedef  ::robot_comm::robot_IsMovingResponse_<std::allocator<void> > robot_IsMovingResponse;

typedef boost::shared_ptr< ::robot_comm::robot_IsMovingResponse> robot_IsMovingResponsePtr;
typedef boost::shared_ptr< ::robot_comm::robot_IsMovingResponse const> robot_IsMovingResponseConstPtr;


struct robot_IsMoving
{

typedef robot_IsMovingRequest Request;
typedef robot_IsMovingResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct robot_IsMoving
} // namespace robot_comm

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::robot_comm::robot_IsMovingRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::robot_comm::robot_IsMovingRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::robot_comm::robot_IsMovingRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const  ::robot_comm::robot_IsMovingRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::robot_comm::robot_IsMovingRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "robot_comm/robot_IsMovingRequest";
  }

  static const char* value(const  ::robot_comm::robot_IsMovingRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::robot_comm::robot_IsMovingRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
\n\
\n\
";
  }

  static const char* value(const  ::robot_comm::robot_IsMovingRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::robot_comm::robot_IsMovingRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::robot_comm::robot_IsMovingResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::robot_comm::robot_IsMovingResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::robot_comm::robot_IsMovingResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "6a8e4a0e30cc934246f28f5db62a1332";
  }

  static const char* value(const  ::robot_comm::robot_IsMovingResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x6a8e4a0e30cc9342ULL;
  static const uint64_t static_value2 = 0x46f28f5db62a1332ULL;
};

template<class ContainerAllocator>
struct DataType< ::robot_comm::robot_IsMovingResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "robot_comm/robot_IsMovingResponse";
  }

  static const char* value(const  ::robot_comm::robot_IsMovingResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::robot_comm::robot_IsMovingResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "bool moving\n\
int64 ret\n\
string msg\n\
\n\
\n\
";
  }

  static const char* value(const  ::robot_comm::robot_IsMovingResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::robot_comm::robot_IsMovingRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct robot_IsMovingRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::robot_comm::robot_IsMovingResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.moving);
    stream.next(m.ret);
    stream.next(m.msg);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct robot_IsMovingResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<robot_comm::robot_IsMoving> {
  static const char* value() 
  {
    return "6a8e4a0e30cc934246f28f5db62a1332";
  }

  static const char* value(const robot_comm::robot_IsMoving&) { return value(); } 
};

template<>
struct DataType<robot_comm::robot_IsMoving> {
  static const char* value() 
  {
    return "robot_comm/robot_IsMoving";
  }

  static const char* value(const robot_comm::robot_IsMoving&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<robot_comm::robot_IsMovingRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "6a8e4a0e30cc934246f28f5db62a1332";
  }

  static const char* value(const robot_comm::robot_IsMovingRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<robot_comm::robot_IsMovingRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "robot_comm/robot_IsMoving";
  }

  static const char* value(const robot_comm::robot_IsMovingRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<robot_comm::robot_IsMovingResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "6a8e4a0e30cc934246f28f5db62a1332";
  }

  static const char* value(const robot_comm::robot_IsMovingResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<robot_comm::robot_IsMovingResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "robot_comm/robot_IsMoving";
  }

  static const char* value(const robot_comm::robot_IsMovingResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // ROBOT_COMM_SERVICE_ROBOT_ISMOVING_H
