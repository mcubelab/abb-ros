/* Auto-generated by genmsg_cpp for file /home/mcube/hands_MLab/code/nodes/logger/ROS/logger_comm/srv/logger_Stop.srv */
#ifndef LOGGER_COMM_SERVICE_LOGGER_STOP_H
#define LOGGER_COMM_SERVICE_LOGGER_STOP_H
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




namespace logger_comm
{
template <class ContainerAllocator>
struct logger_StopRequest_ {
  typedef logger_StopRequest_<ContainerAllocator> Type;

  logger_StopRequest_()
  {
  }

  logger_StopRequest_(const ContainerAllocator& _alloc)
  {
  }


  typedef boost::shared_ptr< ::logger_comm::logger_StopRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::logger_comm::logger_StopRequest_<ContainerAllocator>  const> ConstPtr;
}; // struct logger_StopRequest
typedef  ::logger_comm::logger_StopRequest_<std::allocator<void> > logger_StopRequest;

typedef boost::shared_ptr< ::logger_comm::logger_StopRequest> logger_StopRequestPtr;
typedef boost::shared_ptr< ::logger_comm::logger_StopRequest const> logger_StopRequestConstPtr;



template <class ContainerAllocator>
struct logger_StopResponse_ {
  typedef logger_StopResponse_<ContainerAllocator> Type;

  logger_StopResponse_()
  : ret(0)
  , msg()
  {
  }

  logger_StopResponse_(const ContainerAllocator& _alloc)
  : ret(0)
  , msg(_alloc)
  {
  }

  typedef int64_t _ret_type;
  int64_t ret;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _msg_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  msg;


  typedef boost::shared_ptr< ::logger_comm::logger_StopResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::logger_comm::logger_StopResponse_<ContainerAllocator>  const> ConstPtr;
}; // struct logger_StopResponse
typedef  ::logger_comm::logger_StopResponse_<std::allocator<void> > logger_StopResponse;

typedef boost::shared_ptr< ::logger_comm::logger_StopResponse> logger_StopResponsePtr;
typedef boost::shared_ptr< ::logger_comm::logger_StopResponse const> logger_StopResponseConstPtr;


struct logger_Stop
{

typedef logger_StopRequest Request;
typedef logger_StopResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct logger_Stop
} // namespace logger_comm

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::logger_comm::logger_StopRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::logger_comm::logger_StopRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::logger_comm::logger_StopRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const  ::logger_comm::logger_StopRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::logger_comm::logger_StopRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "logger_comm/logger_StopRequest";
  }

  static const char* value(const  ::logger_comm::logger_StopRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::logger_comm::logger_StopRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
\n\
";
  }

  static const char* value(const  ::logger_comm::logger_StopRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::logger_comm::logger_StopRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::logger_comm::logger_StopResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::logger_comm::logger_StopResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::logger_comm::logger_StopResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "1e32786be6359fbbb6259aee4f579d10";
  }

  static const char* value(const  ::logger_comm::logger_StopResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x1e32786be6359fbbULL;
  static const uint64_t static_value2 = 0xb6259aee4f579d10ULL;
};

template<class ContainerAllocator>
struct DataType< ::logger_comm::logger_StopResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "logger_comm/logger_StopResponse";
  }

  static const char* value(const  ::logger_comm::logger_StopResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::logger_comm::logger_StopResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int64 ret\n\
string msg\n\
\n\
\n\
";
  }

  static const char* value(const  ::logger_comm::logger_StopResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::logger_comm::logger_StopRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct logger_StopRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::logger_comm::logger_StopResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.ret);
    stream.next(m.msg);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct logger_StopResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<logger_comm::logger_Stop> {
  static const char* value() 
  {
    return "1e32786be6359fbbb6259aee4f579d10";
  }

  static const char* value(const logger_comm::logger_Stop&) { return value(); } 
};

template<>
struct DataType<logger_comm::logger_Stop> {
  static const char* value() 
  {
    return "logger_comm/logger_Stop";
  }

  static const char* value(const logger_comm::logger_Stop&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<logger_comm::logger_StopRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "1e32786be6359fbbb6259aee4f579d10";
  }

  static const char* value(const logger_comm::logger_StopRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<logger_comm::logger_StopRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "logger_comm/logger_Stop";
  }

  static const char* value(const logger_comm::logger_StopRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<logger_comm::logger_StopResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "1e32786be6359fbbb6259aee4f579d10";
  }

  static const char* value(const logger_comm::logger_StopResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<logger_comm::logger_StopResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "logger_comm/logger_Stop";
  }

  static const char* value(const logger_comm::logger_StopResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // LOGGER_COMM_SERVICE_LOGGER_STOP_H
