/* Auto-generated by genmsg_cpp for file /home/mcube/hands_MLab/code/nodes/logger/ROS/logger_comm/srv/logger_Start.srv */
#ifndef LOGGER_COMM_SERVICE_LOGGER_START_H
#define LOGGER_COMM_SERVICE_LOGGER_START_H
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
struct logger_StartRequest_ {
  typedef logger_StartRequest_<ContainerAllocator> Type;

  logger_StartRequest_()
  : id(0)
  , folder()
  {
  }

  logger_StartRequest_(const ContainerAllocator& _alloc)
  : id(0)
  , folder(_alloc)
  {
  }

  typedef int64_t _id_type;
  int64_t id;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _folder_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  folder;


  typedef boost::shared_ptr< ::logger_comm::logger_StartRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::logger_comm::logger_StartRequest_<ContainerAllocator>  const> ConstPtr;
}; // struct logger_StartRequest
typedef  ::logger_comm::logger_StartRequest_<std::allocator<void> > logger_StartRequest;

typedef boost::shared_ptr< ::logger_comm::logger_StartRequest> logger_StartRequestPtr;
typedef boost::shared_ptr< ::logger_comm::logger_StartRequest const> logger_StartRequestConstPtr;



template <class ContainerAllocator>
struct logger_StartResponse_ {
  typedef logger_StartResponse_<ContainerAllocator> Type;

  logger_StartResponse_()
  : filename()
  , ret(0)
  , msg()
  {
  }

  logger_StartResponse_(const ContainerAllocator& _alloc)
  : filename(_alloc)
  , ret(0)
  , msg(_alloc)
  {
  }

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _filename_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  filename;

  typedef int64_t _ret_type;
  int64_t ret;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _msg_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  msg;


  typedef boost::shared_ptr< ::logger_comm::logger_StartResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::logger_comm::logger_StartResponse_<ContainerAllocator>  const> ConstPtr;
}; // struct logger_StartResponse
typedef  ::logger_comm::logger_StartResponse_<std::allocator<void> > logger_StartResponse;

typedef boost::shared_ptr< ::logger_comm::logger_StartResponse> logger_StartResponsePtr;
typedef boost::shared_ptr< ::logger_comm::logger_StartResponse const> logger_StartResponseConstPtr;


struct logger_Start
{

typedef logger_StartRequest Request;
typedef logger_StartResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct logger_Start
} // namespace logger_comm

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::logger_comm::logger_StartRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::logger_comm::logger_StartRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::logger_comm::logger_StartRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "8a2f83175a4a2157ff798d04511637ea";
  }

  static const char* value(const  ::logger_comm::logger_StartRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x8a2f83175a4a2157ULL;
  static const uint64_t static_value2 = 0xff798d04511637eaULL;
};

template<class ContainerAllocator>
struct DataType< ::logger_comm::logger_StartRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "logger_comm/logger_StartRequest";
  }

  static const char* value(const  ::logger_comm::logger_StartRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::logger_comm::logger_StartRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
int64 id\n\
string folder\n\
\n\
";
  }

  static const char* value(const  ::logger_comm::logger_StartRequest_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::logger_comm::logger_StartResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::logger_comm::logger_StartResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::logger_comm::logger_StartResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "c38c43175a19a935e6ed91a689ba16f4";
  }

  static const char* value(const  ::logger_comm::logger_StartResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xc38c43175a19a935ULL;
  static const uint64_t static_value2 = 0xe6ed91a689ba16f4ULL;
};

template<class ContainerAllocator>
struct DataType< ::logger_comm::logger_StartResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "logger_comm/logger_StartResponse";
  }

  static const char* value(const  ::logger_comm::logger_StartResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::logger_comm::logger_StartResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "string filename\n\
int64 ret\n\
string msg\n\
\n\
\n\
";
  }

  static const char* value(const  ::logger_comm::logger_StartResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::logger_comm::logger_StartRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.id);
    stream.next(m.folder);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct logger_StartRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::logger_comm::logger_StartResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.filename);
    stream.next(m.ret);
    stream.next(m.msg);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct logger_StartResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<logger_comm::logger_Start> {
  static const char* value() 
  {
    return "90487e5efdcd9de2a1cd8bed7b6e2d81";
  }

  static const char* value(const logger_comm::logger_Start&) { return value(); } 
};

template<>
struct DataType<logger_comm::logger_Start> {
  static const char* value() 
  {
    return "logger_comm/logger_Start";
  }

  static const char* value(const logger_comm::logger_Start&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<logger_comm::logger_StartRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "90487e5efdcd9de2a1cd8bed7b6e2d81";
  }

  static const char* value(const logger_comm::logger_StartRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<logger_comm::logger_StartRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "logger_comm/logger_Start";
  }

  static const char* value(const logger_comm::logger_StartRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<logger_comm::logger_StartResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "90487e5efdcd9de2a1cd8bed7b6e2d81";
  }

  static const char* value(const logger_comm::logger_StartResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<logger_comm::logger_StartResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "logger_comm/logger_Start";
  }

  static const char* value(const logger_comm::logger_StartResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // LOGGER_COMM_SERVICE_LOGGER_START_H

