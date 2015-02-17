/* Auto-generated by genmsg_cpp for file /home/mcube/hands_MLab/code/nodes/hand/ROS/hand_comm/srv/hand_WaitRest.srv */
#ifndef HAND_COMM_SERVICE_HAND_WAITREST_H
#define HAND_COMM_SERVICE_HAND_WAITREST_H
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




namespace hand_comm
{
template <class ContainerAllocator>
struct hand_WaitRestRequest_ {
  typedef hand_WaitRestRequest_<ContainerAllocator> Type;

  hand_WaitRestRequest_()
  : delay(0.0)
  {
  }

  hand_WaitRestRequest_(const ContainerAllocator& _alloc)
  : delay(0.0)
  {
  }

  typedef double _delay_type;
  double delay;


  typedef boost::shared_ptr< ::hand_comm::hand_WaitRestRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hand_comm::hand_WaitRestRequest_<ContainerAllocator>  const> ConstPtr;
}; // struct hand_WaitRestRequest
typedef  ::hand_comm::hand_WaitRestRequest_<std::allocator<void> > hand_WaitRestRequest;

typedef boost::shared_ptr< ::hand_comm::hand_WaitRestRequest> hand_WaitRestRequestPtr;
typedef boost::shared_ptr< ::hand_comm::hand_WaitRestRequest const> hand_WaitRestRequestConstPtr;



template <class ContainerAllocator>
struct hand_WaitRestResponse_ {
  typedef hand_WaitRestResponse_<ContainerAllocator> Type;

  hand_WaitRestResponse_()
  : ret(0)
  , msg()
  {
  }

  hand_WaitRestResponse_(const ContainerAllocator& _alloc)
  : ret(0)
  , msg(_alloc)
  {
  }

  typedef int64_t _ret_type;
  int64_t ret;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _msg_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  msg;


  typedef boost::shared_ptr< ::hand_comm::hand_WaitRestResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hand_comm::hand_WaitRestResponse_<ContainerAllocator>  const> ConstPtr;
}; // struct hand_WaitRestResponse
typedef  ::hand_comm::hand_WaitRestResponse_<std::allocator<void> > hand_WaitRestResponse;

typedef boost::shared_ptr< ::hand_comm::hand_WaitRestResponse> hand_WaitRestResponsePtr;
typedef boost::shared_ptr< ::hand_comm::hand_WaitRestResponse const> hand_WaitRestResponseConstPtr;


struct hand_WaitRest
{

typedef hand_WaitRestRequest Request;
typedef hand_WaitRestResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct hand_WaitRest
} // namespace hand_comm

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::hand_comm::hand_WaitRestRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::hand_comm::hand_WaitRestRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::hand_comm::hand_WaitRestRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "f60229add3267e5e23ed968ef0720885";
  }

  static const char* value(const  ::hand_comm::hand_WaitRestRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xf60229add3267e5eULL;
  static const uint64_t static_value2 = 0x23ed968ef0720885ULL;
};

template<class ContainerAllocator>
struct DataType< ::hand_comm::hand_WaitRestRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "hand_comm/hand_WaitRestRequest";
  }

  static const char* value(const  ::hand_comm::hand_WaitRestRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::hand_comm::hand_WaitRestRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
float64 delay\n\
\n\
";
  }

  static const char* value(const  ::hand_comm::hand_WaitRestRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::hand_comm::hand_WaitRestRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::hand_comm::hand_WaitRestResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::hand_comm::hand_WaitRestResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::hand_comm::hand_WaitRestResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "1e32786be6359fbbb6259aee4f579d10";
  }

  static const char* value(const  ::hand_comm::hand_WaitRestResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x1e32786be6359fbbULL;
  static const uint64_t static_value2 = 0xb6259aee4f579d10ULL;
};

template<class ContainerAllocator>
struct DataType< ::hand_comm::hand_WaitRestResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "hand_comm/hand_WaitRestResponse";
  }

  static const char* value(const  ::hand_comm::hand_WaitRestResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::hand_comm::hand_WaitRestResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int64 ret\n\
string msg\n\
\n\
\n\
";
  }

  static const char* value(const  ::hand_comm::hand_WaitRestResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::hand_comm::hand_WaitRestRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.delay);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct hand_WaitRestRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::hand_comm::hand_WaitRestResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.ret);
    stream.next(m.msg);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct hand_WaitRestResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<hand_comm::hand_WaitRest> {
  static const char* value() 
  {
    return "db3a0abc5a7b38d65070db42a856d203";
  }

  static const char* value(const hand_comm::hand_WaitRest&) { return value(); } 
};

template<>
struct DataType<hand_comm::hand_WaitRest> {
  static const char* value() 
  {
    return "hand_comm/hand_WaitRest";
  }

  static const char* value(const hand_comm::hand_WaitRest&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<hand_comm::hand_WaitRestRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "db3a0abc5a7b38d65070db42a856d203";
  }

  static const char* value(const hand_comm::hand_WaitRestRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<hand_comm::hand_WaitRestRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "hand_comm/hand_WaitRest";
  }

  static const char* value(const hand_comm::hand_WaitRestRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<hand_comm::hand_WaitRestResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "db3a0abc5a7b38d65070db42a856d203";
  }

  static const char* value(const hand_comm::hand_WaitRestResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<hand_comm::hand_WaitRestResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "hand_comm/hand_WaitRest";
  }

  static const char* value(const hand_comm::hand_WaitRestResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // HAND_COMM_SERVICE_HAND_WAITREST_H

