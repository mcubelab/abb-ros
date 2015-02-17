/* Auto-generated by genmsg_cpp for file /home/mcube/hands_MLab/code/nodes/robot/ROS/robot_comm/msg/robot_ForceLog.msg */
#ifndef ROBOT_COMM_MESSAGE_ROBOT_FORCELOG_H
#define ROBOT_COMM_MESSAGE_ROBOT_FORCELOG_H
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


namespace robot_comm
{
template <class ContainerAllocator>
struct robot_ForceLog_ {
  typedef robot_ForceLog_<ContainerAllocator> Type;

  robot_ForceLog_()
  : date()
  , time()
  , timeStamp(0.0)
  , fx(0.0)
  , fy(0.0)
  , fz(0.0)
  , tx(0.0)
  , ty(0.0)
  , tz(0.0)
  {
  }

  robot_ForceLog_(const ContainerAllocator& _alloc)
  : date(_alloc)
  , time(_alloc)
  , timeStamp(0.0)
  , fx(0.0)
  , fy(0.0)
  , fz(0.0)
  , tx(0.0)
  , ty(0.0)
  , tz(0.0)
  {
  }

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _date_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  date;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _time_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  time;

  typedef double _timeStamp_type;
  double timeStamp;

  typedef double _fx_type;
  double fx;

  typedef double _fy_type;
  double fy;

  typedef double _fz_type;
  double fz;

  typedef double _tx_type;
  double tx;

  typedef double _ty_type;
  double ty;

  typedef double _tz_type;
  double tz;


  typedef boost::shared_ptr< ::robot_comm::robot_ForceLog_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::robot_comm::robot_ForceLog_<ContainerAllocator>  const> ConstPtr;
}; // struct robot_ForceLog
typedef  ::robot_comm::robot_ForceLog_<std::allocator<void> > robot_ForceLog;

typedef boost::shared_ptr< ::robot_comm::robot_ForceLog> robot_ForceLogPtr;
typedef boost::shared_ptr< ::robot_comm::robot_ForceLog const> robot_ForceLogConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::robot_comm::robot_ForceLog_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::robot_comm::robot_ForceLog_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace robot_comm

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::robot_comm::robot_ForceLog_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::robot_comm::robot_ForceLog_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::robot_comm::robot_ForceLog_<ContainerAllocator> > {
  static const char* value() 
  {
    return "e69a688c0d45a806a3e0dc3ba264486c";
  }

  static const char* value(const  ::robot_comm::robot_ForceLog_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xe69a688c0d45a806ULL;
  static const uint64_t static_value2 = 0xa3e0dc3ba264486cULL;
};

template<class ContainerAllocator>
struct DataType< ::robot_comm::robot_ForceLog_<ContainerAllocator> > {
  static const char* value() 
  {
    return "robot_comm/robot_ForceLog";
  }

  static const char* value(const  ::robot_comm::robot_ForceLog_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::robot_comm::robot_ForceLog_<ContainerAllocator> > {
  static const char* value() 
  {
    return "string date\n\
string time\n\
float64 timeStamp\n\
float64 fx\n\
float64 fy\n\
float64 fz\n\
float64 tx\n\
float64 ty\n\
float64 tz\n\
\n\
";
  }

  static const char* value(const  ::robot_comm::robot_ForceLog_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::robot_comm::robot_ForceLog_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.date);
    stream.next(m.time);
    stream.next(m.timeStamp);
    stream.next(m.fx);
    stream.next(m.fy);
    stream.next(m.fz);
    stream.next(m.tx);
    stream.next(m.ty);
    stream.next(m.tz);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct robot_ForceLog_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::robot_comm::robot_ForceLog_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::robot_comm::robot_ForceLog_<ContainerAllocator> & v) 
  {
    s << indent << "date: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.date);
    s << indent << "time: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.time);
    s << indent << "timeStamp: ";
    Printer<double>::stream(s, indent + "  ", v.timeStamp);
    s << indent << "fx: ";
    Printer<double>::stream(s, indent + "  ", v.fx);
    s << indent << "fy: ";
    Printer<double>::stream(s, indent + "  ", v.fy);
    s << indent << "fz: ";
    Printer<double>::stream(s, indent + "  ", v.fz);
    s << indent << "tx: ";
    Printer<double>::stream(s, indent + "  ", v.tx);
    s << indent << "ty: ";
    Printer<double>::stream(s, indent + "  ", v.ty);
    s << indent << "tz: ";
    Printer<double>::stream(s, indent + "  ", v.tz);
  }
};


} // namespace message_operations
} // namespace ros

#endif // ROBOT_COMM_MESSAGE_ROBOT_FORCELOG_H

