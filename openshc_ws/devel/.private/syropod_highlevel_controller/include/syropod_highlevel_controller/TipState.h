// Generated by gencpp from file syropod_highlevel_controller/TipState.msg
// DO NOT EDIT!


#ifndef SYROPOD_HIGHLEVEL_CONTROLLER_MESSAGE_TIPSTATE_H
#define SYROPOD_HIGHLEVEL_CONTROLLER_MESSAGE_TIPSTATE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Vector3.h>

namespace syropod_highlevel_controller
{
template <class ContainerAllocator>
struct TipState_
{
  typedef TipState_<ContainerAllocator> Type;

  TipState_()
    : header()
    , name()
    , wrench()
    , step_plane()  {
    }
  TipState_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , name(_alloc)
    , wrench(_alloc)
    , step_plane(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> _name_type;
  _name_type name;

   typedef std::vector< ::geometry_msgs::Wrench_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::geometry_msgs::Wrench_<ContainerAllocator> >> _wrench_type;
  _wrench_type wrench;

   typedef std::vector< ::geometry_msgs::Vector3_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::geometry_msgs::Vector3_<ContainerAllocator> >> _step_plane_type;
  _step_plane_type step_plane;





  typedef boost::shared_ptr< ::syropod_highlevel_controller::TipState_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::syropod_highlevel_controller::TipState_<ContainerAllocator> const> ConstPtr;

}; // struct TipState_

typedef ::syropod_highlevel_controller::TipState_<std::allocator<void> > TipState;

typedef boost::shared_ptr< ::syropod_highlevel_controller::TipState > TipStatePtr;
typedef boost::shared_ptr< ::syropod_highlevel_controller::TipState const> TipStateConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::syropod_highlevel_controller::TipState_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::syropod_highlevel_controller::TipState_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::syropod_highlevel_controller::TipState_<ContainerAllocator1> & lhs, const ::syropod_highlevel_controller::TipState_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.name == rhs.name &&
    lhs.wrench == rhs.wrench &&
    lhs.step_plane == rhs.step_plane;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::syropod_highlevel_controller::TipState_<ContainerAllocator1> & lhs, const ::syropod_highlevel_controller::TipState_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace syropod_highlevel_controller

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::syropod_highlevel_controller::TipState_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::syropod_highlevel_controller::TipState_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::syropod_highlevel_controller::TipState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::syropod_highlevel_controller::TipState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::syropod_highlevel_controller::TipState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::syropod_highlevel_controller::TipState_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::syropod_highlevel_controller::TipState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "24a3486efb85ea2d52231aaad58ea97c";
  }

  static const char* value(const ::syropod_highlevel_controller::TipState_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x24a3486efb85ea2dULL;
  static const uint64_t static_value2 = 0x52231aaad58ea97cULL;
};

template<class ContainerAllocator>
struct DataType< ::syropod_highlevel_controller::TipState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "syropod_highlevel_controller/TipState";
  }

  static const char* value(const ::syropod_highlevel_controller::TipState_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::syropod_highlevel_controller::TipState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"string[] name\n"
"geometry_msgs/Wrench[] wrench \n"
"geometry_msgs/Vector3[] step_plane\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Wrench\n"
"# This represents force in free space, separated into\n"
"# its linear and angular parts.\n"
"Vector3  force\n"
"Vector3  torque\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Vector3\n"
"# This represents a vector in free space. \n"
"# It is only meant to represent a direction. Therefore, it does not\n"
"# make sense to apply a translation to it (e.g., when applying a \n"
"# generic rigid transformation to a Vector3, tf2 will only apply the\n"
"# rotation). If you want your data to be translatable too, use the\n"
"# geometry_msgs/Point message instead.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::syropod_highlevel_controller::TipState_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::syropod_highlevel_controller::TipState_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.name);
      stream.next(m.wrench);
      stream.next(m.step_plane);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TipState_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::syropod_highlevel_controller::TipState_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::syropod_highlevel_controller::TipState_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "name[]" << std::endl;
    for (size_t i = 0; i < v.name.size(); ++i)
    {
      s << indent << "  name[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.name[i]);
    }
    s << indent << "wrench[]" << std::endl;
    for (size_t i = 0; i < v.wrench.size(); ++i)
    {
      s << indent << "  wrench[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::geometry_msgs::Wrench_<ContainerAllocator> >::stream(s, indent + "    ", v.wrench[i]);
    }
    s << indent << "step_plane[]" << std::endl;
    for (size_t i = 0; i < v.step_plane.size(); ++i)
    {
      s << indent << "  step_plane[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "    ", v.step_plane[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // SYROPOD_HIGHLEVEL_CONTROLLER_MESSAGE_TIPSTATE_H
