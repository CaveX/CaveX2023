// Generated by gencpp from file syropod_remote/AndroidJoy.msg
// DO NOT EDIT!


#ifndef SYROPOD_REMOTE_MESSAGE_ANDROIDJOY_H
#define SYROPOD_REMOTE_MESSAGE_ANDROIDJOY_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int8.h>

namespace syropod_remote
{
template <class ContainerAllocator>
struct AndroidJoy_
{
  typedef AndroidJoy_<ContainerAllocator> Type;

  AndroidJoy_()
    : header()
    , id_name()
    , override_priority_interface()
    , primary_control_axis()
    , secondary_control_axis()
    , system_state()
    , robot_state()
    , gait_selection()
    , cruise_control_mode()
    , auto_navigation_mode()
    , posing_mode()
    , pose_reset_mode()
    , primary_leg_selection()
    , secondary_leg_selection()
    , primary_leg_state()
    , secondary_leg_state()
    , parameter_selection()
    , parameter_adjustment()  {
    }
  AndroidJoy_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , id_name(_alloc)
    , override_priority_interface(_alloc)
    , primary_control_axis(_alloc)
    , secondary_control_axis(_alloc)
    , system_state(_alloc)
    , robot_state(_alloc)
    , gait_selection(_alloc)
    , cruise_control_mode(_alloc)
    , auto_navigation_mode(_alloc)
    , posing_mode(_alloc)
    , pose_reset_mode(_alloc)
    , primary_leg_selection(_alloc)
    , secondary_leg_selection(_alloc)
    , primary_leg_state(_alloc)
    , secondary_leg_state(_alloc)
    , parameter_selection(_alloc)
    , parameter_adjustment(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::std_msgs::String_<ContainerAllocator>  _id_name_type;
  _id_name_type id_name;

   typedef  ::std_msgs::Bool_<ContainerAllocator>  _override_priority_interface_type;
  _override_priority_interface_type override_priority_interface;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _primary_control_axis_type;
  _primary_control_axis_type primary_control_axis;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _secondary_control_axis_type;
  _secondary_control_axis_type secondary_control_axis;

   typedef  ::std_msgs::Int8_<ContainerAllocator>  _system_state_type;
  _system_state_type system_state;

   typedef  ::std_msgs::Int8_<ContainerAllocator>  _robot_state_type;
  _robot_state_type robot_state;

   typedef  ::std_msgs::Int8_<ContainerAllocator>  _gait_selection_type;
  _gait_selection_type gait_selection;

   typedef  ::std_msgs::Int8_<ContainerAllocator>  _cruise_control_mode_type;
  _cruise_control_mode_type cruise_control_mode;

   typedef  ::std_msgs::Int8_<ContainerAllocator>  _auto_navigation_mode_type;
  _auto_navigation_mode_type auto_navigation_mode;

   typedef  ::std_msgs::Int8_<ContainerAllocator>  _posing_mode_type;
  _posing_mode_type posing_mode;

   typedef  ::std_msgs::Int8_<ContainerAllocator>  _pose_reset_mode_type;
  _pose_reset_mode_type pose_reset_mode;

   typedef  ::std_msgs::Int8_<ContainerAllocator>  _primary_leg_selection_type;
  _primary_leg_selection_type primary_leg_selection;

   typedef  ::std_msgs::Int8_<ContainerAllocator>  _secondary_leg_selection_type;
  _secondary_leg_selection_type secondary_leg_selection;

   typedef  ::std_msgs::Int8_<ContainerAllocator>  _primary_leg_state_type;
  _primary_leg_state_type primary_leg_state;

   typedef  ::std_msgs::Int8_<ContainerAllocator>  _secondary_leg_state_type;
  _secondary_leg_state_type secondary_leg_state;

   typedef  ::std_msgs::Int8_<ContainerAllocator>  _parameter_selection_type;
  _parameter_selection_type parameter_selection;

   typedef  ::std_msgs::Int8_<ContainerAllocator>  _parameter_adjustment_type;
  _parameter_adjustment_type parameter_adjustment;





  typedef boost::shared_ptr< ::syropod_remote::AndroidJoy_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::syropod_remote::AndroidJoy_<ContainerAllocator> const> ConstPtr;

}; // struct AndroidJoy_

typedef ::syropod_remote::AndroidJoy_<std::allocator<void> > AndroidJoy;

typedef boost::shared_ptr< ::syropod_remote::AndroidJoy > AndroidJoyPtr;
typedef boost::shared_ptr< ::syropod_remote::AndroidJoy const> AndroidJoyConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::syropod_remote::AndroidJoy_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::syropod_remote::AndroidJoy_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::syropod_remote::AndroidJoy_<ContainerAllocator1> & lhs, const ::syropod_remote::AndroidJoy_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.id_name == rhs.id_name &&
    lhs.override_priority_interface == rhs.override_priority_interface &&
    lhs.primary_control_axis == rhs.primary_control_axis &&
    lhs.secondary_control_axis == rhs.secondary_control_axis &&
    lhs.system_state == rhs.system_state &&
    lhs.robot_state == rhs.robot_state &&
    lhs.gait_selection == rhs.gait_selection &&
    lhs.cruise_control_mode == rhs.cruise_control_mode &&
    lhs.auto_navigation_mode == rhs.auto_navigation_mode &&
    lhs.posing_mode == rhs.posing_mode &&
    lhs.pose_reset_mode == rhs.pose_reset_mode &&
    lhs.primary_leg_selection == rhs.primary_leg_selection &&
    lhs.secondary_leg_selection == rhs.secondary_leg_selection &&
    lhs.primary_leg_state == rhs.primary_leg_state &&
    lhs.secondary_leg_state == rhs.secondary_leg_state &&
    lhs.parameter_selection == rhs.parameter_selection &&
    lhs.parameter_adjustment == rhs.parameter_adjustment;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::syropod_remote::AndroidJoy_<ContainerAllocator1> & lhs, const ::syropod_remote::AndroidJoy_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace syropod_remote

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::syropod_remote::AndroidJoy_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::syropod_remote::AndroidJoy_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::syropod_remote::AndroidJoy_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::syropod_remote::AndroidJoy_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::syropod_remote::AndroidJoy_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::syropod_remote::AndroidJoy_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::syropod_remote::AndroidJoy_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f798248626a520efb6e3973bbe95d25a";
  }

  static const char* value(const ::syropod_remote::AndroidJoy_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf798248626a520efULL;
  static const uint64_t static_value2 = 0xb6e3973bbe95d25aULL;
};

template<class ContainerAllocator>
struct DataType< ::syropod_remote::AndroidJoy_<ContainerAllocator> >
{
  static const char* value()
  {
    return "syropod_remote/AndroidJoy";
  }

  static const char* value(const ::syropod_remote::AndroidJoy_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::syropod_remote::AndroidJoy_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"std_msgs/String id_name\n"
"std_msgs/Bool override_priority_interface\n"
"geometry_msgs/Point primary_control_axis\n"
"geometry_msgs/Point secondary_control_axis\n"
"std_msgs/Int8 system_state\n"
"std_msgs/Int8 robot_state\n"
"std_msgs/Int8 gait_selection\n"
"std_msgs/Int8 cruise_control_mode\n"
"std_msgs/Int8 auto_navigation_mode\n"
"std_msgs/Int8 posing_mode\n"
"std_msgs/Int8 pose_reset_mode\n"
"std_msgs/Int8 primary_leg_selection\n"
"std_msgs/Int8 secondary_leg_selection\n"
"std_msgs/Int8 primary_leg_state\n"
"std_msgs/Int8 secondary_leg_state\n"
"std_msgs/Int8 parameter_selection\n"
"std_msgs/Int8 parameter_adjustment\n"
"\n"
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
"MSG: std_msgs/String\n"
"string data\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Bool\n"
"bool data\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Int8\n"
"int8 data\n"
;
  }

  static const char* value(const ::syropod_remote::AndroidJoy_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::syropod_remote::AndroidJoy_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.id_name);
      stream.next(m.override_priority_interface);
      stream.next(m.primary_control_axis);
      stream.next(m.secondary_control_axis);
      stream.next(m.system_state);
      stream.next(m.robot_state);
      stream.next(m.gait_selection);
      stream.next(m.cruise_control_mode);
      stream.next(m.auto_navigation_mode);
      stream.next(m.posing_mode);
      stream.next(m.pose_reset_mode);
      stream.next(m.primary_leg_selection);
      stream.next(m.secondary_leg_selection);
      stream.next(m.primary_leg_state);
      stream.next(m.secondary_leg_state);
      stream.next(m.parameter_selection);
      stream.next(m.parameter_adjustment);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct AndroidJoy_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::syropod_remote::AndroidJoy_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::syropod_remote::AndroidJoy_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "id_name: ";
    s << std::endl;
    Printer< ::std_msgs::String_<ContainerAllocator> >::stream(s, indent + "  ", v.id_name);
    s << indent << "override_priority_interface: ";
    s << std::endl;
    Printer< ::std_msgs::Bool_<ContainerAllocator> >::stream(s, indent + "  ", v.override_priority_interface);
    s << indent << "primary_control_axis: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.primary_control_axis);
    s << indent << "secondary_control_axis: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.secondary_control_axis);
    s << indent << "system_state: ";
    s << std::endl;
    Printer< ::std_msgs::Int8_<ContainerAllocator> >::stream(s, indent + "  ", v.system_state);
    s << indent << "robot_state: ";
    s << std::endl;
    Printer< ::std_msgs::Int8_<ContainerAllocator> >::stream(s, indent + "  ", v.robot_state);
    s << indent << "gait_selection: ";
    s << std::endl;
    Printer< ::std_msgs::Int8_<ContainerAllocator> >::stream(s, indent + "  ", v.gait_selection);
    s << indent << "cruise_control_mode: ";
    s << std::endl;
    Printer< ::std_msgs::Int8_<ContainerAllocator> >::stream(s, indent + "  ", v.cruise_control_mode);
    s << indent << "auto_navigation_mode: ";
    s << std::endl;
    Printer< ::std_msgs::Int8_<ContainerAllocator> >::stream(s, indent + "  ", v.auto_navigation_mode);
    s << indent << "posing_mode: ";
    s << std::endl;
    Printer< ::std_msgs::Int8_<ContainerAllocator> >::stream(s, indent + "  ", v.posing_mode);
    s << indent << "pose_reset_mode: ";
    s << std::endl;
    Printer< ::std_msgs::Int8_<ContainerAllocator> >::stream(s, indent + "  ", v.pose_reset_mode);
    s << indent << "primary_leg_selection: ";
    s << std::endl;
    Printer< ::std_msgs::Int8_<ContainerAllocator> >::stream(s, indent + "  ", v.primary_leg_selection);
    s << indent << "secondary_leg_selection: ";
    s << std::endl;
    Printer< ::std_msgs::Int8_<ContainerAllocator> >::stream(s, indent + "  ", v.secondary_leg_selection);
    s << indent << "primary_leg_state: ";
    s << std::endl;
    Printer< ::std_msgs::Int8_<ContainerAllocator> >::stream(s, indent + "  ", v.primary_leg_state);
    s << indent << "secondary_leg_state: ";
    s << std::endl;
    Printer< ::std_msgs::Int8_<ContainerAllocator> >::stream(s, indent + "  ", v.secondary_leg_state);
    s << indent << "parameter_selection: ";
    s << std::endl;
    Printer< ::std_msgs::Int8_<ContainerAllocator> >::stream(s, indent + "  ", v.parameter_selection);
    s << indent << "parameter_adjustment: ";
    s << std::endl;
    Printer< ::std_msgs::Int8_<ContainerAllocator> >::stream(s, indent + "  ", v.parameter_adjustment);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SYROPOD_REMOTE_MESSAGE_ANDROIDJOY_H
