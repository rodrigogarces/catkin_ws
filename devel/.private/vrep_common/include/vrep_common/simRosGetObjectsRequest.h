// Generated by gencpp from file vrep_common/simRosGetObjectsRequest.msg
// DO NOT EDIT!


#ifndef VREP_COMMON_MESSAGE_SIMROSGETOBJECTSREQUEST_H
#define VREP_COMMON_MESSAGE_SIMROSGETOBJECTSREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace vrep_common
{
template <class ContainerAllocator>
struct simRosGetObjectsRequest_
{
  typedef simRosGetObjectsRequest_<ContainerAllocator> Type;

  simRosGetObjectsRequest_()
    : objectType(0)  {
    }
  simRosGetObjectsRequest_(const ContainerAllocator& _alloc)
    : objectType(0)  {
  (void)_alloc;
    }



   typedef int32_t _objectType_type;
  _objectType_type objectType;




  typedef boost::shared_ptr< ::vrep_common::simRosGetObjectsRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vrep_common::simRosGetObjectsRequest_<ContainerAllocator> const> ConstPtr;

}; // struct simRosGetObjectsRequest_

typedef ::vrep_common::simRosGetObjectsRequest_<std::allocator<void> > simRosGetObjectsRequest;

typedef boost::shared_ptr< ::vrep_common::simRosGetObjectsRequest > simRosGetObjectsRequestPtr;
typedef boost::shared_ptr< ::vrep_common::simRosGetObjectsRequest const> simRosGetObjectsRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::vrep_common::simRosGetObjectsRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::vrep_common::simRosGetObjectsRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace vrep_common

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'vrep_common': ['/home/raphael/catkin_ws/src/vrep_common/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::vrep_common::simRosGetObjectsRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::vrep_common::simRosGetObjectsRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::vrep_common::simRosGetObjectsRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::vrep_common::simRosGetObjectsRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::vrep_common::simRosGetObjectsRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::vrep_common::simRosGetObjectsRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::vrep_common::simRosGetObjectsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f18b65c1a814107158daa3b6bc83f24b";
  }

  static const char* value(const ::vrep_common::simRosGetObjectsRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf18b65c1a8141071ULL;
  static const uint64_t static_value2 = 0x58daa3b6bc83f24bULL;
};

template<class ContainerAllocator>
struct DataType< ::vrep_common::simRosGetObjectsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "vrep_common/simRosGetObjectsRequest";
  }

  static const char* value(const ::vrep_common::simRosGetObjectsRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::vrep_common::simRosGetObjectsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
\n\
\n\
\n\
int32 objectType\n\
";
  }

  static const char* value(const ::vrep_common::simRosGetObjectsRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::vrep_common::simRosGetObjectsRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.objectType);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct simRosGetObjectsRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::vrep_common::simRosGetObjectsRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::vrep_common::simRosGetObjectsRequest_<ContainerAllocator>& v)
  {
    s << indent << "objectType: ";
    Printer<int32_t>::stream(s, indent + "  ", v.objectType);
  }
};

} // namespace message_operations
} // namespace ros

#endif // VREP_COMMON_MESSAGE_SIMROSGETOBJECTSREQUEST_H
