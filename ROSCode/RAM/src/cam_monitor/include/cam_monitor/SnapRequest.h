// Generated by gencpp from file cam_monitor/SnapRequest.msg
// DO NOT EDIT!


#ifndef CAM_MONITOR_MESSAGE_SNAPREQUEST_H
#define CAM_MONITOR_MESSAGE_SNAPREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace cam_monitor
{
template <class ContainerAllocator>
struct SnapRequest_
{
  typedef SnapRequest_<ContainerAllocator> Type;

  SnapRequest_()
    {
    }
  SnapRequest_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::cam_monitor::SnapRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cam_monitor::SnapRequest_<ContainerAllocator> const> ConstPtr;

}; // struct SnapRequest_

typedef ::cam_monitor::SnapRequest_<std::allocator<void> > SnapRequest;

typedef boost::shared_ptr< ::cam_monitor::SnapRequest > SnapRequestPtr;
typedef boost::shared_ptr< ::cam_monitor::SnapRequest const> SnapRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::cam_monitor::SnapRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::cam_monitor::SnapRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace cam_monitor

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::cam_monitor::SnapRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cam_monitor::SnapRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cam_monitor::SnapRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cam_monitor::SnapRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cam_monitor::SnapRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cam_monitor::SnapRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::cam_monitor::SnapRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::cam_monitor::SnapRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::cam_monitor::SnapRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cam_monitor/SnapRequest";
  }

  static const char* value(const ::cam_monitor::SnapRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::cam_monitor::SnapRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
";
  }

  static const char* value(const ::cam_monitor::SnapRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::cam_monitor::SnapRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SnapRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cam_monitor::SnapRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::cam_monitor::SnapRequest_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // CAM_MONITOR_MESSAGE_SNAPREQUEST_H
