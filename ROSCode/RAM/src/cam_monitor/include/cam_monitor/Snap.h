// Generated by gencpp from file cam_monitor/Snap.msg
// DO NOT EDIT!


#ifndef CAM_MONITOR_MESSAGE_SNAP_H
#define CAM_MONITOR_MESSAGE_SNAP_H

#include <ros/service_traits.h>


#include <cam_monitor/SnapRequest.h>
#include <cam_monitor/SnapResponse.h>


namespace cam_monitor
{

struct Snap
{

typedef SnapRequest Request;
typedef SnapResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct Snap
} // namespace cam_monitor


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::cam_monitor::Snap > {
  static const char* value()
  {
    return "03da474bc61cfeb81a8854b4ca05bafa";
  }

  static const char* value(const ::cam_monitor::Snap&) { return value(); }
};

template<>
struct DataType< ::cam_monitor::Snap > {
  static const char* value()
  {
    return "cam_monitor/Snap";
  }

  static const char* value(const ::cam_monitor::Snap&) { return value(); }
};


// service_traits::MD5Sum< ::cam_monitor::SnapRequest> should match 
// service_traits::MD5Sum< ::cam_monitor::Snap > 
template<>
struct MD5Sum< ::cam_monitor::SnapRequest>
{
  static const char* value()
  {
    return MD5Sum< ::cam_monitor::Snap >::value();
  }
  static const char* value(const ::cam_monitor::SnapRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::cam_monitor::SnapRequest> should match 
// service_traits::DataType< ::cam_monitor::Snap > 
template<>
struct DataType< ::cam_monitor::SnapRequest>
{
  static const char* value()
  {
    return DataType< ::cam_monitor::Snap >::value();
  }
  static const char* value(const ::cam_monitor::SnapRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::cam_monitor::SnapResponse> should match 
// service_traits::MD5Sum< ::cam_monitor::Snap > 
template<>
struct MD5Sum< ::cam_monitor::SnapResponse>
{
  static const char* value()
  {
    return MD5Sum< ::cam_monitor::Snap >::value();
  }
  static const char* value(const ::cam_monitor::SnapResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::cam_monitor::SnapResponse> should match 
// service_traits::DataType< ::cam_monitor::Snap > 
template<>
struct DataType< ::cam_monitor::SnapResponse>
{
  static const char* value()
  {
    return DataType< ::cam_monitor::Snap >::value();
  }
  static const char* value(const ::cam_monitor::SnapResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // CAM_MONITOR_MESSAGE_SNAP_H
