// Generated by gencpp from file vrep_common/simRosGetInfo.msg
// DO NOT EDIT!


#ifndef VREP_COMMON_MESSAGE_SIMROSGETINFO_H
#define VREP_COMMON_MESSAGE_SIMROSGETINFO_H

#include <ros/service_traits.h>


#include <vrep_common/simRosGetInfoRequest.h>
#include <vrep_common/simRosGetInfoResponse.h>


namespace vrep_common
{

struct simRosGetInfo
{

typedef simRosGetInfoRequest Request;
typedef simRosGetInfoResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct simRosGetInfo
} // namespace vrep_common


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::vrep_common::simRosGetInfo > {
  static const char* value()
  {
    return "2ab24cc264f8f17af7e013147c57dbc0";
  }

  static const char* value(const ::vrep_common::simRosGetInfo&) { return value(); }
};

template<>
struct DataType< ::vrep_common::simRosGetInfo > {
  static const char* value()
  {
    return "vrep_common/simRosGetInfo";
  }

  static const char* value(const ::vrep_common::simRosGetInfo&) { return value(); }
};


// service_traits::MD5Sum< ::vrep_common::simRosGetInfoRequest> should match 
// service_traits::MD5Sum< ::vrep_common::simRosGetInfo > 
template<>
struct MD5Sum< ::vrep_common::simRosGetInfoRequest>
{
  static const char* value()
  {
    return MD5Sum< ::vrep_common::simRosGetInfo >::value();
  }
  static const char* value(const ::vrep_common::simRosGetInfoRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::vrep_common::simRosGetInfoRequest> should match 
// service_traits::DataType< ::vrep_common::simRosGetInfo > 
template<>
struct DataType< ::vrep_common::simRosGetInfoRequest>
{
  static const char* value()
  {
    return DataType< ::vrep_common::simRosGetInfo >::value();
  }
  static const char* value(const ::vrep_common::simRosGetInfoRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::vrep_common::simRosGetInfoResponse> should match 
// service_traits::MD5Sum< ::vrep_common::simRosGetInfo > 
template<>
struct MD5Sum< ::vrep_common::simRosGetInfoResponse>
{
  static const char* value()
  {
    return MD5Sum< ::vrep_common::simRosGetInfo >::value();
  }
  static const char* value(const ::vrep_common::simRosGetInfoResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::vrep_common::simRosGetInfoResponse> should match 
// service_traits::DataType< ::vrep_common::simRosGetInfo > 
template<>
struct DataType< ::vrep_common::simRosGetInfoResponse>
{
  static const char* value()
  {
    return DataType< ::vrep_common::simRosGetInfo >::value();
  }
  static const char* value(const ::vrep_common::simRosGetInfoResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // VREP_COMMON_MESSAGE_SIMROSGETINFO_H