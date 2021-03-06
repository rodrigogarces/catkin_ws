// Generated by gencpp from file vrep_common/simRosSetObjectQuaternion.msg
// DO NOT EDIT!


#ifndef VREP_COMMON_MESSAGE_SIMROSSETOBJECTQUATERNION_H
#define VREP_COMMON_MESSAGE_SIMROSSETOBJECTQUATERNION_H

#include <ros/service_traits.h>


#include <vrep_common/simRosSetObjectQuaternionRequest.h>
#include <vrep_common/simRosSetObjectQuaternionResponse.h>


namespace vrep_common
{

struct simRosSetObjectQuaternion
{

typedef simRosSetObjectQuaternionRequest Request;
typedef simRosSetObjectQuaternionResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct simRosSetObjectQuaternion
} // namespace vrep_common


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::vrep_common::simRosSetObjectQuaternion > {
  static const char* value()
  {
    return "f5fe7b4813e58c37e0cb1c1585da009e";
  }

  static const char* value(const ::vrep_common::simRosSetObjectQuaternion&) { return value(); }
};

template<>
struct DataType< ::vrep_common::simRosSetObjectQuaternion > {
  static const char* value()
  {
    return "vrep_common/simRosSetObjectQuaternion";
  }

  static const char* value(const ::vrep_common::simRosSetObjectQuaternion&) { return value(); }
};


// service_traits::MD5Sum< ::vrep_common::simRosSetObjectQuaternionRequest> should match 
// service_traits::MD5Sum< ::vrep_common::simRosSetObjectQuaternion > 
template<>
struct MD5Sum< ::vrep_common::simRosSetObjectQuaternionRequest>
{
  static const char* value()
  {
    return MD5Sum< ::vrep_common::simRosSetObjectQuaternion >::value();
  }
  static const char* value(const ::vrep_common::simRosSetObjectQuaternionRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::vrep_common::simRosSetObjectQuaternionRequest> should match 
// service_traits::DataType< ::vrep_common::simRosSetObjectQuaternion > 
template<>
struct DataType< ::vrep_common::simRosSetObjectQuaternionRequest>
{
  static const char* value()
  {
    return DataType< ::vrep_common::simRosSetObjectQuaternion >::value();
  }
  static const char* value(const ::vrep_common::simRosSetObjectQuaternionRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::vrep_common::simRosSetObjectQuaternionResponse> should match 
// service_traits::MD5Sum< ::vrep_common::simRosSetObjectQuaternion > 
template<>
struct MD5Sum< ::vrep_common::simRosSetObjectQuaternionResponse>
{
  static const char* value()
  {
    return MD5Sum< ::vrep_common::simRosSetObjectQuaternion >::value();
  }
  static const char* value(const ::vrep_common::simRosSetObjectQuaternionResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::vrep_common::simRosSetObjectQuaternionResponse> should match 
// service_traits::DataType< ::vrep_common::simRosSetObjectQuaternion > 
template<>
struct DataType< ::vrep_common::simRosSetObjectQuaternionResponse>
{
  static const char* value()
  {
    return DataType< ::vrep_common::simRosSetObjectQuaternion >::value();
  }
  static const char* value(const ::vrep_common::simRosSetObjectQuaternionResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // VREP_COMMON_MESSAGE_SIMROSSETOBJECTQUATERNION_H
