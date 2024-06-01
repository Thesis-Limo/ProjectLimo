// Generated by gencpp from file limo_motion_controller/OverrideMotion.msg
// DO NOT EDIT!


#ifndef LIMO_MOTION_CONTROLLER_MESSAGE_OVERRIDEMOTION_H
#define LIMO_MOTION_CONTROLLER_MESSAGE_OVERRIDEMOTION_H

#include <ros/service_traits.h>


#include <limo_motion_controller/OverrideMotionRequest.h>
#include <limo_motion_controller/OverrideMotionResponse.h>


namespace limo_motion_controller
{

struct OverrideMotion
{

typedef OverrideMotionRequest Request;
typedef OverrideMotionResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct OverrideMotion
} // namespace limo_motion_controller


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::limo_motion_controller::OverrideMotion > {
  static const char* value()
  {
    return "4955e86101808a9dfce48b4756da7f72";
  }

  static const char* value(const ::limo_motion_controller::OverrideMotion&) { return value(); }
};

template<>
struct DataType< ::limo_motion_controller::OverrideMotion > {
  static const char* value()
  {
    return "limo_motion_controller/OverrideMotion";
  }

  static const char* value(const ::limo_motion_controller::OverrideMotion&) { return value(); }
};


// service_traits::MD5Sum< ::limo_motion_controller::OverrideMotionRequest> should match
// service_traits::MD5Sum< ::limo_motion_controller::OverrideMotion >
template<>
struct MD5Sum< ::limo_motion_controller::OverrideMotionRequest>
{
  static const char* value()
  {
    return MD5Sum< ::limo_motion_controller::OverrideMotion >::value();
  }
  static const char* value(const ::limo_motion_controller::OverrideMotionRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::limo_motion_controller::OverrideMotionRequest> should match
// service_traits::DataType< ::limo_motion_controller::OverrideMotion >
template<>
struct DataType< ::limo_motion_controller::OverrideMotionRequest>
{
  static const char* value()
  {
    return DataType< ::limo_motion_controller::OverrideMotion >::value();
  }
  static const char* value(const ::limo_motion_controller::OverrideMotionRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::limo_motion_controller::OverrideMotionResponse> should match
// service_traits::MD5Sum< ::limo_motion_controller::OverrideMotion >
template<>
struct MD5Sum< ::limo_motion_controller::OverrideMotionResponse>
{
  static const char* value()
  {
    return MD5Sum< ::limo_motion_controller::OverrideMotion >::value();
  }
  static const char* value(const ::limo_motion_controller::OverrideMotionResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::limo_motion_controller::OverrideMotionResponse> should match
// service_traits::DataType< ::limo_motion_controller::OverrideMotion >
template<>
struct DataType< ::limo_motion_controller::OverrideMotionResponse>
{
  static const char* value()
  {
    return DataType< ::limo_motion_controller::OverrideMotion >::value();
  }
  static const char* value(const ::limo_motion_controller::OverrideMotionResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // LIMO_MOTION_CONTROLLER_MESSAGE_OVERRIDEMOTION_H
