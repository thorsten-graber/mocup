#ifndef _ROS_SERVICE_SetAlternativeTolerance_h
#define _ROS_SERVICE_SetAlternativeTolerance_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "actionlib_msgs/GoalID.h"

namespace mocup_msgs
{

static const char SETALTERNATIVETOLERANCE[] = "mocup_msgs/SetAlternativeTolerance";

  class SetAlternativeToleranceRequest : public ros::Msg
  {
    public:
      actionlib_msgs::GoalID goalID;
      float linearTolerance;
      float angularTolerance;

    SetAlternativeToleranceRequest():
      goalID(),
      linearTolerance(0),
      angularTolerance(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->goalID.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->linearTolerance);
      offset += serializeAvrFloat64(outbuffer + offset, this->angularTolerance);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->goalID.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->linearTolerance));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->angularTolerance));
     return offset;
    }

    const char * getType(){ return SETALTERNATIVETOLERANCE; };
    const char * getMD5(){ return "9332f7ba5e819792f5504c48f062b9f5"; };

  };

  class SetAlternativeToleranceResponse : public ros::Msg
  {
    public:

    SetAlternativeToleranceResponse()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return SETALTERNATIVETOLERANCE; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class SetAlternativeTolerance {
    public:
    typedef SetAlternativeToleranceRequest Request;
    typedef SetAlternativeToleranceResponse Response;
  };

}
#endif
