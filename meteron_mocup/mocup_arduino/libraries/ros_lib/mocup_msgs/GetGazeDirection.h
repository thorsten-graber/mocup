#ifndef _ROS_SERVICE_GetGazeDirection_h
#define _ROS_SERVICE_GetGazeDirection_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/PointStamped.h"
#include "ros/time.h"

namespace mocup_msgs
{

static const char GETGAZEDIRECTION[] = "mocup_msgs/GetGazeDirection";

  class GetGazeDirectionRequest : public ros::Msg
  {
    public:
      ros::Time stamp;

    GetGazeDirectionRequest():
      stamp()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->stamp.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stamp.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stamp.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stamp.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stamp.sec);
      *(outbuffer + offset + 0) = (this->stamp.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stamp.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stamp.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stamp.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stamp.nsec);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->stamp.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->stamp.sec);
      this->stamp.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->stamp.nsec);
     return offset;
    }

    const char * getType(){ return GETGAZEDIRECTION; };
    const char * getMD5(){ return "84d365d08d5fc49dde870daba1c7992c"; };

  };

  class GetGazeDirectionResponse : public ros::Msg
  {
    public:
      geometry_msgs::PointStamped pointOut;

    GetGazeDirectionResponse():
      pointOut()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->pointOut.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->pointOut.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return GETGAZEDIRECTION; };
    const char * getMD5(){ return "575e2bfcd2844c893c892df880361661"; };

  };

  class GetGazeDirection {
    public:
    typedef GetGazeDirectionRequest Request;
    typedef GetGazeDirectionResponse Response;
  };

}
#endif
