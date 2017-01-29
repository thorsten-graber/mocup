#ifndef _ROS_mocup_msgs_Status_h
#define _ROS_mocup_msgs_Status_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace mocup_msgs
{

  class Status : public ros::Msg
  {
    public:
      std_msgs::Header header;
      float voltage1;
      float voltage2;
      uint16_t status;
      enum { AUTONOMOUS = 1 };
      enum { NAVIGATION_READY = 2 };
      enum { NAVIGATION_ODOMETRY = 4 };
      enum { NAVIGATION_GPS = 8 };
      enum { NAVIGATION_COMPASS = 16 };
      enum { NAVIGATION_UPDATE_2D = 32 };
      enum { NAVIGATION_UPDATE_3D = 64 };

    Status():
      header(),
      voltage1(0),
      voltage2(0),
      status(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_voltage1;
      u_voltage1.real = this->voltage1;
      *(outbuffer + offset + 0) = (u_voltage1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_voltage1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_voltage1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_voltage1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->voltage1);
      union {
        float real;
        uint32_t base;
      } u_voltage2;
      u_voltage2.real = this->voltage2;
      *(outbuffer + offset + 0) = (u_voltage2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_voltage2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_voltage2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_voltage2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->voltage2);
      *(outbuffer + offset + 0) = (this->status >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->status >> (8 * 1)) & 0xFF;
      offset += sizeof(this->status);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_voltage1;
      u_voltage1.base = 0;
      u_voltage1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_voltage1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_voltage1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_voltage1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->voltage1 = u_voltage1.real;
      offset += sizeof(this->voltage1);
      union {
        float real;
        uint32_t base;
      } u_voltage2;
      u_voltage2.base = 0;
      u_voltage2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_voltage2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_voltage2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_voltage2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->voltage2 = u_voltage2.real;
      offset += sizeof(this->voltage2);
      this->status =  ((uint16_t) (*(inbuffer + offset)));
      this->status |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->status);
     return offset;
    }

    const char * getType(){ return "mocup_msgs/Status"; };
    const char * getMD5(){ return "b401e47c89d5600c3cddee66fb6b4851"; };

  };

}
#endif