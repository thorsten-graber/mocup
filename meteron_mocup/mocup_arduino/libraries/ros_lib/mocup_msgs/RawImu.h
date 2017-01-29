#ifndef _ROS_mocup_msgs_RawImu_h
#define _ROS_mocup_msgs_RawImu_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace mocup_msgs
{

  class RawImu : public ros::Msg
  {
    public:
      std_msgs::Header header;
      float w_x;
      float w_y;
      float w_z;
      float a_x;
      float a_y;
      float a_z;

    RawImu():
      header(),
      w_x(0),
      w_y(0),
      w_z(0),
      a_x(0),
      a_y(0),
      a_z(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_w_x;
      u_w_x.real = this->w_x;
      *(outbuffer + offset + 0) = (u_w_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_w_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_w_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_w_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->w_x);
      union {
        float real;
        uint32_t base;
      } u_w_y;
      u_w_y.real = this->w_y;
      *(outbuffer + offset + 0) = (u_w_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_w_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_w_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_w_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->w_y);
      union {
        float real;
        uint32_t base;
      } u_w_z;
      u_w_z.real = this->w_z;
      *(outbuffer + offset + 0) = (u_w_z.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_w_z.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_w_z.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_w_z.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->w_z);
      union {
        float real;
        uint32_t base;
      } u_a_x;
      u_a_x.real = this->a_x;
      *(outbuffer + offset + 0) = (u_a_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_a_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_a_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_a_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->a_x);
      union {
        float real;
        uint32_t base;
      } u_a_y;
      u_a_y.real = this->a_y;
      *(outbuffer + offset + 0) = (u_a_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_a_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_a_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_a_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->a_y);
      union {
        float real;
        uint32_t base;
      } u_a_z;
      u_a_z.real = this->a_z;
      *(outbuffer + offset + 0) = (u_a_z.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_a_z.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_a_z.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_a_z.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->a_z);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_w_x;
      u_w_x.base = 0;
      u_w_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_w_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_w_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_w_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->w_x = u_w_x.real;
      offset += sizeof(this->w_x);
      union {
        float real;
        uint32_t base;
      } u_w_y;
      u_w_y.base = 0;
      u_w_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_w_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_w_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_w_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->w_y = u_w_y.real;
      offset += sizeof(this->w_y);
      union {
        float real;
        uint32_t base;
      } u_w_z;
      u_w_z.base = 0;
      u_w_z.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_w_z.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_w_z.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_w_z.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->w_z = u_w_z.real;
      offset += sizeof(this->w_z);
      union {
        float real;
        uint32_t base;
      } u_a_x;
      u_a_x.base = 0;
      u_a_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_a_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_a_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_a_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->a_x = u_a_x.real;
      offset += sizeof(this->a_x);
      union {
        float real;
        uint32_t base;
      } u_a_y;
      u_a_y.base = 0;
      u_a_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_a_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_a_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_a_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->a_y = u_a_y.real;
      offset += sizeof(this->a_y);
      union {
        float real;
        uint32_t base;
      } u_a_z;
      u_a_z.base = 0;
      u_a_z.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_a_z.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_a_z.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_a_z.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->a_z = u_a_z.real;
      offset += sizeof(this->a_z);
     return offset;
    }

    const char * getType(){ return "mocup_msgs/RawImu"; };
    const char * getMD5(){ return "46b8b9dc3bb20946232996e98df291dc"; };

  };

}
#endif