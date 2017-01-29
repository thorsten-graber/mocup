#ifndef _ROS_mocup_msgs_PositionFeedback_h
#define _ROS_mocup_msgs_PositionFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace mocup_msgs
{

  class PositionFeedback : public ros::Msg
  {
    public:
      std_msgs::Header header;
      float x;
      float y;
      float yaw;
      float varianceX;
      float varianceY;
      float varianceYaw;
      float varianceXY;
      float varianceXYaw;
      float varianceYYaw;

    PositionFeedback():
      header(),
      x(0),
      y(0),
      yaw(0),
      varianceX(0),
      varianceY(0),
      varianceYaw(0),
      varianceXY(0),
      varianceXYaw(0),
      varianceYYaw(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_x;
      u_x.real = this->x;
      *(outbuffer + offset + 0) = (u_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->x);
      union {
        float real;
        uint32_t base;
      } u_y;
      u_y.real = this->y;
      *(outbuffer + offset + 0) = (u_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->y);
      union {
        float real;
        uint32_t base;
      } u_yaw;
      u_yaw.real = this->yaw;
      *(outbuffer + offset + 0) = (u_yaw.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_yaw.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_yaw.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_yaw.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->yaw);
      union {
        float real;
        uint32_t base;
      } u_varianceX;
      u_varianceX.real = this->varianceX;
      *(outbuffer + offset + 0) = (u_varianceX.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_varianceX.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_varianceX.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_varianceX.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->varianceX);
      union {
        float real;
        uint32_t base;
      } u_varianceY;
      u_varianceY.real = this->varianceY;
      *(outbuffer + offset + 0) = (u_varianceY.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_varianceY.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_varianceY.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_varianceY.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->varianceY);
      union {
        float real;
        uint32_t base;
      } u_varianceYaw;
      u_varianceYaw.real = this->varianceYaw;
      *(outbuffer + offset + 0) = (u_varianceYaw.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_varianceYaw.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_varianceYaw.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_varianceYaw.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->varianceYaw);
      union {
        float real;
        uint32_t base;
      } u_varianceXY;
      u_varianceXY.real = this->varianceXY;
      *(outbuffer + offset + 0) = (u_varianceXY.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_varianceXY.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_varianceXY.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_varianceXY.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->varianceXY);
      union {
        float real;
        uint32_t base;
      } u_varianceXYaw;
      u_varianceXYaw.real = this->varianceXYaw;
      *(outbuffer + offset + 0) = (u_varianceXYaw.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_varianceXYaw.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_varianceXYaw.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_varianceXYaw.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->varianceXYaw);
      union {
        float real;
        uint32_t base;
      } u_varianceYYaw;
      u_varianceYYaw.real = this->varianceYYaw;
      *(outbuffer + offset + 0) = (u_varianceYYaw.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_varianceYYaw.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_varianceYYaw.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_varianceYYaw.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->varianceYYaw);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_x;
      u_x.base = 0;
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->x = u_x.real;
      offset += sizeof(this->x);
      union {
        float real;
        uint32_t base;
      } u_y;
      u_y.base = 0;
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->y = u_y.real;
      offset += sizeof(this->y);
      union {
        float real;
        uint32_t base;
      } u_yaw;
      u_yaw.base = 0;
      u_yaw.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_yaw.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_yaw.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_yaw.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->yaw = u_yaw.real;
      offset += sizeof(this->yaw);
      union {
        float real;
        uint32_t base;
      } u_varianceX;
      u_varianceX.base = 0;
      u_varianceX.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_varianceX.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_varianceX.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_varianceX.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->varianceX = u_varianceX.real;
      offset += sizeof(this->varianceX);
      union {
        float real;
        uint32_t base;
      } u_varianceY;
      u_varianceY.base = 0;
      u_varianceY.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_varianceY.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_varianceY.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_varianceY.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->varianceY = u_varianceY.real;
      offset += sizeof(this->varianceY);
      union {
        float real;
        uint32_t base;
      } u_varianceYaw;
      u_varianceYaw.base = 0;
      u_varianceYaw.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_varianceYaw.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_varianceYaw.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_varianceYaw.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->varianceYaw = u_varianceYaw.real;
      offset += sizeof(this->varianceYaw);
      union {
        float real;
        uint32_t base;
      } u_varianceXY;
      u_varianceXY.base = 0;
      u_varianceXY.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_varianceXY.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_varianceXY.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_varianceXY.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->varianceXY = u_varianceXY.real;
      offset += sizeof(this->varianceXY);
      union {
        float real;
        uint32_t base;
      } u_varianceXYaw;
      u_varianceXYaw.base = 0;
      u_varianceXYaw.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_varianceXYaw.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_varianceXYaw.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_varianceXYaw.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->varianceXYaw = u_varianceXYaw.real;
      offset += sizeof(this->varianceXYaw);
      union {
        float real;
        uint32_t base;
      } u_varianceYYaw;
      u_varianceYYaw.base = 0;
      u_varianceYYaw.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_varianceYYaw.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_varianceYYaw.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_varianceYYaw.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->varianceYYaw = u_varianceYYaw.real;
      offset += sizeof(this->varianceYYaw);
     return offset;
    }

    const char * getType(){ return "mocup_msgs/PositionFeedback"; };
    const char * getMD5(){ return "0b421832e703e2cf9d54d09ba6e26172"; };

  };

}
#endif