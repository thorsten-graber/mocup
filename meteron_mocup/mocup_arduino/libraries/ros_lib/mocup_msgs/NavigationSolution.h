#ifndef _ROS_mocup_msgs_NavigationSolution_h
#define _ROS_mocup_msgs_NavigationSolution_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace mocup_msgs
{

  class NavigationSolution : public ros::Msg
  {
    public:
      std_msgs::Header header;
      float roll;
      float nick;
      float yaw;
      float droll;
      float dnick;
      float dyaw;
      float x;
      float y;
      float z;
      float d_x;
      float d_y;
      float d_z;

    NavigationSolution():
      header(),
      roll(0),
      nick(0),
      yaw(0),
      droll(0),
      dnick(0),
      dyaw(0),
      x(0),
      y(0),
      z(0),
      d_x(0),
      d_y(0),
      d_z(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_roll;
      u_roll.real = this->roll;
      *(outbuffer + offset + 0) = (u_roll.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_roll.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_roll.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_roll.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->roll);
      union {
        float real;
        uint32_t base;
      } u_nick;
      u_nick.real = this->nick;
      *(outbuffer + offset + 0) = (u_nick.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_nick.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_nick.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_nick.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->nick);
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
      } u_droll;
      u_droll.real = this->droll;
      *(outbuffer + offset + 0) = (u_droll.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_droll.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_droll.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_droll.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->droll);
      union {
        float real;
        uint32_t base;
      } u_dnick;
      u_dnick.real = this->dnick;
      *(outbuffer + offset + 0) = (u_dnick.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dnick.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_dnick.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_dnick.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dnick);
      union {
        float real;
        uint32_t base;
      } u_dyaw;
      u_dyaw.real = this->dyaw;
      *(outbuffer + offset + 0) = (u_dyaw.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dyaw.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_dyaw.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_dyaw.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dyaw);
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
      } u_z;
      u_z.real = this->z;
      *(outbuffer + offset + 0) = (u_z.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_z.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_z.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_z.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->z);
      union {
        float real;
        uint32_t base;
      } u_d_x;
      u_d_x.real = this->d_x;
      *(outbuffer + offset + 0) = (u_d_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_d_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_d_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_d_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->d_x);
      union {
        float real;
        uint32_t base;
      } u_d_y;
      u_d_y.real = this->d_y;
      *(outbuffer + offset + 0) = (u_d_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_d_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_d_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_d_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->d_y);
      union {
        float real;
        uint32_t base;
      } u_d_z;
      u_d_z.real = this->d_z;
      *(outbuffer + offset + 0) = (u_d_z.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_d_z.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_d_z.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_d_z.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->d_z);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_roll;
      u_roll.base = 0;
      u_roll.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_roll.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_roll.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_roll.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->roll = u_roll.real;
      offset += sizeof(this->roll);
      union {
        float real;
        uint32_t base;
      } u_nick;
      u_nick.base = 0;
      u_nick.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_nick.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_nick.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_nick.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->nick = u_nick.real;
      offset += sizeof(this->nick);
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
      } u_droll;
      u_droll.base = 0;
      u_droll.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_droll.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_droll.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_droll.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->droll = u_droll.real;
      offset += sizeof(this->droll);
      union {
        float real;
        uint32_t base;
      } u_dnick;
      u_dnick.base = 0;
      u_dnick.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_dnick.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_dnick.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_dnick.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->dnick = u_dnick.real;
      offset += sizeof(this->dnick);
      union {
        float real;
        uint32_t base;
      } u_dyaw;
      u_dyaw.base = 0;
      u_dyaw.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_dyaw.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_dyaw.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_dyaw.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->dyaw = u_dyaw.real;
      offset += sizeof(this->dyaw);
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
      } u_z;
      u_z.base = 0;
      u_z.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_z.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_z.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_z.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->z = u_z.real;
      offset += sizeof(this->z);
      union {
        float real;
        uint32_t base;
      } u_d_x;
      u_d_x.base = 0;
      u_d_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_d_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_d_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_d_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->d_x = u_d_x.real;
      offset += sizeof(this->d_x);
      union {
        float real;
        uint32_t base;
      } u_d_y;
      u_d_y.base = 0;
      u_d_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_d_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_d_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_d_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->d_y = u_d_y.real;
      offset += sizeof(this->d_y);
      union {
        float real;
        uint32_t base;
      } u_d_z;
      u_d_z.base = 0;
      u_d_z.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_d_z.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_d_z.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_d_z.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->d_z = u_d_z.real;
      offset += sizeof(this->d_z);
     return offset;
    }

    const char * getType(){ return "mocup_msgs/NavigationSolution"; };
    const char * getMD5(){ return "45d93b8c71cb9a03c6be120126e5ff84"; };

  };

}
#endif