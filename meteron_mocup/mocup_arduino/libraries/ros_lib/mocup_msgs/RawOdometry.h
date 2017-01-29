#ifndef _ROS_mocup_msgs_RawOdometry_h
#define _ROS_mocup_msgs_RawOdometry_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace mocup_msgs
{

  class RawOdometry : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint32_t tics_fl;
      uint32_t tics_fr;
      uint32_t tics_rl;
      uint32_t tics_rr;
      float v_fl;
      float v_fr;
      float v_rl;
      float v_rr;
      float speed;
      float yawRate;

    RawOdometry():
      header(),
      tics_fl(0),
      tics_fr(0),
      tics_rl(0),
      tics_rr(0),
      v_fl(0),
      v_fr(0),
      v_rl(0),
      v_rr(0),
      speed(0),
      yawRate(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->tics_fl >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->tics_fl >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->tics_fl >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->tics_fl >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tics_fl);
      *(outbuffer + offset + 0) = (this->tics_fr >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->tics_fr >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->tics_fr >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->tics_fr >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tics_fr);
      *(outbuffer + offset + 0) = (this->tics_rl >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->tics_rl >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->tics_rl >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->tics_rl >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tics_rl);
      *(outbuffer + offset + 0) = (this->tics_rr >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->tics_rr >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->tics_rr >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->tics_rr >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tics_rr);
      union {
        float real;
        uint32_t base;
      } u_v_fl;
      u_v_fl.real = this->v_fl;
      *(outbuffer + offset + 0) = (u_v_fl.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_v_fl.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_v_fl.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_v_fl.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->v_fl);
      union {
        float real;
        uint32_t base;
      } u_v_fr;
      u_v_fr.real = this->v_fr;
      *(outbuffer + offset + 0) = (u_v_fr.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_v_fr.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_v_fr.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_v_fr.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->v_fr);
      union {
        float real;
        uint32_t base;
      } u_v_rl;
      u_v_rl.real = this->v_rl;
      *(outbuffer + offset + 0) = (u_v_rl.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_v_rl.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_v_rl.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_v_rl.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->v_rl);
      union {
        float real;
        uint32_t base;
      } u_v_rr;
      u_v_rr.real = this->v_rr;
      *(outbuffer + offset + 0) = (u_v_rr.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_v_rr.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_v_rr.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_v_rr.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->v_rr);
      union {
        float real;
        uint32_t base;
      } u_speed;
      u_speed.real = this->speed;
      *(outbuffer + offset + 0) = (u_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_speed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_speed.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->speed);
      union {
        float real;
        uint32_t base;
      } u_yawRate;
      u_yawRate.real = this->yawRate;
      *(outbuffer + offset + 0) = (u_yawRate.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_yawRate.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_yawRate.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_yawRate.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->yawRate);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->tics_fl =  ((uint32_t) (*(inbuffer + offset)));
      this->tics_fl |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->tics_fl |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->tics_fl |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->tics_fl);
      this->tics_fr =  ((uint32_t) (*(inbuffer + offset)));
      this->tics_fr |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->tics_fr |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->tics_fr |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->tics_fr);
      this->tics_rl =  ((uint32_t) (*(inbuffer + offset)));
      this->tics_rl |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->tics_rl |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->tics_rl |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->tics_rl);
      this->tics_rr =  ((uint32_t) (*(inbuffer + offset)));
      this->tics_rr |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->tics_rr |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->tics_rr |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->tics_rr);
      union {
        float real;
        uint32_t base;
      } u_v_fl;
      u_v_fl.base = 0;
      u_v_fl.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_v_fl.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_v_fl.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_v_fl.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->v_fl = u_v_fl.real;
      offset += sizeof(this->v_fl);
      union {
        float real;
        uint32_t base;
      } u_v_fr;
      u_v_fr.base = 0;
      u_v_fr.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_v_fr.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_v_fr.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_v_fr.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->v_fr = u_v_fr.real;
      offset += sizeof(this->v_fr);
      union {
        float real;
        uint32_t base;
      } u_v_rl;
      u_v_rl.base = 0;
      u_v_rl.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_v_rl.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_v_rl.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_v_rl.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->v_rl = u_v_rl.real;
      offset += sizeof(this->v_rl);
      union {
        float real;
        uint32_t base;
      } u_v_rr;
      u_v_rr.base = 0;
      u_v_rr.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_v_rr.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_v_rr.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_v_rr.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->v_rr = u_v_rr.real;
      offset += sizeof(this->v_rr);
      union {
        float real;
        uint32_t base;
      } u_speed;
      u_speed.base = 0;
      u_speed.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_speed.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_speed.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->speed = u_speed.real;
      offset += sizeof(this->speed);
      union {
        float real;
        uint32_t base;
      } u_yawRate;
      u_yawRate.base = 0;
      u_yawRate.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_yawRate.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_yawRate.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_yawRate.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->yawRate = u_yawRate.real;
      offset += sizeof(this->yawRate);
     return offset;
    }

    const char * getType(){ return "mocup_msgs/RawOdometry"; };
    const char * getMD5(){ return "16be9e146c33fd79f2291a429164cfb3"; };

  };

}
#endif