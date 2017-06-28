#ifndef _ROS_mocup_msgs_RawSensors_h
#define _ROS_mocup_msgs_RawSensors_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mocup_msgs
{

  class RawSensors : public ros::Msg
  {
    public:
      int16_t wheel_r;
      int16_t steer_r;
      int16_t wheel_l;
      int16_t steer_l;
      int16_t cam_yaw;
      int16_t cam_pit;
      uint8_t us_fr;
      uint8_t us_fl;
      uint8_t us_rr;
      uint8_t us_rl;

    RawSensors():
      wheel_r(0),
      steer_r(0),
      wheel_l(0),
      steer_l(0),
      cam_yaw(0),
      cam_pit(0),
      us_fr(0),
      us_fl(0),
      us_rr(0),
      us_rl(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_wheel_r;
      u_wheel_r.real = this->wheel_r;
      *(outbuffer + offset + 0) = (u_wheel_r.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_wheel_r.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->wheel_r);
      union {
        int16_t real;
        uint16_t base;
      } u_steer_r;
      u_steer_r.real = this->steer_r;
      *(outbuffer + offset + 0) = (u_steer_r.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_steer_r.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->steer_r);
      union {
        int16_t real;
        uint16_t base;
      } u_wheel_l;
      u_wheel_l.real = this->wheel_l;
      *(outbuffer + offset + 0) = (u_wheel_l.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_wheel_l.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->wheel_l);
      union {
        int16_t real;
        uint16_t base;
      } u_steer_l;
      u_steer_l.real = this->steer_l;
      *(outbuffer + offset + 0) = (u_steer_l.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_steer_l.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->steer_l);
      union {
        int16_t real;
        uint16_t base;
      } u_cam_yaw;
      u_cam_yaw.real = this->cam_yaw;
      *(outbuffer + offset + 0) = (u_cam_yaw.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cam_yaw.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->cam_yaw);
      union {
        int16_t real;
        uint16_t base;
      } u_cam_pit;
      u_cam_pit.real = this->cam_pit;
      *(outbuffer + offset + 0) = (u_cam_pit.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cam_pit.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->cam_pit);
      *(outbuffer + offset + 0) = (this->us_fr >> (8 * 0)) & 0xFF;
      offset += sizeof(this->us_fr);
      *(outbuffer + offset + 0) = (this->us_fl >> (8 * 0)) & 0xFF;
      offset += sizeof(this->us_fl);
      *(outbuffer + offset + 0) = (this->us_rr >> (8 * 0)) & 0xFF;
      offset += sizeof(this->us_rr);
      *(outbuffer + offset + 0) = (this->us_rl >> (8 * 0)) & 0xFF;
      offset += sizeof(this->us_rl);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_wheel_r;
      u_wheel_r.base = 0;
      u_wheel_r.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_wheel_r.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->wheel_r = u_wheel_r.real;
      offset += sizeof(this->wheel_r);
      union {
        int16_t real;
        uint16_t base;
      } u_steer_r;
      u_steer_r.base = 0;
      u_steer_r.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_steer_r.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->steer_r = u_steer_r.real;
      offset += sizeof(this->steer_r);
      union {
        int16_t real;
        uint16_t base;
      } u_wheel_l;
      u_wheel_l.base = 0;
      u_wheel_l.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_wheel_l.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->wheel_l = u_wheel_l.real;
      offset += sizeof(this->wheel_l);
      union {
        int16_t real;
        uint16_t base;
      } u_steer_l;
      u_steer_l.base = 0;
      u_steer_l.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_steer_l.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->steer_l = u_steer_l.real;
      offset += sizeof(this->steer_l);
      union {
        int16_t real;
        uint16_t base;
      } u_cam_yaw;
      u_cam_yaw.base = 0;
      u_cam_yaw.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cam_yaw.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->cam_yaw = u_cam_yaw.real;
      offset += sizeof(this->cam_yaw);
      union {
        int16_t real;
        uint16_t base;
      } u_cam_pit;
      u_cam_pit.base = 0;
      u_cam_pit.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cam_pit.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->cam_pit = u_cam_pit.real;
      offset += sizeof(this->cam_pit);
      this->us_fr =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->us_fr);
      this->us_fl =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->us_fl);
      this->us_rr =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->us_rr);
      this->us_rl =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->us_rl);
     return offset;
    }

    const char * getType(){ return "mocup_msgs/RawSensors"; };
    const char * getMD5(){ return "9ded4a063536b1734421d7cfb1ac5fec"; };

  };

}
#endif