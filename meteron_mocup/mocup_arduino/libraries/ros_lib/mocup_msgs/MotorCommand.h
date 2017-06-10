#ifndef _ROS_mocup_msgs_MotorCommand_h
#define _ROS_mocup_msgs_MotorCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mocup_msgs
{

  class MotorCommand : public ros::Msg
  {
    public:
      int16_t speed_r;
      int16_t steer_r;
      int16_t speed_l;
      int16_t steer_l;
      int16_t cam_yaw;
      int16_t cam_pit;

    MotorCommand():
      speed_r(0),
      steer_r(0),
      speed_l(0),
      steer_l(0),
      cam_yaw(0),
      cam_pit(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_speed_r;
      u_speed_r.real = this->speed_r;
      *(outbuffer + offset + 0) = (u_speed_r.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed_r.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->speed_r);
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
      } u_speed_l;
      u_speed_l.real = this->speed_l;
      *(outbuffer + offset + 0) = (u_speed_l.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed_l.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->speed_l);
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
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_speed_r;
      u_speed_r.base = 0;
      u_speed_r.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed_r.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->speed_r = u_speed_r.real;
      offset += sizeof(this->speed_r);
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
      } u_speed_l;
      u_speed_l.base = 0;
      u_speed_l.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed_l.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->speed_l = u_speed_l.real;
      offset += sizeof(this->speed_l);
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
     return offset;
    }

    const char * getType(){ return "mocup_msgs/MotorCommand"; };
    const char * getMD5(){ return "9f2bdf884e2704d7e1ef4a03eec3192d"; };

  };

}
#endif