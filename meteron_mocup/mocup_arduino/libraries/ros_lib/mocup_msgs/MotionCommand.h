#ifndef _ROS_mocup_msgs_MotionCommand_h
#define _ROS_mocup_msgs_MotionCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mocup_msgs
{

  class MotionCommand : public ros::Msg
  {
    public:
      float speed;
      float steerAngleFront;
      float steerAngleRear;
      bool brake;
      const char* mode;

    MotionCommand():
      speed(0),
      steerAngleFront(0),
      steerAngleRear(0),
      brake(0),
      mode("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
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
      } u_steerAngleFront;
      u_steerAngleFront.real = this->steerAngleFront;
      *(outbuffer + offset + 0) = (u_steerAngleFront.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_steerAngleFront.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_steerAngleFront.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_steerAngleFront.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->steerAngleFront);
      union {
        float real;
        uint32_t base;
      } u_steerAngleRear;
      u_steerAngleRear.real = this->steerAngleRear;
      *(outbuffer + offset + 0) = (u_steerAngleRear.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_steerAngleRear.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_steerAngleRear.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_steerAngleRear.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->steerAngleRear);
      union {
        bool real;
        uint8_t base;
      } u_brake;
      u_brake.real = this->brake;
      *(outbuffer + offset + 0) = (u_brake.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->brake);
      uint32_t length_mode = strlen(this->mode);
      memcpy(outbuffer + offset, &length_mode, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->mode, length_mode);
      offset += length_mode;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
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
      } u_steerAngleFront;
      u_steerAngleFront.base = 0;
      u_steerAngleFront.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_steerAngleFront.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_steerAngleFront.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_steerAngleFront.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->steerAngleFront = u_steerAngleFront.real;
      offset += sizeof(this->steerAngleFront);
      union {
        float real;
        uint32_t base;
      } u_steerAngleRear;
      u_steerAngleRear.base = 0;
      u_steerAngleRear.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_steerAngleRear.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_steerAngleRear.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_steerAngleRear.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->steerAngleRear = u_steerAngleRear.real;
      offset += sizeof(this->steerAngleRear);
      union {
        bool real;
        uint8_t base;
      } u_brake;
      u_brake.base = 0;
      u_brake.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->brake = u_brake.real;
      offset += sizeof(this->brake);
      uint32_t length_mode;
      memcpy(&length_mode, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_mode; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_mode-1]=0;
      this->mode = (char *)(inbuffer + offset-1);
      offset += length_mode;
     return offset;
    }

    const char * getType(){ return "mocup_msgs/MotionCommand"; };
    const char * getMD5(){ return "24de0f88aab481c661b20c2a8ac59247"; };

  };

}
#endif