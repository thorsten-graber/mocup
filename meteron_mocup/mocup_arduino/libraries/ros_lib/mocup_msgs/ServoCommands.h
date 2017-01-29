#ifndef _ROS_mocup_msgs_ServoCommands_h
#define _ROS_mocup_msgs_ServoCommands_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "mocup_msgs/ServoCommand.h"

namespace mocup_msgs
{

  class ServoCommands : public ros::Msg
  {
    public:
      uint8_t servo_length;
      mocup_msgs::ServoCommand st_servo;
      mocup_msgs::ServoCommand * servo;

    ServoCommands():
      servo_length(0), servo(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = servo_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < servo_length; i++){
      offset += this->servo[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t servo_lengthT = *(inbuffer + offset++);
      if(servo_lengthT > servo_length)
        this->servo = (mocup_msgs::ServoCommand*)realloc(this->servo, servo_lengthT * sizeof(mocup_msgs::ServoCommand));
      offset += 3;
      servo_length = servo_lengthT;
      for( uint8_t i = 0; i < servo_length; i++){
      offset += this->st_servo.deserialize(inbuffer + offset);
        memcpy( &(this->servo[i]), &(this->st_servo), sizeof(mocup_msgs::ServoCommand));
      }
     return offset;
    }

    const char * getType(){ return "mocup_msgs/ServoCommands"; };
    const char * getMD5(){ return "370e1168616fe3787707a7aa29404a95"; };

  };

}
#endif