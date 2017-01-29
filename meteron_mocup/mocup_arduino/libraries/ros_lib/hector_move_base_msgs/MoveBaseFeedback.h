#ifndef _ROS_hector_move_base_msgs_MoveBaseFeedback_h
#define _ROS_hector_move_base_msgs_MoveBaseFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace hector_move_base_msgs
{

  class MoveBaseFeedback : public ros::Msg
  {
    public:

    MoveBaseFeedback()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return "hector_move_base_msgs/MoveBaseFeedback"; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

}
#endif