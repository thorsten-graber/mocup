#ifndef _ROS_hector_move_base_msgs_MoveBaseActionGeneric_h
#define _ROS_hector_move_base_msgs_MoveBaseActionGeneric_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalID.h"

namespace hector_move_base_msgs
{

  class MoveBaseActionGeneric : public ros::Msg
  {
    public:
      std_msgs::Header header;
      actionlib_msgs::GoalID goal_id;
      uint8_t type;
      uint8_t goal_length;
      uint8_t st_goal;
      uint8_t * goal;
      enum { GOAL =  1 };
      enum { PATH =  2 };
      enum { EXPLORE =  3 };

    MoveBaseActionGeneric():
      header(),
      goal_id(),
      type(0),
      goal_length(0), goal(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->goal_id.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->type >> (8 * 0)) & 0xFF;
      offset += sizeof(this->type);
      *(outbuffer + offset++) = goal_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < goal_length; i++){
      *(outbuffer + offset + 0) = (this->goal[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->goal[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->goal_id.deserialize(inbuffer + offset);
      this->type =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->type);
      uint8_t goal_lengthT = *(inbuffer + offset++);
      if(goal_lengthT > goal_length)
        this->goal = (uint8_t*)realloc(this->goal, goal_lengthT * sizeof(uint8_t));
      offset += 3;
      goal_length = goal_lengthT;
      for( uint8_t i = 0; i < goal_length; i++){
      this->st_goal =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->st_goal);
        memcpy( &(this->goal[i]), &(this->st_goal), sizeof(uint8_t));
      }
     return offset;
    }

    const char * getType(){ return "hector_move_base_msgs/MoveBaseActionGeneric"; };
    const char * getMD5(){ return "8ce7dbf3bac6ada8b079fa43ad02e966"; };

  };

}
#endif