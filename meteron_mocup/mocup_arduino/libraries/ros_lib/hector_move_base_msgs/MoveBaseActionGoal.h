#ifndef _ROS_hector_move_base_msgs_MoveBaseActionGoal_h
#define _ROS_hector_move_base_msgs_MoveBaseActionGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalID.h"
#include "hector_move_base_msgs/MoveBaseGoal.h"

namespace hector_move_base_msgs
{

  class MoveBaseActionGoal : public ros::Msg
  {
    public:
      std_msgs::Header header;
      actionlib_msgs::GoalID goal_id;
      hector_move_base_msgs::MoveBaseGoal goal;

    MoveBaseActionGoal():
      header(),
      goal_id(),
      goal()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->goal_id.serialize(outbuffer + offset);
      offset += this->goal.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->goal_id.deserialize(inbuffer + offset);
      offset += this->goal.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "hector_move_base_msgs/MoveBaseActionGoal"; };
    const char * getMD5(){ return "0ccfb483cbd1e6f40fa90f23e0469177"; };

  };

}
#endif