#ifndef _ROS_hector_move_base_msgs_MoveBaseActionExplore_h
#define _ROS_hector_move_base_msgs_MoveBaseActionExplore_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalID.h"
#include "hector_move_base_msgs/MoveBaseExplore.h"

namespace hector_move_base_msgs
{

  class MoveBaseActionExplore : public ros::Msg
  {
    public:
      std_msgs::Header header;
      actionlib_msgs::GoalID goal_id;
      hector_move_base_msgs::MoveBaseExplore goal;

    MoveBaseActionExplore():
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

    const char * getType(){ return "hector_move_base_msgs/MoveBaseActionExplore"; };
    const char * getMD5(){ return "9f099efa6d81450c557f59f9dfac4454"; };

  };

}
#endif