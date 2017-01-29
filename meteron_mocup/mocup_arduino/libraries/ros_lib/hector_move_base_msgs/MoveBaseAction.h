#ifndef _ROS_hector_move_base_msgs_MoveBaseAction_h
#define _ROS_hector_move_base_msgs_MoveBaseAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "hector_move_base_msgs/MoveBaseActionGoal.h"
#include "hector_move_base_msgs/MoveBaseActionResult.h"
#include "hector_move_base_msgs/MoveBaseActionFeedback.h"

namespace hector_move_base_msgs
{

  class MoveBaseAction : public ros::Msg
  {
    public:
      hector_move_base_msgs::MoveBaseActionGoal action_goal;
      hector_move_base_msgs::MoveBaseActionResult action_result;
      hector_move_base_msgs::MoveBaseActionFeedback action_feedback;

    MoveBaseAction():
      action_goal(),
      action_result(),
      action_feedback()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->action_goal.serialize(outbuffer + offset);
      offset += this->action_result.serialize(outbuffer + offset);
      offset += this->action_feedback.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->action_goal.deserialize(inbuffer + offset);
      offset += this->action_result.deserialize(inbuffer + offset);
      offset += this->action_feedback.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "hector_move_base_msgs/MoveBaseAction"; };
    const char * getMD5(){ return "ef19eb22e9e3d9474f8acbc89c91876b"; };

  };

}
#endif