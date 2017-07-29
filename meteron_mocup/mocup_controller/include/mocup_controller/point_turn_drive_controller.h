/*
    Copyright (c) 2016, Thorsten Graber
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
        * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
        * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
        * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY Antons Rebguns <email> ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL Antons Rebguns <email> BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef POINT_TURN_DRIVE_CONTROLLER_H
#define POINT_TURN_DRIVE_CONTROLLER_H

#include <ros/ros.h>
#include "vehicle_control_interface.h"

class PointTurnDriveController: public VehicleControlInterface
{
  public:
    virtual void configure(ros::NodeHandle& params, MotionParameters* mp) // todo: does not take alternative goal tolerances (set by service) into account
    {
      mp_ = mp;

      ros::NodeHandle nh;
      drivePublisher_ = nh.advertise<mocup_msgs::MotionCommand>("drive", 1);

      max_steeringangle = 45.0 * M_PI/180.0;
      params.getParam("max_steeringangle", max_steeringangle);
    }

    virtual void executeTwist(const geometry_msgs::Twist& velocity)
    {
      float backward = (velocity.linear.x < 0) ? -1.0 : 1.0;
      float speed = backward * sqrt(velocity.linear.x*velocity.linear.x + velocity.linear.y*velocity.linear.y);
      mp_->limitSpeed(speed);

      float kappa = velocity.angular.z * speed;
      float tan_gamma = tan(velocity.linear.y / velocity.linear.x);

      setDriveCommand(speed, kappa, tan_gamma);
    }

    virtual void executeMotionCommand(double carrot_relative_angle, double carrot_orientation_error, double carrot_distance, double speed)
    {
        ROS_DEBUG("carrot_relative_angle: %.5f",carrot_relative_angle);
        ROS_DEBUG("speed: %.1f",speed);

        if (fabs(carrot_relative_angle) > mp_->goal_angle_tolerance) {
            // -> point turn to align with target
            float sign;
            if(carrot_relative_angle > 0.0) {
                // rotate clock wise
                sign = 1.0;
            } else {
                // rotate counter clock wise
                sign = -1.0;
            }
            drive.speed = sign*fabs(speed);
            drive.mode = "point_turn";
            drivePublisher_.publish(drive);
            return;
        }

        // relative angle OK, drive straight to target
        drive.speed = fabs(speed);
        drive.mode = "continuous";
        drivePublisher_.publish(drive);
    }

    virtual void stop()
    {
      drive.speed = 0.0;
      drive.mode = "continuous";
      drivePublisher_.publish(drive);
    }

    virtual double getCommandedSpeed() const
    {
      return drive.speed;
    }

    virtual std::string getName()
    {
      return "Point Turn Drive Controller";
    }

    void setDriveCommand(float speed, float kappa, float tan_gamma) {

      float B = 0.16; // half wheel distance (front - rear)

      drive.speed = speed;
      mp_->limitSpeed(drive.speed);

      if (drive.speed != 0.0) {
        float max_kappa = tan(max_steeringangle) / B;
        if (kappa >= max_kappa) {
          kappa = max_kappa;
          tan_gamma = 0;

        } else if (kappa <= -max_kappa) {
          kappa = -max_kappa;
          tan_gamma = 0;

        } else {
          float max_tan_gamma = tan(max_steeringangle) - fabs(kappa) * B;
          if (tan_gamma >  max_tan_gamma) tan_gamma =  max_tan_gamma;
          if (tan_gamma < -max_tan_gamma) tan_gamma = -max_tan_gamma;
        }

        drive.steer = atan( tan_gamma + kappa * B);
        if(speed < 0) {
            drive.steer = -drive.steer;
        }
        drive.steer  = -drive.steer;//atan(-tan_gamma + kappa * B);
        drive.mode = "continuous";
      }
      drivePublisher_.publish(drive);
    }

  protected:
    ros::Publisher drivePublisher_;

    mocup_msgs::MotionCommand drive;

    MotionParameters* mp_;

    double max_steeringangle;
};

#endif
