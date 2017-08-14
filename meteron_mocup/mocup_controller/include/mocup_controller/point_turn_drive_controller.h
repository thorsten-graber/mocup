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

        wheelBase = 0.304;
        params.getParam("wheelBase", wheelBase);
        wheelTrack = 0.295;
        params.getParam("wheelTrack", wheelTrack);

        reset();
    }

    virtual void executeTwist(const geometry_msgs::Twist& velocity)
    {
        float backward = (velocity.linear.x < 0) ? -1.0 : 1.0;
        float speed = backward * sqrt(velocity.linear.x*velocity.linear.x + velocity.linear.y*velocity.linear.y);
        mp_->limitSpeed(speed);

//        float omega = velocity.angular.z;
//        float tan_gamma = velocity.linear.y / velocity.linear.x;
//        setDriveCommand(speed, omega, tan_gamma);

//          float omega = velocity.angular.z;
//          float atan_gamma = atan2(velocity.linear.y / velocity.linear.x);
//          publishDriveCommand(speed, omega, atan_gamma);

          float omega = velocity.angular.z;
          float atan_gamma = atan2(velocity.linear.y , velocity.linear.x);
          driveCommand(speed, omega, atan_gamma);

    }

    virtual void executeMotionCommand(double carrot_relative_angle, double carrot_orientation_error, double carrot_distance, double speed)
    {
        ROS_DEBUG("carrot_relative_angle: %.5f",carrot_relative_angle);
        ROS_DEBUG("speed: %.1f",speed);

        if ((fabs(carrot_relative_angle) > mp_->goal_angle_tolerance) || (fabs(carrot_relative_angle) > mp_->goal_angle_tolerance/8 && point_turn)) { // todo expose second parameter to launch file
            point_turn = true;
            // -> point turn to align with target
            float sign;
            if(carrot_relative_angle > 0.0) {
                // rotate clock wise
                sign = 1.0;
            } else {
                // rotate counter clock wise
                sign = -1.0;
            }

            ROS_DEBUG("carrot_relative_angle: %f, mode: point_turn", carrot_relative_angle);

            drive.speed = sign*fabs(speed)/10; //todo:expose parameter to launch file
            drive.steer = M_PI_4;
            drive.mode = "point_turn";
            drivePublisher_.publish(drive);
            return;
        }

        ROS_DEBUG("carrot_relative_angle: %f, mode: contineous", carrot_relative_angle);
        point_turn = false;

        // relative angle OK, drive straight to target
        drive.speed = fabs(speed);
        drive.steer = 0.0;
        drive.mode = "continuous";
        drivePublisher_.publish(drive);
    }

    virtual void stop()
    {
        drive.speed = 0.0;
        drivePublisher_.publish(drive);
    }

    virtual void reset()
    {
        point_turn = true;
    }

    virtual double getCommandedSpeed() const
    {
        return drive.speed;
    }

    virtual std::string getName()
    {
        return "Point Turn Drive Controller";
    }

    void driveCommand(float speed, float omega, float atan_gamma) {
        float l, b;

        b = wheelTrack;
        l = wheelBase;

        float A,B;

        if(speed != 0.0 || omega != 0.0) {
            float sign = (omega < 0) ? -1.0 : 1.0;

            A = (omega*b/2) / (speed+(omega*b/2));
            B = speed / (speed+(omega*b/2));

            drive.speed = A * (omega*b/2)   + B * speed;
            drive.steer = A * sign * M_PI_2 + B * atan_gamma;
            drive.mode  = "continuous";
        } else {
            drive.speed = 0.0;
            drive.steer = 0.0;
            drive.mode  = "continuous";
        }

        ROS_INFO("A: %f, B: %f", A, B);

        drivePublisher_.publish(drive);
    }

    void setDriveCommand(float speed, float omega, float tan_gamma) {

        float l, b;
        float speed_l, speed_r;

        b = wheelTrack;
        l = wheelBase;

        speed_l = 0.0;
        speed_r = 0.0;

        if(speed != 0.0) {
            speed_l = speed*(1+b*tan_gamma/l);
            speed_r = speed*(1-b*tan_gamma/l);
        }
        if(omega != 0.0) {
            speed_l += -omega * b;
            speed_r +=  omega * b;
        }

        if(speed_l == speed_r) {
            drive.steer = 0.0;
        } else {
            drive.steer = atan(((speed_r - speed_l)*l)/((speed_l + speed_r)*b));
            if(speed < 0) drive.steer = atan(((speed_l - speed_r)*l)/((speed_r + speed_l)*b));
        }

        drive.speed = speed + fabs(omega*b);
        mp_->limitSpeed(drive.speed);
        drive.mode = "continuous";

        drivePublisher_.publish(drive);
    }

    void publishDriveCommand(float speed, float omega, float atan_gamma) {
        float l = wheelBase;

        float sign = (speed < 0) ? -1.0 : 1.0;
        drive.steer = sign*atan_gamma + atan((omega*l)/(2*fabs(speed)));
        if(speed == 0.0) drive.steer = 0;

        drive.speed = speed;
        mp_->limitSpeed(drive.speed);
        drive.mode = "continuous";

        drivePublisher_.publish(drive);
    }

protected:
    ros::Publisher drivePublisher_;

    mocup_msgs::MotionCommand drive;

    MotionParameters* mp_;

    double wheelRadius, wheelBase, wheelTrack;
    bool point_turn;
};

#endif
