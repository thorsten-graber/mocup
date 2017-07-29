/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/*
 * Desc: ROS interface to a Position2d controller for a Differential drive.
 * Author: Daniel Hewlett (adapted from Nathan Koenig)
 */

#include <algorithm>
#include <assert.h>
#include <cmath>

#include <mocup_gazebo_plugins/all_wheel_steering_plugin.h>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>

#if (GAZEBO_MAJOR_VERSION > 1) || (GAZEBO_MINOR_VERSION >= 2)
#define RADIAN Radian
#else
#define RADIAN GetAsRadian
#endif

namespace gazebo {

GZ_REGISTER_MODEL_PLUGIN(AllWheelSteeringPlugin)

enum
{
    FRONT_RIGHT = 0, FRONT_LEFT = 1, MIDDLE_RIGHT = 2, MIDDLE_LEFT = 3, REAR_RIGHT = 4, REAR_LEFT = 5
};

// Constructor
AllWheelSteeringPlugin::AllWheelSteeringPlugin()
{
    rosnode_ = 0;
}

// Destructor
AllWheelSteeringPlugin::~AllWheelSteeringPlugin()
{
    sub_.shutdown();
    delete rosnode_;
}

// Load the controller
void AllWheelSteeringPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    // get physics
    model = _model;
    world = _model->GetWorld();

    // default parameters
    topicName = "drive";
    odomTopicName = "odom";
    jointStateName = "joint_states";
    proportionalControllerGain = 8.0;
    derivativeControllerGain = 0.0;
    wheelTrack = 0.30;
    wheelBase = 0.305;
    wheelRadius = 0.035;
    jointMaxTorque = 10.0;
    wheelMaxTorque = 10.0;
    maxVelX = 0.1;

    // load parameters
    if (_sdf->HasElement("robotNamespace")) robotNamespace = _sdf->Get<std::string>("robotNamespace");
    if (_sdf->HasElement("topicName")) topicName = _sdf->Get<std::string>("topicName");
    if (_sdf->HasElement("odomTopicName")) odomTopicName = _sdf->Get<std::string>("odomTopicName");
    if (_sdf->HasElement("jointStateName")) jointStateName = _sdf->Get<std::string>("jointStateName");
    if (_sdf->HasElement("frontLeftAxle")) wheels[FRONT_LEFT].axleName = _sdf->Get<std::string>("frontLeftAxle");
    if (_sdf->HasElement("frontRightAxle")) wheels[FRONT_RIGHT].axleName = _sdf->Get<std::string>("frontRightAxle");
    if (_sdf->HasElement("middleLeftAxle")) wheels[MIDDLE_LEFT].axleName = _sdf->Get<std::string>("middleLeftAxle");
    if (_sdf->HasElement("middleRightAxle")) wheels[MIDDLE_RIGHT].axleName = _sdf->Get<std::string>("middleRightAxle");
    if (_sdf->HasElement("rearLeftAxle")) wheels[REAR_LEFT].axleName = _sdf->Get<std::string>("rearLeftAxle");
    if (_sdf->HasElement("rearRightAxle")) wheels[REAR_RIGHT].axleName = _sdf->Get<std::string>("rearRightAxle");
    if (_sdf->HasElement("frontLeftJoint")) wheels[FRONT_LEFT].jointName = _sdf->Get<std::string>("frontLeftJoint");
    if (_sdf->HasElement("frontRightJoint")) wheels[FRONT_RIGHT].jointName = _sdf->Get<std::string>("frontRightJoint");
    if (_sdf->HasElement("middleLeftJoint")) wheels[MIDDLE_LEFT].jointName = _sdf->Get<std::string>("middleLeftJoint");
    if (_sdf->HasElement("middleRightJoint")) wheels[MIDDLE_RIGHT].jointName = _sdf->Get<std::string>("middleRightJoint");
    if (_sdf->HasElement("rearLeftJoint")) wheels[REAR_LEFT].jointName = _sdf->Get<std::string>("rearLeftJoint");
    if (_sdf->HasElement("rearRightJoint")) wheels[REAR_RIGHT].jointName = _sdf->Get<std::string>("rearRightJoint");
    if (_sdf->HasElement("proportionalControllerGain")) proportionalControllerGain = _sdf->Get<double>("proportionalControllerGain");
    if (_sdf->HasElement("derivativeControllerGain")) derivativeControllerGain = _sdf->Get<double>("derivativeControllerGain");
    if (_sdf->HasElement("wheelTrack")) wheelTrack = _sdf->Get<double>("wheelTrack");
    if (_sdf->HasElement("wheelTrack")) wheelBase = _sdf->Get<double>("wheelBase");
    if (_sdf->HasElement("wheelRadius")) wheelRadius = _sdf->Get<double>("wheelRadius");
    if (_sdf->HasElement("jointMaxTorque")) jointMaxTorque = _sdf->Get<double>("jointMaxTorque");
    if (_sdf->HasElement("wheelMaxTorque")) wheelMaxTorque = _sdf->Get<double>("wheelMaxTorque");
    if (_sdf->HasElement("maxVelX")) maxVelX = _sdf->Get<double>("maxVelX");

    double controlRate = 0.0;
    if (_sdf->HasElement("controlRate")) controlRate = _sdf->Get<double>("controlRate");
    controlPeriod = controlRate > 0.0 ? 1.0/controlRate : 0.0;

    for(int i = 0; i < 6; ++i) {
        wheels[i].axle  = _model->GetJoint(wheels[i].axleName);
        wheels[i].joint = _model->GetJoint(wheels[i].jointName);
    }

    if (!wheels[FRONT_LEFT].axle)
        gzthrow("The controller couldn't get front left axle");

    if (!wheels[FRONT_RIGHT].axle)
        gzthrow("The controller couldn't get front right axle");

    if (!wheels[MIDDLE_LEFT].axle)
        gzthrow("The controller couldn't get middle left axle");

    if (!wheels[MIDDLE_RIGHT].axle)
        gzthrow("The controller couldn't get middle right axle");

    if (!wheels[REAR_LEFT].axle)
        gzthrow("The controller couldn't get rear left axle");

    if (!wheels[REAR_RIGHT].axle)
        gzthrow("The controller couldn't get rear right axle");

    if (!wheels[FRONT_LEFT].joint)
        gzthrow("The controller couldn't get front left hinge joint");

    if (!wheels[FRONT_RIGHT].joint)
        gzthrow("The controller couldn't get front right hinge joint");

    if (!wheels[MIDDLE_LEFT].joint)
        gzthrow("The controller couldn't get middle left hinge joint");

    if (!wheels[MIDDLE_RIGHT].joint)
        gzthrow("The controller couldn't get middle right hinge joint");

    if (!wheels[REAR_LEFT].joint)
        gzthrow("The controller couldn't get rear left hinge joint");

    if (!wheels[REAR_RIGHT].joint)
        gzthrow("The controller couldn't get rear right hinge joint");

    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
    }
    rosnode_ = new ros::NodeHandle(robotNamespace);

    // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
    if (!topicName.empty()) {
        ros::SubscribeOptions so =
                ros::SubscribeOptions::create<mocup_msgs::MotionCommand>(topicName, 1,
                                                                         boost::bind(&AllWheelSteeringPlugin::motionCommandCallback, this, _1),
                                                                         ros::VoidPtr(), &queue_);
        sub_ = rosnode_->subscribe(so);
    }

    if (!odomTopicName.empty()) {
        odomPub_ = rosnode_->advertise<nav_msgs::Odometry>(odomTopicName, 10);
    }

    if (!jointStateName.empty()) {
        jointStatePub_ = rosnode_->advertise<sensor_msgs::JointState>(jointStateName, 10);
    }

    std::string tf_prefix = tf::getPrefixParam(*rosnode_);
    joint_state.header.frame_id = tf::resolve(tf_prefix, model->GetLink()->GetName());
    odom_.header.frame_id = tf::resolve(tf_prefix, "odom");
    odom_.child_frame_id = tf::resolve(tf_prefix, "base_footprint");

    // New Mechanism for Updating every World Cycle
    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    updateConnection = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&AllWheelSteeringPlugin::Update, this));
}

// Initialize the controller
void AllWheelSteeringPlugin::Init()
{
    Reset();
}

// Reset
void AllWheelSteeringPlugin::Reset()
{
    // Reset odometric pose
    odomPose[0] = 0.0;
    odomPose[1] = 0.0;
    odomPose[2] = 0.0;

    odomVel[0] = 0.0;
    odomVel[1] = 0.0;
    odomVel[2] = 0.0;

    enableMotors = false;

    wheels[FRONT_RIGHT].jointSpeed = 0;
    wheels[FRONT_LEFT].jointSpeed = 0;
    wheels[MIDDLE_RIGHT].jointSpeed = 0;
    wheels[MIDDLE_LEFT].jointSpeed = 0;
    wheels[REAR_RIGHT].jointSpeed = 0;
    wheels[REAR_LEFT].jointSpeed = 0;

    wheels[FRONT_RIGHT].wheelSpeed = 0;
    wheels[FRONT_LEFT].wheelSpeed = 0;
    wheels[MIDDLE_RIGHT].wheelSpeed = 0;
    wheels[MIDDLE_LEFT].wheelSpeed = 0;
    wheels[REAR_RIGHT].wheelSpeed = 0;
    wheels[REAR_LEFT].wheelSpeed = 0;

    prevUpdateTime = world->GetSimTime();
}

// Update the controller
void AllWheelSteeringPlugin::Update()
{
    // TODO: Step should be in a parameter of this function
    double l, b, r;
    double omega_fl, omega_fr, omega_ml, omega_mr, omega_rl, omega_rr, omega_phi;
    double phi_fl, phi_fr, phi_ml, phi_mr, phi_rl, phi_rr;
    double v;

    // handle callbacks
    queue_.callAvailable();

    b = wheelTrack;
    l = wheelBase;
    r = wheelRadius;

    common::Time stepTime;
    //stepTime = World::Instance()->GetPhysicsEngine()->GetStepTime();
    stepTime = world->GetSimTime() - prevUpdateTime;

    if (controlPeriod == 0.0 || stepTime > controlPeriod) {
        GetPositionCmd();

        prevUpdateTime = world->GetSimTime();

        // odometry calculation
        omega_fl = wheels[FRONT_LEFT].axle->GetVelocity(0);
        omega_fr = wheels[FRONT_RIGHT].axle->GetVelocity(0);
        omega_ml = wheels[MIDDLE_LEFT].axle->GetVelocity(0);
        omega_mr = wheels[MIDDLE_RIGHT].axle->GetVelocity(0);
        omega_rl = wheels[REAR_LEFT].axle->GetVelocity(0);
        omega_rr = wheels[REAR_RIGHT].axle->GetVelocity(0);

        phi_fl = wheels[FRONT_LEFT].joint->GetAngle(0).RADIAN();
        phi_fr = wheels[FRONT_RIGHT].joint->GetAngle(0).RADIAN();
        phi_ml = wheels[MIDDLE_LEFT].joint->GetAngle(0).RADIAN();
        phi_mr = wheels[MIDDLE_RIGHT].joint->GetAngle(0).RADIAN();
        phi_rl = wheels[REAR_LEFT].joint->GetAngle(0).RADIAN();
        phi_rr = wheels[REAR_RIGHT].joint->GetAngle(0).RADIAN();

        if (phi_fl < -M_PI_2 || phi_fl > M_PI_2) {
            // gzthrow("phi_fl: %f out of bounds" << phi_fl);
        }
        if (phi_fr < -M_PI_2 || phi_fr > M_PI_2) {
            // gzthrow("phi_fr: %f out of bounds" << phi_fr);
        }
        if (phi_rl < -M_PI_2 || phi_rl > M_PI_2) {
            // gzthrow("phi_rl: %f out of bounds" << phi_rl);
        }
        if (phi_rr < -M_PI_2 || phi_rr > M_PI_2) {
            // gzthrow("phi_rr: %f out of bounds" << phi_rr);
        }

        // Compute angular velocity for ICC which is same as angular velocity of vehicle
        //omega_phi = (omega_fl * sin(phi_fl) * r / b); //+ omega_fr * sin(phi_fr) + omega_rl * sin(-phi_rl) + omega_rr * sin(-phi_rr)) * r / (4 * b);
        omega_phi = ((omega_fl * sin(phi_fl) + omega_fr * sin(phi_fr)) * r) / b;

        v = r * (omega_fl + omega_fr)/2;

        // Compute odometric pose
        odomPose[0] += v * stepTime.Double() * cos(odomPose[2]);
        odomPose[1] += v * stepTime.Double() * sin(odomPose[2]);
        odomPose[2] += omega_phi * stepTime.Double() * 0.9;  //TODO MAGIC NUMBER

        // Compute odometric instantaneous velocity
        odomVel[0] = v;
        odomVel[1] = 0.0;
        odomVel[2] = omega_phi;

        //  write_position_data();
        publish_odometry();
        publish_joint_states();
    }

    double tan_steer= tan(cmd_.steer);
    double steer_l = atan2(l*tan_steer,l-b*tan_steer);
    double steer_r = atan2(l*tan_steer,l+b*tan_steer);

//    // hack for precission issue in tan
//    if(cmd_.steer >= M_PI_2) {
//        steer_l = 3*M_PI_4;
//        steer_r = M_PI_4;
//        //ROS_INFO("steer > 90, set to steering manual");
//    } else if(cmd_.steer <= -M_PI_2) {
//        steer_l = -M_PI_4;
//        steer_r = -3*M_PI_4;
//        //ROS_INFO("steer < -90, set to steering manual");
//    }

    if (!cmd_.mode.compare("continuous")) {
        // calculate wheel speeds
        double speed_l = cmd_.speed*(1-b*tan_steer/l);
        double speed_r = cmd_.speed*(1+b*tan_steer/l);

        // limit wheel speeds for small radius
        if(speed_l > maxVelX) {
            speed_l = maxVelX;
            speed_r = speed_l*(l+b*tan_steer)/(l-b*tan_steer);
        } else if(speed_l < -maxVelX) {
            speed_l = -maxVelX;
            speed_r = speed_l*(l+b*tan_steer)/(l-b*tan_steer);
        } else if(speed_r > maxVelX) {
            speed_r = maxVelX;
            speed_l = speed_r*(l-b*tan_steer)/(l+b*tan_steer);
        } else if(speed_r < -maxVelX) {
            speed_r = -maxVelX;
            speed_l = speed_r*(l-b*tan_steer)/(l+b*tan_steer);
        }

        //ROS_INFO("cmd_vel: %f, speed_l: %f, speed_r: %f", cmd_.speed, speed_l, speed_r);

        wheels[FRONT_LEFT].wheelSpeed   = speed_l / (cos(steer_l) * r);
        wheels[FRONT_RIGHT].wheelSpeed  = speed_r / (cos(steer_r) * r);
        wheels[MIDDLE_LEFT].wheelSpeed  = speed_l / r;
        wheels[MIDDLE_RIGHT].wheelSpeed = speed_r / r;
        wheels[REAR_LEFT].wheelSpeed    = speed_l / (cos(steer_l) * r);
        wheels[REAR_RIGHT].wheelSpeed   = speed_r / (cos(steer_r) * r);
    }

    if (!cmd_.mode.compare("point_turn")) {
        // calculate wheel speeds
        wheels[FRONT_LEFT].wheelSpeed    = -cmd_.speed / (cos(steer_l) * r);
        wheels[FRONT_RIGHT].wheelSpeed   =  cmd_.speed / (cos(steer_r) * r);
        wheels[MIDDLE_LEFT].wheelSpeed   = -cmd_.speed / r;
        wheels[MIDDLE_RIGHT].wheelSpeed  =  cmd_.speed / r;
        wheels[REAR_LEFT].wheelSpeed     = -cmd_.speed / (cos(steer_l) * r);
        wheels[REAR_RIGHT].wheelSpeed    =  cmd_.speed / (cos(steer_r) * r);
    }

    ROS_DEBUG_STREAM_NAMED("all_wheel_steering_plugin", "Wheel speeds:\n"
                           << "front left:  "  << wheels[FRONT_LEFT].wheelSpeed << "\n"
                           << "front right:  " << wheels[FRONT_RIGHT].wheelSpeed << "\n"
                           << "middle left:  " << wheels[MIDDLE_LEFT].wheelSpeed << "\n"
                           << "middle right: " << wheels[MIDDLE_RIGHT].wheelSpeed << "\n"
                           << "rear left: "    << wheels[REAR_LEFT].wheelSpeed << "\n"
                           << "rear right: "   << wheels[REAR_RIGHT].wheelSpeed);

    ROS_DEBUG_STREAM_NAMED("all_wheel_steering_plugin", "Wheel poses:\n"
                           << "front left:  "   << wheels[FRONT_LEFT].joint->GetChild()->GetWorldPose() << "\n"
                           << "front right:  "  << wheels[FRONT_RIGHT].joint->GetChild()->GetWorldPose() << "\n"
                           << "middle leftt:  " << wheels[MIDDLE_LEFT].joint->GetChild()->GetWorldPose() << "\n"
                           << "middle right:  " << wheels[MIDDLE_RIGHT].joint->GetChild()->GetWorldPose() << "\n"
                           << "rear left:  "    << wheels[REAR_LEFT].joint->GetChild()->GetWorldPose() << "\n"
                           << "rear right:  "   << wheels[REAR_RIGHT].joint->GetChild()->GetWorldPose());

    if (enableMotors)
    {
        wheels[FRONT_LEFT].joint->SetVelocity(0, wheels[FRONT_LEFT].jointSpeed);
        wheels[FRONT_RIGHT].joint->SetVelocity(0, wheels[FRONT_RIGHT].jointSpeed);
        wheels[MIDDLE_LEFT].joint->SetVelocity(0, wheels[MIDDLE_LEFT].jointSpeed);
        wheels[MIDDLE_RIGHT].joint->SetVelocity(0, wheels[MIDDLE_RIGHT].jointSpeed);
        wheels[REAR_LEFT].joint->SetVelocity(0, wheels[REAR_LEFT].jointSpeed);
        wheels[REAR_RIGHT].joint->SetVelocity(0, wheels[REAR_RIGHT].jointSpeed);

        wheels[FRONT_LEFT].joint->SetMaxForce(0, jointMaxTorque);
        wheels[FRONT_RIGHT].joint->SetMaxForce(0, jointMaxTorque);
        wheels[MIDDLE_LEFT].joint->SetMaxForce(0, jointMaxTorque);
        wheels[MIDDLE_RIGHT].joint->SetMaxForce(0, jointMaxTorque);
        wheels[REAR_LEFT].joint->SetMaxForce(0, jointMaxTorque);
        wheels[REAR_RIGHT].joint->SetMaxForce(0, jointMaxTorque);

        wheels[FRONT_LEFT].axle->SetVelocity(0, wheels[FRONT_LEFT].wheelSpeed);
        wheels[FRONT_RIGHT].axle->SetVelocity(0, wheels[FRONT_RIGHT].wheelSpeed);
        wheels[MIDDLE_LEFT].axle->SetVelocity(0, wheels[MIDDLE_LEFT].wheelSpeed);
        wheels[MIDDLE_RIGHT].axle->SetVelocity(0, wheels[MIDDLE_RIGHT].wheelSpeed);
        wheels[REAR_LEFT].axle->SetVelocity(0, wheels[REAR_LEFT].wheelSpeed);
        wheels[REAR_RIGHT].axle->SetVelocity(0, wheels[REAR_RIGHT].wheelSpeed);

        wheels[FRONT_LEFT].axle->SetMaxForce(0, wheelMaxTorque);
        wheels[FRONT_RIGHT].axle->SetMaxForce(0, wheelMaxTorque);
        wheels[MIDDLE_LEFT].axle->SetMaxForce(0, wheelMaxTorque);
        wheels[MIDDLE_RIGHT].axle->SetMaxForce(0, wheelMaxTorque);
        wheels[REAR_LEFT].axle->SetMaxForce(0, wheelMaxTorque);
        wheels[REAR_RIGHT].axle->SetMaxForce(0, wheelMaxTorque);
    } else {
        wheels[FRONT_LEFT].joint->SetMaxForce(0, 0.0);
        wheels[FRONT_RIGHT].joint->SetMaxForce(0, 0.0);
        wheels[MIDDLE_LEFT].joint->SetMaxForce(0, 0.0);
        wheels[MIDDLE_RIGHT].joint->SetMaxForce(0, 0.0);
        wheels[REAR_LEFT].joint->SetMaxForce(0, 0.0);
        wheels[REAR_RIGHT].joint->SetMaxForce(0, 0.0);
        wheels[FRONT_LEFT].axle->SetMaxForce(0, 0.0);
        wheels[FRONT_RIGHT].axle->SetMaxForce(0, 0.0);
        wheels[MIDDLE_LEFT].axle->SetMaxForce(0, 0.0);
        wheels[MIDDLE_RIGHT].axle->SetMaxForce(0, 0.0);
        wheels[REAR_LEFT].axle->SetMaxForce(0, 0.0);
        wheels[REAR_RIGHT].axle->SetMaxForce(0, 0.0);
    }
}

// NEW: Now uses the target velocities from the ROS message, not the Iface 
void AllWheelSteeringPlugin::GetPositionCmd()
{
    boost::mutex::scoped_lock lock(mutex);

    double current_phi_fl, current_phi_fr, current_phi_ml, current_phi_mr, current_phi_rl, current_phi_rr;
    double vel_phi_fl, vel_phi_fr, vel_phi_ml, vel_phi_mr, vel_phi_rl, vel_phi_rr;

    if (cmd_.speed > maxVelX) {
        cmd_.speed = maxVelX;
    } else if (cmd_.speed < -maxVelX) {
        cmd_.speed = -maxVelX;
    }

//    //ROS_INFO("steer before %a", cmd_.steer);
//    if(cmd_.steer >= M_PI_2) {
//        cmd_.steer = M_PI_2;
//        //ROS_INFO("steer >= 90");
//    } else if(cmd_.steer <= -M_PI_2) {
//        cmd_.steer = -M_PI_2;
//        //ROS_INFO("steer <= -90");
//    }
//    //ROS_INFO("steer after %a", cmd_.steer);

    current_phi_fl = wheels[FRONT_LEFT].joint->GetAngle(0).RADIAN();
    current_phi_fr = wheels[FRONT_RIGHT].joint->GetAngle(0).RADIAN();
    current_phi_ml = wheels[MIDDLE_LEFT].joint->GetAngle(0).RADIAN();
    current_phi_mr = wheels[MIDDLE_RIGHT].joint->GetAngle(0).RADIAN();
    current_phi_rl = wheels[REAR_LEFT].joint->GetAngle(0).RADIAN();
    current_phi_rr = wheels[REAR_RIGHT].joint->GetAngle(0).RADIAN();

    vel_phi_fl = wheels[FRONT_LEFT].joint->GetVelocity(0);
    vel_phi_fr = wheels[FRONT_RIGHT].joint->GetVelocity(0);
    vel_phi_ml = wheels[MIDDLE_LEFT].joint->GetVelocity(0);
    vel_phi_mr = wheels[MIDDLE_RIGHT].joint->GetVelocity(0);
    vel_phi_rl = wheels[REAR_LEFT].joint->GetVelocity(0);
    vel_phi_rr = wheels[REAR_RIGHT].joint->GetVelocity(0);

    double tan_steer= tan(cmd_.steer);
    double b = wheelTrack;
    double l = wheelBase;
    double steer_l = atan2(l*tan_steer,l-b*tan_steer);
    double steer_r = atan2(l*tan_steer,l+b*tan_steer);

    // hack for precission issue in tan
    if(cmd_.steer >= M_PI_2) {
        steer_l = 3*M_PI_4;
        steer_r = M_PI_4;
        //ROS_INFO("steer > 90, set to steering manual");
    } else if(cmd_.steer <= -M_PI_2) {
        steer_l = -M_PI_4;
        steer_r = -3*M_PI_4;
        //ROS_INFO("steer < -90, set to steering manual");
    }

    if (!cmd_.mode.compare("continuous")) {
        wheels[FRONT_LEFT].jointSpeed   = ( ((steer_l - current_phi_fl) * proportionalControllerGain) - vel_phi_fl * derivativeControllerGain);
        wheels[FRONT_RIGHT].jointSpeed  = ( ((steer_r - current_phi_fr) * proportionalControllerGain) - vel_phi_fr * derivativeControllerGain);
        wheels[MIDDLE_LEFT].jointSpeed  = ( ((0  - current_phi_ml) * proportionalControllerGain) - vel_phi_ml* derivativeControllerGain); // fixed joint
        wheels[MIDDLE_RIGHT].jointSpeed = ( ((0  - current_phi_mr) * proportionalControllerGain) - vel_phi_mr * derivativeControllerGain); // fixed joint
        wheels[REAR_LEFT].jointSpeed    = ( ((-steer_l  - current_phi_rl) * proportionalControllerGain) - vel_phi_rl * derivativeControllerGain);
        wheels[REAR_RIGHT].jointSpeed   = ( ((-steer_r  - current_phi_rr) * proportionalControllerGain) - vel_phi_rr * derivativeControllerGain);
    }
    if (!cmd_.mode.compare("point_turn")) {
        wheels[FRONT_LEFT].jointSpeed   = ( ((-M_PI_4 - current_phi_fl) * proportionalControllerGain) - vel_phi_fl * derivativeControllerGain);
        wheels[FRONT_RIGHT].jointSpeed  = ( ((M_PI_4 - current_phi_fr) * proportionalControllerGain) - vel_phi_fr * derivativeControllerGain);
        wheels[MIDDLE_LEFT].jointSpeed  = ( ((0  - current_phi_ml) * proportionalControllerGain) - vel_phi_ml * derivativeControllerGain); // fixed joint
        wheels[MIDDLE_RIGHT].jointSpeed = ( ((0  - current_phi_mr) * proportionalControllerGain) - vel_phi_mr * derivativeControllerGain); // fixed joint
        wheels[REAR_LEFT].jointSpeed    = ( ((M_PI_4  - current_phi_rl) * proportionalControllerGain) - vel_phi_rl * derivativeControllerGain);
        wheels[REAR_RIGHT].jointSpeed   = ( ((-M_PI_4  - current_phi_rr) * proportionalControllerGain) - vel_phi_rr * derivativeControllerGain);
    }

    //  ROS_INFO("i: fl: [%f] fr: [%f] rl: [%f] rr: [%f]", current_phi_fl, current_phi_fr, current_phi_rl, current_phi_rr);
    //  ROS_INFO("s: fl: [%f] fr: [%f] rl: [%f] rr: [%f]", vel_phi_fl, vel_phi_fr, vel_phi_rl, vel_phi_rr);

    //  ROS_INFO("v: fl: [%f] fr: [%f] rl: [%f] rr: [%f]\n", wheels[FRONT_LEFT].jointSpeed, wheels[FRONT_RIGHT].jointSpeed, wheels[REAR_LEFT].jointSpeed, wheels[REAR_RIGHT].jointSpeed);

    // Changed motors to be always on, which is probably what we want anyway
    enableMotors = true;

    ROS_DEBUG_STREAM_NAMED("all_wheel_steering_plugin", enableMotors);
}

// NEW: Store the velocities from the ROS message
void AllWheelSteeringPlugin::motionCommandCallback(const mocup_msgs::MotionCommand::ConstPtr& cmd_msg)
{
    boost::mutex::scoped_lock lock(mutex);
    cmd_ = *cmd_msg;
}

// NEW: Update this to publish odometry topic
void AllWheelSteeringPlugin::publish_odometry()
{
    if (!odomPub_) return;

    // publish odom topic
    odom_.pose.pose.position.x = odomPose[0];
    odom_.pose.pose.position.y = odomPose[1];

    tf::Quaternion qt = tf::createQuaternionFromRPY(0.0, 0.0, fmod(odomPose[2] + M_PI, 2*M_PI) - M_PI);
    tf::quaternionTFToMsg(qt, odom_.pose.pose.orientation);

    odom_.twist.twist.linear.x = odomVel[0];
    odom_.twist.twist.linear.y = odomVel[1];
    odom_.twist.twist.angular.z = odomVel[2];

    odom_.header.stamp.sec = world->GetSimTime().sec;
    odom_.header.stamp.nsec = world->GetSimTime().nsec;

    odomPub_.publish(odom_);
}

void AllWheelSteeringPlugin::publish_joint_states()
{
    if (!jointStatePub_) return;

    joint_state.header.stamp.sec = world->GetSimTime().sec;
    joint_state.header.stamp.nsec = world->GetSimTime().nsec;
    joint_state.name.resize(12);
    joint_state.position.resize(12);
    joint_state.velocity.resize(12);
    joint_state.effort.resize(12);

    for (unsigned int i = 0; i < 6; i++) {
        joint_state.name[i] = wheels[i].joint->GetName();
        joint_state.position[i] = wheels[i].joint->GetAngle(0).RADIAN();
        joint_state.velocity[i] = wheels[i].joint->GetVelocity(0);
        joint_state.effort[i] = wheels[i].joint->GetForce(0u);
    }

    for (unsigned int i = 0; i < 6; i++) {
        joint_state.name[6+i] = wheels[i].axle->GetName();
        joint_state.position[6+i] = wheels[i].axle->GetAngle(0).RADIAN();
        joint_state.velocity[6+i] = wheels[i].axle->GetVelocity(0);
        joint_state.effort[6+i] = wheels[i].axle->GetForce(0u);
    }

    jointStatePub_.publish(joint_state);
}

} // namespace gazebo

