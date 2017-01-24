#ifndef MOCUP_DRIVER_H
#define MOCUP_DRIVER_H

#include <ros/ros.h>
#include <mocup_msgs/MotionCommand.h>
#include <std_msgs/Int16MultiArray.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

class Driver {
public:
    typedef struct {
        float min_velocity;
        float max_velocity;
        float min_object_distance;
        double wheel_base;
        double wheel_radius;
        double wheel_gear;
        double stearing_gear;
        double camera_pan_gear;
        double camera_tilt_gear;
        std_msgs::Int16MultiArray control_input;
        sensor_msgs::JointState motor_states;
    } MotorControlParameters;

    typedef struct {
        double x;
        double y;
        double yaw;
    } Pose;

    typedef struct {
        float range_fl;
        float range_fr;
        float range_rl;
        float range_rr;
    } UltraSonic;

    typedef struct {
        double right_wheel_joint;
        double left_wheel_joint;
        double right_suspension_wheel_joint;
        double left_suspension_wheel_joint;
        double camera_pan;
        double camera_tilt;
    } Angle;

    typedef struct {
        long right_wheel_joint;
        long left_wheel_joint;
        long right_suspension_wheel_joint;
        long left_suspension_wheel_joint;
        long camera_pan;
        long camera_tilt;
    } Encoder;

    typedef struct {
        Encoder overflow;
        Encoder position;
        Angle angle;
    } Motor;

    typedef struct {
        ros::Time stamp;
        UltraSonic ultrasonic;
        Motor motor;
    } Sensors;

    Driver(const std::string &ns = std::string());
    virtual ~Driver();

    friend int main(int, char**);

protected:
    virtual bool configure();
    virtual void update();
    virtual void reset();
    virtual void stop();
    virtual void cleanup();

    virtual float limitVelocity(float speed);
    virtual int16_t floatToInt(float value);

    virtual void motionCommandCallback(const mocup_msgs::MotionCommand& cmd_msg);
    virtual void cameraCommandCallback(const geometry_msgs::QuaternionStamped& cmd_msg);
    virtual void readSensorsCallback(const std_msgs::Int16MultiArray& sensor_msg);

    virtual void publishOdometry();
    virtual void publishJointStates();

private:
    ros::NodeHandle nh;

    MotorControlParameters motor_control_parameters;
    Pose pose;
    Sensors actual_readings;
    Sensors previous_readings;

    geometry_msgs::TransformStamped odom_trans;
    tf::TransformBroadcaster odom_broadcaster;

    ros::Publisher motor_command_publisher;
    ros::Publisher joint_state_publisher;

    ros::Subscriber motion_command_subscriber;
    ros::Subscriber camera_command_subscriber;
    ros::Subscriber read_sensors_subscriber;
};

#endif // MOCUP_DRIVER_H
