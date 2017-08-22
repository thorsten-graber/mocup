#ifndef MOCUP_DRIVER_H
#define MOCUP_DRIVER_H

#include <ros/ros.h>
#include <mocup_msgs/MotionCommand.h>
#include <mocup_msgs/MotorCommand.h>
#include <mocup_msgs/RawSensors.h>
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
        float chassis_width;
        float chassis_length;
        float wheel_radius;
        float wheel_gear;
        float stearing_gear;
        float camera_pan_gear;
        float camera_tilt_gear;
        mocup_msgs::MotorCommand control_input;
        sensor_msgs::JointState motor_states;
    } MotorControlParameters;

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
    ~Driver();

    int main(int, char**);
    void update(const ros::TimerEvent&);

    void reset();
    void stop();

    void ComputeLocomotion(float speed, float steer, float& speed_l, float& speed_r, float& steer_l, float& steer_r);
    float limitVelocity(float speed);
    int16_t floatToInt(float value);

    void motionCommandCallback(const mocup_msgs::MotionCommand& cmd_msg);
    void cameraCommandCallback(const geometry_msgs::QuaternionStamped& cmd_msg);
    void readSensorsCallback(const mocup_msgs::RawSensors &sensor_msg);

    void publishOdometry();
    void publishJointStates();

    ros::NodeHandle nh;
    ros::Timer timer;

    MotorControlParameters motor_control_parameters;
    Sensors actual_readings;
    Sensors previous_readings;

    nav_msgs::Odometry odom;

    geometry_msgs::TransformStamped odom_trans;
    tf::TransformBroadcaster odom_broadcaster;

    ros::Publisher motor_command_publisher;
    ros::Publisher joint_state_publisher;
    ros::Publisher odom_publisher;

    ros::Subscriber motion_command_subscriber;
    ros::Subscriber camera_command_subscriber;
    ros::Subscriber read_sensors_subscriber;

    float steer_old;
};

#endif // MOCUP_DRIVER_H
