#include <mocup_driver/mocup_driver.h>
#include <ros/ros.h>

#include "quaternions.h"

#define NO_OBSTICAL 2.55f

static double roundRadToDeg(double rad) {
    return round (rad * 180.0 / M_PI);
}

static double degToRad(double deg) {
    return (deg * M_PI / 180.0);
}

static int16_t posToDeg(long position) {
    return (int16_t) (position / 2l % 360l);
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

static long correctPosition(const int16_t actual_position_raw, long &previous_position, long &overflow) {
    long delta_position = previous_position - actual_position_raw + overflow*USHRT_MAX;

    // check if encoder overrun has happend, and correct if so
    if (delta_position > SHRT_MAX) { overflow++; }
    if (delta_position < SHRT_MIN) { overflow--; }

    return (long)actual_position_raw + overflow*USHRT_MAX;
}

static float centimeterToMeter(int centimeter) {
    return (float) (centimeter / 100.0f);
}

static double maxDeltaFilter(double y, double x, double c) {
    double delta = y-x;
    double test = x+(double)sgn(delta) * c;
    return (fabs(delta) > c) ? test : y;
}

Driver::Driver(const std::string& ns)
    : nh(ns)
{
    motor_control_parameters.min_velocity        = 0.05;
    motor_control_parameters.max_velocity        = 1.0;
    motor_control_parameters.min_object_distance = 0.2;

    motor_control_parameters.chassis_width       = 0.3;
    motor_control_parameters.chassis_length      = 0.3;

    motor_control_parameters.wheel_radius        = 0.03; 

    motor_control_parameters.motor_states.header.frame_id = "base_link";
    motor_control_parameters.motor_states.name.push_back("front_right_wheel_suspension_joint");
    motor_control_parameters.motor_states.name.push_back("middle_right_wheel_suspension_joint");
    motor_control_parameters.motor_states.name.push_back("rear_right_wheel_suspension_joint");
    motor_control_parameters.motor_states.name.push_back("front_left_wheel_suspension_joint");
    motor_control_parameters.motor_states.name.push_back("middle_left_wheel_suspension_joint");
    motor_control_parameters.motor_states.name.push_back("rear_left_wheel_suspension_joint");
    motor_control_parameters.motor_states.name.push_back("front_right_wheel_joint");
    motor_control_parameters.motor_states.name.push_back("middle_right_wheel_joint");
    motor_control_parameters.motor_states.name.push_back("rear_right_wheel_joint");
    motor_control_parameters.motor_states.name.push_back("front_left_wheel_joint");
    motor_control_parameters.motor_states.name.push_back("middle_left_wheel_joint");
    motor_control_parameters.motor_states.name.push_back("rear_left_wheel_joint");
    motor_control_parameters.motor_states.name.push_back("camera/z");
    motor_control_parameters.motor_states.name.push_back("camera/y");
    motor_control_parameters.motor_states.effort.resize(14);
    motor_control_parameters.motor_states.position.resize(14);
    motor_control_parameters.motor_states.velocity.resize(14);
}

Driver::~Driver()
{
}

bool Driver::configure()
{
    ros::NodeHandle params("~");
    params.getParam("min_velocity", motor_control_parameters.min_velocity);
    params.getParam("max_velocity", motor_control_parameters.max_velocity);

    motion_command_subscriber = nh.subscribe("drive", 10, &Driver::motionCommandCallback, this);
    camera_command_subscriber = nh.subscribe("camera/command", 10, &Driver::cameraCommandCallback, this);
    read_sensors_subscriber   = nh.subscribe("sensor_readings", 10, &Driver::readSensorsCallback,this);

    motor_command_publisher = nh.advertise<mocup_msgs::MotorCommand>("motor_command", 1, true);
    joint_state_publisher = nh.advertise<sensor_msgs::JointState>("joint_states",10,true);
    odom_publisher = nh.advertise<nav_msgs::Odometry>("state", 10, true);

    timer = nh.createTimer(ros::Duration(0.02),&Driver::update,this);

    reset();

    return true;
}

void Driver::cleanup()
{
}

void Driver::reset()
{
    // reset motor control data
    motor_control_parameters.control_input.speed_r = 0;
    motor_control_parameters.control_input.steer_r = 0;
    motor_control_parameters.control_input.speed_l = 0;
    motor_control_parameters.control_input.steer_r = 0;
    motor_control_parameters.control_input.cam_yaw = 0;
    motor_control_parameters.control_input.cam_pit = 0;

    alpha_old = 0;

    // reset sensor readings
    previous_readings.stamp = ros::Time::now();

    actual_readings.motor.position.left_wheel_joint             = 0;
    actual_readings.motor.position.right_wheel_joint            = 0;
    actual_readings.motor.position.left_suspension_wheel_joint  = 0;
    actual_readings.motor.position.right_suspension_wheel_joint = 0;
    actual_readings.motor.position.camera_pan                   = 0;
    actual_readings.motor.position.camera_tilt                  = 0;

    actual_readings.motor.overflow.left_wheel_joint             = 0;
    actual_readings.motor.overflow.right_wheel_joint            = 0;
    actual_readings.motor.overflow.left_suspension_wheel_joint  = 0;
    actual_readings.motor.overflow.right_suspension_wheel_joint = 0;
    actual_readings.motor.overflow.camera_pan                   = 0;
    actual_readings.motor.overflow.camera_tilt                  = 0;\

    actual_readings.ultrasonic.range_fl = NO_OBSTICAL;
    actual_readings.ultrasonic.range_fr = NO_OBSTICAL;
    actual_readings.ultrasonic.range_rl = NO_OBSTICAL;
    actual_readings.ultrasonic.range_rr = NO_OBSTICAL;

    previous_readings.stamp = ros::Time::now();

    previous_readings.motor.position.left_wheel_joint             = 0;
    previous_readings.motor.position.right_wheel_joint            = 0;
    previous_readings.motor.position.left_suspension_wheel_joint  = 0;
    previous_readings.motor.position.right_suspension_wheel_joint = 0;
    previous_readings.motor.position.camera_pan                   = 0;
    previous_readings.motor.position.camera_tilt                  = 0;

    previous_readings.motor.overflow.left_wheel_joint             = 0;
    previous_readings.motor.overflow.right_wheel_joint            = 0;
    previous_readings.motor.overflow.left_suspension_wheel_joint  = 0;
    previous_readings.motor.overflow.right_suspension_wheel_joint = 0;
    previous_readings.motor.overflow.camera_pan                   = 0;
    previous_readings.motor.overflow.camera_tilt                  = 0;\

    previous_readings.ultrasonic.range_fl = NO_OBSTICAL;
    previous_readings.ultrasonic.range_fr = NO_OBSTICAL;
    previous_readings.ultrasonic.range_rl = NO_OBSTICAL;
    previous_readings.ultrasonic.range_rr = NO_OBSTICAL;

    // todo: reset odometric pose and tf
    odom.pose.pose.position.x    = 0;
    odom.pose.pose.position.y    = 0;
    odom.pose.pose.position.z    = 0;
    odom.pose.pose.orientation.w = 1;
    odom.pose.pose.orientation.x = 0;
    odom.pose.pose.orientation.y = 0;
    odom.pose.pose.orientation.z = 0;
}

void Driver::stop()
{
    motor_control_parameters.control_input.speed_r = 0;
    motor_control_parameters.control_input.steer_r = 0;
    motor_control_parameters.control_input.speed_l = 0;
    motor_control_parameters.control_input.steer_r = 0;
}

void Driver::update(const ros::TimerEvent&)
{
    motor_command_publisher.publish(motor_control_parameters.control_input);
}

void Driver::motionCommandCallback(const mocup_msgs::MotionCommand& cmd_msg)
{
    float cmd_vel = limitVelocity(cmd_msg.speed);

    // update desired motor velocity/position based on control mode
    if(!strcmp(cmd_msg.mode.c_str(), "continuous")) {
        double alpha    = (double)cmd_msg.steerAngleFront;
        if(alpha > M_PI_2) {
            alpha = M_PI_2;
        } else if(alpha < -M_PI_2) {
            alpha = -M_PI_2;
        }

        alpha = maxDeltaFilter(alpha, alpha_old, 0.0025);
        alpha_old = alpha;

        double tan_alpha = tan(alpha);
        double b = motor_control_parameters.chassis_width;
        double l = motor_control_parameters.chassis_length;
        double steer_l = atan2(l*tan_alpha,l-b*tan_alpha);
        double steer_r = atan2(l*tan_alpha,l+b*tan_alpha);

        motor_control_parameters.control_input.steer_r = roundRadToDeg(steer_r);
        motor_control_parameters.control_input.steer_l = roundRadToDeg(steer_l);

        double speed_l = cmd_vel*(1-b*tan_alpha/l);
        double speed_r = cmd_vel*(1+b*tan_alpha/l);

        // limit wheel speeds for small radius
        if(speed_l > motor_control_parameters.max_velocity) {
            speed_l = motor_control_parameters.max_velocity;
            speed_r = speed_l*(l+b*tan_alpha)/(l-b*tan_alpha);
        } else if(speed_l < -motor_control_parameters.max_velocity) {
            speed_l = -motor_control_parameters.max_velocity;
            speed_r = speed_l*(l+b*tan_alpha)/(l-b*tan_alpha);
        } else if(speed_r > motor_control_parameters.max_velocity) {
            speed_r = motor_control_parameters.max_velocity;
            speed_l = speed_r*(l-b*tan_alpha)/(l+b*tan_alpha);
        } else if(speed_r < -motor_control_parameters.max_velocity) {
            speed_r = -motor_control_parameters.max_velocity;
            speed_l = speed_r*(l-b*tan_alpha)/(l+b*tan_alpha);
        }

        // Velocity [m/s] to PWM (max velocity = 255 pwm)
        int16_t pwm_l = floatToInt(speed_l);
        int16_t pwm_r = floatToInt(speed_r);
        motor_control_parameters.control_input.speed_r = pwm_r;
        motor_control_parameters.control_input.speed_l = pwm_l;
    }
    if(!strcmp(cmd_msg.mode.c_str(), "point_turn")) {
        int16_t pwm = floatToInt(cmd_vel);

        motor_control_parameters.control_input.speed_r = pwm;
        motor_control_parameters.control_input.steer_r = 45;
        motor_control_parameters.control_input.speed_l = -pwm;
        motor_control_parameters.control_input.steer_l = -45;
    }
}

void Driver::cameraCommandCallback(const geometry_msgs::QuaternionStamped& cmd_msg)
{
    double angles[3];
    quaternion2angles(cmd_msg.quaternion, angles);

    motor_control_parameters.control_input.cam_yaw = roundRadToDeg(angles[0]);
    motor_control_parameters.control_input.cam_pit = roundRadToDeg(angles[1]);
}

void Driver::readSensorsCallback(const mocup_msgs::RawSensors &sensor_msg) {

    actual_readings.stamp = ros::Time::now();

    actual_readings.motor.position.right_wheel_joint         = correctPosition(sensor_msg.wheel_r,previous_readings.motor.position.right_wheel_joint,previous_readings.motor.overflow.right_wheel_joint);
    actual_readings.motor.position.left_wheel_joint          = correctPosition(sensor_msg.wheel_l,previous_readings.motor.position.left_wheel_joint,previous_readings.motor.overflow.left_wheel_joint);
    actual_readings.motor.angle.right_wheel_joint            = degToRad(posToDeg(actual_readings.motor.position.right_wheel_joint));
    actual_readings.motor.angle.left_wheel_joint             = degToRad(posToDeg(actual_readings.motor.position.left_wheel_joint));

    actual_readings.motor.angle.right_suspension_wheel_joint = degToRad(sensor_msg.steer_r);
    actual_readings.motor.angle.left_suspension_wheel_joint  = degToRad(sensor_msg.steer_l);
    actual_readings.motor.angle.camera_pan                   = degToRad(sensor_msg.cam_yaw);
    actual_readings.motor.angle.camera_tilt                  = degToRad(sensor_msg.cam_pit);

    actual_readings.ultrasonic.range_fl                      = centimeterToMeter(sensor_msg.us_fl);
    actual_readings.ultrasonic.range_fr                      = centimeterToMeter(sensor_msg.us_fr);
    actual_readings.ultrasonic.range_rl                      = centimeterToMeter(sensor_msg.us_rl);
    actual_readings.ultrasonic.range_rr                      = centimeterToMeter(sensor_msg.us_rr);

    // publish joint states
    publishJointStates();

    // publish odometry
    publishOdometry();

    // safe sensor readings
    previous_readings = actual_readings;
}

void Driver::publishOdometry()
{
    double b, r;
    double alpha_l, alpha_r;
    double x, y, yaw;
    double phi_l, phi_r;
    double s;

    // vehicle geometry
    b = motor_control_parameters.chassis_width / 2.0;
    r = motor_control_parameters.wheel_radius;

    alpha_r = degToRad(posToDeg(actual_readings.motor.position.right_wheel_joint - previous_readings.motor.position.right_wheel_joint));
    alpha_l = degToRad(posToDeg(actual_readings.motor.position.left_wheel_joint - previous_readings.motor.position.left_wheel_joint));
    phi_r   = actual_readings.motor.angle.right_suspension_wheel_joint;
    phi_l   = actual_readings.motor.angle.left_suspension_wheel_joint;

    // Compute angular velocity for ICC which is same as angular velocity of vehicle
    yaw = ((alpha_l * sin(phi_l) + alpha_r * sin(phi_r))* r) / (2.0 * b );

    // Compute translation using previous yaw angle using last pose
    double euler[3];
    quaternion2euler(odom.pose.pose.orientation, euler);
    s = r * (alpha_l + alpha_r)/2;
    x = s * cos(euler[0]);
    y = s * sin(euler[0]);

    // Compute delta t
    double dt = actual_readings.stamp.toSec() - previous_readings.stamp.toSec();

    // Publish Odom
    // set header
    odom.header.stamp    = actual_readings.stamp;
    odom.header.frame_id = "map";                   // todo: magic string

    //Compute velocity
    odom.child_frame_id        = "base_link";       // todo: magic string
    odom.twist.twist.linear.x  = x / dt;
    odom.twist.twist.linear.y  = y / dt;
    odom.twist.twist.angular.z = yaw / dt;

    // Compute odometric pose
    geometry_msgs::Quaternion yaw_quat = tf::createQuaternionMsgFromYaw(yaw);
    odom.pose.pose.position.x  += x;
    odom.pose.pose.position.y  += y;
    odom.pose.pose.position.z  = 0.1;                // todo: magic number
    odom.pose.pose.orientation = odom.pose.pose.orientation*yaw_quat;

    //publish message
    odom_publisher.publish(odom);

    // Publish tf
    odom_trans.header.stamp    = actual_readings.stamp;
    odom_trans.header.frame_id = "map";             // todo: magic string
    odom_trans.child_frame_id  = "base_link";       // todo: magic string

    odom_trans.transform.translation.x = odom.pose.pose.position.x;
    odom_trans.transform.translation.y = odom.pose.pose.position.y;
    odom_trans.transform.translation.z = odom.pose.pose.position.z;
    odom_trans.transform.rotation      = odom.pose.pose.orientation;

    odom_broadcaster.sendTransform(odom_trans);
}

void Driver::publishJointStates()
{
    // publish motor joints
    motor_control_parameters.motor_states.position[0]  = actual_readings.motor.angle.right_suspension_wheel_joint;
    motor_control_parameters.motor_states.position[1]  = 0;
    motor_control_parameters.motor_states.position[2]  = -actual_readings.motor.angle.right_suspension_wheel_joint;

    motor_control_parameters.motor_states.position[3]  = actual_readings.motor.angle.left_suspension_wheel_joint;
    motor_control_parameters.motor_states.position[4]  = 0;
    motor_control_parameters.motor_states.position[5]  = -actual_readings.motor.angle.left_suspension_wheel_joint;

    motor_control_parameters.motor_states.position[6]  = actual_readings.motor.angle.right_wheel_joint;
    motor_control_parameters.motor_states.position[7]  = actual_readings.motor.angle.right_wheel_joint;
    motor_control_parameters.motor_states.position[8]  = actual_readings.motor.angle.right_wheel_joint;

    motor_control_parameters.motor_states.position[9]  = actual_readings.motor.angle.left_wheel_joint;
    motor_control_parameters.motor_states.position[10] = actual_readings.motor.angle.left_wheel_joint;
    motor_control_parameters.motor_states.position[11] = actual_readings.motor.angle.left_wheel_joint;

    motor_control_parameters.motor_states.position[12] = actual_readings.motor.angle.camera_pan;
    motor_control_parameters.motor_states.position[13] = actual_readings.motor.angle.camera_tilt;

    motor_control_parameters.motor_states.header.stamp = actual_readings.stamp;

    joint_state_publisher.publish(motor_control_parameters.motor_states);
}

float Driver::limitVelocity(float value) { //todo check if cases
    float velocity = value;
    if (value > 0.0) {
        if (value > motor_control_parameters.max_velocity) velocity = motor_control_parameters.max_velocity;
        if (value < motor_control_parameters.min_velocity) velocity = motor_control_parameters.min_velocity;
        if (previous_readings.ultrasonic.range_fl < motor_control_parameters.min_object_distance || previous_readings.ultrasonic.range_fr < motor_control_parameters.min_object_distance) {
            velocity = 0.0f;
            ROS_WARN("OBSTACLE in the FRONT, STOPPING MOTORS distance to obstacle: %f", motor_control_parameters.min_object_distance);
        }
    } else if (value < 0.0) {
        if (value < -motor_control_parameters.max_velocity) velocity = -motor_control_parameters.max_velocity;
        if (value > -motor_control_parameters.min_velocity) velocity = -motor_control_parameters.min_velocity;
        if (previous_readings.ultrasonic.range_rl < motor_control_parameters.min_object_distance || previous_readings.ultrasonic.range_rr < motor_control_parameters.min_object_distance) {
            velocity = 0.0f;
            ROS_WARN("OBSTACLE in the BACK, STOPPING MOTORS distance to obstacle: %f", motor_control_parameters.min_object_distance);
        }
    }
    return velocity;
}

int16_t Driver::floatToInt(float value) {
    return (int16_t) round(value * 255.0f / motor_control_parameters.max_velocity);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, ROS_PACKAGE_NAME);

    Driver d;
    d.configure();
    while(ros::ok())
    {
        ros::spin();
    }
    d.cleanup();

    ros::shutdown();
    return 0;
}
