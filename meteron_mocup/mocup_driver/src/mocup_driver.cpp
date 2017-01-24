#include <mocup_driver/mocup_driver.h>
#include <ros/ros.h>

#include "quaternions.h"

#define NO_OBSTICAL 2.5f

static double roundRadToDeg(double rad) {
    return round (rad * 180.0 / M_PI);
}

static double degToRad(double deg) {
    return (deg * M_PI / 180.0);
}

static int16_t posToDeg(long position) {
    return (int16_t) (position / 2l % 360l);
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

Driver::Driver(const std::string& ns)
    : nh(ns)
{
    motor_control_parameters.min_velocity        = 0.1;
    motor_control_parameters.max_velocity        = 1.0;
    motor_control_parameters.min_object_distance = 0.10;

    motor_control_parameters.wheel_base          = 0.3;
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

    motor_control_parameters.control_input.layout.dim.push_back(std_msgs::MultiArrayDimension());
    motor_control_parameters.control_input.layout.dim[0].size = 6;
    motor_control_parameters.control_input.layout.dim[0].stride = 1;
    motor_control_parameters.control_input.data.resize(6);
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

    motor_command_publisher = nh.advertise<std_msgs::Int16MultiArray>("motor_command", 1, true);
    joint_state_publisher = nh.advertise<sensor_msgs::JointState>("joint_states",10,true);

    reset();

    return true;
}

void Driver::cleanup()
{
}

void Driver::reset()
{
    // reset motor control data
    motor_control_parameters.control_input.data[0] = 0;
    motor_control_parameters.control_input.data[1] = 0;
    motor_control_parameters.control_input.data[2] = 0;
    motor_control_parameters.control_input.data[3] = 0;
    motor_control_parameters.control_input.data[4] = 0;
    motor_control_parameters.control_input.data[5] = 0;
    update();

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

}

void Driver::stop()
{
    motor_control_parameters.control_input.data[0] = 0;
    motor_control_parameters.control_input.data[1] = 0;
    motor_control_parameters.control_input.data[2] = 0;
    motor_control_parameters.control_input.data[3] = 0;

    update();
}

void Driver::update()
{
    motor_command_publisher.publish(motor_control_parameters.control_input);
}

void Driver::motionCommandCallback(const mocup_msgs::MotionCommand& cmd_msg)
{
    float cmd_vel = limitVelocity(cmd_msg.speed);

    // Velocity [m/s] to PWM (max velocity = 255 pwm)
    int16_t pwm = floatToInt(cmd_vel);

    // update desired motor velocity/position based on control mode
    if(!strcmp(cmd_msg.mode.c_str(), "ackermann")) {
        int16_t angle = roundRadToDeg(cmd_msg.steerAngleFront);
        motor_control_parameters.control_input.data[0] = pwm;
        motor_control_parameters.control_input.data[1] = angle;
        motor_control_parameters.control_input.data[2] = pwm;
        motor_control_parameters.control_input.data[3] = angle;
    }
    if(!strcmp(cmd_msg.mode.c_str(), "point_turn")) {
        // Steer wheels to 45deg
        motor_control_parameters.control_input.data[0] = pwm;
        motor_control_parameters.control_input.data[1] = 45;
        motor_control_parameters.control_input.data[2] = pwm;
        motor_control_parameters.control_input.data[3] = 45;
    }

    update();
}

void Driver::cameraCommandCallback(const geometry_msgs::QuaternionStamped& cmd_msg)
{
    double angles[3];
    quaternion2angles(cmd_msg.quaternion, angles);

    motor_control_parameters.control_input.data[4] = roundRadToDeg(angles[0]);
    motor_control_parameters.control_input.data[5] = roundRadToDeg(angles[1]);

    update();
}

void Driver::readSensorsCallback(const std_msgs::Int16MultiArray &sensor_msg) {

    if(sensor_msg.data.size() == 10) {
        actual_readings.stamp = ros::Time::now();

        actual_readings.motor.position.right_wheel_joint            = correctPosition(sensor_msg.data[0],previous_readings.motor.position.right_wheel_joint,previous_readings.motor.overflow.right_wheel_joint);
        actual_readings.motor.position.left_wheel_joint             = correctPosition(sensor_msg.data[2],previous_readings.motor.position.left_wheel_joint,previous_readings.motor.overflow.left_wheel_joint);
        actual_readings.motor.position.right_suspension_wheel_joint = correctPosition(sensor_msg.data[1],previous_readings.motor.position.right_suspension_wheel_joint,previous_readings.motor.overflow.right_suspension_wheel_joint);
        actual_readings.motor.position.left_suspension_wheel_joint  = correctPosition(sensor_msg.data[3],previous_readings.motor.position.left_suspension_wheel_joint,previous_readings.motor.overflow.left_suspension_wheel_joint);
        actual_readings.motor.position.camera_pan                   = correctPosition(sensor_msg.data[4],previous_readings.motor.position.camera_pan,previous_readings.motor.overflow.camera_pan);
        actual_readings.motor.position.camera_tilt                  = correctPosition(sensor_msg.data[5],previous_readings.motor.position.camera_tilt,previous_readings.motor.overflow.camera_tilt);

        actual_readings.motor.angle.right_wheel_joint              = degToRad(posToDeg(actual_readings.motor.position.right_wheel_joint));
        actual_readings.motor.angle.left_wheel_joint               = degToRad(posToDeg(actual_readings.motor.position.left_wheel_joint));
        actual_readings.motor.angle.right_suspension_wheel_joint   = degToRad(posToDeg(actual_readings.motor.position.right_suspension_wheel_joint));
        actual_readings.motor.angle.left_suspension_wheel_joint    = degToRad(posToDeg(actual_readings.motor.position.left_suspension_wheel_joint));
        actual_readings.motor.angle.camera_pan                     = degToRad(posToDeg(actual_readings.motor.position.camera_pan));
        actual_readings.motor.angle.camera_tilt                    = degToRad(posToDeg(actual_readings.motor.position.camera_tilt));

        actual_readings.ultrasonic.range_fl = (centimeterToMeter(sensor_msg.data[6]) != 0.0f) ? centimeterToMeter(sensor_msg.data[6]) : NO_OBSTICAL;
        actual_readings.ultrasonic.range_fr = (centimeterToMeter(sensor_msg.data[7]) != 0.0f) ? centimeterToMeter(sensor_msg.data[7]) : NO_OBSTICAL;
        actual_readings.ultrasonic.range_rl = (centimeterToMeter(sensor_msg.data[8]) != 0.0f) ? centimeterToMeter(sensor_msg.data[6]) : NO_OBSTICAL;
        actual_readings.ultrasonic.range_rr = (centimeterToMeter(sensor_msg.data[9]) != 0.0f) ? centimeterToMeter(sensor_msg.data[9]) : NO_OBSTICAL;

        // publish joint states
        publishJointStates();

        // publish odometry
        publishOdometry();

        // safe sensor readings
        previous_readings = actual_readings;
    }
}

void Driver::publishOdometry()
{
    double b, r;
    double alpha_l, alpha_r, yaw;
    double phi_l, phi_r;
    double s;

    // Calculate vehicle geometry
    b = motor_control_parameters.wheel_base / 2;
    r = motor_control_parameters.wheel_radius;

    alpha_r = degToRad(posToDeg(actual_readings.motor.position.right_wheel_joint - previous_readings.motor.position.right_wheel_joint));
    alpha_l = degToRad(posToDeg(actual_readings.motor.position.left_wheel_joint - previous_readings.motor.position.left_wheel_joint));
    phi_r   = actual_readings.motor.angle.right_suspension_wheel_joint;
    phi_l   = actual_readings.motor.angle.left_suspension_wheel_joint;

    // Calculate angular velocity for ICC which is same as angular velocity of vehicle
    yaw = (alpha_l * sin(phi_l) + alpha_r * sin(phi_r)* r) / (2 * b);

    s = r * (alpha_l + alpha_r)/2;

    // Compute odometric pose
    pose.x   += s * cos(yaw);
    pose.y   += s * sin(yaw);
    pose.yaw += yaw;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw);

    // Publish tf
    odom_trans.header.stamp    = actual_readings.stamp;
    odom_trans.header.frame_id = "map";
    odom_trans.child_frame_id  = "base_link";

    odom_trans.transform.translation.x = pose.x;
    odom_trans.transform.translation.y = pose.y;
    odom_trans.transform.translation.z = 0.1;
    odom_trans.transform.rotation = odom_quat;

    odom_broadcaster.sendTransform(odom_trans);

    // Publish Odom
    //http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom
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

float Driver::limitVelocity(float value) {
    float velocity = value;
    if (value > 0.0) {
        if (value > motor_control_parameters.max_velocity) velocity = motor_control_parameters.max_velocity;
        if (value < motor_control_parameters.min_velocity) velocity = motor_control_parameters.min_velocity;
       if (previous_readings.ultrasonic.range_fl < motor_control_parameters.min_object_distance || previous_readings.ultrasonic.range_fr < motor_control_parameters.min_object_distance) {
           velocity = 0.0f;
           ROS_WARN("OBSTACLE in FRONT, STOPPING MOTORS distance to obstacle: %f", motor_control_parameters.min_object_distance);
       }
    } else if (value < 0.0) {
        if (value < -motor_control_parameters.max_velocity) velocity = -motor_control_parameters.max_velocity;
        if (value > -motor_control_parameters.min_velocity) velocity = -motor_control_parameters.min_velocity;
        if (previous_readings.ultrasonic.range_rl < motor_control_parameters.min_object_distance || previous_readings.ultrasonic.range_rr < motor_control_parameters.min_object_distance) {
            velocity = 0.0f;
            ROS_WARN("OBSTACLE in FRONT, STOPPING MOTORS distance to obstacle: %f", motor_control_parameters.min_object_distance);
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
