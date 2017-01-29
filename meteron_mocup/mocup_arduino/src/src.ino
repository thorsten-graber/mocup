// Include ros libraries
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>

// Include the Bricktronics libraries
#include <BricktronicsMegashield.h>
#include <BricktronicsMotor.h>
#include <BricktronicsUltrasonic.h>

#define BAUD 1000000 // 1Mbps
#define UPDATE_INTERVAL  10  // 1/update rate in ms -> 10ms = 50Hz

// Select the desired motor port (MOTOR_1 through MOTOR_6) in the constructor below.
BricktronicsMotor m1(BricktronicsMegashield::MOTOR_1);
BricktronicsMotor m2(BricktronicsMegashield::MOTOR_2);
BricktronicsMotor m3(BricktronicsMegashield::MOTOR_3);
BricktronicsMotor m4(BricktronicsMegashield::MOTOR_4);
BricktronicsMotor m5(BricktronicsMegashield::MOTOR_5);
BricktronicsMotor m6(BricktronicsMegashield::MOTOR_6);

// Select the sensor port for the sensor (SENSOR_1 through SENSOR_4) below.
// Use the jumpers to connect pins 1-2 and 4-5 for the ultrasonic sensor.
BricktronicsUltrasonic u1(BricktronicsMegashield::SENSOR_1);
BricktronicsUltrasonic u2(BricktronicsMegashield::SENSOR_2);
BricktronicsUltrasonic u3(BricktronicsMegashield::SENSOR_3);
BricktronicsUltrasonic u4(BricktronicsMegashield::SENSOR_4);

// ROS stuff
ros::NodeHandle  nh;
std_msgs::Int16MultiArray sensor_readings;

void motorCommandCallback(const std_msgs::Int16MultiArray& cmd_msg)
{   
  m1.setFixedDrive(cmd_msg.data[0]);
  m2.goToAngle(cmd_msg.data[1]);
  m3.setFixedDrive(cmd_msg.data[2]);
  m4.goToAngle(cmd_msg.data[3]);
  m5.goToAngle(cmd_msg.data[4]);
  m6.goToAngle(cmd_msg.data[5]);
}

ros::Publisher sensor_publisher("sensor_readings", &sensor_readings);
ros::Subscriber<std_msgs::Int16MultiArray> motor_command_subscriber("motor_command", motorCommandCallback);

SIGNAL(TIMER5_COMPA_vect)
{ 
    m1.update();
    m2.update();
    m3.update();
    m4.update();
    m5.update();
    m6.update();
    
    publishSensorReadings();
    
    TCNT5H=0x00;
    TCNT5L=0x00;
}



void publishSensorReadings()
{
  // motor encoders
  sensor_readings.data[0] = m1.getPosition();
  sensor_readings.data[1] = m2.getPosition();
  sensor_readings.data[2] = m3.getPosition();
  sensor_readings.data[3] = m4.getPosition();
  sensor_readings.data[4] = m5.getPosition();
  sensor_readings.data[5] = m6.getPosition();

  sensor_publisher.publish(&sensor_readings);
}

void setup()
{
  // Initialize ROS node
  nh.getHardware()->setBaud(BAUD);   
  nh.initNode(); 
  nh.advertise(sensor_publisher);
  nh.subscribe(motor_command_subscriber);

  // Initialize sensor readings container with constants
  sensor_readings.data_length = 10;
  sensor_readings.data = (int16_t *)malloc(sizeof(int16_t)*10);

  // Initialize the motor connections
  m1.pidSetUpdateFrequencyMS(UPDATE_INTERVAL);
  m1.begin();
  m2.pidSetUpdateFrequencyMS(UPDATE_INTERVAL);
  m2.begin();
  m3.pidSetUpdateFrequencyMS(UPDATE_INTERVAL);
  m3.begin();
  m4.pidSetUpdateFrequencyMS(UPDATE_INTERVAL);    
  m4.begin();
  m5.pidSetUpdateFrequencyMS(UPDATE_INTERVAL);
  m5.begin();
  m6.pidSetUpdateFrequencyMS(UPDATE_INTERVAL);
  m6.begin();

  // Initialize the ultrasonic sensor connections
  u1.begin();
  u2.begin();
  u3.begin();
  u4.begin();

  // Timer/Counter 5 initialization
  // Clock source: System Clock
  // Clock value: 16000,000 kHz
  // Mode: Normal top=0xFFFF
  // OC5A output: Disconnected
  // OC5B output: Disconnected
  // OC5C output: Disconnected
  // Noise Canceler: Off
  // Input Capture on Falling Edge
  // Timer Period: 4,096 ms
  // Timer5 Overflow Interrupt: Off
  // Input Capture Interrupt: Off
  // Compare A Match Interrupt: On
  // Compare B Match Interrupt: Off
  // Compare C Match Interrupt: Off
  TCCR5A=(0<<COM5A1) | (0<<COM5A0) | (0<<COM5B1) | (0<<COM5B0) | (0<<COM5C1) | (0<<COM5C0) | (0<<WGM51) | (0<<WGM50);
  TCCR5B=(0<<ICNC5) | (0<<ICES5) | (0<<WGM53) | (0<<WGM52) | (0<<CS52) | (1<<CS51) | (1<<CS50);
  TCNT5H=0x00;
  TCNT5L=0x00;
  ICR5H=0x00;
  ICR5L=0x00;
  OCR5AH=0x08;
  OCR5AL=0x00;
  OCR5BH=0x00;
  OCR5BL=0x00;
  OCR5CH=0x00;
  OCR5CL=0x00;
  
  // Timer/Counter 5 Interrupt(s) initialization
  TIMSK5=(0<<ICIE5) | (0<<OCIE5C) | (0<<OCIE5B) | (1<<OCIE5A) | (0<<TOIE5);

}

void loop() 
{
  // read ultrasonic sensors as soon as there is time
  sensor_readings.data[6] = u1.getDistance();
  sensor_readings.data[7] = u2.getDistance();
  sensor_readings.data[8] = u3.getDistance();
  sensor_readings.data[9] = u4.getDistance();

  nh.spinOnce();
}

