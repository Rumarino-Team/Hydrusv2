
#ifndef DEVICES_H

#define DEVICES_H

#include <ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>

// TODO: Could change the message structure, so each msg class doesn't use
// a header to define it

// SG90 Servo (for camera)

void initializeCameraServo(void);
void setCameraAngle(const std_msgs::UInt16&);
extern ros::Subscriber<std_msgs::UInt16> camera_angle_sub;

// Newton Subsea Gripper

void initializeGripper(void);
void manipulateGripper(const std_msgs::UInt8&);
extern ros::Subscriber<std_msgs::UInt8> gripper_mode_sub;

void initializeThrustersArduino(void);
// T100 thrusters declarations
void setThruster_1(const geometry_msgs::Vector3& thusterVector);
void setThruster_2(const geometry_msgs::Vector3& thusterVector);
void setThruster_3(const geometry_msgs::Vector3& thusterVector);
void setThruster_4(const geometry_msgs::Vector3& thusterVector);
void setThruster_5(const geometry_msgs::Vector3& thusterVector);
void setThruster_6(const geometry_msgs::Vector3& thusterVector);
void setThruster_7(const geometry_msgs::Vector3& thusterVector);
void setThruster_8(const geometry_msgs::Vector3& thusterVector);
extern ros::Subscriber<geometry_msgs::Vector3> thruster_sub_1;
extern ros::Subscriber<geometry_msgs::Vector3> thruster_sub_2;
extern ros::Subscriber<geometry_msgs::Vector3> thruster_sub_3;
extern ros::Subscriber<geometry_msgs::Vector3> thruster_sub_4;
extern ros::Subscriber<geometry_msgs::Vector3> thruster_sub_5;
extern ros::Subscriber<geometry_msgs::Vector3> thruster_sub_6;
extern ros::Subscriber<geometry_msgs::Vector3> thruster_sub_7;
extern ros::Subscriber<geometry_msgs::Vector3> thruster_sub_8;

// MS5837 pressure sensor declarations

void publishCurrentDepth(void);
void initializePressureSensor(void);
extern ros::Publisher current_depth_pub;

#endif