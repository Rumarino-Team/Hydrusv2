#ifndef DEVICES_H
#define DEVICES_H

#include <ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>

void initializeThrustersArduino(void);

// T100 thrusters declarations
void setThruster_1(const std_msgs::UInt16& thrusterValue);
void setThruster_2(const std_msgs::UInt16& thrusterValue);
void setThruster_3(const std_msgs::UInt16& thrusterValue);
void setThruster_4(const std_msgs::UInt16& thrusterValue);
void setThruster_5(const std_msgs::UInt16& thrusterValue);
void setThruster_6(const std_msgs::UInt16& thrusterValue);
void setThruster_7(const std_msgs::UInt16& thrusterValue);
void setThruster_8(const std_msgs::UInt16& thrusterValue);

extern ros::Subscriber<std_msgs::UInt16> thruster_sub_1;
extern ros::Subscriber<std_msgs::UInt16> thruster_sub_2;
extern ros::Subscriber<std_msgs::UInt16> thruster_sub_3;
extern ros::Subscriber<std_msgs::UInt16> thruster_sub_4;
extern ros::Subscriber<std_msgs::UInt16> thruster_sub_5;
extern ros::Subscriber<std_msgs::UInt16> thruster_sub_6;
extern ros::Subscriber<std_msgs::UInt16> thruster_sub_7;
extern ros::Subscriber<std_msgs::UInt16> thruster_sub_8;

#endif
