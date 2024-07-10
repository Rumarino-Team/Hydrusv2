#ifndef DEVICES_H
#define DEVICES_H

#include <ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>

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
#endif
