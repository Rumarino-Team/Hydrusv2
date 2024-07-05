

#include <Servo.h>
#include "devices.h"
#include "geometry_msgs/Vector3.h"


// Macro and enum declarations

#define MOTOR_NUM 8  // Total number of motors for the actuator
#define PWM_NEUTRAL 1500  // The thruster's output force is 0 lbf at this PWM value
#define model_name "hydrus"  // The name of the model
static Servo motors[MOTOR_NUM]; 
static char* thruster_topics[MOTOR_NUM];
static bool init_motors = false;


char* concatenateTopic(char* prefix, char* name, char* postfix) {
    // Calculate the total length
    int totalLength = 0;
    for (char* p = prefix; *p; ++p) totalLength++;
    for (char* p = name; *p; ++p) totalLength++;
    for (char* p = postfix; *p; ++p) totalLength++;

    // Allocate memory for the concatenated string
    char* result = new char[totalLength + 1]; // +1 for the null terminator

    // Copy the strings
    char* current = result;
    for (char* p = prefix; *p; ++p) *current++ = *p;
    for (char* p = name; *p; ++p) *current++ = *p;
    for (char* p = postfix; *p; ++p) *current++ = *p;

    // Null-terminate the result
    *current = '\0';

    return result;
}
//  This is not inside a function because of this error
// /root/Arduino/libraries/Hydrus/src/rosserial_hydrus_node/ros_embedded_node.cpp:17: undefined reference to `thruster_sub_1'
char* thruster_topics_0 =  concatenateTopic("/", model_name, "/thrusters/1/");
char* thruster_topics_1 =  concatenateTopic("/", model_name, "/thrusters/2/");
char* thruster_topics_2 =  concatenateTopic("/", model_name, "/thrusters/3/");
char* thruster_topics_3 =  concatenateTopic("/", model_name, "/thrusters/4/");
char* thruster_topics_4 =  concatenateTopic("/", model_name, "/thrusters/5/");
char* thruster_topics_5 =  concatenateTopic("/", model_name, "/thrusters/6/");
char* thruster_topics_6 =  concatenateTopic("/", model_name, "/thrusters/7/");
char* thruster_topics_7 =  concatenateTopic("/", model_name, "/thrusters/8/");


ros::Subscriber<geometry_msgs::Vector3> thruster_sub_1(thruster_topics_0, setThruster_1);
ros::Subscriber<geometry_msgs::Vector3> thruster_sub_2(thruster_topics_1, setThruster_2);
ros::Subscriber<geometry_msgs::Vector3> thruster_sub_3(thruster_topics_2, setThruster_3);
ros::Subscriber<geometry_msgs::Vector3> thruster_sub_4(thruster_topics_3, setThruster_4);
ros::Subscriber<geometry_msgs::Vector3> thruster_sub_5(thruster_topics_4, setThruster_5);
ros::Subscriber<geometry_msgs::Vector3> thruster_sub_6(thruster_topics_5, setThruster_6);
ros::Subscriber<geometry_msgs::Vector3> thruster_sub_7(thruster_topics_6, setThruster_7);
ros::Subscriber<geometry_msgs::Vector3> thruster_sub_8(thruster_topics_7, setThruster_8);



void initializeThrustersArduino(void)
{     
    init_motors = true;
    for (uint8_t i = 0; i < MOTOR_NUM; i++){
        motors[i].attach(i);
        motors[i].writeMicroseconds(PWM_NEUTRAL);  // This sets the thrusters output force to 0 lbf
    }   
}

//----------------------
//----------------------
//  Callbacks

// Setter for the thruster motor PWM values
void setThruster_1(const std_msgs::UInt16& thrusterValue)
{ 
    // The Vector ffrom the parameter is tecnically a scalar inside 
    // a 3d vector. Don't ask me why it is like that. uuv_simulator
    // implement it like that The y and z parameter of the vector is 
    // always 0.
    if(init_motors){
    motors[0].writeMicroseconds(thrusterValue);
    
}
}
void setThruster_2(const std_msgs::UInt16& thrusterValue)
{ 
    if(init_motors){
    motors[1].writeMicroseconds(thrusterValue);
    }
}

void setThruster_3(const std_msgs::UInt16& thrusterValue)
{ 
    if(init_motors){
    motors[2].writeMicroseconds(thrusterValue);
    }
}
void setThruster_4(const std_msgs::UInt16& thrusterValue)
{ 
    if(init_motors){
    motors[3].writeMicroseconds(thrusterValue);
    }
}
void setThruster_5(const std_msgs::UInt16& thrusterValue)
{ 
    if(init_motors){
    motors[4].writeMicroseconds(thrusterValue);
    }
}
void setThruster_6(const std_msgs::UInt16& thrusterValue)
{ 
    if(init_motors){
    motors[5].writeMicroseconds(thrusterValue);
    }
}
void setThruster_7(const std_msgs::UInt16& thrusterValue)
{ 
    if(init_motors){
    motors[6].writeMicroseconds(thrusterValue);
    }
}
void setThruster_8(const std_msgs::UInt16& thrusterValue)
{ 
    if(init_motors){
    motors[7].writeMicroseconds(thrusterValue);
    }
}