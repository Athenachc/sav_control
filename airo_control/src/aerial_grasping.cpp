#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <airo_message/FSMInfo.h>
#include <airo_message/TakeoffLandTrigger.h>
#include <airo_message/ReferenceStamped.h>
#include "std_msgs/Float32.h"
#include <fstream>

geometry_msgs::PoseStamped local_pose,object_pose, current_object_pose;
airo_message::FSMInfo fsm_info;
int cnt = 0;

enum State{
    TAKEOFF,
    COMMAND,
    LAND
};

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_pose.header = msg->header;
    local_pose.pose = msg->pose;
}

void fsm_info_cb(const airo_message::FSMInfo::ConstPtr& msg){
    fsm_info.header = msg->header;
    fsm_info.is_landed = msg->is_landed;
    fsm_info.is_waiting_for_command = msg->is_waiting_for_command;
}

void object_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& object_pose){
    current_object_pose.pose.position.x = object_pose->pose.position.x;
    current_object_pose.pose.position.y = object_pose->pose.position.y;
    current_object_pose.pose.position.z = object_pose->pose.position.z;   
}

void setDutyCycle(int dutyCycle) {
    std::ofstream pwmFile;
    pwmFile.open("/sys/class/pwm/pwmchip4/pwm1/duty_cycle");
    pwmFile << dutyCycle;
    pwmFile.close();
}

void flightCommandCallback(const std_msgs::Float32::ConstPtr& msg) {
  // Extract the desired duty cycle from the message
  float dutyCycle = msg->data;

  // Update the duty cycle
  setDutyCycle(dutyCycle);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    ros::Rate rate(20.0);
    State state = TAKEOFF;

    airo_message::ReferenceStamped target_pose_1;
    airo_message::ReferenceStamped target_pose_2;
    airo_message::ReferenceStamped target_pose_3;
    airo_message::ReferenceStamped target_pose_4;
    airo_message::ReferenceStamped target_pose_5;
    airo_message::ReferenceStamped target_pose_6;
    airo_message::TakeoffLandTrigger takeoff_land_trigger;
    bool target_1_reached = false;
    bool target_2_reached = false;
    bool target_3_reached = false;
    bool target_4_reached = false;
    bool target_5_reached = false;
    bool target_6_reached = false;

    // Subscribe to the MAVROS topic for flight commands
    ros::Subscriber sub = nh.subscribe("pwm_commands", 1, flightCommandCallback);
    // Enable the PWM controller
    std::ofstream exportFile;
    exportFile.open("/sys/class/pwm/pwmchip4/export");
    exportFile << "1";
    exportFile.close();

    // Set the period time
    std::ofstream periodFile;
    periodFile.open("/sys/class/pwm/pwmchip4/pwm1/period");
    periodFile << "2000000";
    periodFile.close();

    // Set the duty cycle to 800000 (gripper initially opens)
    setDutyCycle(800000);

    // Enable output from the PWM pin
    std::ofstream enableFile;
    enableFile.open("/sys/class/pwm/pwmchip4/pwm1/enable");
    enableFile << "1";
    enableFile.close();

    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",100,pose_cb);
    ros::Subscriber object_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/gh034_sav_obj/pose", 10, object_pose_cb);
    ros::Subscriber fsm_info_sub = nh.subscribe<airo_message::FSMInfo>("/airo_control/fsm_info",10,fsm_info_cb);
    ros::Publisher command_pub = nh.advertise<airo_message::ReferenceStamped>("/airo_control/setpoint",10);
    ros::Publisher takeoff_land_pub = nh.advertise<airo_message::TakeoffLandTrigger>("/airo_control/takeoff_land_trigger",10);

    while(ros::ok()){
        //Get current object pose as initial object pose
        object_pose.pose.position.x = current_object_pose.pose.position.x;
        object_pose.pose.position.y = current_object_pose.pose.position.y;
        object_pose.pose.position.z = current_object_pose.pose.position.z;

        // Hovering pt
        target_pose_1.ref.pose.position.x = current_object_pose.pose.position.x-0.0175; //shuttlerock: -0.025; ball: -0.017; dyed water: -0.037
        target_pose_1.ref.pose.position.y = current_object_pose.pose.position.y-0.012; //shuttlerock: +0.03; ball: +0.012; dyed water: -0.1
        target_pose_1.ref.pose.position.z = 1; //shuttlerock: 1; ball: 1; dyed water: 1
        target_pose_1.ref.pose.orientation.w = 1.0;
        target_pose_1.ref.pose.orientation.x = 0.0;
        target_pose_1.ref.pose.orientation.y = 0.0;
        target_pose_1.ref.pose.orientation.z = 0.0;

        // Approaching pt
        target_pose_2.ref.pose.position.x = current_object_pose.pose.position.x-0.0175; //shuttlerock: -0.025; ball: -0.017; dyed water: -0.03
        target_pose_2.ref.pose.position.y = current_object_pose.pose.position.y-0.012; //shuttlerock: +0.03; ball: +0.012; dyed water: +0.00
        target_pose_2.ref.pose.position.z = 0.6; //shuttlerock: 0.6; ball: 0.6; dyed water: 0.6
        target_pose_2.ref.pose.orientation.w = 1.0;
        target_pose_2.ref.pose.orientation.x = 0.0;
        target_pose_2.ref.pose.orientation.y = 0.0;
        target_pose_2.ref.pose.orientation.z = 0.0;


        // Grasping pt
        target_pose_3.ref.pose.position.x = current_object_pose.pose.position.x-0.0175; //shuttlerock: -0.025; ball: -0.017; dyed water: -0.03
        target_pose_3.ref.pose.position.y = current_object_pose.pose.position.y-0.012; //shuttlerock: +0.03; ball: +0.012; dyed water: +0.00
        target_pose_3.ref.pose.position.z = 0.28; //shuttlerock: 0.31; ball: 0.28; dyed water: 0.29
        target_pose_3.ref.pose.orientation.w = 1.0;
        target_pose_3.ref.pose.orientation.x = 0.0;
        target_pose_3.ref.pose.orientation.y = 0.0;
        target_pose_3.ref.pose.orientation.z = 0.0;

        // Lifting pt
        target_pose_4.ref.pose.position.x = -2;
        target_pose_4.ref.pose.position.y = 0;
        target_pose_4.ref.pose.position.z = 1;
        target_pose_4.ref.pose.orientation.w = 1.0;
        target_pose_4.ref.pose.orientation.x = 0.0;
        target_pose_4.ref.pose.orientation.y = 0.0;
        target_pose_4.ref.pose.orientation.z = 0.0;

        // Releasing pt
        target_pose_5.ref.pose.position.x = -1.653;
        target_pose_5.ref.pose.position.y = -1.03;
        target_pose_5.ref.pose.position.z = 0.6;
        target_pose_5.ref.pose.orientation.w = 1.0;
        target_pose_5.ref.pose.orientation.x = 0.0;
        target_pose_5.ref.pose.orientation.y = 0.0;
        target_pose_5.ref.pose.orientation.z = 0.0;

        // Pre-landing pt
        target_pose_6.ref.pose.position.x = 0;
        target_pose_6.ref.pose.position.y = -1.03;
        target_pose_6.ref.pose.position.z = 0.6;
        target_pose_6.ref.pose.orientation.w = 1.0;
        target_pose_6.ref.pose.orientation.x = 0.0;
        target_pose_6.ref.pose.orientation.y = 0.0;
        target_pose_6.ref.pose.orientation.z = 0.0;

        switch(state){
            case TAKEOFF:{
                if(fsm_info.is_landed == true){
                    setDutyCycle(1200000);
                    while(ros::ok()){
                        takeoff_land_trigger.takeoff_land_trigger = true; // Takeoff
                        takeoff_land_trigger.header.stamp = ros::Time::now();
                        takeoff_land_pub.publish(takeoff_land_trigger);
                        ros::spinOnce();
                        ros::Duration(0.5).sleep();
                        if(fsm_info.is_waiting_for_command){
                            state = COMMAND;
                            break;
                        }
                    }
                }
                break;
            }

            case COMMAND:{
                if(fsm_info.is_waiting_for_command){
                    if(!target_1_reached){
                        target_pose_1.header.stamp = ros::Time::now();
                        command_pub.publish(target_pose_1);
                        if(abs(local_pose.pose.position.x - target_pose_1.ref.pose.position.x) < 0.08
                        && abs(local_pose.pose.position.y - target_pose_1.ref.pose.position.y) < 0.08
                        && abs(local_pose.pose.position.z - target_pose_1.ref.pose.position.z) < 0.1){
                        std::cout<<"x tracking error: "<< local_pose.pose.position.x - target_pose_1.ref.pose.position.x<<std::endl;
                        std::cout<<"y tracking error: "<< local_pose.pose.position.y - target_pose_1.ref.pose.position.y<<std::endl;
                        std::cout<<"z tracking error: "<< local_pose.pose.position.z - target_pose_1.ref.pose.position.z<<std::endl;
                        std::cout<<"hover over obj"<<std::endl;
                        std::cout<<"cnt: "<<cnt<<std::endl;
                            
                            target_1_reached = true;
                            
                        }
                    }
                    if(target_1_reached){
                        target_pose_2.header.stamp = ros::Time::now();
                        command_pub.publish(target_pose_2);
                        if(abs(local_pose.pose.position.x - target_pose_2.ref.pose.position.x) < 0.08
                        && abs(local_pose.pose.position.y - target_pose_2.ref.pose.position.y) < 0.08
                        && abs(local_pose.pose.position.z - target_pose_2.ref.pose.position.z) < 0.1){
                        std::cout<<"x tracking error: "<< local_pose.pose.position.x - target_pose_2.ref.pose.position.x<<std::endl;
                        std::cout<<"y tracking error: "<< local_pose.pose.position.y - target_pose_2.ref.pose.position.y<<std::endl;
                        std::cout<<"z tracking error: "<< local_pose.pose.position.z - target_pose_2.ref.pose.position.z<<std::endl;
                        std::cout<<"approaching obj"<<std::endl;
                        std::cout<<"cnt: "<<cnt<<std::endl;
                        target_2_reached = true;
                        }
                    }
                    if(target_2_reached){
                        target_pose_3.header.stamp = ros::Time::now();
                        command_pub.publish(target_pose_3);    
                        if(abs(local_pose.pose.position.x - target_pose_3.ref.pose.position.x) < 0.08
                        && abs(local_pose.pose.position.y - target_pose_3.ref.pose.position.y) < 0.08
                        && abs(local_pose.pose.position.z - target_pose_3.ref.pose.position.z) < 0.08){
                            std::cout<<"reach desired grasping point"<<std::endl;
                            std::cout<<"x tracking error: "<< local_pose.pose.position.x - target_pose_3.ref.pose.position.x << std::endl;
                            std::cout<<"y tracking error: "<< local_pose.pose.position.y - target_pose_3.ref.pose.position.y << std::endl;
                            std::cout<<"z tracking error: "<< local_pose.pose.position.z - target_pose_3.ref.pose.position.z << std::endl;
                            
                            target_3_reached = true;
                            setDutyCycle(1600000);

                            
                        }
                    }
                    if(target_3_reached){
                        target_pose_3.header.stamp = ros::Time::now();
                        command_pub.publish(target_pose_3);
                        setDutyCycle(1600000);
                        if(abs(local_pose.pose.position.x - target_pose_3.ref.pose.position.x) 
                        +  abs(local_pose.pose.position.y - target_pose_3.ref.pose.position.y) 
                        +  abs(local_pose.pose.position.z - target_pose_3.ref.pose.position.z) < 0.3){
                            std::cout<<"grasping"<<std::endl;
                            std::cout<<"x tracking error: "<< local_pose.pose.position.x - target_pose_3.ref.pose.position.x << std::endl;
                            std::cout<<"y tracking error: "<< local_pose.pose.position.y - target_pose_3.ref.pose.position.y << std::endl;
                            std::cout<<"z tracking error: "<< local_pose.pose.position.z - target_pose_3.ref.pose.position.z << std::endl;
                            std::cout<<"cnt: "<<cnt<<std::endl;
                            if (cnt < 150){
                                cnt++;
                                std::cout<<"grasping"<<std::endl;
                            }
                            else{
                                cnt = 0;
                                target_4_reached = true;
                            } 
                        }
                    }
                    if(target_4_reached){
                        target_pose_4.header.stamp = ros::Time::now();
                        command_pub.publish(target_pose_4);
                        if(abs(local_pose.pose.position.x - target_pose_4.ref.pose.position.x)
                        + abs(local_pose.pose.position.y - target_pose_4.ref.pose.position.y)
                        + abs(local_pose.pose.position.z - target_pose_4.ref.pose.position.z) < 0.3){
                            std::cout<<"--- ---- current tracking error -------"<<std::endl;
                            std::cout<<"x tracking error: "<< local_pose.pose.position.x - target_pose_4.ref.pose.position.x << std::endl;
                            std::cout<<"y tracking error: "<< local_pose.pose.position.y - target_pose_4.ref.pose.position.y << std::endl;
                            std::cout<<"z tracking error: "<< local_pose.pose.position.z - target_pose_4.ref.pose.position.z << std::endl;
                            std::cout<<"cnt: "<<cnt<<std::endl;
                            std::cout<<"holding and lifting object"<<std::endl;
                            
                            if (cnt < 1000){
                                cnt++;
                            }
                            else{
                                state = LAND;
                            }
                            // if (cnt < 90){
                            //     cnt++;
                            //     std::cout<<"holding and lifting object"<<std::endl;
                            // }
                            // else{
                            //     cnt = 0;
                            //     target_5_reached = true;
                            // }
                        }
                    }
                    if (target_5_reached){
                        target_pose_5.header.stamp = ros::Time::now();
                        command_pub.publish(target_pose_5);
                        if(abs(local_pose.pose.position.x - target_pose_5.ref.pose.position.x)
                         + abs(local_pose.pose.position.y - target_pose_5.ref.pose.position.y)
                         + abs(local_pose.pose.position.z - target_pose_5.ref.pose.position.z) < 0.3){
                            std::cout<<"--- ---- current tracking error -------"<<std::endl;
                            std::cout<<"x tracking error: "<< local_pose.pose.position.x - target_pose_5.ref.pose.position.x << std::endl;
                            std::cout<<"y tracking error: "<< local_pose.pose.position.y - target_pose_5.ref.pose.position.y << std::endl;
                            std::cout<<"z tracking error: "<< local_pose.pose.position.z - target_pose_5.ref.pose.position.z << std::endl;
                            std::cout<<"cnt: "<<cnt<<std::endl;
                            if (cnt<30){
                                cnt++;
                                //Update the duty cycle to 1200000 (Gripper fully opens)
                                setDutyCycle(1200000);
                                std::cout<<"releasing object"<<std::endl;
                            }
                            else{
                                cnt = 0;
                                
                                target_6_reached = true;
                            }
                        }
                    }
                    if(target_6_reached){
                        target_pose_6.header.stamp = ros::Time::now();
                        command_pub.publish(target_pose_6);
                        setDutyCycle(1200000);
                        if(abs(local_pose.pose.position.x - target_pose_6.ref.pose.position.x)
                         + abs(local_pose.pose.position.y - target_pose_6.ref.pose.position.y)
                         + abs(local_pose.pose.position.z - target_pose_6.ref.pose.position.z) < 0.3){
                            std::cout<<"--- ---- current tracking error -------"<<std::endl;
                            std::cout<<"x tracking error: "<< local_pose.pose.position.x - target_pose_6.ref.pose.position.x << std::endl;
                            std::cout<<"y tracking error: "<< local_pose.pose.position.y - target_pose_6.ref.pose.position.y << std::endl;
                            std::cout<<"z tracking error: "<< local_pose.pose.position.z - target_pose_6.ref.pose.position.z << std::endl;
                            std::cout<<"cnt: "<<cnt<<std::endl;
                            //state = LAND;
                        }
                    }
                }
                break;
            }

            case LAND:{
                if(fsm_info.is_waiting_for_command){
                    // Update the duty cycle to 1200000 (Gripper fully opens)
                    setDutyCycle(1200000);
                    takeoff_land_trigger.takeoff_land_trigger = false; // Land
                    takeoff_land_trigger.header.stamp = ros::Time::now();
                    takeoff_land_pub.publish(takeoff_land_trigger);
                    std::cout<<"landing"<<std::endl;
                }
                break;
            }
        }

        ros::spinOnce();
        ros::Duration(rate).sleep();
    }

    return 0;
}
