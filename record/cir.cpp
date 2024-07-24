#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <cmath>
#include <fstream>
#include <sstream>
#include <geometry_msgs/TwistStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


geometry_msgs::PoseStamped local_pose;
geometry_msgs::TwistStamped local_twist;
mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void twist_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    local_twist.header.stamp = msg->header.stamp;
    local_twist.twist = msg->twist;
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_pose.header = msg->header;
    local_pose.pose = msg->pose;
}

bool takeoff = false;

bool is_near(double x1, double y1, double x2, double y2, double threshold) {
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2)) < threshold;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/local_position/pose",100,pose_cb);
    ros::Subscriber twist_sub = nh.subscribe<geometry_msgs::TwistStamped>
            ("/mavros/local_position/velocity_local",10,twist_cb);
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 10);

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    // Parameters for circular trajectory
    double radius = 1.8;
    double speed = 2.0;
    double altitude = 1.0;
    double center_x = 0.5;
    double center_y = 0.5;
    double center_z = altitude;
    double psi;

    // Calculate time for one complete circle
    double circle_duration = 2 * M_PI * radius / speed;
    int num_circles = 3; // Number of circles to complete

    geometry_msgs::PoseStamped pose;
    ros::Time start_time = ros::Time::now();

    bool was_near_start = false;
    int circle_count = 0;
    double threshold = 0.2;

    // Send a few setpoints before starting
    for (int i = 100; ros::ok() && i > 0; --i) {
        pose.pose.position.x = center_x + radius * cos(0);
        pose.pose.position.y = center_y + radius * sin(0);
        pose.pose.position.z = center_z;
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while (ros::ok()) {
        if (current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if (set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if (!current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if (arming_client.call(arm_cmd) &&
                    arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        
        
        if(!takeoff){
            //takeoff
            std::cout<<"start takeoff"<<std::endl;
            pose.pose.position.x = 0;
            pose.pose.position.y = 0;
            pose.pose.position.z = 1;
            local_pos_pub.publish(pose);
            if(abs(local_pose.pose.position.x - 0) < 0.08 
            && abs(local_pose.pose.position.y - 0) < 0.08
            && abs(local_pose.pose.position.z - 1) < 0.1){
                std::cout<<"done takeoff"<<std::endl;
                takeoff = true;}
        }
        
        if (takeoff){
            // Calculate time elapsed
            ros::Duration elapsed_time = ros::Time::now() - start_time;
            double t = elapsed_time.toSec();

            // Calculate circular trajectory
            pose.pose.position.x = center_x - radius * cos(speed * t / radius);
            pose.pose.position.y = center_y - radius * sin(speed * t / radius);
            pose.pose.position.z = center_z;
            
            std::cout<<"start circular path"<<std::endl;
            local_pos_pub.publish(pose);

            // Calculate circular trajectory
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = -radius * cos(t * speed / radius) + center_x;
            pose.pose.position.y = -radius * sin(t * speed / radius) + center_y;
            pose.pose.position.z = altitude;

            // Calculate psi (yaw angle)
            psi = atan2(sin(t * speed / radius), cos(t * speed / radius));

            // Convert yaw angle to quaternion
            tf2::Quaternion quat;
            quat.setRPY(0, 0, psi);
            pose.pose.orientation = tf2::toMsg(quat);

            local_pos_pub.publish(pose);

            std::ofstream save("/home/athena/airo_observer_ws/src/airo_control_interface/airo_control/log_cir/pid_cir8.csv", std::ios::app);
            save<<std::setprecision(20)<<ros::Time::now().toSec()<<
                ","<<local_pose.pose.position.x <<","<<pose.pose.position.x
                <<","<<local_pose.pose.position.y <<","<<pose.pose.position.y
                <<","<<local_pose.pose.position.z <<","<<pose.pose.position.z<<std::endl;
            save.close();

            // Check if UAV crosses the starting point
        bool is_near_start = is_near(pose.pose.position.x, pose.pose.position.y, center_x + radius, center_y, threshold);
        if (is_near_start && !was_near_start) {
            circle_count++;
            ROS_INFO("Completed circle %d", circle_count);
            if (circle_count > 3) {
                ROS_INFO("Completed 3 circles. Exiting.");
                break;
            }
        }
        was_near_start = is_near_start;}
        

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
