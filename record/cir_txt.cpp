#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>

geometry_msgs::PoseStamped local_pose;
bool takeoff = false;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_pose.header = msg->header;
    local_pose.pose = msg->pose;
}


struct TrajectoryPoint {
    double x, y, z, u, v, w, du, dv, dw, psi;
};

std::vector<TrajectoryPoint> readTrajectory(const std::string& file_path) {
    std::ifstream file(file_path);
    std::vector<TrajectoryPoint> trajectory;
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        TrajectoryPoint point;
        iss >> point.x >> point.y >> point.z >> point.u >> point.v >> point.w >> point.du >> point.dv >> point.dw >> point.psi;
        trajectory.push_back(point);
    }
    return trajectory;
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

    ros::Rate rate(20.0);

    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    
    

    // Load trajectory from file
    // std::string package_path = ros::package::getPath("airo_trajectory");
    // std::string file_path = package_path + "/scripts/circle.txt";
    std::vector<TrajectoryPoint> trajectory = readTrajectory("/home/athena/airo_observer_ws/src/airo_control_interface/airo_trajectory/scripts/circle.txt");

    // Send a few setpoints before starting
    geometry_msgs::PoseStamped pose;
    for (int i = 100; ros::ok() && i > 0; --i) {
        pose.pose.position.x = trajectory[0].x;
        pose.pose.position.y = trajectory[0].y;
        pose.pose.position.z = trajectory[0].z;
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    size_t point_index = 0;
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

        if (!takeoff){
            std::cout<<"takeoff start"<<std::endl;
            pose.pose.position.x = 0;
            pose.pose.position.y = 0;
            pose.pose.position.z = 1;
            local_pos_pub.publish(pose);
            // Check if the drone has reached the takeoff position
            double dx = local_pose.pose.position.x - pose.pose.position.x;
            double dy = local_pose.pose.position.y - pose.pose.position.y;
            double dz = local_pose.pose.position.z - pose.pose.position.z;
            double distance = sqrt(dx*dx + dy*dy + dz*dz);
            if (distance <0.1){
                takeoff = true;
                std::cout<<"takeoff done"<<std::endl;
            }
        }
        
        if (takeoff){
            // Publish the next point in the trajectory
            if (point_index < trajectory.size()) {
                TrajectoryPoint& point = trajectory[point_index];
                pose.pose.position.x = point.x;
                pose.pose.position.y = point.y;
                pose.pose.position.z = point.z;
                local_pos_pub.publish(pose);

                std::ofstream save("/home/athena/airo_observer_ws/src/airo_control_interface/airo_control/log_cir/pid_cir3.csv", std::ios::app);
                save<<std::setprecision(20)<<ros::Time::now().toSec()<<
                    ","<<local_pose.pose.position.x <<","<<point.x
                    <<","<<local_pose.pose.position.y <<","<<point.y
                    <<","<<local_pose.pose.position.z <<","<<point.z<<std::endl;
                save.close();

                point_index++;
                } else {
                    ROS_INFO("Completed trajectory. Exiting.");
                    break;
                }

        }
        
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
