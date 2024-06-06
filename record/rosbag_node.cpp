#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/AttitudeTarget.h>
#include "airo_control/observer/ekf_observer.h"
#include "airo_control/observer/rd3_observer.h"
#include <sensor_msgs/BatteryState.h>
#include <fstream>


geometry_msgs::PoseStamped local_pose;
geometry_msgs::TwistStamped local_twist;
geometry_msgs::AccelStamped local_accel;
mavros_msgs::AttitudeTarget attitude_target;
std::string OBSERVER_TYPE, log_path;
std::unique_ptr<BASE_OBSERVER> disturbance_observer;
geometry_msgs::AccelStamped accel_disturbance,rosbag_disturbance;
sensor_msgs::BatteryState battery_state;
double a_z,force,MASS,K1,K2,K3;

void twist_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    local_twist.header.stamp = msg->header.stamp;
    local_twist.twist = msg->twist;
}
void imu_cb(const sensor_msgs::Imu::ConstPtr& msg){
    local_accel.header = msg->header;
    local_accel.accel.linear = msg->linear_acceleration;
    local_accel.accel.angular = msg->angular_velocity;
}
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_pose.header = msg->header;
    local_pose.pose = msg->pose;
}
void thrust_cb(const mavros_msgs::AttitudeTarget::ConstPtr& msg){
    attitude_target.header = msg->header;
    attitude_target.thrust = msg->thrust;
}
void battery_state_cb(const sensor_msgs::BatteryState::ConstPtr& msg){
    battery_state.current = msg->current;
    battery_state.header = msg->header;
    battery_state.percentage = msg->percentage;
    battery_state.voltage = msg->voltage;
}
void accel_disturbance_cb (const geometry_msgs::AccelStamped::ConstPtr& msg){
    rosbag_disturbance.accel = msg->accel;
    rosbag_disturbance.header = msg->header;
}
void log(){
    // Write data to the file
    std::string package_path = ros::package::getPath("airo_control");
    log_path = package_path + "/rosbag_logs/rosbag_debug_" + OBSERVER_TYPE + ".csv";
    std::ofstream save(log_path,std::ios::app);
    if (save.is_open()){
    save << std::setprecision(20) << ros::Time::now().toSec() <<
        "," << accel_disturbance.accel.linear.x << "," << accel_disturbance.accel.linear.y << "," << accel_disturbance.accel.linear.z << 
        "," << rosbag_disturbance.accel.linear.x << "," << rosbag_disturbance.accel.linear.y << "," << rosbag_disturbance.accel.linear.z <<std::endl;
        // "," <<local_accel.accel.linear.x << "," <<local_accel.accel.linear.y<<","<<local_accel.accel.linear.z<<std::endl;
    save.close();
    }
}
ros::Time current_time;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    ros::Rate rate(100.0);

    nh.getParam("rosbag_node/fsm/observer_type",OBSERVER_TYPE);
    nh.getParam("rosbag_node/thrust_model/mass",MASS);
    nh.getParam("rosbag_node/thrust_model/K1",K1);
    nh.getParam("rosbag_node/thrust_model/K2",K2);
    nh.getParam("rosbag_node/thrust_model/K3",K3);

    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose",10,pose_cb);
    ros::Subscriber twist_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local",10,twist_cb);
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data",10,imu_cb);
    ros::Subscriber thrust_sub = nh.subscribe<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude",10,thrust_cb);
    ros::Subscriber battery_state_sub = nh.subscribe<sensor_msgs::BatteryState>("/mavros/battery",10,battery_state_cb);
    ros::Subscriber accel_disturbance_sub = nh.subscribe<geometry_msgs::AccelStamped>("/airo_control/accel_disturbance",10,accel_disturbance_cb);

    if (OBSERVER_TYPE == "ekf"){
    disturbance_observer = std::make_unique<EKF>(nh);
    }
    else if (OBSERVER_TYPE == "rd3"){
        disturbance_observer = std::make_unique<RD3>(nh);
    }

    while(ros::ok()){ 
        force = K1*pow(battery_state.voltage,K2)*(K3*pow(attitude_target.thrust,2)+(1-K3)*attitude_target.thrust);
        a_z = force/MASS;

        if (OBSERVER_TYPE == "ekf" || OBSERVER_TYPE == "rd3"){
            accel_disturbance = disturbance_observer->observe(local_pose, local_twist, local_accel, a_z);
            log();
            
            std::cout<<"observer type: "<<OBSERVER_TYPE<<std::endl;
            std::cout << "[" << accel_disturbance.accel.linear.x 
                    << "," << accel_disturbance.accel.linear.y
                    << "," << accel_disturbance.accel.linear.z
                    << "]" << std::endl;
            std::cout<<"rosbag_disturbance: "<<rosbag_disturbance.accel.linear<<std::endl;
        }

        ros::spinOnce();
        ros::Duration(rate).sleep();
    }

    return 0;
}
