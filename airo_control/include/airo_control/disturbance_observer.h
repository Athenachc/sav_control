#ifndef DISTURBANCE_OBSERVER_H
#define DISTURBANCE_OBSERVER_H

#include <iostream>
#include <fstream>
#include <istream>
#include <sstream>
#include <cstdio>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <geometry_msgs/AccelStamped.h>
#include <tf/tf.h>

class DISTURBANCE_OBSERVER{
    private:
    Eigen::Vector3d q2rpy(const geometry_msgs::Quaternion&);  // Quaternion to euler angle

    struct MEASUREMENT_STATES{
        double x,y,z,u,v,w,phi,theta,psi,thrust_x,thrust_y,thrust_z;
    };
    MEASUREMENT_STATES measurement_states;
    
    struct SYSTEM_STATES{
        double x,y,z,u,v,w,phi,theta,psi,disturbance_x,disturbance_y,disturbance_z,
        p,q,r,phi_dot_w,theta_dot_w,psi_dot_w;
    };
    SYSTEM_STATES system_states;
    
    struct ACCEL{
        double x_b,y_b,z_b,x_w,y_w,z_w;
    };
    ACCEL accel;

    // Parameters
    double FSM_FREQUENCY,hover_thrust,current_euler,R_POS_X,R_POS_Y,R_POS_Z,R_VEL_X,R_VEL_Y,R_VEL_Z,R_CONTROL_X,R_CONTROL_Y,R_CONTROL_Z,Q_POS_X,Q_POS_Y,Q_POS_Z,Q_VEL_X,Q_VEL_Y,Q_VEL_Z,Q_DISTURBANCE_X,Q_DISTURBANCE_Y,Q_DISTURBANCE_Z;
    // double FSM_FREQUENCY,hover_thrust,current_euler,R_VEL_X,R_VEL_Y,R_VEL_Z,R_CONTROL_X,R_CONTROL_Y,R_CONTROL_Z,Q_VEL_X,Q_VEL_Y,Q_VEL_Z,Q_DISTURBANCE_X,Q_DISTURBANCE_Y,Q_DISTURBANCE_Z;
    double g = 9.80665;
    double dt;
    int cout_counter = 0;
    double mass = 1.0;

    // Raw disturbance parameters
    int window_size = 100;
    std::deque<double> delta_x_W_buffer;
    std::deque<double> delta_y_W_buffer;
    std::deque<double> delta_z_W_buffer;
    double meanDelta_x_W = 0.0;
    double meanDelta_y_W = 0.0;
    double meanDelta_z_W = 0.0;

    // Weights
    // int m = 6;
    // int n = 6;
    // Eigen::Matrix<double,6,6> Q_noise,R_noise,P0,esti_P;       // Process noise matrix, Measurement noise matrix, Initial covariance, Estimate covariance
    // Eigen::Matrix<double,1,6> Q_cov,R_cov;                      // Process noise value, Measurement noise value
    // Eigen::Matrix<double,3,3> R_z,R_y,R_x,R_b2w;                 // Rotation matrix in z,y,x, Rotaion matrix from body frame to world frame
    // Eigen::Matrix<double,3,1> euler_body, euler_world;           // Euler angles in body frame and world frame
    // Eigen::Matrix<double,3,1> accel_body, accel_world;           // Linear acceleration in body frame and world frame 

    // Eigen::Matrix<double,3,1> thrust_body, thrust_world;         // Thrust in body frame and world frame 

    // // EKF Parameters
    // Eigen::Matrix<double,3,1> input_u;                           // Inputs
    // Eigen::Matrix<double,6,1> meas_y;                           // Measurement vector
    // Eigen::Matrix<double,6,1> esti_x;    
    int m = 9;
    int n = 9;
    Eigen::Matrix<double,9,9> Q_noise,R_noise,P0,esti_P;       // Process noise matrix, Measurement noise matrix, Initial covariance, Estimate covariance
    Eigen::Matrix<double,1,9> Q_cov,R_cov;                      // Process noise value, Measurement noise value
    
    // EKF Parameters
    Eigen::Matrix<double,3,1> input_u;                           // Inputs
    Eigen::Matrix<double,9,1> meas_y;                           // Measurement vector
    Eigen::Matrix<double,9,1> esti_x;                             // Estimate states

    public:
    DISTURBANCE_OBSERVER(ros::NodeHandle&,const double&);
    geometry_msgs::Vector3Stamped observe(const geometry_msgs::PoseStamped&, const geometry_msgs::TwistStamped&,const mavros_msgs::AttitudeTarget, 
    const geometry_msgs::AccelStamped&);
    Eigen::MatrixXd RK4(Eigen::MatrixXd x, Eigen::MatrixXd u);                   // EKF predict and update
    Eigen::MatrixXd f(Eigen::MatrixXd x, Eigen::MatrixXd u);                     // system process model
    Eigen::MatrixXd h(Eigen::MatrixXd x);                                        // measurement model
    Eigen::MatrixXd compute_jacobian_F(Eigen::MatrixXd x, Eigen::MatrixXd u);    // compute Jacobian of system process model
    Eigen::MatrixXd compute_jacobian_H(Eigen::MatrixXd x);                       // compute Jacobian of measurement model

    Eigen::Matrix3d q2ROT(const geometry_msgs::Quaternion);
    Eigen::Vector3d disturbance_raw(const geometry_msgs::AccelStamped& ,const mavros_msgs::AttitudeTarget& ,const geometry_msgs::PoseStamped& );

};

#endif