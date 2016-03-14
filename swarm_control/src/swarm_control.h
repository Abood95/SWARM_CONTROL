#ifndef SWARM_CONTROL_H_
#define SWARM_CONTROL_H_

#include <queue>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include "trajectory_plan.h"
#include <swarm_control/path.h>
#include <std_msgs/Float64.h>


const double dt = 0.02; //send desired-state messages at fixed rate, e.g. 0.02 sec = 50Hz
//dynamic parameters: should be tuned for target system
const double accel_max = 1.0; //1m/sec^2
const double alpha_max = 0.2; // rad/sec^2
const double speed_max = 1.0; //1 m/sec
const double omega_max = 1.0; //1 rad/sec
const double path_move_tol = 0.01; // if path points are within 1cm, fuggidaboutit

class SwarmControl{
private:
 ros::NodeHandle nh_;

    double accel_max_; 
    double alpha_max_; 
    double speed_max_; 
    double omega_max_; 
    double path_move_tol_; 


    std::vector<nav_msgs::Odometry> des_state_vec_1;
    std::vector<nav_msgs::Odometry> des_state_vec_2;
    std::vector<nav_msgs::Odometry> des_state_vec_3;
    std::vector<nav_msgs::Odometry> des_state_vec_4;
    std::vector<nav_msgs::Odometry> des_state_vec_5;
    std::vector<nav_msgs::Odometry> des_state_vec_6;

    geometry_msgs::PoseStamped start_pose_1;
    geometry_msgs::PoseStamped start_pose_2;
    geometry_msgs::PoseStamped start_pose_3;
    geometry_msgs::PoseStamped start_pose_4;
    geometry_msgs::PoseStamped start_pose_5;
    geometry_msgs::PoseStamped start_pose_6;

    geometry_msgs::PoseStamped end_pose_1;
    geometry_msgs::PoseStamped end_pose_2;
    geometry_msgs::PoseStamped end_pose_3;
    geometry_msgs::PoseStamped end_pose_4;
    geometry_msgs::PoseStamped end_pose_5;
    geometry_msgs::PoseStamped end_pose_6;

    nav_msgs::Odometry des_state_1;
    nav_msgs::Odometry des_state_2;
    nav_msgs::Odometry des_state_3;
    nav_msgs::Odometry des_state_4;
    nav_msgs::Odometry des_state_5;
    nav_msgs::Odometry des_state_6;

    nav_msgs::Odometry current_des_state_1;
    nav_msgs::Odometry current_des_state_2;
    nav_msgs::Odometry current_des_state_3;
    nav_msgs::Odometry current_des_state_4;
    nav_msgs::Odometry current_des_state_5;
    nav_msgs::Odometry current_des_state_6;

   // nav_msgs::Odometry current_vel_state;

    nav_msgs::Odometry halt_state_1;
    nav_msgs::Odometry halt_state_2;
    nav_msgs::Odometry halt_state_3;
    nav_msgs::Odometry halt_state_4;
    nav_msgs::Odometry halt_state_5;
    nav_msgs::Odometry halt_state_6;

    geometry_msgs::Twist halt_twist_;
    
    geometry_msgs::PoseStamped current_pose_1;
    geometry_msgs::PoseStamped current_pose_2;
    geometry_msgs::PoseStamped current_pose_3;
    geometry_msgs::PoseStamped current_pose_4;
    geometry_msgs::PoseStamped current_pose_5;
    geometry_msgs::PoseStamped current_pose_6;

    std_msgs::Float64 float_msg_;

    std::queue<geometry_msgs::PoseStamped> path_queue_1;
    std::queue<geometry_msgs::PoseStamped> path_queue_2;
    std::queue<geometry_msgs::PoseStamped> path_queue_3;
    std::queue<geometry_msgs::PoseStamped> path_queue_4;
    std::queue<geometry_msgs::PoseStamped> path_queue_5;
    std::queue<geometry_msgs::PoseStamped> path_queue_6;

    ros::Publisher desired_state_publisher_1;
    ros::Publisher desired_state_publisher_2;
    ros::Publisher desired_state_publisher_3;
    ros::Publisher desired_state_publisher_4;
    ros::Publisher desired_state_publisher_5;
    ros::Publisher desired_state_publisher_6;


    TrajBuilder trajBuilder_; 

    void initializePublishers();


public:
    SwarmControl(ros::NodeHandle& nh);//constructor  

    bool alarm1;
    bool alarm2;
    bool alarm3;
    bool alarm4;
    bool alarm5;
    bool alarm6;

    void set_init_pose(double x,double y, double psi);
    void pub_next_state();  
    void append_path_queue(geometry_msgs::PoseStamped pose) { path_queue_.push(pose); }  
    void append_path_queue(double x, double y, double psi) 
        { path_queue_.push(trajBuilder_.xyPsi2PoseStamped(x,y,psi)); }
};







#endif