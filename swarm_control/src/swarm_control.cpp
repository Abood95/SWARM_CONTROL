#include "swarm_control.h"
//ExampleRosClass::ExampleRosClass(ros::NodeHandle* nodehandle):nh_(*nodehandle)

SwarmControl::SwarmControl(ros::NodeHandle& nh) : nh_(nh) {
    //as_(nh, "pub_des_state_server", boost::bind(&DesStatePublisher::executeCB, this, _1),false) {
    //as_.start(); //start the server running
    //configure the trajectory builder: 
    //dt_ = dt; //send desired-state messages at fixed rate, e.g. 0.02 sec = 50Hz
    trajBuilder_.set_dt(dt);
    //dynamic parameters: should be tuned for target system
    accel_max_ = accel_max;
    trajBuilder_.set_accel_max(accel_max_);
    alpha_max_ = alpha_max;
    trajBuilder_.set_alpha_max(alpha_max_);
    speed_max_ = speed_max;
    trajBuilder_.set_speed_max(speed_max_);
    omega_max_ = omega_max;
    trajBuilder_.set_omega_max(omega_max_);
    path_move_tol_ = path_move_tol;
    trajBuilder_.set_path_move_tol_(path_move_tol_);
    initializePublishers();

    //define a halt state; zero speed and spin, and fill with viable coords
    halt_twist_.linear.x = 0.0;
    halt_twist_.linear.y = 0.0;
    halt_twist_.linear.z = 0.0;
    halt_twist_.angular.x = 0.0;
    halt_twist_.angular.y = 0.0;
    halt_twist_.angular.z = 0.0;

    current_pose_1 = trajBuilder_.xyPsi2PoseStamped(0, 0, 0);
    start_pose_1 = current_pose_1;
    end_pose_1 = current_pose_1;
    current_des_state_1.twist.twist = halt_twist_;
    current_des_state_1.pose.pose = current_pose_1.pose;
    halt_state_1 = current_des_state_1;


    current_pose_2 = trajBuilder_.xyPsi2PoseStamped(0, 0, 0);
    start_pose_2 = current_pose_2;
    end_pose_2 = current_pose_2;
    current_des_state_2.twist.twist = halt_twist_;
    current_des_state_2.pose.pose = current_pose_2.pose;
    halt_state_2 = current_des_state_2;

    current_pose_3 = trajBuilder_.xyPsi2PoseStamped(0, 0, 0);
    start_pose_3 = current_pose_3;
    end_pose_3 = current_pose_3;
    current_des_state_3.twist.twist = halt_twist_;
    current_des_state_3.pose.pose = current_pose_3.pose;
    halt_state_3 = current_des_state_3;

    current_pose_4 = trajBuilder_.xyPsi2PoseStamped(0, 0, 0);
    start_pose_4 = current_pose_4;
    end_pose_4 = current_pose_4;
    current_des_state_4.twist.twist = halt_twist_;
    current_des_state_4.pose.pose = current_pose_4.pose;
    halt_state_4 = current_des_state_4;    

    current_pose_5 = trajBuilder_.xyPsi2PoseStamped(0, 0, 0);
    start_pose_5 = current_pose_5;
    end_pose_5 = current_pose_5;
    current_des_state_5.twist.twist = halt_twist_;
    current_des_state_5.pose.pose = current_pose_5.pose;
    halt_state_5 = current_des_state_5;

    current_pose_6 = trajBuilder_.xyPsi2PoseStamped(0, 0, 0);
    start_pose_6 = current_pose_6;
    end_pose_6 = current_pose_6;
    current_des_state_6.twist.twist = halt_twist_;
    current_des_state_6.pose.pose = current_pose_6.pose;
    halt_state_6 = current_des_state_6;

    alarm1 = false;
    alarm2 = false;
    alarm3 = false;
    alarm4 = false;
    alarm5 = false;
    alarm6 = false;

}


//member helper function to set up publishers;

void SwarmControl::initializePublishers() {
    ROS_INFO("Initializing Publishers");
    desired_state_publisher_ = nh_.advertise<nav_msgs::Odometry>("/desState", 1, true);
    des_psi_publisher_ = nh_.advertise<std_msgs::Float64>("/desPsi", 1);
}

void SwarmControl::set_init_pose(double x, double y, double psi) {
    current_pose_ = trajBuilder_.xyPsi2PoseStamped(x, y, psi);
}


void SwarmControl::pub_next_state() {
    //state machine; results in publishing a new desired state

}
