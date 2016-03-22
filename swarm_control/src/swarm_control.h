#ifndef SWARM_CONTROL_H_
#define SWARM_CONTROL_H_

#include <queue>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include "trajectory_plan.h"
#include <swarm_control/path.h>
#include <std_msgs/Float64.h>
#include<math.h>


const double dt = 0.02; //send desired-state messages at fixed rate, e.g. 0.02 sec = 50Hz
//dynamic parameters: should be tuned for target system
const double accel_max = 1.0; //1m/sec^2
const double alpha_max = 0.2; // rad/sec^2
const double speed_max = 1.0; //1 m/sec
const double omega_max = 1.0; //1 rad/sec
const double path_move_tol = 0.01; // if path points are within 1cm, fuggidaboutit

const double dange_range_1 = M_PI/4;
const double dange_range_2 = M_PI/8;  //may need to  be tuned!!!!!!!!!

class SwarmControl{
private:
 ros::NodeHandle nh_;

    double accel_max_; 
    double alpha_max_; 
    double speed_max_; 
    double omega_max_; 
    double path_move_tol_; 

    double range_1;
    double range_2;

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

    nav_msgs::Odometry des_pose_1;   ///target pose
    nav_msgs::Odometry des_pose_2;
    nav_msgs::Odometry des_pose_3;
    nav_msgs::Odometry des_pose_4;
    nav_msgs::Odometry des_pose_5;
    nav_msgs::Odometry des_pose_6;

    nav_msgs::Odometry des_state_1;  ////target state
    nav_msgs::Odometry des_state_2;
    nav_msgs::Odometry des_state_3;
    nav_msgs::Odometry des_state_4;
    nav_msgs::Odometry des_state_5;
    nav_msgs::Odometry des_state_6;

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

    ros::Publisher desired_state_publisher_1;
    ros::Publisher desired_state_publisher_2;
    ros::Publisher desired_state_publisher_3;
    ros::Publisher desired_state_publisher_4;
    ros::Publisher desired_state_publisher_5;
    ros::Publisher desired_state_publisher_6;

    ros::Publisher des_psi_publisher_1;
    ros::Publisher des_psi_publisher_2;
    ros::Publisher des_psi_publisher_3;
    ros::Publisher des_psi_publisher_4;
    ros::Publisher des_psi_publisher_5;
    ros::Publisher des_psi_publisher_6;


    TrajBuilder trajBuilder_; 

    void initializePublishers();


public:
    SwarmControl(ros::NodeHandle& nh);//constructor  


    void set_des_pose(double x,double y, double psi);
	
	void set_current_pose(double x, double y, double psi);

	void swarm_obstacles_state(std::vector<geometry_msgs::PoseStamped> obst_posi,
			geometry_msgs::PoseStamped pose,
			geometry_msgs::PoseStamped target_posi,
			std::vector<nav_msgs::Odometry> &vec_of_states);
			
    double SingleConsumpt(geometry_msgs::PoseStamped target,
	                       geometry_msgs::PoseStamped robot,
	      std::vector<geometry_msgs::PoseStamped> obstacle);
		  
    void ComputeConsumption(geometry_msgs::PoseStamped swarm1_pose,
		geometry_msgs::PoseStamped current_swarm2_pose,
		geometry_msgs::PoseStamped current_swarm3_pose,
		geometry_msgs::PoseStamped current_swarm4_pose,
		geometry_msgs::PoseStamped current_swarm5_pose,
		geometry_msgs::PoseStamped current_swarm6_pose,
		std::vector<geometry_msgs::PoseStamped> swarm2_obst,
		std::vector<geometry_msgs::PoseStamped> swarm3_obst,
		std::vector<geometry_msgs::PoseStamped> swarm4_obst,
		std::vector<geometry_msgs::PoseStamped> swarm5_obst,
		std::vector<geometry_msgs::PoseStamped> swarm6_obst,
		std::vector<double> &swarm2_consump,
		std::vector<double> &swarm3_consump,
		std::vector<double> &swarm4_consump,
		std::vector<double> &swarm5_consump,
		std::vector<double> &swarm6_consump,
		);

	void DecisionMaker(std::vector<double> swarm2_consump_vec,
		std::vector<double> swarm3_consump_vec,
		std::vector<double> swarm4_consump_vec,
		std::vector<double> swarm5_consump_vec,
		std::vector<double> swarm6_consump_vec,
		std::vector<int> &vec_of_decision);
		
};



#endif
