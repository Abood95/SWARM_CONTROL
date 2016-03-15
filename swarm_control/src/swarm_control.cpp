#include "swarm_control.h"
#include<time.h>
#include<math.h>
#include<cstdlib>

//ExampleRosClass::ExampleRosClass(ros::NodeHandle* nodehandle):nh_(*nodehandle)

SwarmControl::SwarmControl(ros::NodeHandle& nh) :
		nh_(nh) {
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
	desired_state_publisher_1 = nh_.advertise < nav_msgs::Odometry
			> ("robot1/desState", 1, true);
	des_psi_publisher_1 = nh_.advertise < std_msgs::Float64
			> ("robot1/desPsi", 1);

	desired_state_publisher_2 = nh_.advertise < nav_msgs::Odometry
			> ("robot2/desState", 1, true);
	des_psi_publisher_2 = nh_.advertise < std_msgs::Float64
			> ("robot2/desPsi", 1);

	desired_state_publisher_3 = nh_.advertise < nav_msgs::Odometry
			> ("robot3/desState", 1, true);
	des_psi_publisher_3 = nh_.advertise < std_msgs::Float64
			> ("robot3/desPsi", 1);

	desired_state_publisher_4 = nh_.advertise < nav_msgs::Odometry
			> ("robot4/desState", 1, true);
	des_psi_publisher_4 = nh_.advertise < std_msgs::Float64
			> ("robot4/desPsi", 1);

	desired_state_publisher_5 = nh_.advertise < nav_msgs::Odometry
			> ("robot5/desState", 1, true);
	des_psi_publisher_5 = nh_.advertise < std_msgs::Float64
			> ("robot5/desPsi", 1);

	desired_state_publisher_6 = nh_.advertise < nav_msgs::Odometry
			> ("robot6/desState", 1, true);
	des_psi_publisher_6 = nh_.advertise < std_msgs::Float64
			> ("robot6/desPsi", 1);
}

void SwarmControl::set_init_pose(double x, double y, double psi) {
	current_pose_1 = trajBuilder_.xyPsi2PoseStamped(x, y, psi);
	current_pose_2 = trajBuilder_.xyPsi2PoseStamped(x - 1, y, psi);
}

void SwarmControl::swarm_1_obstacles_state(double dist_obst,
		geometry_msgs::PoseStamped robot_pose,
		std::vector<geometry_msgs::PoseStamped> &vec_of_states) {

	int Num = 200;
	int c1 = 1.1;
	int c2 = 1.1;
	int Miter = 50;
	double w = 0.7298;

	double obst_radius = 2;
	double safe_dist = 10;    // still need be to considered
	double y_range = 5;

	//initialization
	double x_start = position.pose.position.x;
	double y_start = position.pose.position.y;

	double psi = trajBuilder_.convertPlanarQuat2Psi(
			robot_pose.pose.orientation);
	//compute obstacle position
	geometry_msgs::PoseStamped obst;
	double obst_x = x_start + dist_obst * cos(psi);
	double obst_y = y_start + dist_obst * sin(psi);
	obst.pose.position.x = obst_x;
	obst.pose.position.y = obst_y;
	//compute target position
	geometry_msgs::PoseStamped target;
	double target_x = obst_x + safe_dist * cos(psi);
	double target_y = obst_y + safe_dist * sin(psi);
	target.pose.position.x = target_x;
	target.pose.position.y = target_y;

	 srand((unsigned)time( NULL ));

	std::vector<geometry_msgs::PoseStamped> particle_pose;
	std::vector<geometry_msgs::PoseStamped> velocity;
	std::vector<geometry_msgs::PoseStamped> local_pose;
	geometry_msgs::PoseStamped temp;
	geometry_msgs::PoseStamped vel;

	std::vector<double> local_best;

	double valid_dist = 0.0;

	double dx;
	double dy;
	double temp_evalu;

	int N=999;

	for (int i = 0; i < Num; i++) {
		//careful
		temp.pose.position.x = x_start
				+ (obst_x - x_start) * (rand()%(N+1)/(float)(N+1)) * cos(psi);
		temp.pose.position.y = y_start - y_range * cos(psi)
				+ (y_start + y_range * cos(psi)) * (rand()%(N+1)/(float)(N+1));

		vel.pose.position.x = x_start + (obst_x - x_start) * (rand()%(N+1)/(float)(N+1)) * cos(psi);
		vel.pose.position.y = y_start - y_range * cos(psi)
				+ (y_start + y_range * cos(psi)) * (rand()%(N+1)/(float)(N+1));
		//screen
		dx = temp.pose.position.x - obst_x;
		dy = temp.pose.position.y - obst_y;
		valid_dist = sqrt(dx * dx + dy * dy);

		temp_evalu = trajBuilder_.ComputeEvaluation(target, obst, temp);

		if (valid_dist > obst_radius) {
			particle_pose.push_back(temp);
			local_pose.push(temp); //temprary local best
			local_best.push_back(temp_evalu);
			velocity.pose.push_back(vel);
		}

	}

	int valid_num = particle_pose.size();

	geometry_msgs::PoseStamped global_best;

	global_best.pose.position.x = particle_pose[0].pose.position.x;
	global_best.pose.position.y = particle_pose[0].pose.position.y;

	double best_value;
	for (int i = 1; i < valid_num; i++) {
		best_value = trajBuilder_.ComputeEvaluation(target, obst, global_best);
		if (local_best[i] > best_value)
			global_best.pose.position.x = particle_pose[i].pose.position.x; ////////////
		global_best.pose.position.y = particle_pose[i].pose.position.y; ////////////
	}

	////main iteration
	vec_of_states.clear();
    int counter = 0;
	for (int iter = 1; iter < Miter; iter++) {
		for (int n = 0; n < valid_num; n++) {
			velocity[n].pose.position.x = w * velocity[n].pose.position.x
					+ c1 * (rand()%(N+1)/(float)(N+1))
							* (local_pose[n].pose.position.x
									- particle_pose[n].pose.position.x)
					+ c2 * (rand()%(N+1)/(float)(N+1))
							* (global_best.pose.position.x
									- particle_pose[n].pose.position.x);
			velocity[n].pose.position.y = w * velocity[n].pose.position.y
					+ c1 * (rand()%(N+1)/(float)(N+1))
							* (local_pose[n].pose.position.y
									- particle_pose[n].pose.position.y)
					+ c2 * (rand()%(N+1)/(float)(N+1))
							* (global_best.pose.position.y
									- particle_pose[n].pose.position.y);
			particle_pose[n].pose.position.x = particle_pose[n].pose.position.x
					+ velocity[n].pose.position.x;
			particle_pose[n].pose.position.y = particle_pose[n].pose.position.y
					+ velocity[n].pose.position.y;

			temp_evalu = trajBuilder_.ComputeEvaluation(target, obst,
					particle_pose[n]);
			if (local_best[n] < temp_evalu) { /// update local best
				local_best[n] = temp_evalu;
				local_pose[n].pose.position.x =
						particle_pose[n].pose.position.x;
				local_pose[n].pose.position.y =
						particle_pose[n].pose.position.y;
			}

			best_value = trajBuilder_.ComputeEvaluation(target, obst,
					global_best);
			if (local_best[n] > best_value) {
				global_best.pose.position.x = local_pose[n].pose.position.x;
				global_best.pose.position.y = local_pose[n].pose.position.y;
			}
		}

        counter +=1;
        if(counter > 10){
		   vec_of_states.pushback(global_best);
		   counter =0;
        }
	}
	vec_of_states.pushback(target.pose);
}
