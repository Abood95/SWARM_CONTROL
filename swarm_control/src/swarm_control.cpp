#include "swarm_control.h"
#include<time.h>
#include<cstdlib>

using namespace std;
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

	range_1 = dange_range_1;
	range_2 = dange_range_2;

	//define a halt state; zero speed and spin, and fill with viable coords
	halt_twist_.linear.x = 0.0;
	halt_twist_.linear.y = 0.0;
	halt_twist_.linear.z = 0.0;
	halt_twist_.angular.x = 0.0;
	halt_twist_.angular.y = 0.0;
	halt_twist_.angular.z = 0.0;

	//initialization
	current_pose_1 = trajBuilder_.xyPsi2PoseStamped(0, 0, 0);
	start_pose_1 = current_pose_1;
	end_pose_1 = current_pose_1;
	current_des_state_1.twist.twist = halt_twist_;
	current_des_state_1.pose.pose = current_pose_1.pose;
	halt_state_1 = current_des_state_1;

	current_pose_2 = trajBuilder_.xyPsi2PoseStamped(1, 2, 0);
	start_pose_2 = current_pose_2;
	end_pose_2 = current_pose_2;
	current_des_state_2.twist.twist = halt_twist_;
	current_des_state_2.pose.pose = current_pose_2.pose;
	halt_state_2 = current_des_state_2;

	current_pose_3 = trajBuilder_.xyPsi2PoseStamped(2, 2, 0);
	start_pose_3 = current_pose_3;
	end_pose_3 = current_pose_3;
	current_des_state_3.twist.twist = halt_twist_;
	current_des_state_3.pose.pose = current_pose_3.pose;
	halt_state_3 = current_des_state_3;

	current_pose_4 = trajBuilder_.xyPsi2PoseStamped(1, 5, 0);
	start_pose_4 = current_pose_4;
	end_pose_4 = current_pose_4;
	current_des_state_4.twist.twist = halt_twist_;
	current_des_state_4.pose.pose = current_pose_4.pose;
	halt_state_4 = current_des_state_4;

	current_pose_5 = trajBuilder_.xyPsi2PoseStamped(0,3,0);
	start_pose_5 = current_pose_5;
	end_pose_5 = current_pose_5;
	current_des_state_5.twist.twist = halt_twist_;
	current_des_state_5.pose.pose = current_pose_5.pose;
	halt_state_5 = current_des_state_5;

	current_pose_6 = trajBuilder_.xyPsi2PoseStamped(3, 7, 0);
	start_pose_6 = current_pose_6;
	end_pose_6 = current_pose_6;
	current_des_state_6.twist.twist = halt_twist_;
	current_des_state_6.pose.pose = current_pose_6.pose;
	halt_state_6 = current_des_state_6;

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

void SwarmControl::set_des_pose(double x, double y, double psi) {
	des_pose_1 = trajBuilder_.xyPsi2PoseStamped(x, y, psi);
	trajBuilder_.ComputeSubpositions(geometry_msgs::PoseStamped des_pose_1,
			geometry_msgs::PoseStamped &swarm2_posi,
			geometry_msgs::PoseStamped &swarm3_posi,
			geometry_msgs::PoseStamped &swarm4_posi,
			geometry_msgs::PoseStamped &swarm5_posi,
			geometry_msgs::PoseStamped &swarm6_posi,
	);
	des_pose_2 = swarm2_posi;
	des_pose_3 = swarm3_posi;
	des_pose_4 = swarm4_posi;
	des_pose_5 = swarm5_posi;
	des_pose_6 = swarm6_posi;
}

void swarm_obstacles_state(std::vector<geometry_msgs::PoseStamped> obst_posi,
		geometry_msgs::PoseStamped robot_pose,
		geometry_msgs::PoseStamped target_posi,
		std::vector<nav_msgs::Odometry> &vec_of_states) {

	int Num = 150;
	double c1 = 0.5;
	double c2 = 0.5;
	int Miter = 20;
	double w = 0.75;

	//initialization
	double x_start = robot_pose.pose.position.x;
	double y_start = robot_pose.pose.position.y;
    //probabily no use
	//double psi = trajBuilder_.convertPlanarQuat2Psi(
    //robot_pose.pose.orientation);

	srand((unsigned) time( NULL));

	std::vector<geometry_msgs::PoseStamped> particle_pose;
	std::vector<geometry_msgs::PoseStamped> velocity;
	std::vector<geometry_msgs::PoseStamped> local_pose;
	geometry_msgs::PoseStamped temp;
	geometry_msgs::PoseStamped vel;

	std::vector<double> local_best;

	double dx;
	double dy;
	double dist;
	double temp_evalu;

	int N = 999;
	
	for (int i = 0;i < Num; i++){
		temp.pose.position.x = x_start;
		temp.pose.position.y = y_start;

        vel.pose.position.x = rand() % (N + 1) / (float) (N + 1);
        vel.pose.position.y = rand() % (N + 1) / (float) (N + 1);

        //then everything should be same
		particle_pose.push_back(temp);
		local_pose.push_back(temp); //temprary local best


		temp_evalu = trajBuilder_.ComputeEvaluation(target_posi, obst_posi,
				temp);
		local_best.push_back(temp_evalu);
		velocity.push_back(vel);

	}

	int valid_num = particle_pose.size();

	geometry_msgs::PoseStamped global_best;

	global_best.pose.position.x = particle_pose[0].pose.position.x;
	global_best.pose.position.y = particle_pose[0].pose.position.y;

    //initialize global best
	double best_value;
	for (int i = 1; i < valid_num; i++) {
		best_value = trajBuilder_.ComputeEvaluation(target_posi, obst_posi,
				global_best);
		if (local_best[i] < best_value)
		global_best.pose.position.x = particle_pose[i].pose.position.x; ////////////
		global_best.pose.position.y = particle_pose[i].pose.position.y; ////////////
	}

	////main iteration
	vec_of_states.clear();
	for (int iter = 1; iter < Miter; iter++) {
		for (int n = 0; n < valid_num; n++) {
			velocity[n].pose.position.x = w * velocity[n].pose.position.x
					+ c1 * (rand() % (N + 1) / (float) (N + 1)) //need to be refinded
							* (local_pose[n].pose.position.x
									- particle_pose[n].pose.position.x)
					+ c2 * (rand() % (N + 1) / (float) (N + 1)) //need to be refinded
							* (global_best.pose.position.x
									- particle_pose[n].pose.position.x);
			velocity[n].pose.position.y = w * velocity[n].pose.position.y
					+ c1 * (rand() % (N + 1) / (float) (N + 1))
							* (local_pose[n].pose.position.y
									- particle_pose[n].pose.position.y)
					+ c2 * (rand() % (N + 1) / (float) (N + 1))
							* (global_best.pose.position.y
									- particle_pose[n].pose.position.y);
			particle_pose[n].pose.position.x = particle_pose[n].pose.position.x
					+ velocity[n].pose.position.x;
			particle_pose[n].pose.position.y = particle_pose[n].pose.position.y
					+ velocity[n].pose.position.y;

			temp_evalu = trajBuilder_.ComputeEvaluation(target_posi, obst_posi,
					particle_pose[n]);
			if (local_best[n] > temp_evalu) { /// update local best
				local_best[n] = temp_evalu;
				local_pose[n].pose.position.x =
						particle_pose[n].pose.position.x;
				local_pose[n].pose.position.y =
						particle_pose[n].pose.position.y;
			}

			best_value = trajBuilder_.ComputeEvaluation(target_posi, obst_posi,
					global_best);
			if (best_value > local_best[n]) {
				global_best.pose.position.x = local_pose[n].pose.position.x;
				global_best.pose.position.y = local_pose[n].pose.position.y;
			}
		}
          dx = global_best.pose.position.x - target_posi.pose.position.x;
          dy = global_best.pose.position.y - target_posi.pose.position.y;
          dist = sqrt(dx *dx + dy*dy);
		  if( dist > 1){vec_of_states.pose.push_back(global_best);}  ///careful if it's not pose
	}	
	vec_of_states.pose.push_back(target_posi);
}

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
		) {
	//get all target position not acoordingly
trajBuilder_.ComputeSubpositions(geometry_msgs::PoseStamped swarm1_pose,
		geometry_msgs::PoseStamped &swarm2_posi,
		geometry_msgs::PoseStamped &swarm3_posi,
		geometry_msgs::PoseStamped &swarm4_posi,
		geometry_msgs::PoseStamped &swarm5_posi,
		geometry_msgs::PoseStamped &swarm6_posi,
);
//swarm2 ,notice in order
double consump = SingleConsumpt(swarm2_posi, current_swarm2_pose,swarm2_obst);
swarm2_consump.push_back(consump);
consump = SingleConsumpt(swarm3_posi, current_swarm2_pose,swarm2_obst);
swarm2_consump.push_back(consump);
consump = SingleConsumpt(swarm4_posi, current_swarm2_pose,swarm2_obst);
swarm2_consump.push_back(consump);
consump = SingleConsumpt(swarm5_posi, current_swarm2_pose,swarm2_obst);
swarm2_consump.push_back(consump);
consump = SingleConsumpt(swarm6_posi, current_swarm2_pose,swarm2_obst);
swarm2_consump.push_back(consump);

//swarm3, notice in order
consump = SingleConsumpt(swarm2_posi, current_swarm3_pose,swarm3_obst);
swarm3_consump.push_back(consump);
consump = SingleConsumpt(swarm3_posi, current_swarm3_pose,swarm3_obst);
swarm3_consump.push_back(consump);
consump = SingleConsumpt(swarm4_posi, current_swarm3_pose,swarm3_obst);
swarm3_consump.push_back(consump);
consump = SingleConsumpt(swarm5_posi, current_swarm3_pose,swarm3_obst);
swarm3_consump.push_back(consump);
consump = SingleConsumpt(swarm6_posi, current_swarm3_pose,swarm3_obst);
swarm3_consump.push_back(consump);

//swarm4, notice in order
consump = SingleConsumpt(swarm2_posi, current_swarm4_pose,swarm4_obst);
swarm4_consump.push_back(consump);
consump = SingleConsumpt(swarm3_posi, current_swarm4_pose,swarm4_obst);
swarm4_consump.push_back(consump);
consump = SingleConsumpt(swarm4_posi, current_swarm4_pose,swarm4_obst);
swarm4_consump.push_back(consump);
consump = SingleConsumpt(swarm5_posi, current_swarm4_pose,swarm4_obst);
swarm4_consump.push_back(consump);
consump = SingleConsumpt(swarm6_posi, current_swarm4_pose,swarm4_obst);
swarm4_consump.push_back(consump);

//swarm5, notice in order
consump = SingleConsumpt(swarm2_posi, current_swarm5_pose,swarm5_obst);
swarm5_consump.push_back(consump);
consump = SingleConsumpt(swarm3_posi, current_swarm5_pose,swarm5_obst);
swarm5_consump.push_back(consump);
consump = SingleConsumpt(swarm4_posi, current_swarm5_pose,swarm5_obst);
swarm5_consump.push_back(consump);
consump = SingleConsumpt(swarm5_posi, current_swarm5_pose,swarm5_obst);
swarm5_consump.push_back(consump);
consump = SingleConsumpt(swarm6_posi, current_swarm5_pose,swarm5_obst);
swarm5_consump.push_back(consump);

//swarm6, notice in order
consump = SingleConsumpt(swarm2_posi, current_swarm6_pose,swarm6_obst);
swarm6_consump.push_back(consump);
consump = SingleConsumpt(swarm3_posi, current_swarm6_pose,swarm6_obst);
swarm6_consump.push_back(consump);
consump = SingleConsumpt(swarm4_posi, current_swarm6_pose,swarm6_obst);
swarm6_consump.push_back(consump);
consump = SingleConsumpt(swarm5_posi, current_swarm6_pose,swarm6_obst);
swarm6_consump.push_back(consump);
consump = SingleConsumpt(swarm6_posi, current_swarm6_pose,swarm6_obst);
swarm6_consump.push_back(consump);
}

double SingleConsumpt(geometry_msgs::PoseStamped target,
	geometry_msgs::PoseStamped robot,
	std::vector<geometry_msgs::PoseStamped> obstacle) {
double target_dx = target.pose.position.x - robot.pose.position.x;
double target_dy = target.pose.position.y - robot.pose.position.y;
double dist = sqrt(target_dx * target_dx + target_dy * target_dy);

double obst_dx;
double obst_dy;
double obst_dist;

double dot_product;
double mod;
double dang_angle;
///ROS_INFO
double add_consumpt;
double total_consumpt;
int num = obstacle.size();

for (int i = 0; i < num; i++) {
	obst_dx = obstacle[i].pose.position.x - robot.pose.position.x;
	obst_dy = obstacle[i].pose.position.y - robot.pose.position.y;
	obst_dist = sqrt(obst_dx * obst_dx + obst_dy * obst_dy);

	dot_product = target_dx * obst_dx + target_dy * obst_dy;
	mod = dist * obst_dist;
	dang_angle = dot_product / mod;

	if (fabs(dang_angle) <= range_1) {  ///symetric for both positive and negative
		add_consumpt = exp(-fabs(dang_angle)); //¾«¶È  or a Gain
	} else {
		add_consumpt = 0;
	}
	total_consumpt += add_consumpt;
}
return dist + total_consumpt + 1; // for convenience of proba
}

void DecisionMaker(std::vector<double> swarm2_consump_vec,
		std::vector<double> swarm3_consump_vec,
		std::vector<double> swarm4_consump_vec,
		std::vector<double> swarm5_consump_vec,
		std::vector<double> swarm6_consump_vec,
		std::vector<int> &vec_of_decision){

//normalize and change monoton
//	double delta = max - min;
	num2 = swarm2_consump_vec.size();
	for(int i = 0; i < num2; i++){
//		swarm2_consump_vec[i] = (swarm2_consump_vec[i] - min)/delta; //normalized
		swarm2_consump_vec[i] = 1/swarm2_consump_vec[i]; //reverse monotonicity
	}

	num3 = swarm3_consump_vec.size();
	for(int i = 0; i < num3; i++){
//		swarm3_consump_vec[i] = (swarm3_consump_vec[i] - min)/delta;
		swarm3_consump_vec[i] = 1/swarm3_consump_vec[i]; //reverse monotonicity
	}

	num4 = swarm4_consump_vec.size();
	for(int i = 0; i < num4; i++){
//		swarm4_consump_vec[i] = (swarm4_consump_vec[i] - min)/delta;
		swarm4_consump_vec[i] = 1/swarm4_consump_vec[i]; //reverse monotonicity		
	}

	num5 = swarm5_consump_vec.size();
	for(int i = 0; i < num5; i++){
//		swarm5_consump_vec[i] = (swarm5_consump_vec[i] - min)/delta;
		swarm5_consump_vec[i] = 1/swarm5_consump_vec[i]; //reverse monotonicity		
	}	

	num6 = swarm6_consump_vec.size();
	for(int i = 0; i < num6; i++){
//		swarm6_consump_vec[i] = (swarm6_consump_vec[i] - min)/delta;
		swarm6_consump_vec[i] = 1/swarm6_consump_vec[i]; //reverse monotonicity		
	}	
 

    double norm;
    double proby_2;
    double proby_3;
    double proby_4;
    double proby_5;
    double proby_6;
    double Entropy;
    double BestEntropy = 1000;
    std::vector<double> vec_of_proby;
    vec_of_proby.push_back(0.0); //as clear may let core dumpt
    vec_of_decision.push_back(0);

    for(int i_2 = 0; i_2 < num2; i_2++){
    	for(int i_3 = 0; i_3 < num3; i_3++){
            if(i_2 != i_3){
                for(int i_4 = 0; i_4 < num4; i_4++){
                    if(i_4 != i_3 && i_4 != i_2){
                        for(int i_5 = 0; i_5 < num5; i_5++){
                            if(i_5 != i_4 && i_5 != i_3 && i_5 != i_2){
                               for(int i_6 = 0; i_6 < num6; i_6++){
                               	  if(i_6 != i_5 && i_6 != i_4 && i_6 != i_3 && i_6 != i_2){
                               	  	//normalize
	                                     norm = sqrt(swarm2_consump_vec[i_2]*swarm2_consump_vec[i_2] + 
	                                     swarm3_consump_vec[i_3]*swarm3_consump_vec[i_3] + 
	                                     swarm4_consump_vec[i_4]*swarm4_consump_vec[i_4] + 
	                                     swarm5_consump_vec[i_5]*swarm5_consump_vec[i_5] + 
	                                     swarm6_consump_vec[i_6]*swarm6_consump_vec[i_6]);
	                                     proby_2 = swarm2_consump_vec[i_2]/norm;
	                                     proby_3 = swarm3_consump_vec[i_3]/norm;
	                                     proby_4 = swarm4_consump_vec[i_4]/norm;
	                                     proby_5 = swarm5_consump_vec[i_5]/norm;
	                                     proby_6 = swarm6_consump_vec[i_5]/norm;
                                        Entropy = -(proby_2 * log(proby_2) + proby_3 * log(proby_3) + proby_3 * log(proby_3)
                                         + proby_4 * log(proby_4) + proby_5 * log(proby_5) + proby_6 * log(proby_6));
                                        if(Entropy < BestEntropy){   // if not minimal, replace
                                        	BestEntropy = Entropy;
                                        	vec_of_proby.clear();
                                        	vec_of_proby.push_back(swarm2_consump_vec[i_2]);
                                        	vec_of_proby.push_back(swarm3_consump_vec[i_3]);
                                        	vec_of_proby.push_back(swarm4_consump_vec[i_4]);
                                        	vec_of_proby.push_back(swarm5_consump_vec[i_5]);
                                        	vec_of_proby.push_back(swarm6_consump_vec[i_6]);

                                        	vec_of_decision.clear();
                                        	vec_of_decision.push_back(i_2 + 2);  ///index is from 0, but our target position is from 2
                                        	vec_of_decision.push_back(i_3 + 2);
                                        	vec_of_decision.push_back(i_4 + 2);
                                        	vec_of_decision.push_back(i_5 + 2);
                                        	vec_of_decision.push_back(i_6 + 2);
                                        	
                                        }
                               	  }
                               }
                            }
                        }
                    }
                }
            }
       }
    }

void get_target_pose(std::vector<int> vec_of_decision,
	std::vector<geometry_msgs::PoseStamped> vec_of_targets,
	geometry_msgs::PoseStamped &des_pose_2,
	geometry_msgs::PoseStamped &des_pose_3,
	geometry_msgs::PoseStamped &des_pose_4,
	geometry_msgs::PoseStamped &des_pose_5,
	geometry_msgs::PoseStamped &des_pose_6){
    int num = vec_of_decision.size();
    int num_target = vec_of_targets.size();
    for (int i = 0; i < num; i++){
       for (int n = 0; n < num_target; n++){
       	   
       }
    }

}
}