#include "swarm_control.h"
#include <iostream>
#include <swarm_control/obstacles.h>
using namespace std;

int main(int argc, char **argv) {
  ros::init(argc, argv, "des_state_publisher");
  ros::NodeHandle nh;
  //instantiate a desired-state publisher object
  SwarmControl SwarmControl(nh);
  
  //alarm_p1 = &(SwarmControl.alarm1);
  //maybe try give position vectors back directly
  //ros::ServiceServer service1 = n.advertiseService("obsts1", callback1);
  //ros::ServiceServer service2 = n.advertiseService("obsts2", callback2);
  //ros::ServiceServer service3 = n.advertiseService("obsts3", callback3);
  //ros::ServiceServer service4 = n.advertiseService("obsts4", callback4);
  //ros::ServiceServer service5 = n.advertiseService("obsts5", callback5);
  //ros::ServiceServer service6 = n.advertiseService("obsts6", callback6);
 
  ros::Rate looprate(1 / dt); //timer for fixed publication rate
  //put some points in the path queue--hard coded here

  SwarmControl.set_des_pose(double x, double y, double psi);
/*
need to know obst_positions
*/
  SwarmControl.ComputeConsumption(SwarmControl.des_pose_1,
    SwarmControl.current_pose_2,
    SwarmControl.current_pose_3,
    SwarmControl.current_pose_4,
    SwarmControl.current_pose_5,
    SwarmControl.current_pose_6,
    obst_posi,
    obst_posi,
    obst_posi,
    obst_posi,
    obst_posi,
    swarm2_consump,
    swarm3_consump,
    swarm4_consump,
    swarm5_consump,
    swarm6_consump,
    ); 
  SwarmControl.DecisionMaker(SwarmControl.swarm2_consump,
    SwarmControl.swarm3_consump,
    SwarmControl.swarm4_consump,
    SwarmControl.swarm5_consump,
    SwarmControl.swarm6_consump,
    SwarmControl.decision_assign_vector);

  

  ////swarm_1
  SwarmControl.swarm_obstacles_state(obst_posi, SwarmControl.current_pose_1, SwarmControl.des_pose_1,
    SwarmControl.des_state_vec_1); //des_state_vec_1 

  SwarmControl.swarm_obstacles_state(obst_posi, SwarmControl.current_pose_2, SwarmControl.des_pose_2,
    SwarmControl.des_state_vec_2); //des_state_vec_2 

  SwarmControl.swarm_obstacles_state(obst_posi, SwarmControl.current_pose_3, SwarmControl.des_pose_3,
    SwarmControl.des_state_vec_3); //des_state_vec_3   
  
  SwarmControl.swarm_obstacles_state(obst_posi, SwarmControl.current_pose_4, SwarmControl.des_pose_4,
    SwarmControl.des_state_vec_4); //des_state_vec_4   

  SwarmControl.swarm_obstacles_state(obst_posi, SwarmControl.current_pose_5, SwarmControl.des_pose_5,
    SwarmControl.des_state_vec_5); //des_state_vec_5

  SwarmControl.swarm_obstacles_state(obst_posi, SwarmControl.current_pose_6, SwarmControl.des_pose_6,
    SwarmControl.des_state_vec_6); //des_state_vec_6  
  // main loop; publish a desired state every iteration

  while (ros::ok()) {
	  
  
    
    ros::spinOnce();
    looprate.sleep(); //sleep for defined sample period, then do loop again
  }
  return 0;
}

