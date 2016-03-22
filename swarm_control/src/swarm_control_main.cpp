#include "swarm_control.cpp"
#include <iostream>

using namespace std;

int main(int argc, char **argv) {
  ros::init(argc, argv, "des_state_publisher");
  ros::NodeHandle nh;
  //instantiate a desired-state publisher object
  SwarmControl swarm_control(nh);
  

  ros::Rate looprate(1 / dt); //timer for fixed publication rate
  //put some points in the path queue--hard coded here

  swarm_control.set_des_pose(20.0, 20.0, 0.0);
/*
need to know obst_positions
*/
geometry_msgs::PoseStamped temp;
temp.pose.position.x = 0;
temp.pose.position.y = 0;

  std::vector<geometry_msgs::PoseStamped> obst_posi;
  obst_posi.push_back(temp);

  std::vector<geometry_msgs::PoseStamped> swarm2_obst;
  std::vector<geometry_msgs::PoseStamped> swarm3_obst;
  std::vector<geometry_msgs::PoseStamped> swarm4_obst;
  std::vector<geometry_msgs::PoseStamped> swarm5_obst;
  std::vector<geometry_msgs::PoseStamped> swarm6_obst;

  swarm2_obst.push_back(temp);
  swarm3_obst.push_back(temp);
  swarm4_obst.push_back(temp);
  swarm5_obst.push_back(temp);
  swarm6_obst.push_back(temp);

  swarm_control.ComputeConsumption(
    swarm2_obst,
    swarm3_obst,
    swarm4_obst,
    swarm5_obst,
    swarm6_obst);    // give consumptiion to decisionmaker 

  swarm_control.DecisionMaker();   //give corresponding targets

  ////swarm_1
  swarm_control.swarm_obstacles_state(obst_posi, swarm_control.current_swarm1_pose, swarm_control.des_pose_1,   ////PSO compute pose vector
    swarm_control.des_state_vec_1); //des_state_vec_1 

  swarm_control.swarm_obstacles_state(obst_posi, swarm_control.current_swarm2_pose, swarm_control.des_pose_2,
    swarm_control.des_state_vec_2); //des_state_vec_2 

  swarm_control.swarm_obstacles_state(obst_posi, swarm_control.current_swarm3_pose, swarm_control.des_pose_3,
    swarm_control.des_state_vec_3); //des_state_vec_3   
  
  swarm_control.swarm_obstacles_state(obst_posi, swarm_control.current_swarm4_pose, swarm_control.des_pose_4,
    swarm_control.des_state_vec_4); //des_state_vec_4   

  swarm_control.swarm_obstacles_state(obst_posi, swarm_control.current_swarm5_pose, swarm_control.des_pose_5,
    swarm_control.des_state_vec_5); //des_state_vec_5

  swarm_control.swarm_obstacles_state(obst_posi, swarm_control.current_swarm6_pose, swarm_control.des_pose_6,
    swarm_control.des_state_vec_6); //des_state_vec_6  
  // main loop; publish a desired state every iteration

  while (ros::ok()) {   
    ros::spinOnce();
    looprate.sleep(); //sleep for defined sample period, then do loop again
  }
  return 0;
}

