#include "mpc_pr/move.h"
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include "std_msgs/Int32.h"

using namespace std;

ofstream myfile;
int rti_num = 20;

MPC_solver myMpcSolver(rti_num);

// Introduce class to make safer goal change
class GoalFollower 
{ 
    // Access specifier 
    public: 
    // Data Members 
    ros::Publisher chatter_pub;
    ros::Publisher state_pub;
    double goal[2] = {3.14, 0.0000};
    double comand_vel[2] = {0.0000, 0.0000};
    double joint_position[2] = {0.0000, 0.0000};
    double joint_speed[2] = {0.0000, 0.0000};

    void change_states_msg(const std_msgs::Float64MultiArray msg) 
    { 
       for (int i=0; i<2; i++) {
           joint_position[i] = msg.data[i];
           joint_speed[i] = msg.data[i+2];
       }
    }
    
    void SendVelocity(const std_msgs::Float64MultiArray joint_vel_values){
    	chatter_pub.publish(joint_vel_values);
	    return;
    }
    void SendState(const std_msgs::Float64 msg){
    	state_pub.publish(msg);
	    return;
    } 
}; 

int main(int argc, char **argv)
{
  // myfile.open("data_planar.csv", ios::out); 
  ros::init(argc, argv, "pr_controller");
  ros::NodeHandle n;
  ROS_INFO("Node Started");

  GoalFollower my_follower;
  my_follower.chatter_pub = n.advertise<std_msgs::Float64MultiArray>("/MPC_solutions", 1);
  my_follower.state_pub = n.advertise<std_msgs::Float64>("/state_flag", 1);

  ros::Subscriber joint_status = n.subscribe("/pr/joint_states", 1, &GoalFollower::change_states_msg, &my_follower);

  double smallest_dist;
  double sphere_1_dist;
  double sphere_2_dist;
  int queue = 0;
  double min_dist[] = {0.5, 0.5, 0.5, 0.5};
  
  double init_positions[2] = {0.0, 0.0};
  double l1 = 0.5;
  double l2 = 0.4;
  
  ros::Rate loop_rate(20);

  while (ros::ok())
  {
    // MPC input:
    double currentState_targetValue[4];

    // Check if arrived
    float max_diff = 0;
    float temp = 0;
    for (int i = 0; i < 2; i++) {
        temp = abs(my_follower.goal[i] - my_follower.joint_position[i]);
        if (temp > max_diff) max_diff = temp; 
    }
    std_msgs::Float64 state_value;

    if (max_diff<0.05) {
        printf("Arrived\n");
        state_value.data = 0;
        my_follower.SendState(state_value);
    }
    else{
        state_value.data = 1;
        my_follower.SendState(state_value);
    }
    
          // Goal reference position 
    for (int i = 0; i < 2; i++){
        currentState_targetValue[i] = my_follower.joint_position[i];
        currentState_targetValue[i+2] = my_follower.goal[i];
    }
    
    double* solutions = myMpcSolver.solve_mpc(currentState_targetValue);

    // prepare to send commands
    std_msgs::Float64MultiArray joint_vel_values;
    joint_vel_values.data.clear();
    for (int i = 0; i < 2; i++) joint_vel_values.data.push_back(solutions[i]);
    printf("Solutions = %f, %f\n", solutions[0], solutions[1]);
    my_follower.SendVelocity(joint_vel_values);
    ros::spinOnce();
    loop_rate.sleep();
  }

  myfile.close();
  return 0;
}

