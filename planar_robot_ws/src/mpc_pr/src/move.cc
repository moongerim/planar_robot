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

    double robot_spheres[4] = {0.05, 0.05, 0.05, 0.05};
    double obst_spheres[6] = {-0.6, 0.7, 0.2,
                               0.6, 0.7, 0.2};

    double goal[2] = {0.0000, 0.0000};

    double comand_vel[2] = {0.0000, 0.0000};
    double joint_position[2] = {0.0000, 0.0000};
    double joint_speed[2] = {0.0000, 0.0000};
    double dist[4] = {10.0000,10.0000,10.0000,10.000};
    void change_states_msg(const std_msgs::Float64MultiArray msg) 
    { 
       for (int i=0; i<2; i++) joint_position[i] = msg.data[i];
       for (int i=0; i<2; i++) joint_speed[i] = msg.data[i+2];
    }

    void SendVelocity(const std_msgs::Float64MultiArray joint_vel_values){
    	chatter_pub.publish(joint_vel_values);
	return;
    }

    void change_distances(const std_msgs::Float64MultiArray msg){
    	for (int i=0; i<4; i++) dist[i] = msg.data[i];
    }
}; 

int main(int argc, char **argv)
{
  myfile.open("data_planar.csv", ios::out); 
   
  ros::init(argc, argv, "pr_controller");

  ros::NodeHandle n;
  ROS_INFO("Node Started");
  //--------------------------------
  GoalFollower my_follower;
  my_follower.chatter_pub = n.advertise<std_msgs::Float64MultiArray>("/pr/command", 1);

  ROS_INFO("Goal default to: %.3f, %.3ff", my_follower.goal[0], my_follower.goal[1]);

  //--------------------------------

  ros::Subscriber joint_status = n.subscribe("/pr/joint_states_low", 1, &GoalFollower::change_states_msg, &my_follower);
  ros::Subscriber distances = n.subscribe("/CoppeliaSim/distances", 1, &GoalFollower::change_distances, &my_follower);

  double smallest_dist;
  double local_val;
  double min_dist[] = {10000, 10000, 10000, 10000};
  double goal_queue[] = {-3.14, 0.000};
  ros::Rate loop_rate(20);


  while (ros::ok())
  {
    double currentState_targetValue[4];

    // define terminal goal
    for (int i=0; i<2; i++) my_follower.goal[i] = goal_queue[i];

    // Check if arrived
    float max_diff = 0;
    float temp = 0;
    for (int i = 0; i < 2; ++i) {
        temp = abs(my_follower.goal[i] - my_follower.joint_position[i]);
        if (temp > max_diff) {
            max_diff = temp; 
        }
        if (max_diff<0.05){
            printf("Arrived\n");
        }
    }

    //******************* get_min_dist **********************
    local_val = 10000;
    smallest_dist = 10000;
    min_dist[0] = 10000; min_dist[1] = 10000; min_dist[2] = 10000; min_dist[3] = 10000;
    double spheres_dist[] = {0.0, 0.0, 0.0, 0.0};
    
	for (int j = 0; j < 4; j++) {
        local_val = my_follower.dist[j];
	    if (local_val < min_dist[j]){
            min_dist[j] = local_val;
        }
	if (smallest_dist > min_dist[j]) smallest_dist = min_dist[j];
	}
    double* solutions;
    if (smallest_dist >= 0.00) {
          // Goal reference position 
          for (int i = 0; i < 2; ++i) currentState_targetValue[i] = my_follower.joint_position[i];
          for (int i = 0; i < 2; ++i) currentState_targetValue[i+2] = my_follower.goal[i];

          solutions = myMpcSolver.solve_mpc(currentState_targetValue);
    } 
    else{
        for (int i=0; i<5; i++){
            solutions[i] = 0;
        } 
    }
    
    // prepare to send commands
    std_msgs::Float64MultiArray joint_vel_values;
    joint_vel_values.data.clear();
    for (int i = 0; i < 2; i++) joint_vel_values.data.push_back(solutions[i]);
    my_follower.SendVelocity(joint_vel_values);
    
    // save data
    if (myfile.is_open())
	  {
          myfile <<my_follower.joint_position[0]<<" "<<my_follower.joint_position[1]<<" ";
          myfile <<my_follower.goal[0]<<" "<<my_follower.goal[1]<<" ";
          myfile <<my_follower.joint_speed[0]<<" "<<my_follower.joint_speed[1]<<" ";
          myfile <<min_dist[0]<<" "<<min_dist[1]<<" "<<min_dist[2]<<" "<<min_dist[3]<<" ";
          myfile <<solutions[0]<<" "<<solutions[1]<<" "<<solutions[2]<<" "<<solutions[3]<<" "<<solutions[4]<<" "<< endl;
	  }
      else cout << "Unable to open file";

    ros::spinOnce();
    loop_rate.sleep();
  }

  myfile.close();
  return 0;
}

