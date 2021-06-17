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
    ros::Publisher reference_pub;

    double goal[2] = {0.0000, 0.0000};
    double comand_vel[2] = {0.0000, 0.0000};
    double joint_position[2] = {0.0000, 0.0000};
    double joint_speed[2] = {0.0000, 0.0000};
    double dist[10] = {0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6};

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

    void SendReference(const std_msgs::Float64MultiArray msg){
    	reference_pub.publish(msg);
	return;
    }

    void change_distances(const std_msgs::Float64MultiArray msg){
    	for (int i=0; i<10; i++) dist[i] = msg.data[i];
    }
}; 

int main(int argc, char **argv)
{
  myfile.open("data_planar.csv", ios::out); 
  ros::init(argc, argv, "pr_controller");
  ros::NodeHandle n;
  ROS_INFO("Node Started");

  GoalFollower my_follower;
  my_follower.chatter_pub = n.advertise<std_msgs::Float64MultiArray>("/pr/command", 1);
  my_follower.reference_pub = n.advertise<std_msgs::Float64MultiArray>("/reference", 1);

  ros::Subscriber joint_status = n.subscribe("/pr/joint_states", 1, &GoalFollower::change_states_msg, &my_follower);
  ros::Subscriber distances = n.subscribe("/CoppeliaSim/distances", 1, &GoalFollower::change_distances, &my_follower);

  double smallest_dist;
  double sphere_1_dist;
  double sphere_2_dist;
  double min_dist[] = {0.5, 0.5, 0.5, 0.5};
//   double goal_queue[] = {2.5, -1.0};
  double goal_queue[] = {3.14, 0.0};
  double l1 = 0.5;
  double l2 = 0.4;
  double ee_ref[3];
  ros::Rate loop_rate(20);

  while (ros::ok())
  {
    double currentState_targetValue[4];
    // define terminal goal
    for (int i=0; i<2; i++) my_follower.goal[i] = goal_queue[i];
    // Check if arrived
    float max_diff = 0;
    float temp = 0;
    for (int i = 0; i < 2; i++) {
        temp = abs(my_follower.goal[i] - my_follower.joint_position[i]);
        if (temp > max_diff) max_diff = temp; 
    }
    for (int i=0; i<2; i++) if (max_diff<0.05) printf("Arrived\n");

    //******************* get_min_dist **********************
    sphere_1_dist = 10000;
    sphere_2_dist = 10000;
    smallest_dist = 10000;

    min_dist[0] = 1.0; min_dist[1] = 1.0; min_dist[2] = 1.0; min_dist[3] = 1.0;
    double spheres_dist[] = {0.0, 0.0, 0.0, 0.0};
    
    for (int i=0; i<4; i++){
        sphere_1_dist = my_follower.dist[i*2];
        if (sphere_1_dist<min_dist[i]) min_dist[i] = sphere_1_dist;
        sphere_2_dist = my_follower.dist[i*2+1];
        if (sphere_2_dist<min_dist[i]) min_dist[i] = sphere_2_dist;
        if (smallest_dist>min_dist[i]) smallest_dist = min_dist[i];
    }
	
    printf("smallest_dist = %f\n",smallest_dist);
    double* solutions;
    if (smallest_dist >= 0.00) {
          // Goal reference position 
          for (int i = 0; i < 2; i++){
              currentState_targetValue[i] = my_follower.joint_position[i];
              currentState_targetValue[i+2] = my_follower.goal[i];
          }
          solutions = myMpcSolver.solve_mpc(currentState_targetValue);
    } 
    else{
        for (int i=0; i<5; i++) solutions[i] = 0;
    }
    // printf("Current state goal = %f, %f\n", currentState_targetValue[2],currentState_targetValue[3]);
    ee_ref[0] = l1*cos(goal_queue[0])+l2*cos(goal_queue[0]+goal_queue[1]);
    ee_ref[1] = l1*sin(goal_queue[0])+l2*sin(goal_queue[0]+goal_queue[1]);
    ee_ref[2] = 0;
    // printf("goal = %f,%f, ee = %f,%f,%f\n", currentState_targetValue[2],currentState_targetValue[3],ee_ref[0],ee_ref[1],ee_ref[2]);

    // send end effector reference in order to visualize:
    std_msgs::Float64MultiArray ee_ref_values;
    ee_ref_values.data.clear();
    for (int i = 0; i < 3; i++) ee_ref_values.data.push_back(ee_ref[i]);
    my_follower.SendReference(ee_ref_values);

    // prepare to send commands
    std_msgs::Float64MultiArray joint_vel_values;
    joint_vel_values.data.clear();
    for (int i = 0; i < 2; i++) joint_vel_values.data.push_back(solutions[i]);
    printf("Solutions = %f, %f\n", solutions[0], solutions[1]);
    my_follower.SendVelocity(joint_vel_values);
    
    // save data
    if (myfile.is_open())
	  {
          myfile <<my_follower.joint_position[0]<<" "<<my_follower.joint_position[1]<<" ";
          myfile <<my_follower.goal[0]<<" "<<my_follower.goal[1]<<" ";
          myfile <<my_follower.joint_speed[0]<<" "<<my_follower.joint_speed[1]<<" ";
          myfile <<min_dist[0]<<" "<<min_dist[1]<<" "<<min_dist[2]<<" "<<min_dist[3]<<" ";
          myfile <<solutions[0]<<" "<<solutions[1]<<" "<<solutions[2]<<" "<<solutions[3]<<" "<<solutions[4]<<" ";
          myfile <<smallest_dist<<" "<<my_follower.dist[8]<<" "<<my_follower.dist[9]<<" "<<endl;
	  }
      else cout << "Unable to open file";

    ros::spinOnce();
    loop_rate.sleep();
  }

  myfile.close();
  return 0;
}

