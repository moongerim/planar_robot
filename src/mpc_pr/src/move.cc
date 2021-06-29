#include "mpc_pr/move.h"
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include "std_msgs/Int32.h"

using namespace std;

ofstream myfile;
int rti_num = 10;

MPC_solver myMpcSolver(rti_num);

double fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

double config_space(double theta_1, double theta_2){
  double l1 = 0.5;
  double l2 = 0.4;
  double R_S = 0.2;
  double R_quad = (l2/8+R_S)*(l2/8+R_S);
  double s1 = sin(theta_1);
  double c1 = cos(theta_1);
  double s12 = sin(theta_1+theta_2);
  double c12 = cos(theta_1+theta_2);
  double O11 = -0.6;
  double O12 = 0.7;
  double O21 = 0.6;
  double O22 = 0.7;
  double x1 = l1*s1+0.1;
  double x2 = l1*s1+l2*s12+0.1;
  double t1 = (l1*c1+1*l2*c12/8-O11)*(l1*c1+1*l2*c12/8-O11)+(l1*s1+1*l2*s12/8-O12)*(l1*s1+1*l2*s12/8-O12)-R_quad;
  double t2 = (l1*c1+3*l2*c12/8-O11)*(l1*c1+3*l2*c12/8-O11)+(l1*s1+3*l2*s12/8-O12)*(l1*s1+3*l2*s12/8-O12)-R_quad;
  double t3 = (l1*c1+5*l2*c12/8-O11)*(l1*c1+5*l2*c12/8-O11)+(l1*s1+5*l2*s12/8-O12)*(l1*s1+5*l2*s12/8-O12)-R_quad;
  double t4 = (l1*c1+7*l2*c12/8-O11)*(l1*c1+7*l2*c12/8-O11)+(l1*s1+7*l2*s12/8-O12)*(l1*s1+7*l2*s12/8-O12)-R_quad;
  double t5 = (l1*c1+1*l2*c12/8-O21)*(l1*c1+1*l2*c12/8-O21)+(l1*s1+1*l2*s12/8-O22)*(l1*s1+1*l2*s12/8-O22)-R_quad;
  double t6 = (l1*c1+3*l2*c12/8-O21)*(l1*c1+3*l2*c12/8-O21)+(l1*s1+3*l2*s12/8-O22)*(l1*s1+3*l2*s12/8-O22)-R_quad;
  double t7 = (l1*c1+5*l2*c12/8-O21)*(l1*c1+5*l2*c12/8-O21)+(l1*s1+5*l2*s12/8-O22)*(l1*s1+5*l2*s12/8-O22)-R_quad;
  double t8 = (l1*c1+7*l2*c12/8-O21)*(l1*c1+7*l2*c12/8-O21)+(l1*s1+7*l2*s12/8-O22)*(l1*s1+7*l2*s12/8-O22)-R_quad;

  double answer = 0;
  if (x1>0 and x2>0 and t1>0 and t2>0 and t3>0 and t4>0 and t5>0 and t6>0 and t7>0 and t8>0){
     answer = 1;
  }
  return answer;
}



// Introduce class to make safer goal change
class GoalFollower 
{ 
    // Access specifier 
    public: 
    // Data Members 
    ros::Publisher chatter_pub;
    ros::Publisher ee_pub;
    double goal[2] = {3.14, 0.0};
    double comand_vel[2] = {0.0000, 0.0000};
    double joint_position[2] = {0.0000, 0.0000};
    double joint_speed[2] = {0.0000, 0.0000};
    double dist[10] = {0.0000, 0.0000,0.0000, 0.0000,0.0000, 0.0000,0.0000, 0.0000,0.0000, 0.0000};
    // double flag = 1.0;
    void change_states_msg(const std_msgs::Float64MultiArray msg) 
    { 
       for (int i=0; i<2; i++) {
           joint_position[i] = msg.data[i];
           joint_speed[i] = msg.data[i+2];
       }
    }
    void change_dist_msg(const std_msgs::Float64MultiArray msg) 
    { 
       for (int i=0; i<10; i++) dist[i] = msg.data[i];
    }
    
    void SendVelocity(const std_msgs::Float64MultiArray joint_vel_values){
    	chatter_pub.publish(joint_vel_values);
	    return;
    }
    void SendRef(const std_msgs::Float64MultiArray ee_values){
    	ee_pub.publish(ee_values);
	    return;
    }
}; 

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pr_controller");
  ros::NodeHandle n;
  ROS_INFO("Node Started");
  // string filename = "data"
  myfile.open("data_378.csv", ios::out); 
  GoalFollower my_follower;
  my_follower.chatter_pub = n.advertise<std_msgs::Float64MultiArray>("/pr/command", 1);
  my_follower.ee_pub = n.advertise<std_msgs::Float64MultiArray>("/reference", 1);
  ros::Subscriber joint_status = n.subscribe("/pr/joint_states", 1, &GoalFollower::change_states_msg, &my_follower);
  ros::Subscriber dist_status = n.subscribe("/CoppeliaSim/distances", 1, &GoalFollower::change_dist_msg, &my_follower);

  double init[2]={1.3,0.0000};
  double answer = 0.0;
  int fileseq=379;
  ros::Rate loop_rate(20);
  while (ros::ok())
  {
    // MPC input:
    double currentState_targetValue[4];
    answer=0;
    // Check if arrived
    float max_diff = 0;
    float temp = 0;
    for (int i = 0; i < 2; i++) {
        temp = abs(my_follower.goal[i] - my_follower.joint_position[i]);
        if (temp > max_diff) max_diff = temp; 
    }
    double* solutions;
    string filename;
    if (max_diff<0.1) {
        printf("Arrived\n");
        myfile.close();
        while (answer<1){
          // init[0] = fRand(-3.14, 3.14);
          // init[1] = fRand(-1.57,1.57);
          init[1]=init[1]+0.1;
          answer = config_space(init[0],init[1]);
          // printf("thetas = %f,%f,ans=%f\n",init[0],init[1],answer);
        }
        solutions[0]=init[0];
        solutions[1]=init[1];
        solutions[2]=0;
        printf("Init poses=%f,%f\n",init[0],init[1]);
        std_msgs::Float64MultiArray joint_vel_values;
        joint_vel_values.data.clear();
        for (int i = 0; i < 3; i++) joint_vel_values.data.push_back(solutions[i]);
        my_follower.SendVelocity(joint_vel_values);
        double l1 = 0.5;
        double l2 = 0.4;
        double s1 = sin(init[0]);
        double c1 = cos(init[1]);
        double s12 = sin(init[0]+init[1]);
        double c12 = cos(init[0]+init[1]);
        double ee[3] = {0,0,0};
        ee[0] = l1*c1+l2*c12;
        ee[1] = l1*s1+l2*s12;
        ee[2] = 0;
        std_msgs::Float64MultiArray ee_values;
        ee_values.data.clear();
        for (int i = 0; i < 3; i++) ee_values.data.push_back(ee[i]);
        my_follower.SendRef(ee_values);
        filename = "data_"+to_string(fileseq)+".csv";
        myfile.open(filename, ios::out); 
        fileseq++;
        sleep(5);
    }
    else{
      for (int i = 0; i < 2; i++){
        currentState_targetValue[i] = my_follower.joint_position[i];
        currentState_targetValue[i+2] = my_follower.goal[i];
      }
      solutions = myMpcSolver.solve_mpc(currentState_targetValue);
      std_msgs::Float64MultiArray joint_vel_values;
      joint_vel_values.data.clear();
      for (int i = 0; i < 2; i++) joint_vel_values.data.push_back(solutions[i]);
      joint_vel_values.data.push_back(1);
      printf("Solutions = %f, %f\n", solutions[0], solutions[1]);
      my_follower.SendVelocity(joint_vel_values);
    }
    if (myfile.is_open())
	  {
      myfile <<my_follower.joint_position[0]<<" "<<my_follower.joint_position[1]<<" ";
      myfile <<init[0]<<" "<<init[1]<<" "<<max_diff<<" ";
      myfile <<my_follower.joint_speed[0]<<" "<<my_follower.joint_speed[1]<<" ";
      myfile <<my_follower.dist[0]<<" "<<my_follower.dist[1]<<" "<<my_follower.dist[2]<<" "<<my_follower.dist[3]<<" ";
      myfile <<my_follower.dist[4]<<" "<<my_follower.dist[5]<<" "<<my_follower.dist[6]<<" "<<my_follower.dist[7]<<" ";
      myfile <<solutions[0]<<" "<<solutions[1]<<" "<<solutions[2]<<" "<<solutions[3]<<" "<<solutions[4]<< endl;
	  }
      else cout << "Unable to open file";
    
    ros::spinOnce();
    loop_rate.sleep();
  }
  myfile.close();
  return 0;
}

