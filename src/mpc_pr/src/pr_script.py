#!/usr/bin/env python3.6
from pyrep import PyRep
from pyrep.objects.shape import Shape
from pyrep.objects.joint import Joint
from pyrep.backend import sim as sim
import os
import time
import numpy as np
import rospy
from pyrep.robots.arms.arm import Arm
from std_msgs.msg import Float64MultiArray, Float64
import random
import math
from utils import experiment_name, write_mat

run_name = experiment_name()
def fRand(fMin, fMax):
    f = random.uniform(fMin, fMax)
    return f

def config_space(theta_1, theta_2):
  l1 = 0.5
  l2 = 0.4
  R_S = 0.2
  R_quad=(l2/8+R_S)*(l2/8+R_S)
  s1=math.sin(theta_1)
  c1=math.cos(theta_1)
  s12=math.sin(theta_1+theta_2)
  c12=math.cos(theta_1+theta_2)
  O11 = -0.6
  O12 = 0.7
  O21 = 0.6
  O22 = 0.7
  x1=l1*s1+0.1
  x2=l1*s1+l2*s12+0.1
  t1=(l1*c1+1*l2*c12/8-O11)*(l1*c1+1*l2*c12/8-O11)+(l1*s1+1*l2*s12/8-O12)*(l1*s1+1*l2*s12/8-O12)-R_quad
  t2=(l1*c1+3*l2*c12/8-O11)*(l1*c1+3*l2*c12/8-O11)+(l1*s1+3*l2*s12/8-O12)*(l1*s1+3*l2*s12/8-O12)-R_quad
  t3=(l1*c1+5*l2*c12/8-O11)*(l1*c1+5*l2*c12/8-O11)+(l1*s1+5*l2*s12/8-O12)*(l1*s1+5*l2*s12/8-O12)-R_quad
  t4=(l1*c1+7*l2*c12/8-O11)*(l1*c1+7*l2*c12/8-O11)+(l1*s1+7*l2*s12/8-O12)*(l1*s1+7*l2*s12/8-O12)-R_quad
  t5=(l1*c1+1*l2*c12/8-O21)*(l1*c1+1*l2*c12/8-O21)+(l1*s1+1*l2*s12/8-O22)*(l1*s1+1*l2*s12/8-O22)-R_quad
  t6=(l1*c1+3*l2*c12/8-O21)*(l1*c1+3*l2*c12/8-O21)+(l1*s1+3*l2*s12/8-O22)*(l1*s1+3*l2*s12/8-O22)-R_quad
  t7=(l1*c1+5*l2*c12/8-O21)*(l1*c1+5*l2*c12/8-O21)+(l1*s1+5*l2*s12/8-O22)*(l1*s1+5*l2*s12/8-O22)-R_quad
  t8=(l1*c1+7*l2*c12/8-O21)*(l1*c1+7*l2*c12/8-O21)+(l1*s1+7*l2*s12/8-O22)*(l1*s1+7*l2*s12/8-O22)-R_quad

  answer = 0
  if (x1>0 and x2>0 and t1>0 and t2>0 and t3>0 and t4>0 and t5>0 and t6>0 and t7>0 and t8>0):
    # print(theta_1, theta_2, " are feasible")
    answer = 1
  return answer

def PR_ee(theta_1, theta_2):
    l1 = 0.5
    l2 = 0.4
    s1=math.sin(theta_1)
    c1=math.cos(theta_1)
    s12=math.sin(theta_1+theta_2)
    c12=math.cos(theta_1+theta_2)
    ee = [0,0,0]
    ee[0] = l1*c1+l2*c12
    ee[1] = l1*s1+l2*s12
    ee[2] = 0
    return ee

class MyRobot(Arm):
    def __init__(self, count: int = 0):
        super().__init__(count, name = 'PR', num_joints=2)

class MPC:
    def __init__(self, run_name):
        self.scene = '/home/robot/workspaces/planar_robot/scenes/toy_example4.ttt'
        self.states_pub = rospy.Publisher('/pr/joint_states', Float64MultiArray, queue_size=1)
        self.observation = Float64MultiArray()
        self.joint_goals = Float64MultiArray()
        self.first_reset = 0
        self.episodes = 0
        self.run_name = run_name
        self.obs = [0,0,0,0]
        self.u = [0,0]
        self.init_ee = [0,0]
        self._start()

    def _start(self):
        print("_start")
        self.env = PyRep()
        self.env.launch(self.scene, headless = False)
        self.env.start()
        MyRobot()
        self.joint1 = Joint('PR_joint1')
        self.joint2 = Joint('PR_joint2')
        self.init_ee_shape = Shape('initial_ee')
        self._init_variables()
        # print("The robot is launched")

    def _step(self,u):
        # print("_step")
        self.u = u
        self.send_actions(u)
        self.env.step() #step the physics simulation
        self._self_observe()
        self.first_reset = 0
        self.log_variables(self.init_jp, self.obs[0:2], self.u[0:2], self.obs[2:4])

    def _self_observe(self):
        # print("_self_observe")
        self.obs = [self.joint1.get_joint_position(), self.joint2.get_joint_position(), self.joint1.get_joint_velocity(),self.joint2.get_joint_velocity()]
        self.observation.data = self.obs
        self.states_pub.publish(self.observation)
    
    def _reset(self):
        # print("reset")
        if self.first_reset == 0:
            self.env.stop()
            write_mat('../log_reward/' + self.run_name,
                      {'init_q': self.init_q,
                       'q': self.q,
                       'MPC_sol': self.MPC_sol,
                       'real_q_dot': self.q_dot},
                      str(self.episodes))
            self._init_variables()
            # print("stop")
            self.env.start()
            # print("start")
            self._self_observe()
            self.first_reset = 1
            self.episodes += 1

    def _init_variables(self):
        # print("_init_variables")
        self.init_log_variables()
        self.flag=0
        self.init_jp=[0,0]
        while (self.flag==0):
            self.init_jp[0] = fRand(-3.14, 3.14)
            self.init_jp[1] = fRand(-1.57, 1.57)
            self.flag = config_space(self.init_jp[0],self.init_jp[1])
        print("New initial joint positions ", self.init_jp[0],self.init_jp[1])
        self.joint1.set_joint_position(self.init_jp[0])
        self.joint2.set_joint_position(self.init_jp[1])
        self.init_ee = PR_ee(self.init_jp[0],self.init_jp[1]) 
        self.init_ee_shape.set_position(self.init_ee)
        self.flag=0

    def init_log_variables(self):
        # print("init_log_variables")
        self.init_q = [0, 0]
        self.q = [0, 0]
        self.MPC_sol = [0, 0]
        self.q_dot = [0, 0]

    def log_variables(self, init_jp, obs_1, u, obs_2):
        # print("log_variables")/
        self.init_q = np.vstack([self.init_q, init_jp])
        self.q = np.vstack([self.q, obs_1])
        self.MPC_sol = np.vstack([self.MPC_sol, u])
        self.q_dot = np.vstack([self.q_dot, obs_2])

    def send_actions(self,u):
        # print("send_actions")
        self.joint1.set_joint_target_velocity(u[0])
        self.joint2.set_joint_target_velocity(u[1])
 
env = MPC(run_name)
velocity = [0,0]

def talker(data):
    global velocity
    velocity = data.data[0:2]

def state_talker(data):
    global velocity, env
    # print("state talker", data.data)
    if data.data==1:
        env._step(velocity)
        # print("not arrived yet", velocity)
    else:
        # print("arrived")
        env._reset()
    
def main():
    global velocity
    global flag
    rospy.init_node("test_move", anonymous=True)
    rospy.Subscriber("/MPC_solutions", Float64MultiArray, talker)
    rospy.Subscriber("/state_flag", Float64, state_talker)
    rospy.spin()

if __name__ == '__main__': main()