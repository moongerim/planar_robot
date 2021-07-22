import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
import copy
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import os
import stream_tee as stream_tee
import __main__ as main
import csv
import rospy
from std_msgs.msg import Float64MultiArray
import time
torch.manual_seed(1)
import random
from math import sin,cos
from stream_tee import write_mat
class MyModel(nn.Module):
    def __init__(self, dev, input_size = 2, output_size = 2):
        super().__init__()
        self.dev = dev
        self.linear_1 = nn.Linear(in_features=input_size, out_features=2)
        self.linear_2 = nn.Linear(2, output_size)

    def forward(self, x):
        x = self.linear_1(x)
        x = torch.tanh(x)
        return self.linear_2(x)

class ENV:
    def __init__(self,model):
        rospy.Subscriber("/pr/joint_states", Float64MultiArray, self.callback)
        self.NN_solutions = Float64MultiArray()
        self.pub = rospy.Publisher('/pr/command', Float64MultiArray, queue_size=1)
        self.init_poses = [0, 0]
        self.model = model
        self.goal = [3.14, 0.0]
        self.arrive = False

    def callback(self, data):
        self.observation = data.data[0:4]

    def done(self):
        max_diff = 0
        temp = 0
        arrive = False
        for i in range(2):
            temp = abs(self.goal[i]-self.observation[i])
            if temp>max_diff:
                max_diff = temp
        if max_diff<0.05:
            print("-----Arrived------")
            arrive = True
        return arrive

    def config_space(self, theta_1, theta_2):
        l1 = 0.5
        l2 = 0.4
        R_S = 0.2
        R_quad = (l2/8+R_S)*(l2/8+R_S)
        s1 = sin(theta_1)
        c1 = cos(theta_1)
        s12 = sin(theta_1+theta_2)
        c12 = cos(theta_1+theta_2)
        O11 = -0.6
        O12 = 0.7
        O21 = 0.6
        O22 = 0.7
        x1 = l1*s1+0.1
        x2 = l1*s1+l2*s12+0.1
        t1 = (l1*c1+1*l2*c12/8-O11)*(l1*c1+1*l2*c12/8-O11)+(l1*s1+1*l2*s12/8-O12)*(l1*s1+1*l2*s12/8-O12)-R_quad
        t2 = (l1*c1+3*l2*c12/8-O11)*(l1*c1+3*l2*c12/8-O11)+(l1*s1+3*l2*s12/8-O12)*(l1*s1+3*l2*s12/8-O12)-R_quad
        t3 = (l1*c1+5*l2*c12/8-O11)*(l1*c1+5*l2*c12/8-O11)+(l1*s1+5*l2*s12/8-O12)*(l1*s1+5*l2*s12/8-O12)-R_quad
        t4 = (l1*c1+7*l2*c12/8-O11)*(l1*c1+7*l2*c12/8-O11)+(l1*s1+7*l2*s12/8-O12)*(l1*s1+7*l2*s12/8-O12)-R_quad
        t5 = (l1*c1+1*l2*c12/8-O21)*(l1*c1+1*l2*c12/8-O21)+(l1*s1+1*l2*s12/8-O22)*(l1*s1+1*l2*s12/8-O22)-R_quad
        t6 = (l1*c1+3*l2*c12/8-O21)*(l1*c1+3*l2*c12/8-O21)+(l1*s1+3*l2*s12/8-O22)*(l1*s1+3*l2*s12/8-O22)-R_quad
        t7 = (l1*c1+5*l2*c12/8-O21)*(l1*c1+5*l2*c12/8-O21)+(l1*s1+5*l2*s12/8-O22)*(l1*s1+5*l2*s12/8-O22)-R_quad
        t8 = (l1*c1+7*l2*c12/8-O21)*(l1*c1+7*l2*c12/8-O21)+(l1*s1+7*l2*s12/8-O22)*(l1*s1+7*l2*s12/8-O22)-R_quad
        answer = 0
        if (x1>0 and x2>0 and t1>0 and t2>0 and t3>0 and t4>0 and t5>0 and t6>0 and t7>0 and t8>0):
            answer = 1
        return answer

    def test(self, x_test):
        self.model.eval()
        prediction=[]
        with torch.no_grad():
            seq_data = np.array(x_test)
            seq_data = torch.tensor([i for i in seq_data], dtype=torch.float32).to(dev)
            prediction = self.model(seq_data)
        return prediction

    def step(self, u):
        u = [u[0],u[1],1]
        self.NN_solutions.data = u
        self.pub.publish(self.NN_solutions)
        obs = self.observe()
        return obs
    
    def observe(self):
        return self.observation

    def reset(self):   
        self.init_variables()
        time.sleep(5)
        obs = self.observe()
        return obs 
    
    def init_variables(self):
        self.arrive = False
        answer=0
        while (answer<1):
            self.init_poses[0] = random.uniform(0.0, 3.14)
            self.init_poses[1] = random.uniform(-1.57, 1.57)
            answer = self.config_space(self.init_poses[0],self.init_poses[1])
        print("Init pose: ", self.init_poses[0],self.init_poses[1]) 
        self.NN_solutions.data = [self.init_poses[0],self.init_poses[1],0]
        self.pub.publish(self.NN_solutions)

def save_log(run_name, actions, states,i):
    write_mat('Network_log/' + run_name,
                    {'actions':actions,
                    'states': states},
                    str(i))    

if __name__ == '__main__':
    rospy.init_node("pytorch_test", anonymous=True)
    run_name = stream_tee.generate_timestamp()
    dev = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model = MyModel(dev).to(dev)
    model.cuda()
    data_dir = '/home/robot/workspaces/planar_robot/weights'
    os.chdir(data_dir)
    model.load_state_dict(torch.load('model_20210721_184135.pth'))
    env = ENV(model)
    i = 0
    actions = []
    states = []
    t = time.time()
    observation = env.reset()
    seq_data = observation[0:2]
    action = env.test(seq_data)
    print("HELLO")
    rate = rospy.Rate(20) #hz
    while not rospy.is_shutdown():
        done = env.done()
        if done==True:
            elapsed = time.time() - t
            print("Episode ", i, ' time = ', elapsed)
            save_log(run_name, actions, states, i)
            i+=1
            t = time.time()
            print("Episode", i, " is started")
            actions = []
            states = []
            observation = env.reset()
            seq_data = observation[0:2]
            action = env.test(seq_data)
            t = time.time()
        else:
            observation = env.step(action)
            seq_data = observation[0:2]
            action = env.test(seq_data)
            actions.append(action)
            states.append(observation)
        rate.sleep()

    
        