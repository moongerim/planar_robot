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

def test(model, x_test):
    model.eval()
    prediction=[]
    with torch.no_grad():
        seq_data = np.array(x_test)
        seq_data = torch.tensor([i for i in seq_data], dtype=torch.float32).to(dev)
        prediction = model(seq_data)
    return prediction

run_name = stream_tee.generate_timestamp()
dev = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
model = MyModel(dev).to(dev)
model.cuda()
data_dir = '/home/robot/workspaces/planar_robot/weights'
os.chdir(data_dir)
model.load_state_dict(torch.load('model_20210701_105701.pth'))  
logfile = 'real_test_log_{}.csv'.format(run_name)
data_dir = '/home/robot/workspaces/planar_robot/_LOGS/'
os.chdir(data_dir)

pub = rospy.Publisher('/pr/command', Float64MultiArray, queue_size=1)
NN_solutions = Float64MultiArray()
row_data = [0, 0]
first_step = 0


def talker(data):
    global model, logfile, pub, row_data, NN_solutions, first_step
    if first_step==0:
        init = [0.0, 0.1, 0]
        print("init", init)
        NN_solutions.data = init
        pub.publish(NN_solutions)
        time.sleep(5)
        first_step=1
    else:
        x_test = data.data[0:2]
        print("joint positions", x_test)
        predicted_data = test(model, x_test)
        init = list(predicted_data.cpu().numpy())
        row_data = [init[0], init[1], x_test[0], x_test[1]]
        NN_solutions.data = [init[0], init[1], 1]
        pub.publish(NN_solutions)
        with open(logfile,  'a') as fd:
            wr = csv.writer(fd, dialect='excel')
            wr.writerow(row_data)

def main():
    global first_step
    rospy.init_node("pytorch_test", anonymous=True)
    rospy.Subscriber("/pr/joint_states", Float64MultiArray, talker)
    time.sleep(5)
    rospy.spin()

if __name__ == '__main__': main()