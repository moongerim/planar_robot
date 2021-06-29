import pandas as pd
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
import copy
import matplotlib.pyplot as plt
import os
import stream_tee as stream_tee
torch.manual_seed(1)
import os
import stream_tee as stream_tee
import __main__ as main

def experiment_name():
    experiment = os.path.splitext(os.path.split(main.__file__)[1])[0]
    name = experiment + '_' + stream_tee.generate_timestamp()
    return name


class MyModel(nn.Module):
    def __init__(self, dev, input_size = 2, output_size = 2):
        super().__init__()
        self.dev = dev
        self.linear_1 = nn.Linear(in_features=input_size, out_features=2)
        self.linear_2 = nn.Linear(2, output_size)

    def forward(self, x):
        x = self.linear_1(x)
        x = F.tanh(x)
        return self.linear_2(x)

def train(epoch, dev, model, x_train, y_train, optimizer, log_interval, loss_function, log_file):
    runLoss = 0
    model.train()

    for b in range(0, len(x_train), n_batch):
        seq_data = np.array(x_train[b:b+n_batch])
        seq_label = np.array(y_train[b:b+n_batch])
        seq_data = torch.tensor([i for i in seq_data], dtype=torch.float32).to(dev)
        seq_label = torch.tensor([i for i in seq_label], dtype=torch.float32).to(dev)
        optimizer.zero_grad()

        y_pred = model(seq_data)
        single_loss = loss_function(y_pred, seq_label)

        runLoss += single_loss.item()
        single_loss.backward()
        optimizer.step()
        if b % log_interval == 0:
            print ('Train epoch {} [{}/{}] loss: {:.6f}'.format(epoch, b, len(train_data), single_loss.item()))
            with open('_LOGS/{}'.format(log_file), 'a+') as f:
                f.write('Train epoch {} [{}/{}] loss: {:.6f}\n'.format(epoch, b, len(train_data), single_loss.item()))
    return runLoss


def evals(model, x_eval, y_eval, dev, loss_function, log_file):
    total_loss = 0
    correct = 0
    model.eval()

    with torch.no_grad():
        for b in range(0, len(x_eval), n_batch):
            seq_data = np.array(x_eval[b:b+n_batch])
            seq_label = np.array(y_eval[b:b+n_batch])
            seq_data = torch.tensor([i for i in seq_data], dtype=torch.float32).to(dev)
            seq_label = torch.tensor([i for i in seq_label], dtype=torch.float32).to(dev)
            y_pred = model(seq_data)
            single_loss = loss_function(y_pred, seq_label)
            total_loss += single_loss.item()
        total_loss /= len(eval_data)
    print('\nEvaluation Set: Average loss: {:4f}\n'.format(total_loss))
    with open('_LOGS/{}'.format(log_file), 'a+') as f:
                f.write('Train epoch {} [{}/{}] loss: {:.6f}\n'.format(epoch, b, len(eval_data), single_loss.item()))
    return total_loss

def test(model, x_test, y_test):
    model.eval()
    predictions=[]
    real_data=[]
    with torch.no_grad():
        for b in range(0, len(x_test), n_batch):
            seq_data = np.array(x_test[b:b+n_batch])
            seq_label = np.array(y_test[b:b+n_batch])
            real_data.append(seq_label)
            seq_data = torch.tensor([i for i in seq_data], dtype=torch.float32).to(dev)
            predictions.append(model(seq_data))
    return predictions, real_data

def load_data(n_files):
    data = []
    full_data = None
    for i in range(1, n_files):
        raw_data = np.loadtxt('data_{}.csv'.format(i), skiprows = 1)
        if full_data is None:
            full_data = raw_data
        else:
            full_data = np.concatenate((full_data, raw_data), axis=0)
    return full_data

def split_data(data):
    all_size = len(data)
    train_size = int(all_size*0.7)
    eval_size = int(all_size*0.2)
    test_size = all_size-train_size-eval_size
    train_data = data[:train_size]
    eval_data = data[train_size:train_size+eval_size]
    test_data = data[train_size+eval_size:]
    return [train_data, eval_data, test_data]

def visualize(predicted_data, real_data):
    p_data = np.asarray(predicted_data)
    r_data = np.asarray(real_data)
    p_len = len(p_data)
    r_len = len(r_data)

    q_1_dot_p = np.zeros(p_len)
    q_2_dot_p = np.zeros(p_len)
    for i in range(p_data):
        init = p_data[i]
        init = list(init.cpu().numpy())
        q_1_dot_p[i] = init[0]
        q_2_dot_p[i] = init[1]
    q_1_dot_r = np.zeros(r_len) 
    q_2_dot_r = np.zeros(r_len) 
    for i in range(r_data):
        real = r_data[i]
        real = list(real)
        q_1_dot_r[i] = real[0]
        q_2_dot_r[i] = real[1]
    
    a = 2
    fig, axs = plt.subplots(a,1)   
    time = np.arange(0,p_len,1)

    axs[0].plot(time, q_1_dot_p, label = 'predicted')
    axs[0].set_ylabel('q_1_dot')
    axs[0].plot(time, q_1_dot_r, label = 'real')
    axs[0].grid(True)
    axs[1].plot(time, q_2_dot_p, label = 'predicted')
    axs[1].set_ylabel('q_2_dot')
    axs[1].plot(time, q_2_dot_r, label = 'real')
    axs[1].grid(True)
    plt.show()


if __name__ == '__main__':
    run_name = experiment_name()
    train_log = 'train_log_{}.txt'.format(run_name)
    eval_log = 'eval_log_{}.txt'.format(run_name)
    n_files = 377
    n_batch = 10
    # Data loading:
    data_dir = '/home/robot/workspaces/planar_robot/data/'
    os.chdir(data_dir)
    all_data = load_data(n_files)
    train_data, eval_data, test_data = split_data(all_data)

    x_train = train_data[:,0:2]
    y_train = train_data[:, 15:17]

    x_eval = eval_data[:,0:2]
    y_eval = eval_data[:, 15:17]

    x_test = test_data[:,0:2]
    y_test = test_data[:, 15:17]

    data_dir = '/home/robot/workspaces/planar_robot/'
    os.chdir(data_dir)
    dev = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

    model = MyModel(dev).to(dev)
    epoches = 100
    log_interval = 500
    loss_function = nn.MSELoss()
    optimizer = torch.optim.Adam(model.parameters(), lr=1e-3)
    lower_loss = 1

    for epoch in range(epoches):
        loss1 = train(epoch, dev, model, x_train, y_train, optimizer, log_interval, loss_function, train_log)
        loss2 = evals(model, x_eval, y_eval, dev, loss_function, eval_log)
        if lower_loss>loss2:
            lower_loss = loss2
            print('\nThe lowest loss is: {:4f}\n '.format(lower_loss))
            torch.save(model.state_dict(), "model2906.pth")
    
    print("\nA testing part")
    model.cuda()
    model.load_state_dict(torch.load('model2906.pth'))  
    predicted_data, real_data = test(model, x_test, y_test)
    visualize(predicted_data, real_data)

