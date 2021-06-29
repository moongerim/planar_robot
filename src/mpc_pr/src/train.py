#!/usr/bin/env python3.6
import os
import pandas as pd
import numpy as np
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers
from tensorflow.keras.layers.experimental.preprocessing import Normalization

# Data loading:
data_dir = '/home/robot/workspaces/planar_robot/data/'
os.chdir(data_dir)

def load_data():
    data = []
    full_data = None
    for i in range(1, 283):
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

def simple_model():



inputs = keras.Input(shape=(2,), name='q')
x1 = layers.Dense(2,activation='tanh')(inputs)
outputs = layers.Dense(2,name='prediction')(x1)
model = keras.Model(inputs = inputs, outputs = outputs)

optimizer = keras.optimizers.SGD(learning_rate=1e-3)
loss_fn = keras.losses.CategoricalCrossentropy(from_logits=True)
batch_size = 64
train_dataset = tf.data.Dataset.from_tensor_slices((x_train,y_train))
train_dataset = train_dataset.shuffle(buffer_size=1024).batch(batch_size)

epochs = 100
for epoch in range(epochs):
    print("\nStart of epoch %d" % (epoch,))

    # Iterate over the batches of the dataset.
    for step, (x_batch_train, y_batch_train) in enumerate(train_dataset):
        with tf.GradientTape() as tape:
            logits = model(x_batch_train, training=True)
            loss_value = loss_fn(y_batch_train, logits)
        grads = tape.gradient(loss_value, model.trainable_weights)
        optimizer.apply_gradients(zip(grads, model.trainable_weights))
        if step % 200 == 0:
            print(
                "Training loss (for one batch) at step %d: %.4f"
                % (step, float(loss_value))
            )
            print("Seen so far: %s samples" % ((step + 1) * 64))

if __name__ == '__main__':
    log_file1 = 'train_log0.txt'
    log_file2 = 'eval_log0.txt'

    epoches = 100
    log_interval = 500

    all_data = load_data()
    train_data, eval_data, test_data = split_data(all_data)

    x_train = train_data[:,0:2]
    y_train = train_data[:, 15:17]

    x_eval = eval_data[:,0:2]
    y_eval = eval_data[:, 15:17]

    x_test = test_data[:,0:2]
    y_test = test_data[:, 15:17]


    dev = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

    model = LSTM(dev).to(dev)

    loss_function = nn.MSELoss()
    optimizer = torch.optim.Adam(model.parameters(), lr=0.001)
    lower_loss = 1

    for epoch in range(epoches):
        loss1 = train(epoch, dev, model, train_data, optimizer, log_interval, loss_function, log_file1)
        loss2 = evals(model, eval_data, dev, loss_function, log_file2)
        if lower_loss>loss2:
            lower_loss = loss2
            print('\nThe lowest loss is: {:4f}\n '.format(lower_loss))
            torch.save(model.state_dict(), "model1303.pth")
    
    print("\nA testing part")
    model.cuda()
    model.load_state_dict(torch.load('model1303.pth'))  
    predicted_data, real_data = test(model, test_data)
    visualize(predicted_data, real_data)

