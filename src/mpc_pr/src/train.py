#!/usr/bin/env python3.6
from numpy import loadtxt
from keras.models import Sequential
from keras.layers import Dense

dataset = loadtxt('Data_1.csv', delimiter=',')
init_q = dataset[2,0:2]
goal_q = [3.14,0.0]
q_dot = dataset[:,5:7]

model = Sequential()
model.add(Dense(12, input_dim=2,activation='relu'))
model.add(Dense(20, activation = 'relu'))
model.add(Dense(2000, activation='sigmoid'))

model.compile(loss='MSE', optimizer = 'adam', metricss = ['accuracy'])
model.fit(init_q, q_dot, epochs=150, batch_size=10)
_,accuracy = model.evaluate(init_q, q_dot)

print('Accuracy: %.2f' % (accuracy*100))

