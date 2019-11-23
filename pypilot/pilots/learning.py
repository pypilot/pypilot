
#!/usr/bin/env python
#
#   Copyright (C) 2019 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

from signalk.values import Value

import sys
sys.path.append('..')
from autopilot import AutopilotPilot, AutopilotGain
from signalk.values import *

class History(object):
  def __init__(self, samples):
    self.samples = samples
    self.data = []

  def put(self, data):
    self.data = (data+self.data)[:self.samples]

  def get(self):
    if not self.data:
      return False
    while (len(self.data) < self.samples):
      self.data += [0]*len(self.data[0])
    return self.data

samples=4
def BuildModel(samples):
    import tensorflow as tf
    input = tf.keras.layers.Input(shape=2*samples, name='input_layer')
#    hidden = tf.keras.layers.Dense(16, activation='relu')(input)
    output = tf.keras.layers.Dense(1, activation='tanh')(input)
    model = tf.keras.Model(inputs=input, outputs=output)
    model.compile(optimizer='adam', loss='mean_squared_error', metrics=['accuracy'])
    return model

import random
import numpy as np
def PreTrain(model):
  train_x=[]
  train_y=[]
  for i in range(10000):
    x = []
    for j in range(samples):
      p = (random.random()-.5)*10
      d = (random.random()-.5)*2
      x.append(p)
      x.append(d)
    y = x[0]*.05 + x[1]*.1
    train_x.append(x)
    train_y.append(y)

  model.fit(train_x, train_y, epochs=4)
  model.evaluate(train_x,  train_y, verbose=2)

  test_x = []
  for j in range(samples):
      p = (random.random()-.5)*10
      d = (random.random()-.5)*2
      test_x.append(p)
      test_x.append(d)

  test_x = np.array([test_x])
  print('testx', test_x)
#  test_x = test_x.astype('float32')
  y = model.predict(test_x)
  print('result', y)

model=  BuildModel(samples)
PreTrain(model)

exit(0)

def TrainingProcess(queue):
  p = psutil.Process(os.getpid())
  while True:
    # find cpu usage of training process
    if p.cpu_percent() > 50:
      print('learning cpu very high')
    
    # train model from error
    time.sleep(.1)
  
class LearningPilot(AutopilotPilot):
  def __init__(self, ap):
    super(LearningPilot, self).__init__('learning', ap)
    # create filters
    timestamp = self.ap.server.TimeStamp('ap')

    # create simple pid filter
    self.P = self.Register(AutopilotGain, 'P', .001, .0001, .01)
    self.D = self.Register(AutopilotGain, 'D', .03, .01, .1)

    timestamp = self.ap.server.TimeStamp('ap')
    self.dt = self.Register(SensorValue, 'dt', timestamp)

    # Build modal
    samples = 50 # 5 seconds
    self.history = History(samples)
    return
    self.modal = BuildModal(samples)
    try:
      self.modal.load_weights('~/.pypilot/learning_weights')
    except:
      self.reset()

  def reset(self):
    PreTrain(self.modal)

  def process(self, reset):
    ap = self.ap

    P = ap.heading_error.value
    D = ap.boatimu.SensorValues['headingrate_lowpass'].value

    self.history.put([P, D])
    return
    command = self.model.evaluate()

    ap.servo.command.set(command)

pilot = LearningPilot
