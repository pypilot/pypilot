
#!/usr/bin/env python
#
#   Copyright (C) 2019 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

from signalk.values import Value

import sys, multiprocessing, select
import tensorflow as tf
sys.path.append('..')
from autopilot import AutopilotPilot, AutopilotGain
from signalk.values import *
from signalk.pipeserver import NonBlockingPipe

samples = 50 # 5 seconds at 10hz
num_inputs = 12

class History(object):
  def __init__(self):
    self.data = []

  def put(self, data, samples): # store new data discarding old
    self.data = ([data]+self.data)[:samples]

def BuildModel():
    input = tf.keras.layers.Input(shape=(samples, num_inputs), name='input_layer')
    flatten = tf.keras.layers.Flatten()(input)
    hidden = tf.keras.layers.Dense(16, activation='relu')(flatten)
    output = tf.keras.layers.Dense(1, activation='tanh')(hidden)
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
    for j in range(int(samples)):
      y = []
      for k in range(int(num_inputs)):
        p = (random.random()-.5)*10
        y.append(p)
      x.append(y)

    y = x[0][0]*.05 + x[0][1]*.1
    train_x.append(x)
    train_y.append(y)

  print("pretrain")
  model.fit(train_x, train_y, epochs=4)
#  print('eval ')
#  model.evaluate(train_x,  train_y, verbose=2)
  
def LoadModel():
  print('load mode')
  model = BuildModel()
  try:
    self.model.load_weights('~/.pypilot/learning_weights')
  except:
    return model
  
def LearningProcess(pipe, model_pipe):
  uid = 0
  def Convert(model, pipe):
    converter = tf.lite.TFLiteConverter.from_keras_model(model)
    tflite_model = converter.convert()
    global uid
    uid += 1
    pipe.send((tflite_model, uid))
  
  model = LoadModel()
  PreTrain(model)
  Convert(model, model_pipe)

  poller = select.poll()
  poller.register(pipe, select.POLLIN)

  history = History()

  import psutil
  ps = psutil.Process(os.getpid())
  print('learn process')
  batch_size = 600 # 60 seconds
  train_x, train_y = [], []
  while True:
    # find cpu usage of training process
    cpu = ps.cpu_percent()
    if cpu > 50:
      print('learning cpu very high', cpu)

    # train model from error
    time.sleep(.005)

    if not poller.poll(0.001):
      continue
    
    data = pipe.recv()
    if data['uid'] != uid:
      print('received training sample from old model', uid, data['uid'], time.time())
      continue
    
    d, e = data['input'], data['error']
    history.put(d, samples)
    if len(history.data) == samples:
      h = history.data
      train_x.append(h)
      train_y.append([e])
        
    if len(train_x) >= batch_size:
      p = model.predict(train_x)
      y = p + train_y
      y = list(map(lambda x : [min(max(x[0], -.8), .8)], y))
      model.fit(train_x, y, epochs=4)

      Convert(model, model_pipe)
      train_x, train_y = [], []

  
class LearningPilot(AutopilotPilot):
  def __init__(self, ap):
    super(LearningPilot, self).__init__('learning', ap)
    # create filters
    timestamp = self.ap.server.TimeStamp('ap')

    # create simple pid filter
    self.P = self.Register(AutopilotGain, 'P', .001, .0001, .01)
    self.D = self.Register(AutopilotGain, 'D', .03, .01, .1)

    self.lag = self.Register(RangeProperty, 'lag', 1, 0, 5)
    
    timestamp = self.ap.server.TimeStamp('ap')
    self.dt = self.Register(SensorValue, 'dt', timestamp)
    self.initialized = False
    self.start_time = time.time()
    self.model_uid = 0

  def reset(self):
    PreTrain(self.model)

  def initialize(self):
      # Build model
      self.history = History()

      self.learning_pipe, pipe = NonBlockingPipe('learning_pipe')
      model_pipe, self.model_pipe = NonBlockingPipe('learning_model_pipe')
      self.model_pipe_poller = select.poll()
      self.model_pipe_poller.register(pipe, select.POLLIN)

      self.fit_process = multiprocessing.Process(target=LearningProcess, args=(pipe,model_pipe))
      self.fit_process.start()
      print('start training')
      self.initialized = True
    
  def process(self, reset):
    ap = self.ap

    if not self.initialized:
      if time.time() - self.start_time < 2:
        return
      self.initialize()

    P = ap.heading_error.value
    D = ap.boatimu.SensorValues['headingrate_lowpass'].value
    accel = ap.boatimu.SensorValues['accel'].value
    gyro = ap.boatimu.SensorValues['gyro'].value
    wind = ap.sensors.wind

    # input data
    data = [P, D] + list(accel) + list(gyro)
    data += [ap.servo.voltage.value/24, ap.servo.current.value/20]
    data += [wind.direction.value/360, wind.speed.value/60]

    # training data
    lag_samples = int(self.lag.value/self.ap.boatimu.rate.value)
    self.history.put(data, samples + lag_samples)

    learning_history = self.history.data[lag_samples:]
    if len(learning_history) == samples:
      #error = 0
      #for d in self.history.data[:lag_samples]:
      #  error += d[0]*self.P.value + d[1]*self.D.value # calculate error from current state
      #error /= lag_samples
      d = self.history.data[0]
      e = d[0]*self.P.value + d[1]*self.D.value # calculate error from current state

      # see what our command was to find the better command
      data = {'input': learning_history[0], 'error': e, 'uid': self.model_uid}
      self.learning_pipe.send(data)

      history = self.history.data[:samples]
      if len(history) == samples:
        if self.model_pipe_poller.poll():
          tflite_model, self.model_uid = self.model_pipe.recv()
          open('converted.tflite', 'wb').write(tflite_model)

        if self.model_uid:
          t0 = time.time()
          interpreter = tf.lite.Interpreter(model_path="converted.tflite")
          t1 = time.time()
          interpreter.allocate_tensors()
          t2 = time.time()
          input_details = interpreter.get_input_details()
          output_details = interpreter.get_output_details()
          t3 = time.time()
          input_shape = input_details[0]['shape']
          print ('input details', input_details)
          t4 = time.time()
          interpreter.set_tensor(input_details[0]['index'], np.array(history))
          interpreter.invoke()
          t5 = time.time()
          output_data = interpreter.get_tensor(output_details[0]['index'])
          t6 = time.time()
          print('interpreter timings', t1-t0, t2-t1, t3-t2, t4-t3, t5-t4, t6-t5)
      
    if ap.enabled.value and len(self.history.data) >= samples:
        ap.servo.command.set(command)

pilot = LearningPilot

if __name__ == '__main__':
  learning_pipe, pipe = NonBlockingPipe('learning_pipe')
  fit_process = multiprocessing.Process(target=LearningProcess, args=(pipe,))
  fit_process.start()
  x = 0
  while True:
    P = math.sin(x)
    D = math.sin(x+3)
    x += .01

    inp = [0] * num_inputs 
    inp[0] = P
    inp[1] = D
    
    
    error = math.sin(x-1)
    data = {'input': inp,
            'error': error}
    learning_pipe.send(data)
    time.sleep(.1)
