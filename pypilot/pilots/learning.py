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
#import tensorflow as tf
sys.path.append('..')
from pilot import AutopilotPilot, AutopilotGain
from signalk.values import *
from signalk.pipeserver import NonBlockingPipe

samples = 50 # 5 seconds at 10hz
num_inputs = 12

def LoadModel():
  print('load mode')
  model = BuildModel()
  try:
    self.model.load_weights('~/.pypilot/learning_weights')
  except:
    return model
  
class LearningPilot(AutopilotPilot):
  def __init__(self, ap):
    super(LearningPilot, self).__init__('learning', ap)
    # create filters
    timestamp = self.ap.server.TimeStamp('ap')

    # gains for training pilot
    self.P = self.Register(AutopilotGain, 'P', .001, .0001, .01)
    self.D = self.Register(AutopilotGain, 'D', .03, .01, .1)
    self.W = self.Register(AutopilotGain, 'W', 0, 0, .1)

    self.servo_rules = self.Register(BooleanProperty, 'servo_rules', True)

    timestamp = self.ap.server.TimeStamp('ap')
    self.dt = self.Register(SensorValue, 'dt', timestamp)
    self.initialized = False
    self.start_time = time.time()


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
