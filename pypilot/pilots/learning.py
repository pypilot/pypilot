#!/usr/bin/env python
#
#   Copyright (C) 2019 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import multiprocessing, select
from pilot import AutopilotPilot, AutopilotGain
from pypilot.values import * # needed?
#from pypilot.pipeserver import NonBlockingPipe

class LearningPilot(AutopilotPilot):
  def __init__(self, ap):
    super(LearningPilot, self).__init__('learning', ap)
    # create filters
    # gains for training pilot
    self.P = self.register(AutopilotGain, 'P', .001, .0001, .01)
    self.D = self.register(AutopilotGain, 'D', .03, .01, .1)
    self.W = self.register(AutopilotGain, 'W', 0, 0, .1)

    self.servo_rules = self.register(BooleanProperty, 'servo_rules', True)

    self.initialized = False
    self.start_time = time.monotonic()

  def loss(predictions, actions):
    heading = predictions['imu.heading']
    headingrate = predictions['imu.headingrate']
    return self.P.value*heading + self.D.value*headingrate + self.W.value*actions['servo.command']**2
    
  def load():
    try:
      f = open('filename')
      self.meta = json.loads(f.read())
    except Exception as e:
      print('failed to load model')
      self.meta = {'sensors' : []}

  def initialize(self):
      self.load()
      self.initialized = True

  def process(self, reset):
    ap = self.ap

    if not self.initialized:
      if time.monotonic() - self.start_time < 2:
        return
      self.initialize()

    data = {}
    for sensor in self.meta['sensors']:
      v = self.ap.server.values[sensor].value
      if v:
        data[sensor] = v
    self.history.put(data)

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

      if len(history) == samples:
        if self.model_pipe_poller.poll():
          tflite_model, self.model_uid = self.model_pipe.recv()
          open('converted.tflite', 'wb').write(tflite_model)

        if self.model_uid:
          t0 = time.monotonic()
          interpreter = tf.lite.Interpreter(model_path="converted.tflite")
          t1 = time.monotonic()
          interpreter.allocate_tensors()
          t2 = time.monotonic()
          input_details = interpreter.get_input_details()
          output_details = interpreter.get_output_details()
          t3 = time.monotonic()
          input_shape = input_details[0]['shape']
          print ('input details', input_details)
          t4 = time.monotonic()
          interpreter.set_tensor(input_details[0]['index'], np.array(history))
          interpreter.invoke()
          t5 = time.monotonic()
          output_data = interpreter.get_tensor(output_details[0]['index'])
          t6 = time.monotonic()
          print('interpreter timings', t1-t0, t2-t1, t3-t2, t4-t3, t5-t4, t6-t5)
      
    if ap.enabled.value and len(self.history.data) >= samples:
        ap.servo.command.set(command)

        
pilot = LearningPilot
