#!/usr/bin/env python
#
#   Copyright (C) 2020 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import multiprocessing, select
from pilot import AutopilotPilot, AutopilotGain
from intellect import *
disabled = True

def build_actions(current, period_count, count):
    if count <= 0:
        return []
    ret = []
    def actions(command):
      if count <= period_count:
        return [[command]*count]
      return list(map(lambda acts : [command]*period_count + acts, build_actions(command, period_count, count - period_count)))
    if current <= 0:
        ret += actions(-1)
    if current >= 0:
        ret += actions(1)
    ret += actions(0)
    return ret

# use tensor flow lite for prediction to achieve realtime performance
class TFliteModel(Model):
    def __init__(self):
        super(TFliteModel, self).__init__()

    def load(self, state):
        filename = model_filename(state)
        try:
            import tflite_runtime.interpreter as tflite
            f = open(filename + '.conf')
            self.conf = json.loads(f.read())
            f.close()
            t0 = time.monotonic()
            interpreter = tf.lite.Interpreter(model_path=filename + '.tflite_model')
            t1 = time.monotonic()
            interpreter.allocate_tensors()
            t2 = time.monotonic()
            input_details = interpreter.get_input_details()
            output_details = interpreter.get_output_details()
            t3 = time.monotonic()
            input_shape = input_details[0]['shape']
            print ('input details', input_details)
            t4 = time.monotonic()
            t5 = time.monotonic()
            t6 = time.monotonic()
            print('interpreter timings', t1-t0, t2-t1, t3-t2, t4-t3, t5-t4, t6-t5)
            self.interpreter = interpreter
            self.history = History(self.conf, state)
        except Exception as e:
            self.start_time = time.monotonic()          
            print('failed to load model', filename)
            self.interpreter = False

    def predict(self, loss):
        # feed input sensors
        p = self.present()
        # inputs are the sensors over past time
        sensors_data = intellect.inputs(self.history.data, self.conf['sensors'])

        # this many future actions
        count = self.history.samples - self.present()
        rate = state['imu.rate']
        current = self.servo.command.value
        period = .4;

        # actions can be stored to optimize this
        actions = self.build_actions(current, period/rate, count)

        # inputs are past sensors and future actions
        inputs = list(map(lambda action : sensors_data + action, actions))

        self.interpreter.set_tensor(input_details[0]['index'], np.array(inputs))
        self.interpreter.invoke()
        outputs = interpreter.get_tensor(output_details[0]['index'])
            
        pnames = self.conf['predictions']
  
        # find best prediction based on loss
        besti = False
        for i in len(inputs):
            output = outputs[i].reshape(-1, len(pnames))
            weight = 0
            for j in len(output):
                prediction = {}
                for k in len(pnames):
                    prediction[pnames[j]] = output[j][k], self.conf['accuracy'][j][k]
                action = actions[i][j]
                weight += loss(prediction, action)
                
            if not besti or weight < best:
                besti = i
                best = weight
            
        return actions[besti]

class LearningPilot(AutopilotPilot):
    def __init__(self, ap):
        super(LearningPilot, self).__init__('learning', ap)
        self.P = self.register(AutopilotGain, 'P', .001, .0001, .01)
        self.D = self.register(AutopilotGain, 'D', .03, .01, .1)
        self.W = self.register(AutopilotGain, 'W', 0, 0, .1)

        self.state = False
        self.start_time = time.monotonic()

    def loss(predictions, action):
        heading, heading_accuracy = predictions['imu.heading_error']
        headingrate, headingrate_accuracy = predictions['imu.headingrate_lowpass']
        return (self.P.value*heading + self.D.value*headingrate)**2 + self.W.value*action**2

    def process(self, reset):
        ap = self.ap
        state = {'ap.mode', ap.mode.value, 'imu.rate', ap.imu.rate.value}
        if self.state != state or not self.model:
            if time.monotonic() - self.start_time < 2:
                return
            self.load(state)
            if not self.model:
                ap.pilot.set('basic') # fall back to basic pilot if no model loaded
                return

        data = {}
        for sensor in self.conf['sensors'] + self.conf['predictions']:
            data[sensor] = self.ap.client.values[sensor].value
        self.model.history.put(data)

        if not self.history.full():
            # defer to basic pilot control until history fills
            self.ap.pilots['basic'].process(reset)
            return

        if ap.enabled.value:
            actions = self.model.predict(self.loss)
            ap.servo.command.command(actions[0])

        
pilot = LearningPilot
