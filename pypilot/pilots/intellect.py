#!/usr/bin/env python
#
#   Copyright (C) 2019 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import os, sys, time, math
import tensorflow as tf
from signalk.client import SignalKClient

# convenience
def rate(conf):
  return conf['state']['imu.rate']

class History(object):
  def __init__(self, conf):
    self.conf = conf
    self.data = []

  def samples(self):
    dt = (self.conf['past']+self.conf['future'])*rate(self.conf)
    return int(math.ceil(dt))

  def put(self, data):
    self.data = (self.data+[data])[:self.samples()]

def inputs(history, names):
    def select(values, names):
      data = []
      for name in values:
        if not name in names:
            continue
        value = values[name]
        if type(value) == type([]):
            data += value
        else:
            data.append(value)
      return data
    def flatten(values):
        if type(values) != type([]):
          return [float(values)]
        data = []
        for value in values:
            data += flatten(value)
        return data
    return flatten(list(map(lambda input : select(input, names), history)))


def norm_sensor(name, value):
    conversions = {'imu.accel' : 1,
                   'imu.gyro' : .1,
                   'servo.current': 1,
                   'servo.command': 1,
                   'ap.heading_error': .2,
                   'imu.headingrate_lowpass': .1}
    c = conversions[name]
    def norm_value(value):
      return math.tanh(c*value)

    if type(value) == type([]):
      return list(map(norm_value, value))
    return norm_value(value)

class Intellect(object):
    def __init__(self):
        self.train_x, self.train_y = [], []
        self.inputs = {}
        self.conf = {'past': 5, # seconds of sensor data
                     'future': 2, # seconds to consider in the future
                     'sensors': ['imu.accel', 'imu.gyro', 'servo.current', 'servo.command'],
                     'actions':  ['servo.command'],
                     'predictions': ['ap.heading_error', 'imu.headingrate_lowpass'],
                     'state': {'ap.mode': 'none', 'imu.rate': 1}}
        self.ap_enabled = False
        self.history = History(self.conf)

        self.lasttimestamp = 0
        self.firsttimestamp = False
        
    def load(self, mode):
        model = build(self.conf)
        try:
            self.model.load_weights('~/.pypilot/intellect')
        except:
            return model
  
    def train(self):
        if len(self.history.data) != self.history.samples():
            print('train', len(self.history.data), self.history.samples())
            return # not enough data in history yet
        present = rate(self.conf)*self.conf['past']
        # inputs are the sensors and predictions over past time
        sensors_data = inputs(self.history.data[:present], self.conf['sensors'])
        # and the actions in the future
        actions_data = inputs(self.history.data[present:], self.conf['actions'])
        # predictions in the future
        predictions_data = inputs(self.history.data[present:], self.conf['predictions'])
    
        if not self.model:
            self.train_x, self.train_y = [], []
    
        self.train_x.append(sensors_data + actions_data)
        self.train_y.append(predictions_data)

        if not self.model:
            print('build')
            self.build(len(self.train_x[0]), len(self.train_y[0]))

        pool_size = 1000 # how much data to accumulate before training
        l = len(self.train_x)
        if l < pool_size:
            if l%10 == 0:
              print('l', l)
            return
        print('fit', len(self.train_x), len(self.train_x[0]), len(self.train_y), len(self.train_y[0]))
        #print('trainx', self.train_x[0])
        #print('trainy', self.train_y[0])
        self.model.fit(self.train_x, self.train_y, epochs=40)
        self.train_x, self.train_y = [], []

    def build(self, input_size, output_size):
        conf = self.conf        
        input = tf.keras.layers.Input(shape=(input_size,), name='input_layer')
        hidden = tf.keras.layers.Dense(64, activation='relu')(input)
        output = tf.keras.layers.Dense(output_size, activation='relu')(hidden)
        self.model = tf.keras.Model(inputs=input, outputs=output)
        self.model.compile(optimizer='adam', loss='mean_squared_error', metrics=['accuracy'])

    def save(self, filename):
        converter = tf.lite.TFLiteConverter.from_keras_model(self.model)
        tflite_model = converter.convert()
        try:
          import json
          f = open(filename, 'w')
          conf['model_filename'] = filename + '.tflite_model'
          f.write(json.dumps(conf))
          f.close()
          f = open(conf['model_filename'], 'w')
          f.write(tflite_model)
          f.close()
        except Exception as e:
          print('failed to save', f)

    def receive_single(self, name, msg):
        value = msg['value']

        if name == 'ap.enabled':
          self.ap_enabled = value
                     
        elif name in self.conf['state']:
            self.conf['state'][name] = value
            self.history.data = []
            self.model = False
            return
          
        elif name in self.conf['sensors'] and (1 or self.ap_enabled):
            self.inputs[name] = norm_sensor(name, value)

        elif name == 'timestamp':
            t0 = time.time()
            if not self.firsttimestamp:
              self.firsttimestamp = value, t0
            else:
              first_value, first_t0 = self.firsttimestamp
              dt = value - first_value
              dtl = t0 - first_t0
              if(abs(dt-dtl) > 10.0):
                print('computation not keep up!!', dtl-dt)
              
            dt = value - self.lasttimestamp
            self.lasttimestamp = value
            dte = abs(dt - 1.0/float(rate(self.conf)))
            if dte > .05:
                self.history.data = []
                return

            for s in self.conf['sensors']:
                if not s in self.inputs:
                    print('missing input', s)
                    return

            self.history.put(self.inputs)
            self.train()

    def receive(self):
        msg = self.client.receive_single(1)
        while msg:
            name, value = msg
            self.receive_single(name, value)
            msg = self.client.receive_single(-1)

    def run_replay(self, filename):
        try:
            f = open(filename)
            print('opened replay file', filename)
            while True:
                line = f.readline()
                if not line:
                    f.close()
                    return True
                intellect.receive(json.loads(line))
        except Exception as e:
            return False
            
    def run(self):
      # ensure we sample all predictions
      for p in self.conf['predictions']:
          if not p in self.conf['sensors']:
              print('adding prediction', p)
              self.conf['sensors'].append(p)
      
      host = 'localhost'
      if len(sys.argv) > 1:
          if self.run_replay(sys.argv[1]):
              return
          host = sys.argv[1]
      # couldn't load try to connect
      watches = self.conf['sensors'] + list(self.conf['state'])
      watches.append('ap.enabled')
      watches.append('timestamp')
      def on_con(client):
        for name in watches:
          client.watch(name)

      t0 = time.time()

      self.client = False
      while True:
          #try:
          if 1:
              if not self.client:
                  print('connecting to', host)
                  self.client = SignalKClient(on_con, host, autoreconnect=False)
              self.receive()
          #except Exception as e:
          #    print('error', e)
          #    self.client = False
          #    time.sleep(1)
              
          if time.time() - t0 > 600:
              filename = os.getenv('HOME')+'/.pypilot/intellect_'+self.conf['mode']+'.conf'
              self.save(filename)
              
          # find cpu usage of training process
          #cpu = ps.cpu_percent()
          #if cpu > 50:
          #    print('learning cpu very high', cpu)
          
def main():
    intellect = Intellect()
    intellect.run()

if __name__ == '__main__':
    main()
