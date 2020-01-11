#!/usr/bin/env python
#
#   Copyright (C) 2019 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import os, sys, time
import tensorflow as tf
from signalk.client import SignalKClient

class History(object):
  def __init__(self, conf):
    self.conf = conf
    self.data = []

  def samples(self):
    return (self.conf['past']+self.conf['future'])*self.conf['rate']

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
        data = []
        for value in values:
            data += flatten(value)
        return data
    return flatten(map(lambda input : select(input, names), history))
  
class Intellect(object):
    def __init__(self):
        self.train_x, self.train_y = [], []
        self.inputs = {}
        self.models = {}
        self.conf = {'past': 10, # seconds of sensor data
                 'future': 3, # seconds to consider in the future
                 'sensors': ['imu.accel', 'imu.gyro', 'imu.heading', 'imu.headingrate', 'servo.current', 'servo.command'],
                 'actions':  ['servo.command'],
                 'predictions': ['imu.heading', 'imu.headingrate']}
        self.state = {'ap.enabled': False,
                      'ap.mode': 'none',
                      'imu.rate': 1}
        self.history = History(self.conf)

        self.last_timestamp = {}
        for name in self.conf['sensors']:
            self.sensor_timestamps[name] = 0

    def load(self, mode):
        model = build(self.conf)
        try:
            self.model.load_weights('~/.pypilot/intellect')
        except:
            return model
  
    def train(self):
        if len(self.history.data) != self.history.samples:
            return # not enough data in history yet
        present = rate*past
        # inputs are the sensors over past time
        sensors_data = inputs(self.history.data[:present], sensors)
        # and the actions in the future
        actions_data = inputs(self.history.data[present:], actions)
        # predictions in the future
        predictions_data = inputs(self.history.data[present:], predictions)
    
        conf = {'sensors': sensor, 'actions': actions, 'rate': rate, 'mode': self.mode,
                'predictions': predictions, 'past': past, 'future': future}
        if not self.model or self.model.conf == conf:
            self.model = self.build(conf)
            self.train_x, self.train_y = [], []
    
        self.train_x.append(sensors_data + actions_data)
        self.train_y.append(predictions_data)

        pool_size = 100 # how much data to accumulate before training
        if len(self.train_x) >= pool_size:        
            self.model.fit(train_x, train_y, epochs=4)
            self.train_x, self.train_y = [], []

    def build(self, conf):
        input_size = conf['rate']*(conf['past']*len(conf['sensors']) + conf['future']*len(conf['actions']))
        output_size = conf['rate']*conf['future']*len(conf['predictions'])
        input = tf.keras.layers.Input(shape=(input_size,), name='input_layer')
        hidden = tf.keras.layers.Dense(16*output_size, activation='relu')(input)
        output = tf.keras.layers.Dense(output_size, activation='tanh')(hidden)
        self.model = tf.keras.Model(inputs=input, outputs=output)
        self.model.compile(optimizer='adam', loss='mean_squared_error', metrics=['accuracy'])
        self.model.conf = conf

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
        if name in self.state:
            self.state[name] = value
            return

        if name in self.conf['sensors'] and (1 or self.state['ap.enabled']):
            timestamp = msg['timestamp']

            dt = timestamp - self.sensor_timestamps[name]
            dte = abs(dt - 1.0/float(self.state['imu.rate']))
            if dte > .05:
                self.history.data = []
                self.inputs = {}
                return

            if name in self.inputs:
                print('input already for', name, self.inputs[name], name, timestamp)

            self.inputs[name] = value
            # see if we have all sensor values, and if so store in the history
            if all(map(lambda sensor : sensor in inputs, sensors)):                  
                s = ''
                for name in inputs:
                    s += name + ' ' + inputs[name]
                    print('input', time.time(), s)
                self.history.put(inputs)
                self.train()
                self.inputs = {}

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
      host = 'localhost'
      if len(sys.argv) > 1:
          if self.run_replay(sys.argv[1]):
              return
          host = sys.argv[1]
      # couldn't load try to connect
      watches = self.conf['sensors'] + list(self.state)
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
