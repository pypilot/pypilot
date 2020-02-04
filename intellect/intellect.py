#!/usr/bin/env python
#
#   Copyright (C) 2019 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

pool_size = 100 # how much data to accumulate before training
from pypilot.pilots.learning import History

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
    self.history = History(meta)
    self.models = {}
    self.meta = {'past': 10, # seconds of sensor data
                 'future': 3, # seconds to consider in the future
                 'sensors': ['imu.accel', 'imu.gyro', 'imu.heading', 'imu.headingrate', 'servo.current', 'servo.command']
                 'actions':  ['servo.command'],
                 'predictions': ['imu.heading', 'imu.headingrate']}
    self.states = {'ap.enabled': False,
                   'ap.mode': 'none',
                   'imu.rate': 0}
                 
  def load(self, mode):
      model = build(self.meta)
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
    
    meta = {'sensors': sensor, 'actions': actions, 'rate': rate, 'mode': self.mode
            'predictions': predictions, 'past': past, 'future': future)
    if not self.model or self.model.meta == meta:
        self.model = self.build(meta)
        self.train_x, self.train_y = [], []
    
    self.train_x.append(sensors_data + actions_data)
    self.train_y.append(predictions_data)

    if len(self.train_x) >= pool_size:        
        self.model.fit(train_x, train_y, epochs=4)
        self.train_x, self.train_y = [], []

    def build(self, meta):
        input_size = meta['rate']*(meta['past']*len(meta['sensors']) + meta['future']*len(meta['actions')))
        output_size = meta['rate']*meta['future']*len(meta['predictions'])
        input = tf.keras.layers.Input(shape=(input_size,), name='input_layer')
        hidden = tf.keras.layers.Dense(16*output_size, activation='relu')(input)
        output = tf.keras.layers.Dense(output_size, activation='tanh')(hidden)
        self.model = tf.keras.Model(inputs=input, outputs=output)
        self.model.compile(optimizer='adam', loss='mean_squared_error', metrics=['accuracy'])
        self.model.meta = meta

    def save(self, filename):
        converter = tf.lite.TFLiteConverter.from_keras_model(self.model)
        tflite_model = converter.convert()
        try:
          import json
          f = open(filename, 'w')
          meta['model_filename'] = filename + '.tflite_model'
          f.write(json.dumps(meta))
          f.close()
          f = open(meta['model_filename'], 'w')
          f.write(tflite_model)
          f.close()
        except Exception as e:
          print('failed to save', f)

      def reset_receive(self):
        self.inputs = {}

      def receive_single(self, name, value):
          if name in self.states:
              self.state[name] value
              
          current_timestamp = 0
          rate = 0
          if name in self.meta['sensors'] and self.states['enabled']:
            
              # ensure inputs are in order
              for n in sensors:
                  if n == name:
                      break
              if not inputs[n]:
                inputs = {}
            inputs[name] = value

            # skipped data, clear history and current input
            if msg['timestamp'] - current_timestamp > .1:
              inputs = {}
              history.data = []

                # see if we have all sensor values, and if so store in the history
                if all(map(lambda sensor : sensor in inputs, sensors)):                  
                    self.history.put(inputs)
                    self.train()
                    inputs = {}
                current_timestamp = msg['timestamp']


      def recieve(self, msg):
        for name in msg:
          self.recieve_single(name, msg[name]['value'])
            
    def run_replay(self, filename):
        try:
            f = open(sys.argv[1])
            while True:
                line = f.readline()
                if not line:
                    f.close()
                    return True
                intellect.receive(json.loads(line))
        except Exception as e:
            return False
            
    def run(self):
      if len(sys.argv) > 1:
          if run_replay(sys.argv[1]):
              return
          # couldn't load try to connect
      watches = sensors + list(self.states)
      self.client = SignalKClientFromArgs(sys.argv, watches)
      t0 = time.time()

      while True:
          msg = client.receive_single(1)
          if msg:
              intellect.receive(msg)
              
          if time.time() - t0 > 600:
              filename = os.getenv('HOME')+'/.pypilot/intellect_'+self.meta['mode']+'.conf'
              self.save(filename)
              
          # find cpu usage of training process
          cpu = ps.cpu_percent()
          if cpu > 50:
              print('learning cpu very high', cpu)
          
def main():
    intellect = Intellect()
    intellect.run()

if __name__ == '__main__':
    main()
