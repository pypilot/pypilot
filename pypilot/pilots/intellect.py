#!/usr/bin/env python
#
#   Copyright (C) 2019 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import os, sys, time, math, json
import struct, lzma
from pypilot.client import pypilotClient

class stopwatch(object):
    def __init__(self):
        self.total = 0
        self.starttime = False
    def start(self):
        self.starttime = time.time()
    def stop(self):
        self.total += time.time() - self.starttime
    def time(self):
      if not self.starttime:
        return 0
      return self.total + time.time() - self.starttime

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

class records(object):
    def __init__(self):
        self.d = {}

    def id(self, name, record_file):
        if not name in self.d:
            i = len(self.d)
            hdr = struct.pack('hhh', 0, i, len(name))
            record_file.write(str(hdr) + name)
            self.d[name] = i
        return self.d[name]
    
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
    def __init__(self, host):
        self.host = host
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
        self.record_file = False
        self.records = records()
        self.playback_file = False

        self.loading = stopwatch()
        self.fitting = stopwatch()
        self.totaltime = stopwatch()
        self.totaltime.start()
        
    def load(self, mode):
        model = build(self.conf)
        try:
            self.model.load_weights('~/.pypilot/intellect')
        except:
            return model
  
    def train(self):
        if len(self.history.data) != self.history.samples():
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
            self.loading.start()
            self.build(len(self.train_x[0]), len(self.train_y[0]))
            self.loading.stop()

        pool_size = 6000 # how much data to accumulate before training
        l = len(self.train_x)
        if l < pool_size:
            if l%100 == 0:
                sys.stdout.write('pooling... ' + str(l) + '\r')
                sys.stdout.flush()
            return
        print('fit', len(self.train_x), len(self.train_x[0]), len(self.train_y), len(self.train_y[0]))
        #print('trainx', self.train_x[0])
        #print('trainy', self.train_y[0])
        self.fitting.start()
        history = self.model.fit(self.train_x, self.train_y, epochs=8)
        self.fitting.stop()
        mse = history.history['mse']
        print('mse', mse)
        self.train_x, self.train_y = [], []

    def build(self, input_size, output_size):
        conf = self.conf
        print('loading...')
        import tensorflow as tf
        print('building...')
        input = tf.keras.layers.Input(shape=(input_size,), name='input_layer')
        #hidden1 = tf.keras.layers.Dense(256, activation='relu')(input)
        hidden2 = tf.keras.layers.Dense(16, activation='relu')(input)
        output = tf.keras.layers.Dense(output_size, activation='tanh')(hidden2)
        self.model = tf.keras.Model(inputs=input, outputs=output)
        self.model.compile(optimizer='adam', loss='mean_squared_error', metrics=['mse'])

    def save(self, filename):
        converter = tf.lite.TFLiteConverter.from_keras_model(self.model)
        tflite_model = converter.convert()
        try:
          f = open(filename, 'w')
          conf['model_filename'] = filename + '.tflite_model'
          f.write(json.dumps(conf))
          f.close()
          f = open(conf['model_filename'], 'w')
          f.write(tflite_model)
          f.close()
        except Exception as e:
          print('failed to save', f)

    def receive_single(self, name, value):
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
                if(dtl-dt > 10.0):
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
        if self.playback_file:
            line = self.playback_file.readline()
            if not line:
                print('end of file')
                exit(0)
            msg = json.loads(line)
            for name in msg:
                self.receive_single(name, msg[name])
            return

        self.client.poll(1)
        msg = self.client.receive_single()
        while msg:
            name, data = msg
            value = data['value']
            if self.record_file:
                self.record_data(name, value)
            else:
                self.receive_single(name, value)
            msg = self.client.receive_single(-1)

    def record_data(self, name, value):
        self.record_file.write(json.dumps({name: value}))
        self.record_file.write('\n')
        return

        i = self.records.id(value, self.record_file)
        data = struct.pack('h', i)
        if type(value) == type(''):
            data += struct.pack('hh', 0, self.records.id(value, self.record_file))
        elif type(value) == type(1.0):
            data += struct.pack('hf', 1, value)
        elif type(value) == type([]):
            if not all(map(lambda x : type(x) == type(float()), value)):
                print('unhandled type in array for storage', value)
            else:
                data += struct.pack('h'+str(len(value))+'f', len(value), *value)
        else:
            print('unhandled storage for type', value)
            data = ''
                    
        self.record_file.write(str(data))
        self.record_file.lines += 1
        if self.record_file.lines%100 == 0:
            sys.stdout.write('recording ' + str(self.record_file.lines) + '\r')
            sys.stdout.flush()

    def record(self, filename):
        try:
            #self.record_file = lzma.open(filename, 'wt', encoding='ascii')
            self.record_file = open(filename, 'w')
            self.record_file.lines = 0
        except Exception as e:
            print('unable to open for recording', filename, e)

    def playback(self, filename):
        try:
            self.playback_file = open(filename, 'rb')
        except Exception as e:
            print('failed to open replay file', filename, e)

    def run(self):
      from signal import signal
      def cleanup(a, b):
          print('time spent loading', self.loading.time())
          print('time spent fitting', self.fitting.time())
          print('time spent total', self.totaltime.time())
          exit(0)
      signal(2, cleanup)
      # ensure we sample all predictions
      for p in self.conf['predictions']:
          if not p in self.conf['sensors']:
              #print('adding prediction', p)
              self.conf['sensors'].append(p)
      
      t0 = time.time()

      print('connecting to', self.host)
      self.client = pypilotClient(self.host)

      watches = self.conf['sensors'] + list(self.conf['state']) + 'ap.enabled' + 'timestamp'
      for name in watches:
          client.watch(name)

      while True:
          self.receive()
              
          if time.time() - t0 > 600:
              filename = os.getenv('HOME')+'/.pypilot/intellect_'
              state = self.conf['state']
              for n in state:
                  filename += '_' + state[n]
              filename += '.conf'
              self.save(filename)
              
          # find cpu usage of training process
          #cpu = ps.cpu_percent()
          #if cpu > 50:
          #    print('learning cpu very high', cpu)

def main():
      
    try:
        import getopt
        args, host = getopt.getopt(sys.argv[1:], 'p:r:h')
        if host:
            host = host[0]
        else:
            host = 'localhost'
    except Exception as e:
        print('failed to parse command line arguments:', e)
        return

    intellect = Intellect(host)
    for arg in args:
        name, value = arg
        if name == '-h':
            print(sys.argv[0] + ' [ARGS] [HOST]\n')
            print('-p filename -- playback from filename instead of live')
            print('-r filename -- record to file data for playback, no processing')
            print('-h          -- Display this message')
            return
        elif name == '-p':
            intellect.playback(value)
        elif name == '-r':
            intellect.record(value)
  
    intellect.run()

if __name__ == '__main__':
    main()
