#!/usr/bin/env python
#
#   Copyright (C) 2020 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import os, sys, time, math, json
import lzma
from pypilot.client import pypilotClient


'''
eye:

64x64x64                     2e29
cnn 4x4x4x32              2e11 2e11-2e13
max poll 16x16x16            2e25
cnn 4x4x4x128             2e13 2e18
max poll 4x4x4               2e21
cnn 4x4x4x512             2e14 2e22
max poll 1x1x1               2e20
dense 2048                   2e19
dense 256                    2e12
dense 16                     


cnn 4x4x4x512                2e15
max poll 4x4x4               2e18
flatten 4x4x4x1024           2e
dense 256                    2e18
dense 64                     2e14
dense 16                     2e10



256x256x64                    67108864
cnn 2x2x4                    16
max poll 128x128x32x4          67108864
cnn 2x2x8                    32
max poll 64x64x16x8            33554432
cnn 2x2x16                   64
max pool 32x32x8x16           16777216
cnn 2x2x32                   128
max poll 16x16x4x32           8388608
cnn 2x2x64                   256
max poll 8x8x2x64             4194304
cnn 2x2x128                  512
max poll 4x4x1x128            2097152
cnn 2x2x256                  1024
max poll 2x2x256              2097152
cnn 2x2x512                  2048
max poll 512                  65536   
dense    128                 65536
dense 8                      1024

flatten 1024
dense 256                     262144
dense 32                      8192
dense 8





can load and select between different models

uses compute nodes to send distributed learning jobs


model predicts future state
inputs - wind speed imu servo camera, future commands
outputs - boat heading, power consumed,  errors for all outputs


predict error of output
measure output error: if output error < error, reduce error slightly, otherwise increase error by a larger factor
predict output error: use smaller loss factor if < error, otherwise use larger loss

optimal trajectory is differential equation




support multiple models

tensorboard for analysis

if logging enabled, hdf5 format to store union of inputs and predictions compressed size per day?
   store in .pypilot with maximum total data size 4gb?   Make sure do not fill disk completely.


new layer type in tensorflow which uses parameters to change weights, otherwise it is same as dense
   can you embed model in model?   Can model do this rather than layer?
   seastate (multi dimensional) -  wind speed, wind direction, water speed (or gps speed), accelerometer/gyro frequency/amplitude??
   light (average intensity) for eye

language to define pypilot models??

models which learn to steer from limited inputs, for example, eye only

models include:
    tensor flow model architecture, knows if online/offline, some layers may only train offline
    filename calculation to store/load weights/bias from file, this file should include what data already trained it to avoid re-training
    function for optimal trajectory, or the function used to predict the output from the model which may be another tensorflow model
    data showing accuracy

visualizations:
    hyperspace of explored sea states in the model
    the realtime/replay plot of intended trajectory, actual trajectory (in playback), and the various predicted trajectories
    for each prediction variable

predictions may include wattage, wind direction, wind speed etc...

main proccess learning pilot:
    executes calculation for current model from inputs
    send all inputs and predictions to ai process

separate ai process includes
    settings for:
          ncpus to use (all idle priority)
          status to indicate if sufficient processing power, if keeping up with all models, only active or not keeping up
          max log size
          
    receive inputs and predictions
    log data to disk for future offline processing
    train active/enabled models

separate program
    to train all models from log data
    support tensorboard from here

'''



def model_filename(state):
  filename = os.getenv('HOME')+'/.pypilot/model_'
  for name, value in state.items():
      filename += '_' + str(value)
  return filename

class stopwatch(object):
    def __init__(self):
        self.total = 0
        self.starttime = False

    def start(self):
        self.starttime = time.monotonic()

    def stop(self):
        self.total += time.monotonic() - self.starttime

    def time(self):
      if not self.starttime:
          return 0
      return self.total + time.monotonic() - self.starttime


class History(object):
    def __init__(self, conf, state, future=False):
        future = self.conf['future'] if future else 0
        dt = (self.conf['past']+future)*self.state['imu.rate']
        self.samples = int(math.ceil(dt))
        self.data = []

    def put(self, data):
        if self.full():
            self.data = self.data[1:]
        self.data.append(data)

    def clear(self):
        self.data = []

    def full(self):
        return len(self.data) == self.samples


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



# normalize a sensor input to -1 to 1 range
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


class Model(object):
    def __init__(self):
        self.history = False

    def present(self):
        return self.conf['state']['imu.rate']*self.conf['past']

    def receive(self, name, value):
        if name in self.conf['sensors'] and self.enabled:
            self.inputs[name] = norm_sensor(name, value)
    

class KerasModel(Model):
    def __init__(self, host):
        super(KerasModel, self).__init__()
        self.host = host
        self.train_x, self.train_y = [], []
        self.inputs = {}
        self.conf = {'past': 5, # seconds of sensor data
                     'future': 2, # seconds to consider in the future
                     'sensors': ['imu.accel', 'imu.headingrate', 'servo.current', 'servo.command'],
                     'actions': ['servo.command'],
                     'predictions': ['ap.heading_error', 'imu.headingrate_lowpass'],
                     'state': ['ap.mode', 'imu.rate']}
        self.state = {}
        self.ap_enabled = False
        self.lasttimestamp = 0
        self.firsttimestamp = False
        self.record_file = False
        self.playback_file = False

        self.load_time = stopwatch()
        self.fit_time = stopwatch()
        self.total_time = stopwatch()
        self.total_time.start()
        
    def train(self):
        if len(self.history.data) != self.history.samples():
            return # not enough data in history yet

        p = self.present()
        # inputs are the sensors over past time
        sensors_data = inputs(self.history.data[:p], self.conf['sensors'])
        # and the actions in the future
        actions_data = inputs(self.history.data[p:], self.conf['actions'])
        # predictions in the future
        predictions_data = inputs(self.history.data[p:], self.conf['predictions'])
    
        if not self.model:
            self.train_x, self.train_y = [], []

        inputs = sensors_data + actions_data
        self.train_x.append(inputs)
        self.train_y.append(predictions_data)

        if not self.model:
            self.load_time.start()
            self.build(len(self.train_x[0]), len(self.train_y[0]))
            try:
                self.model.load_weights(model_filename(self.state)+'model')
            except:
                print('failed to load model, starting from new')
            self.load_time.stop()

        predict = self.model.predict(inputs)
        pl = len(self.conf['predictions'])
        # compare predict to predictions_data to compute accuracy for each output
        a = [1]*pl
        accuracy = []
        for i in range(0, len(predict), pl):
            for j in range(pl):
                computed = predict[i+j]
                measured = predictions_data[i+j]
                square_error = (computed-measured)**2
                a[j] *= max(1-square_error, 0)
                lp = .01
                accuracy = self.conf['accuracy']
                accuracy[i][j] = accuracy[i][j]*(1-lp) + a[j]*lp
            
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
        self.fit_time.start()
        history = self.model.fit(self.train_x, self.train_y, epochs=8)
        self.fit_time.stop()
        mse = history.history['mse']
        print('mse', mse)
        self.train_x, self.train_y = [], []

    def build(self, input_size, output_size):
        conf = self.conf
        print('load_time...')
        import tensorflow as tf
        print('building...')
        input = tf.keras.layers.Input(shape=(input_size,), name='input_layer')
        #hidden1 = tf.keras.layers.Dense(256, activation='relu')(input)
        hidden2 = tf.keras.layers.Dense(16, activation='relu')(input)
        output = tf.keras.layers.Dense(output_size, activation='tanh')(hidden2)
        self.model = tf.keras.Model(inputs=input, outputs=output)
        self.model.compile(optimizer='adam', loss='mean_squared_error', metrics=['mse'])
        #self.accuracy = 

    def save(self):
        filename = learning.model_filename(self.state)
        converter = tf.lite.TFLiteConverter.from_keras_model(self.model)
        tflite_model = converter.convert()
        try:
            f = open(filename + 'conf', 'w')
            f.write(json.dumps(self.conf))
            f.close()
            f = open(filename + '.tflite_model', 'w')
            f.write(tflite_model)
            f.close()
        except Exception as e:
            print('failed to save', f)

    def receive_single(self, name, value):
        if name == 'ap.enabled':
            if self.ap_enabled != value:
                self.model.history = False
                self.ap_enabled = value
        elif name in self.conf['state']:
            if self.state[name] != value:
                print('state changed:', self.state)
                self.state[name] = value
                self.model = False
        elif name == 'timestamp':
            t0 = time.monotonic()
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
                self.history.clear()
                return

            for s in self.conf['sensors']:
                if not s in self.inputs:
                    print('missing input', s)
                    return

            if not self.history:
                self.history = History(self.conf, self.state, True)
            self.history.put(self.inputs)
            self.train()
        else:
            self.model.data(name, value)                

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
        for name, value in self.client.received:
            name, value = msg
            if self.record_file:
                self.record_data(name, value)
            else:
                self.receive_single(name, value)
        self.client.received = []

    def record_data(self, name, value):
        self.record_file.write(json.dumps({name: value}))
        self.record_file.write('\n')

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
            print('time spent load time', self.load_time.time())
            print('time spent fit time', self.fit_time.time())
            print('time spent total', self.total_time.time())
            exit(0)
        signal(2, cleanup)

        # add predictions to the list of sensors
        for p in self.conf['predictions']:
            if not p in self.conf['sensors']:
                self.conf['sensors'].append(p)
      
        t0 = time.monotonic()

        print('connecting to', self.host)
        self.client = pypilotClient(self.host)

        watches = self.conf['sensors'] + self.conf['state'] + 'ap.enabled' + 'timestamp'
        for name in watches:
            client.watch(name)

        while True:
            self.receive()
              
            if time.monotonic() - t0 > 600:
                self.save()
              
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

    kerasmodel = Kerasmodel(host)
    for arg in args:
        name, value = arg
        if name == '-h':
            print(sys.argv[0] + ' [ARGS] [HOST]\n')
            print('-p filename -- playback from filename instead of live')
            print('-r filename -- record to file data for playback, no processing')
            print('-h          -- Display this message')
            return
        elif name == '-p':
            kerasmodel.playback(value)
        elif name == '-r':
            kerasmodel.record(value)
  
    kerasmodel.run()

if __name__ == '__main__':
    main()
