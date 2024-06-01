# PyPilot - Open Source Marine Autopilot

## Installation

Clone the repository by running the following command:

`git clone https://github.com/pypilot/pypilot`

### Dependencies

You could install the dependencies with the following command:

`sudo python3 setup.py install`

### Configuration

You may want to run pypilot as a service, see the scripts/debian directory

## Usage

Most of the scripts can be run individually as standalone 
or test programs, some function as clients, other as servers

### servers (only one executes at a time)

These server scripts can be run as tests:
* execute this script directly

```pypilot -- autopilot```

instead of running the complete autopilot these scripts provide a server with specific functionallity.

```pypilot_boatimu```    -- imu specific to boat motions
                      includes automatic 2d/3d calibration and alignment of magnetic sensors
                      * useful for testing the imu (gyros) or even just reading gyros
                      
```pypilot_sensors```    -- test sensor inputs only
                       reads nmea0183 from serial ports or from tcp connections, and multiplexes
                       the output to both nmea0183.
                       listed on tcp port 20220 by default
                       * convert and multiplex nmea0183 data

```pypilot_servo```   --   use to test or verify a working motor controller is detected,
                      can be used to control and calibrate the servo

### clients (run as many of these to connect to a server):

```pypilot_control``` -- simple interface to command autopilot

```pypilot_calibration``` -- interactive gui for all autopilot calibrations

```pypilot_kivy``` -- work in progress kivy control app

```pypilot_client_wx``` -- graphical client (wx widgets)

```pypilot_scope``` -- plot client with wx widgets (for checked listbox)

```pypilot_client``` -- console client

```pypilot_hat``` -- autopilot control interface using GPIO pins, with web configuration on port 33333

```pypilot_web``` -- python flask application for browser autopilot control
