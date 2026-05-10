# PyPilot - Open Source Marine Autopilot

## Installation

Clone the repository by running the following command:

```
git clone https://github.com/pypilot/pypilot
```

### Dependencies (on raspberry pi)

```
sudo apt install gettext libgpiod-dev
```

### Build and Install
```
cd pypilot
sudo pip install .[optimze,ui,hat,web] --break-system-packages
```

### Install data (fonts and 3d boat rendering, optional but nice)

git clone https://github.com/pypilot/pypilot_data
cd pypilot_data
sudo pip install . --break-system-packages


### Configuration

You may want to run pypilot as a service, see the scripts/debian directory, eg:
sudo cp -rv scripts/debian/etc/systemd /etc
sudo systemctl daemon-reload

## Usage

Most of the scripts can be run individually as standalone 
or test programs, some function as clients, other as servers

### servers (only one executes at a time)

These server scripts can be run as tests:
* execute this script directly

`pypilot -- autopilot`

instead of running the complete autopilot these scripts provide a server with specific functionallity.

`pypilot_boatimu`    -- imu specific to boat motions
                      includes automatic 2d/3d calibration and alignment of magnetic sensors
                      * useful for testing the imu (gyros) or even just reading gyros
                      
`pypilot_servo`   --   use to test or verify a working motor controller is detected,
                      can be used to control and calibrate the servo

### clients (run as many of these to connect to a server):

`pypilot_control` -- simple interface to command autopilot

`pypilot_calibration` -- interactive gui for all autopilot calibrations

`pypilot_client_wx` -- graphical client (wx widgets)

`pypilot_scope` -- plot client with wx widgets (for checked listbox)

`pypilot_client` -- console client

`pypilot_hat` -- autopilot control interface using GPIO pins, with web configuration on port 33333

`pypilot_web` -- python flask application for browser autopilot control
