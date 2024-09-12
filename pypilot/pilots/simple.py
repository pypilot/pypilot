#!/usr/bin/env python
#
#   Copyright (C) 2017 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

from pilot import AutopilotPilot
disabled = True # IMPORTANT:  remove this line enable the simple pilot

class SimplePilot(AutopilotPilot):
    def __init__(self, ap):
        super(SimplePilot, self).__init__('simple', ap)

        # create simple pid filter
        self.Gain('P', .005, 0, .025)
        self.Gain('I', 0, 0, .05)
        self.Gain('D', .15, 0, .5)

    def process(self, reset):
        ap = self.ap
        headingrate = ap.boatimu.SensorValues['headingrate_lowpass'].value
        gain_values = {'P': ap.heading_error.value,
                       'I': ap.heading_error_int.value,
                       'D': headingrate}

        command = self.Compute(gain_values)

        if ap.enabled.value:
            ap.servo.command.command(command)

pilot = SimplePilot
