#!/usr/bin/env python
# Copyright 2025 Domink Röttsches
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
remote.py

The evdev_remote module provides a command-line tool for interfacing keyboard-like
input devices with the pypilot autopilot.
Devices can be connected through any means that the Linux input event
evdev kernel subsystem supports.
This is commonly USB or Bluetooth, or Bluetooth Low Energy (BLE).

(Any devices listed as an evdev device that produces key events should work, too,
but make sure the selected keyboard device is not your main keyboard,
as it will be grabbed and not be available for typing.)

It listens for key events from selected input devices , maps them to autopilot actions,
and communicates these actions to pypilot.

For customization, users can modify the `KEY_ACTION_MAP` dictionary to change
the mapping of key events to actions. Run the script to see the generated key events.

When using a Bluetooth device, make sure that the device is connected and
paired with the system before running this script.
(For example, using `bluetoothctl` to pair, connect to and trust the device,
 or using the GUI Bluetooth manager on your system.)

Example devices that this was tested with:
* Bluetooth "Media Button" remote controls, showing up with BLE name "SmartRemote":
  * https://www.aliexpress.com/item/1005007423472957.html
  * https://www.aliexpress.com/item/1005007811649260.html
* Apple Bluetooth keyboard
* Xbox game controller

Run the script with the path to the device as an argument,
from the pypilot top level directory:
```bash
$ python3 -m pypilot.evdev_remote 4c:5e:a5:86:19:02
```

Or run it with multiple devices, using udev device paths:
```bash
$ python3 -m pypilot.evdev_remote /dev/input/event7 4c:5e:a5:86:19:02
```

Or leave the argument empty to select one device interactively.

Further info:
* Documentation for evdev input device handling https://python-evdev.readthedocs.io/en/latest/
* Short demo video https://youtu.be/LODzTn3Ht4Q
"""

import asyncio
import sys
import threading


from evdev import categorize, ecodes, InputDevice

from .device_selection import select_device, select_device_interactive
from .pypilot_state import Action, pypilot_comms_loop, queue_action
from evdev.events import KeyEvent
from .log_config import logger

KEY_ACTION_MAP = {
    ecodes.KEY_UP: Action.PLUS_ONE,
    ecodes.KEY_DOWN: Action.MINUS_ONE,
    ecodes.KEY_SLASH: Action.PLUS_FIVE,
    ecodes.KEY_DOT: Action.MINUS_FIVE,
    ecodes.KEY_ENTER: Action.TOGGLE,
    ecodes.KEY_SPACE: Action.TOGGLE,
    # SmartRemote
    ecodes.KEY_PLAYPAUSE: Action.TOGGLE,
    ecodes.KEY_PREVIOUSSONG: Action.MINUS_ONE,
    ecodes.KEY_NEXTSONG: Action.PLUS_ONE,
    ecodes.KEY_VOLUMEUP: Action.PLUS_FIVE,
    ecodes.KEY_VOLUMEDOWN: Action.MINUS_FIVE,
    # Long right key press
    # ecodes.KEY_POWER: Action.TOGGLE,
    # Long left key press
    # Example: XBOX game controller
    ecodes.BTN_B: Action.PLUS_ONE,
    ecodes.BTN_X: Action.MINUS_ONE,
    ecodes.BTN_TR: Action.PLUS_FIVE,
    ecodes.BTN_TL: Action.MINUS_FIVE,
    ecodes.BTN_THUMBR: Action.TOGGLE,
}


def start_pypilot_comms():
    # Start the pypilot communication loop in a separate thread
    comms_thread = threading.Thread(target=pypilot_comms_loop, daemon=True)
    comms_thread.start()


async def grab_and_handle_events_for_device(device: InputDevice):
    device.grab()
    logger.debug(f"Device {device.name} {device.path} {device.uniq} grabbed.")
    async for event in device.async_read_loop():
        key_event = categorize(event)
        if not isinstance(key_event, KeyEvent):
            continue
        logger.info(
            f"Key: {key_event.keycode}, State: {key_event.keystate} from {device.name}"
        )
        if key_event.keystate == 0:
            keycode = key_event.keycode
            action = None
            # For some devices, such as game controllers the keycode is a tuple or list.
            if isinstance(keycode, (list, tuple)):
                codes = [ecodes.ecodes.get(k) for k in keycode]
            else:
                codes = [ecodes.ecodes.get(keycode)]
            action = next(
                (KEY_ACTION_MAP.get(code) for code in codes if code in KEY_ACTION_MAP),
                None,
            )
            if action:
                queue_action(action)


def main() -> int:
    selected_devices: list[InputDevice | None] = []
    if len(sys.argv) > 1:
        selected_devices = [select_device(arg) for arg in sys.argv[1:]]
    else:
        selected_devices = [select_device_interactive()]

    valid_devices = [device for device in selected_devices if device is not None]
    if not valid_devices:
        logger.error(
            "One or more devices could not be found. Please check the device paths."
        )
        return 1

    logger.info("Listening for events. Press Ctrl+C to stop.")
    start_pypilot_comms()

    try:
        loop = asyncio.get_event_loop()
        tasks = [
            loop.create_task(grab_and_handle_events_for_device(device))
            for device in valid_devices
        ]
        loop.run_until_complete(asyncio.wait(tasks))
    except KeyboardInterrupt:
        logger.info("\nStopped listening for events.")
    except Exception as e:
        logger.info(f"An error occurred: {e}")
    return 0


if __name__ == "__main__":
    main()
