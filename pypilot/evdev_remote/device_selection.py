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

from evdev import InputDevice, list_devices


def select_device(device_spec) -> InputDevice | None:
    """
    Selects a device based on the provided path or MAC address.
    """
    try:
        device: InputDevice | None = None
        if "/" in device_spec:
            device = InputDevice(device_spec)
        else:
            devices = [InputDevice(path) for path in list_devices()]
            device = next((dev for dev in devices if dev.uniq == device_spec), None)
            if device is None:
                raise FileNotFoundError(f"No device with uniq '{device_spec}' found.")
        return device
    except FileNotFoundError:
        print(f"Device not found: {device_spec}")
        return None


def select_device_interactive() -> InputDevice | None:
    """
    Interactively select a device from the list of available devices.
    """
    devices = [InputDevice(path) for path in list_devices()]
    print("Available devices:")
    for idx, device in enumerate(devices):
        print(f"{idx}: {device.name} (Path: {device.path}, Unique ID: {device.uniq})")

    try:
        device_index = int(input("Enter the number of the device you want to select: "))
        if 0 <= device_index < len(devices):
            return devices[device_index]
        else:
            print("Invalid number. Please try again.")
            return None
    except ValueError:
        print("Invalid input. Please enter a number.")
        return None
