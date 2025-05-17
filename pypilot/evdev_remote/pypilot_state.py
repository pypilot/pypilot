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


from enum import Enum

from pypilot.client import pypilotClientFromArgs
from queue import Queue, Empty
from typing import TypedDict
from .log_config import logger

HOST = "localhost"
PORT = 23322


class APState(TypedDict):
    ap_enabled: bool
    ap_heading_command: float
    ap_heading: float
    imu_heading: float
    ap_mode: str


ap_state: APState = {
    "ap_enabled": False,
    "ap_heading_command": 0,
    "ap_heading": 0,
    "imu_heading": 0,
    "ap_mode": "",
}

send_queue: Queue = Queue()

HEADING_STATE_CHANGE_THRESHOLD = 1  # degrees


def state_changed_significantly(old_ap_state: APState, new_ap_state: APState) -> bool:
    return (
        abs(new_ap_state["imu_heading"] - old_ap_state["imu_heading"])
        >= HEADING_STATE_CHANGE_THRESHOLD
        or abs(new_ap_state["ap_heading"] - old_ap_state["ap_heading"])
        >= HEADING_STATE_CHANGE_THRESHOLD
        or new_ap_state["ap_enabled"] != old_ap_state["ap_enabled"]
        or abs(new_ap_state["ap_heading_command"] - old_ap_state["ap_heading_command"])
        >= HEADING_STATE_CHANGE_THRESHOLD
    )


def update_state_from_readings(datum: str, value: str | float | bool):
    old_ap_state = ap_state.copy()
    if datum == "ap.heading":
        ap_state["ap_heading"] = float(value)
    elif datum == "ap.enabled":
        ap_state["ap_enabled"] = bool(value)
    elif datum == "imu.heading":
        ap_state["imu_heading"] = float(value)
    elif datum == "ap.heading_command":
        ap_state["ap_heading_command"] = float(value)
    elif datum == "ap.mode":
        ap_state["ap_mode"] = str(value)
    if state_changed_significantly(old_ap_state, ap_state):
        logger.debug(f"AP State changed: {ap_state}")


def send_from_queue(client):
    try:
        pypilot_write_msg = send_queue.get_nowait()
        logger.debug(f"Retrieved queue item: {pypilot_write_msg}")
        client.send(pypilot_write_msg.decode())
        logger.debug(f"Sent to pypilot: {pypilot_write_msg}")
    except Empty:
        pass


class Action(Enum):
    ENGAGE = "Engage"
    DISENGAGE = "Disengage"
    PLUS_ONE = "PlusOne"
    MINUS_ONE = "MinusOne"
    PLUS_FIVE = "PlusFive"
    MINUS_FIVE = "MinusFive"
    TOGGLE = "Toggle"


def queue_action(action: Action):
    logger.info(f"Queued action: {action}")
    # TODO: Handle wind mode, similar to hat.py
    # TODO: Handle servo command, when not engaged
    if action == Action.ENGAGE:
        send_queue.put_nowait(
            b"ap.heading_command=%f\n" % float(ap_state["ap_heading"])
        )
        send_queue.put_nowait(b"ap.enabled=true\n")
    elif action == Action.DISENGAGE:
        send_queue.put_nowait(b"ap.enabled=false\n")
    elif action == Action.PLUS_ONE:
        send_queue.put_nowait(
            b"ap.heading_command=%f\n" % (ap_state["ap_heading_command"] + 1.0)
        )
    elif action == Action.PLUS_FIVE:
        send_queue.put_nowait(
            b"ap.heading_command=%f\n" % (ap_state["ap_heading_command"] + 5)
        )
    elif action == Action.MINUS_FIVE:
        send_queue.put_nowait(
            b"ap.heading_command=%f\n" % (ap_state["ap_heading_command"] - 5)
        )
    elif action == Action.MINUS_ONE:
        send_queue.put_nowait(
            b"ap.heading_command=%f\n" % (ap_state["ap_heading_command"] - 1)
        )
    elif action == Action.TOGGLE:
        if ap_state["ap_enabled"]:
            queue_action(Action.DISENGAGE)
        else:
            queue_action(Action.ENGAGE)


def pypilot_comms_loop():
    # Client with specified watches, period 0 (=update watches on any change), scan for host.
    watches = [
        "ap.heading",
        "ap.heading_command",
        "imu.heading",
        "ap.mode",
        "ap.enabled",
    ]
    client = pypilotClientFromArgs(watches, 0, False)

    try:
        while True:
            client.poll(1)
            send_from_queue(client)
            msg = client.receive_single()
            while msg:
                update_state_from_readings(*msg)
                msg = client.receive_single()
    except Exception as e:
        logger.error(f"Error in pypilot communication loop: {e}")


if __name__ == "__main__":
    pypilot_comms_loop()
