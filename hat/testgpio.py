#!/usr/bin/env python3
import gpiod

PINS = [17, 23, 27, 22, 18, 5, 6]

config = {}
for pin in PINS:
    config[pin] = gpiod.LineSettings(
        direction=gpiod.line.Direction.INPUT,
        edge_detection=gpiod.line.Edge.BOTH,
        bias=gpiod.line.Bias.PULL_UP,
    )

req = gpiod.request_lines(
    "/dev/gpiochip0",
    consumer="gpiod-multipin-edge-test",
    config=config,
)

print("waiting for edges:", PINS)

while True:
    if not req.wait_edge_events(timeout=1.0):
        print("timeout")
        continue

    for ev in req.read_edge_events():
        value = req.get_value(ev.line_offset)
        print("edge pin=", ev.line_offset, "event=", ev.event_type, "value_now=", value)
        
