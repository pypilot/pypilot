#!/usr/local/bin/python
#
#   Copyright (C) 2017 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.

"""Keep the system clock synchronized with gpsd."""

import datetime
import json
import socket
import subprocess
import time

GPSD_ADDRESS = ("127.0.0.1", 2947)
GPS_TIMEOUT = 30
RETRY_DELAY = 3
SYNC_INTERVAL = 3 * 24 * 60 * 60


def read_gps_time():
    with socket.create_connection(GPSD_ADDRESS, timeout=5) as connection:
        connection.settimeout(GPS_TIMEOUT)
        connection.sendall(b'?WATCH={"enable":true,"json":true};\n')

        with connection.makefile("r", encoding="ascii", errors="replace") as stream:
            for line in stream:
                try:
                    report = json.loads(line)
                except json.JSONDecodeError:
                    continue

                if report.get("class") == "TPV" and report.get("time"):
                    return report["time"]

    raise RuntimeError("gpsd closed the connection without a GPS timestamp")


def set_system_time(gps_timestamp):
    timestamp = datetime.datetime.fromisoformat(gps_timestamp.replace("Z", "+00:00")).strftime("%Y-%m-%d %H:%M:%S")
    subprocess.run(["date", "-u", "-s", timestamp], check=True)


def main():
    print("gpsdate started; waiting for gpsd time", flush=True)

    while True:
        try:
            gps_timestamp = read_gps_time()
            print(f"setting system time from GPS: {gps_timestamp}", flush=True)
            set_system_time(gps_timestamp)
            print(
                f"GPS time synchronized; next sync in {SYNC_INTERVAL} seconds",
                flush=True,
            )
            time.sleep(SYNC_INTERVAL)
        except Exception as error:
            print(f"GPS time sync failed: {error}; retrying in {RETRY_DELAY}s", flush=True)
            time.sleep(RETRY_DELAY)


if __name__ == "__main__":
    main()
