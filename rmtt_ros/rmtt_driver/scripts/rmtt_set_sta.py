#!/usr/bin/env python3

import robomaster
from robomaster import robot

if __name__ == '__main__':
    tl_drone = robot.Drone()
    tl_drone.initialize()
    version = tl_drone.get_sdk_version()
    print("Drone SDK Version: {0}".format(version))
    tl_drone.config_sta(ssid="shine", password="12345678")
    print("Done")
    tl_drone.close()