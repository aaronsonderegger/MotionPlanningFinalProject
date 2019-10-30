#!/usr/bin/env python
from environments import *

class Robot:
    def __init__(self, sensor_configuration, map_file):
        self.sensor_config = sensor_configuration
        self.GridMap = GridMap(map_file)

    def queue_sensors(self,state):
        return

    def get_possible_states(self,state):
        return

    def display_probability_map(self):
        return

    def display_traveled_map(self):
        return
