#!/usr/bin/env python
from environments import *
import random

class Robot:
    def __init__(self, sensor_configuration, map_file, actions):
        self.sensor_config = sensor_configuration
        self.GridMap = GridMap(map_file)
        self.actions = actions

    def random_movement(self):
        rand_act =  random.choice(self.actions)
        print(rand_act)

    def queue_sensors(self,state):
        return

    def get_possible_states(self,state):
        return

    def display_probability_map(self):
        self.GridMap.display_probability_map()

    def display_traveled_map(self):
        return
