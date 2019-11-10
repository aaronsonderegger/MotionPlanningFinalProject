#!/usr/bin/env python
from environments import *
import random
import matplotlib
from lidar import LidarSensor

class Robot:
    def __init__(self, sensor_configuration, map_file, actions):
        self.GridMap = GridMap(map_file)
        self.actions = actions
        self.sensor_config = LidarSensor(sensor_configuration,self.GridMap)

    def transition_function(self, action):
        '''
        returns possible states transitioned to.
        '''
        return 

    def queue_sensors(self,state):
        '''
        Sensor Reading, u->blocked, r->open, etc.
        returns distance to obsticle for sensor obsticles
        '''
        return self.sensor_config.queue_sensors(state)

    def get_possible_states(self,state):
        '''
        Here is where we return possible states the robot could be in
        return list of states
        '''
        return

    def distribution_from_possible_states(self,states):
        '''
        Get the probability distriubtion from the possible states
        '''
        return

    def update_prob_map(self):
        '''
        Use the distribution from possible states and the current probability map
        to get the new distribution of where we are
        '''
        return

    def get_prob_from_transition(self):
        '''
        When we take an action, uncertainty will grow because we are now
        less sure of our state. We need to update our probability map 
        using the possible states from our transition function...
        '''
        return

    def display_probability_map(self):
        self.GridMap.display_probability_map()

    def display_traveled_map(self):
        return

    def random_movement(self):
        rand_act =  random.choice(self.actions)
        print(rand_act)
