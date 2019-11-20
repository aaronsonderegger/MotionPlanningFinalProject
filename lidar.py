#!/usr/bin/env python
import random
import numpy as np
import math
_T = 2
_X = 1
_Y = 0
_DEBUG = False

class LidarSensor:
    """docstring for LidarSensor"""
    def __init__(self, sensorConfig, GridMap, rangeNoise=1,NoiseProbability=0.20):

        # Parse the sensor config if it is in the format of string rather than a dictionary.
        if isinstance(sensorConfig,str):
            sensorRange = sensorConfig.split(',')
            sensorConfig = dict()
            for s in sensorRange:
                angle,rng = s.split(':')
                sensorConfig[int(angle)] = int(rng)

        self.Sensor_Config = sensorConfig
        self.GridMap = GridMap  # This is the gridmap already initialized
        self.Obstacles = [tuple(o) for o in self.GridMap.getObstacles()]
        self.rangeNoise = rangeNoise
        # This just normalizes if the probability is greater than 1
        self.NoiseProbability = NoiseProbability if NoiseProbability<1 else NoiseProbability/100

    def queue_sensors(self, state):
        '''
        This returns the noisy sensor reading for the robot.
        '''
        if _DEBUG:
            state = list(self.GridMap.init_pos[:])
            state[_T] = 45
            print(state)
        sensorReading = list()
        if _DEBUG:
            looking = list()
        for key in self.Sensor_Config.keys():
            obState = list(state[:])
            # Sensor Takes a perfect measurement
            for i in range(1,self.Sensor_Config[key]+1):
                # Have to round before casting to int because result of sine and cosine
                # will make weird results.
                obState[_X] = int(round(state[_X] + i*math.cos((state[_T] + key)*np.pi/180.0)))
                obState[_Y] = int(round(state[_Y] + i*math.sin((state[_T] + key)*np.pi/180.0)))

                # Bounds Checking, set distance to 1 iteration less for proper distance
                if obState[_X] < 0:
                    obState[_X] = 0
                    distance = i-1
                    break
                if obState[_Y] < 0:
                    obState[_Y] = 0
                    distance = i-1
                    break
                if obState[_X] >= self.GridMap.cols:
                    obState[_X] = self.GridMap.cols-1
                    distance = i-1
                    break
                if obState[_Y] >= self.GridMap.rows:
                    obState[_Y] = self.GridMap.rows-1
                    distance = i-1
                    break
                if self.GridMap.occupancy_grid[obState[0], obState[1]]:
                    distance = i-1
                    break

                distance = i

                if _DEBUG:
                    looking.append(tuple(obState))

            # Add Noise to the sensor
            prob = random.random()
            if prob < self.NoiseProbability:
                distance = distance - 1 if distance>0 else 0 # Don't want a negative distance
            sensorReading.append((key,distance))
        if _DEBUG:
            # print(looking)
            self.GridMap.display_map([], looking)
        return sensorReading
