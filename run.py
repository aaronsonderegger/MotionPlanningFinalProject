#!/usr/bin/env python
from robot import Robot
import sys

_ACTIONS = ['u','d','l','r']
_ACTIONS_R = ['r','l','d','u']
_ACTIONS_2 = ['u','d','l','r','ne','nw','sw','se']
_ACTIONS_3 = ['f', 'tl','tr']

#Parse the commandline arguments
for arg in sys.argv:
    if arg[0:3] == 'map':
        map_file = arg
    elif arg == 'euclid' or arg == 'manhat':
        heuristic = arg
        print('Chosen euristic is ' + heuristic)
    elif arg == 'A1':
        actions = _ACTIONS
    elif arg == 'A2':
        actions = _ACTIONS_2
    elif arg ==  'A3':
        actions = _ACTIONS_3

sensor_config = {0:1, 90:1, 180:1, 270:1} #{lidar_angle:distance_it_can_see}
agent = Robot(sensor_config, map_file, actions)
