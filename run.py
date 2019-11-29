#!/usr/bin/env python
from robot import Robot
import optparse
import sys


_ACTIONS   = ['u','r','d','l']
_ACTIONS_R = ['r','l','d','u']
_ACTIONS_2 = ['u','d','l','r','ne','nw','sw','se']
_ACTIONS_3 = ['f', 'tl','tr']


def readCommand(argv):
    '''
    For Help type -h:
    To change something from command line you type -a A1 or --actions A1 ...
    '''
    parser = optparse.OptionParser(description = 'Run public tests on code')
    # parser.set_defaults(generateSolutions=False, edxOutput=False, muteOutput=False, print)
    # Variable -h cannot be used since default is help
    parser.add_option('--actions','-a',
                    dest = 'ACTIONS',
                    default = 'A1',
                    help = 'Action set:     A1->_ACTIONS, A2->_ACTIONS_2, A3->_ACTIONS_3')
    parser.add_option('--corners','-c',
                    dest = 'CORNERS',
                    default = 'None',
                    help = 'corners: Adds penalty to corners, input should be a negative value such as -5')
    parser.add_option('--discount','-d',
                    dest = 'DISCOUNT',
                    default = '0.8',
                    help = 'Discount:   This is discount for MDP problems')
    parser.add_option('--heuristic','-e',
                    dest = 'HEURISTIC',
                    default = 'e',
                    help = 'Heuristic to use:   e->euclidean, m->manhattain')
    parser.add_option('--filename','-f',
                    dest = 'FILENAME',
                    default = 'None',
                    help = 'filename:   for figure save')
    parser.add_option('--goalReward','-g',
                    dest = 'GOAL_REWARD',
                    default = '10',
                    help = 'Goal Reward:    This is the reward for the goal')
    parser.add_option('--lidar','-l',
                    dest = 'LIDAR_SENSOR',
                    default = '0:1,90:1,180:1,270:1',
                    help = 'Sets the lidar sensor readings angle:distance')
    parser.add_option('--map','-m',
                    dest = 'MAP',
                    default = 'map0',
                    help = 'Chooses map:   map0, map1, map2')
    parser.add_option('--probability','-p',
                    dest = 'ACTION_PROBABILITY',
                    default = '0.1,0.8,0.1',
                    help = 'Probability:    This is set up to be taken as Prob(left),...,Prob(Correct),...,Prob(Right). Ex1: 0.1,0.8,0.1 Ex2: 0.05,0.1,0.7,0.1,0.05, If the values exceed 1, they are normalized making it possible to enter 10,80,10. It is important not to have space inbetween values just a comma.')
    parser.add_option('--policy','-q',
                    dest = 'POLICY',
                    default = 'u',
                    help = 'policy: Adds initial policy such as "u","r","d","l","ne","nw","se",sw"')
    parser.add_option('--stateReward','-r',
                    dest = 'STATE_REWARD',
                    default = '0',
                    help = 'State Reward:    This is the reward for each state')
    parser.add_option('--search','-s',
                    dest = 'SEARCH_METHOD',
                    default = 'pomdp',
                    help = 'Type of search to implement:')
    parser.add_option('--transition','-t',
                    dest = 'TRANSITION',
                    default = '1',
                    help = 'Transition Type:    1->Transition from project1, 2->Transition returning state and probability, 3->Transition returning state')
    parser.add_option('--iterations','-i',
                    dest = 'num_iterations',
                    default = '1',
                    help = 'Number of iterations of Markov Localization')

    (options, args) = parser.parse_args(argv)
    return options


if __name__ == "__main__":
    options = readCommand(sys.argv)
    map_file = './' + options.MAP.lower() + '.txt'
    if options.ACTIONS.lower() == 'a1':
        actions = _ACTIONS
    elif options.ACTIONS.lower() == 'a2':
        actions = _ACTIONS_2
    elif options.ACTIONS.lower() == 'a3':
        actions = _ACTIONS_3


    agent = Robot(options.LIDAR_SENSOR, map_file, actions)
    agent.GridMap.setProbabilisticActions(options.ACTION_PROBABILITY)
    # agent.initialize_sensor_probability()
    # Robot on start up
    agent.display_probability_map()
    #Steps
    for i in range(int(options.num_iterations)):
        #1 Get distances in each direction using queue_sensors (lidar)
        sensor_reading = agent.queue_sensors()

        #2 Find states that match sensor_reading
        possible_locations = agent.get_possible_states(sensor_reading)
        #2.2 Find states and probability from sensor_reading
        # possible_locations = agent.get_possible_states2(sensor_reading)
        # agent.display_possible_states([x[1] for x in possible_locations])

        #3 Update probability_map using the possible states...
        agent.update_prob_map(possible_states = possible_locations)
        #3.2 Update probability_map using the possible states...
        # agent.update_prob_map2(possible_states = possible_locations)

        #4 Carry out some action
        desired_action = agent.random_movement()
        # desired_action = agent.movement_from_policy()
        print("DESIRED ACTION = ",desired_action)

        #5 Update probability_map using gaussian kernel or transition function
        agent.update_prob_map(action = desired_action, sensor_reading=sensor_reading)
        #5.2 Update probability_map using the probability of sensors or transition function
        # agent.update_prob_map2(action = desired_action, sensor_reading=sensor_reading)

        #? Display resulting probability map
        agent.display_probability_map()

    print(agent.path_taken)
