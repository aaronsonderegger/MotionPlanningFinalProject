#!/usr/bin/env python
from environments import *
import random
import matplotlib
import copy
from lidar import LidarSensor
from scipy.ndimage import convolve

class Robot:
    def __init__(self, sensor_configuration, map_file, actions):
        self.GridMap = GridMap(map_file)
        self.truth_position = self.GridMap.init_pos
        self.path_taken = []
        self.rows = self.GridMap.rows
        self.columns = self.GridMap.cols
        self.probability_map = self.GridMap.probability_map
        self.actions = actions
        self.sensor_config = LidarSensor(sensor_configuration,self.GridMap)
        
        # Build a look up table of P(O|S), access by self.p_o_given_s
        # self.p_o_given_s.keys() will return sensor readings
        # self.p_o_given_s.items() will return states that could have sensor reading
        self.initalize_sensor_probability()
        
        self.gauss_3x3 = np.array(([1, 2, 1],[2, 4, 2],[1, 2, 1]))/16.0 #3x3 gaussian kernel

    def transition_function(self, action):
        '''
        returns possible states transitioned to.
        '''
        return

    def queue_sensors(self,state=None):
        '''
        Sensor Reading, u->blocked, r->open, etc.
        returns distance to obsticle for sensor obsticles
        '''
        if(state):
            return self.sensor_config.queue_sensors(state)
        else:
            return self.sensor_config.queue_sensors(self.truth_position)

    def initalize_sensor_probability(self):
        '''
        Call this once during the initialization to build a look up table for the possible 
        sensor readings. 
        This is set up to take rotaional moves _ACTIONS_3
        '''
        if len(self.actions) == 3:
            angles = [0,45,90,135,180,225,270,315]
        else:
            angles = [0]

        # Create a dictionary that will keep track of list. 
        # ProbObsStateDict Class Defined below Robot Class
        self.PossibleStatesFromObservations = ProbObsStateDict()

        # Stores the noise so we can get look up table with and with out noise
        holdNoise = self.sensor_config.NoiseProbability

        # iterate over (x,y,theta) for rotational moves
        for r in range(self.rows):
            for c in range(self.columns):
                for angle in angles:
                    # Don't care about the obsticles
                    if not self.GridMap.occupancy_grid[r][c]:
                        # Get the Perfect Sensor Reading
                        self.sensor_config.NoiseProbability = 0.0
                        pureSensor = self.sensor_config.queue_sensors((r,c,angle))
                        # Get the Noisiest Sensor Reading
                        self.sensor_config.NoiseProbability = 1.0
                        noisySensor = self.sensor_config.queue_sensors((r,c,angle))

                        # iterate to make sure that there is probabilty that sums to 1
                        # if noisyReading and pureReading are the same then probabilty is set to 1.
                        for pAngle,pRange in pureSensor:
                            for nAngle, nRange in noisySensor:
                                if pAngle == nAngle and pRange == nRange:
                                    self.PossibleStatesFromObservations[(pAngle,pRange)] = (1.0,(r,c,angle))
                                elif pAngle == nAngle and pRange != nRange:
                                    self.PossibleStatesFromObservations[(pAngle,pRange)] = (1.0-holdNoise,(r,c,angle))
                                    self.PossibleStatesFromObservations[(nAngle,nRange)] = (holdNoise,(r,c,angle))

        # Ensure noise is enabled so it's not 100% noisy
        self.sensor_config.NoiseProbability = holdNoise
        # for k in self.PossibleStatesFromObservations.keys():
        #     print(k,len(self.PossibleStatesFromObservations[k]))
            # print(self.PossibleStatesFromObservations[k])


    def get_possible_states2(self, sensor_reading):
        '''
        Loop over every state with non-zero probabilities, get it's sensor
        reading, and see if it matches our current one.
        '''
        possible_states = []
        for r in range(self.rows):
            for c in range(self.columns):
                if(self.probability_map[r][c] and not self.GridMap.occupancy_grid[r][c]):
                    if(self.queue_sensors((r,c,0)) == sensor_reading):
                        possible_states.append((r,c,0))

        return possible_states

    def get_possible_states(self, sensor_readings):
        '''
        look up the states that have the probability of seeing the range.
        This accounts for the noise in the range sensor.
        '''
        possible_states = []
        for reading in sensor_readings:
            for p,state in self.PossibleStatesFromObservations[reading]:
                possible_states.append(state)
        return possible_states


    def distribution_from_possible_states(self,states):
        '''
        Get the probability distriubtion from the possible states
        '''
        return

    def update_prob_map(self, possible_states=None, action=None, sensor_reading=None):
        '''
        Use the distribution from possible states and the current probability map
        to get the new distribution of where we are
        '''
        if(possible_states):
            #We could add a gaussian kernel at each of these states?
            probability_of_possible_states = 1.0 / (len(possible_states))
            likelihood = np.zeros(self.probability_map.shape)
            for state in possible_states:
                likelihood[state[0:2]] = probability_of_possible_states
            #Bayes Rule...
            posterior = likelihood * self.probability_map
            #Normalize the resulting distribution
            posterior /= np.sum(posterior)
            #The posterior now becomes the prior (our self.probability_map)
            self.probability_map = copy.copy(posterior)

        #The sensor reading makes sure we can actually go that direction...
        if(action and sensor_reading):
            for angle,rng in sensor_reading:
                if action == 'u' and (angle == 270 and rng):
                    self.probability_map = np.roll(self.probability_map,-1,0)
                    # print(sensor_reading[0],'u','success')
                elif action == 'd' and (angle == 90 and rng):
                    self.probability_map = np.roll(self.probability_map,1,0)
                    # print(sensor_reading[0],'d','success')
                elif action == 'l' and (angle == 180 and rng):
                    self.probability_map = np.roll(self.probability_map,-1,1)
                    # print(sensor_reading[0],'l','success')
                elif action == 'r' and (angle == 0 and rng):
                    self.probability_map = np.roll(self.probability_map,1,1)

            self.probability_map = convolve(self.probability_map, self.gauss_3x3)


        return

    def get_prob_from_transition(self):
        '''
        When we take an action, uncertainty will grow because we are now
        less sure of our state. We need to update our probability map
        using the possible states from our transition function...
        '''
        return

    def display_probability_map(self):
        fig = plt.figure()
        temp = copy.copy(self.probability_map)
        # temp[self.truth_position[0:2]] += -1
        imgplot = plt.imshow(temp)
        # Set interpolation to nearest to create sharp boundaries
        imgplot.set_interpolation('nearest')
        # Set color map to diverging style for contrast
        # imgplot1.set_cmap('gray')
        imgplot.set_cmap('Spectral')

        fig.suptitle("Probability Map", fontsize=16)
        plt.draw()
        plt.waitforbuttonpress(0) # this will wait for indefinite time
        plt.close(fig)

    def display_possible_states(self, possible_states):
        self.GridMap.display_map(visited = possible_states, curr_pos=self.truth_position)


    def display_traveled_map(self):
        return

    def random_movement(self):
        rand_act =  random.choice(self.actions)
        #Keep track of where we go...
        self.path_taken.append(self.truth_position)
        self.truth_position = self.GridMap.transition(self.truth_position, rand_act)
        print("Current Position = ", self.truth_position)
        return rand_act


class ProbObsStateDict:
    def __init__(self):
        self.ProbObserveState = dict()

    def keys(self):
        return self.ProbObserveState.keys()

    def items(self):
        return self.ProbObserveState.items()

    def __getitem__(self,key):
        try:
            return self.ProbObserveState[key]
        except:
            self.ProbObserveState[key] = []
            return self.ProbObserveState[key]

    def __setitem__(self, key, state):
        try:
            states = self.ProbObserveState[key]
        except:
            states = []
        if state not in states:
            states.append(state)
        self.ProbObserveState[key] = states
