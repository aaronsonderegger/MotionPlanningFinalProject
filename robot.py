#!/usr/bin/env python
from environments import *
import random
import matplotlib
import copy
from lidar import LidarSensor
from scipy.ndimage import convolve
import pickle
from mpl_toolkits.mplot3d import Axes3D

_MOVIE = True

class Robot:
    def __init__(self, sensor_configuration, map_file, actions):
        self.GridMap = GridMap(map_file,actions=actions)
        self.truth_position = self.GridMap.init_pos
        self.path_taken = []
        self.rows = self.GridMap.rows
        self.columns = self.GridMap.cols
        self.probability_map = self.GridMap.probability_map
        self.actions = actions
        self.sensor_config = LidarSensor(sensor_configuration,self.GridMap)
        self.optimal_policy = []

        # Build a look up table of P(O|S), access by self.p_o_given_s
        # self.p_o_given_s.keys() will return sensor readings
        # self.p_o_given_s.items() will return states that could have sensor reading
        map_path = map_file.split('/')[1].split('.')[0]
        try:
            # Load the PossibleStatesFromObservations Dictionary
            probFile = open('ProbabilityMap_'+map_path+'_'+str(len(actions))+'.obj','rb')
            self.PossibleStatesFromObservations = pickle.load(probFile)
        except:
            # Create and Save the PossibleStatesFromObservations Dictionary
            print('creating Sensor Probabilities')
            self.initalize_sensor_probability()
            probFile = open('ProbabilityMap_'+map_path+'_'+str(len(actions))+'.obj', 'wb')
            pickle.dump(self.PossibleStatesFromObservations, probFile)


        self.gauss_3x3 = np.array(([1, 2, 1],[2, 4, 2],[1, 2, 1]))/16.0 #3x3 gaussian kernel
        if _MOVIE:
            self.fig, self.ax = plt.subplots()

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
                                if pAngle == nAngle and pRange == nRange and pRange == 0:
                                    # for i in rang(1,1+self.sensor_config.Sensor_Config[pAngle]):
                                    #     self.PossibleStatesFromObservations[pAngle,pRange+i] = (0.0,(r,c,angle))
                                    self.PossibleStatesFromObservations[(pAngle,pRange)] = (1.0,(r,c,angle))
                                    self.PossibleStatesFromObservations[pAngle,pRange+1] = (0.0,(r,c,angle))
                                elif pAngle == nAngle:
                                    self.PossibleStatesFromObservations[(pAngle,pRange)] = (1.0-holdNoise,(r,c,angle))
                                    self.PossibleStatesFromObservations[(nAngle,nRange)] = (holdNoise,(r,c,angle))

        # Ensure noise is enabled so it's not 100% noisy
        self.sensor_config.NoiseProbability = holdNoise

        # for k in self.PossibleStatesFromObservations.keys():
        #     print(k,len(self.PossibleStatesFromObservations[k]))
        #     print(self.PossibleStatesFromObservations[k])
        #     print('')


    def get_possible_states(self, sensor_reading):
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

    def get_possible_states2(self, sensor_readings):
        '''
        look up the states that have the probability of seeing the range.
        This accounts for the noise in the range sensor.
        '''
        possible_states = []
        prob_state_dict = dict()
        for reading in sensor_readings:
            for p,state in self.PossibleStatesFromObservations[reading]:
                try:
                    prob_state_dict[state] *= p
                except:
                    prob_state_dict[state] = p

        for key in prob_state_dict.keys():
            if prob_state_dict[key] > 0.0:
                possible_states.append((round(prob_state_dict[key],15),key))
        return possible_states


    def distribution_from_possible_states(self,states):
        '''
        Get the probability distriubtion from the possible states
        '''
        return

    def update_prob_map2(self, possible_states=None, action=None, sensor_reading=None):
        '''
        Takes in Noisy sensor readings and updates the probability based on them.
        '''
        if possible_states:
            likelihood = np.zeros(self.probability_map.shape)
            for prob, state in possible_states:
                likelihood[state[0:2]] = prob

            self.probability_map *= likelihood
            self.probability_map /= np.sum(self.probability_map)

        if action:
            if len(self.actions) == 3:
                angles = [0,45,90,135,180,225,270,315]
            else:
                angles = [0]

            likelihood = np.zeros(self.probability_map.shape)
            # likelihood = copy.copy(self.probability_map)
            for r in range(self.rows):
                for c in range(self.columns):
                    if not self.GridMap.occupancy_grid[r][c]:
                        for angle in angles:
                            successor = self.GridMap.uncertainty_transition((r,c,angle),action)
                            for prob, statePrime in successor:
                                likelihood[statePrime[0:2]] += (prob)*self.probability_map[r,c]

            likelihood /= np.sum(likelihood)
            self.probability_map = copy.deepcopy(likelihood)

        if np.any(np.isnan(self.probability_map)):
            '''
            Safety Check to make sure that we don't get a 0 probability in all states.
            '''
            self.GridMap.create_probability_map()
            self.probability_map = self.GridMap.probability_map





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
                elif action == 'd' and (angle == 90 and rng):
                    self.probability_map = np.roll(self.probability_map,1,0)
                elif action == 'l' and (angle == 180 and rng):
                    self.probability_map = np.roll(self.probability_map,-1,1)
                elif action == 'r' and (angle == 0 and rng):
                    self.probability_map = np.roll(self.probability_map,1,1)

            self.probability_map = convolve(self.probability_map, self.gauss_3x3)


        return

    def display_probability_map(self):
        if _MOVIE:
            fig = self.fig
            ax = self.ax
            ax.clear()
            try:
                self.cbar.remove()
            except:
                pass
        else:
            fig, ax = plt.subplots()

        robot_location = np.zeros(self.probability_map.shape)
        robot_location[self.truth_position[0:2]] = 100
        temp_map = copy.copy(self.probability_map)
        
        # temp_map[self.GridMap.occupancy_grid] = 0.27
        # temp[self.truth_position[0:2]] += -1

        norm = matplotlib.colors.Normalize(vmin=0., vmax=1.0)
        pc_kwargs = {'rasterized': True, 'cmap': 'Spectral', 'norm': norm}
        im = ax.imshow(temp_map, **pc_kwargs)
        self.cbar = ax.figure.colorbar(im, ax=ax)
        self.cbar.ax.set_ylabel('Belief Space',rotation=-90,va='center',ha='left')

        grays = matplotlib.colors.Normalize(0, 1, clip=True)(self.GridMap.occupancy_grid)
        cmap = plt.cm.Greys
        grays = cmap(grays)
        grays[...,-1] = self.GridMap.occupancy_grid
        occ_grid_overlay = ax.imshow(grays)
        occ_grid_overlay.set_cmap('gray')

        goal_location = np.zeros(self.probability_map.shape)
        goal_location[self.GridMap.goal[0:2]] = 1.0
        display_goal = matplotlib.colors.Normalize(0, 1, clip=True)(goal_location)
        cmap = plt.cm.Greens
        display_goal = cmap(display_goal)
        display_goal[...,-1] = goal_location


        # imgplot = ax.imshow(temp_map)
        robot_loc_overlay = ax.imshow(display_goal)
        # robot_loc_overlay.set_cmap('Oranges')

        # Set interpolation to nearest to create sharp boundaries
        # imgplot.set_interpolation('nearest')
        # Set color map to diverging style for contrast
        # occ_grid_overlay.set_cmap('gray')
        # robot_loc_overlay.set_cmap('gray')
        # imgplot.set_cmap('Spectral')

        # for r in range(self.rows):
        #     for c in range(self.columns):
        #         if (r,c,0) == self.truth_position:
        #             rbt = '\n* *\n U '
        #         else:
        #             rbt = ''
        #         text = ax.text(c,r, str(round(temp_map[r,c],4))+rbt, ha='center',va='center',color='k')
        # print(self.truth_position)
        r,c,_ = self.truth_position
        text = ax.text(c,r, 'R' ,ha='center',va='center',color='k',weight='bold')
        r,c = self.GridMap.goal
        text = ax.text(c,r,'G',ha='center',va='center',color='w',weight='bold')

        fig.suptitle("Probability Map", fontsize=16)
        
        plt.draw()
        # # fig, ax = plt.subplots()
        # fig = plt.figure()
        # ax = fig.gca(projection='3d')
        # ax.plot_surface(range(self.columns), range(self.rows), temp_map.flatten())
        # plt.show()

        if _MOVIE:
            plt.pause(0.1)
        else:
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

    def movement_from_policy(self):
        action_probabilities = np.zeros(len(self.actions))
        for r in range(self.rows):
            for c in range(self.columns):
                if(self.GridMap.policies[(r,c,0)] != 0):
                    # print(np.array(self.actions) == self.GridMap.policies[(r,c,0)], self.GridMap.policies[(r,c,0)])
                    truth_array = np.array(np.array(self.actions) == self.GridMap.policies[(r,c,0)])
                    action_probabilities += truth_array*self.probability_map[r][c]
        most_probable_action = self.actions[np.argmax(action_probabilities)]
        self.truth_position = self.GridMap.transition_with_random_movement(self.truth_position, most_probable_action)
        self.path_taken.append(self.truth_position)
        # print("Current Position = ", self.truth_position)

        return most_probable_action

    def movement_from_max_policy(self):
        row,col = np.where(self.probability_map == np.max(self.probability_map))
        key = (row[0],col[0],0)
        policy = self.GridMap.policies[key]
        # print(policy)
        self.truth_position = self.GridMap.transition_with_random_movement(self.truth_position, policy)
        self.path_taken.append(self.truth_position)

        if not self.check_if_in_goal_state() and self.GridMap.is_goal(key):
            # Update Probability_map if it thinks it's at the goal
            self.probability_map[row[0],col[0]] = 0.0

        return policy



    def check_if_in_goal_state(self):
        return self.truth_position[0:2] == self.GridMap.goal






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
