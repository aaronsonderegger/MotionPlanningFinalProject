#!/usr/bin/env python
'''
Package providing helper classes and functions for performing graph search operations for planning.
'''
import sys
import numpy as np
import heapq
import matplotlib.pyplot as plt
from math import hypot, sqrt

_DEBUG = False
_DEBUG_END = True
_ACTIONS   = ['u','r','d','l']
_ACTIONS_2 = ['u','ne','r','se','d','sw','l','nw']
_T = 2
_X = 1
_Y = 0
_TURN_ANGLE = 45
# _GOAL_COLOR = 0.75
_GOAL_COLOR = 0.0
# _INIT_COLOR = 0.25
_INIT_COLOR = 0.0
_CURR_POS_COLOR = 0.5
_PATH_COLOR_RANGE = _GOAL_COLOR-_INIT_COLOR
_VISITED_COLOR = 0.27


class GridMap:
    '''
    Class to hold a grid map for navigation. Reads in a map.txt file of the format
    0 - free cell, x - occupied cell, g - goal location, i - initial location.
    Additionally provides a simple transition model for grid maps and a convience function
    for displaying maps.
    '''
    def __init__(self, map_path=None, heuristic='euclid',actions=_ACTIONS):
        '''
        Constructor. Makes the necessary class variables. Optionally reads in a provided map
        file given by map_path.

        map_path (optional) - a string of the path to the file on disk
        '''
        self.rows = None
        self.cols = None
        self.goal = None
        self.init_pos = None
        self.occupancy_grid = None
        self.c_heuristic = heuristic #Specify the chosen heuristic here
        self.action_set = actions
        if map_path is not None:
            self.read_map(map_path)
        self.create_probability_map()

    def create_probability_map(self):
        self.probability_map = np.ones((self.rows,self.cols))
        self.probability_map[self.occupancy_grid] = 0.0
        # self.probability_map = np.random.random((self.rows,self.cols))
        self.probability_map /= np.sum(self.probability_map);


    def setProbabilisticActions(self, probabiltyString):
        '''
        This takes in a string of probabilities and makes them floats.
        For the purpose of the assignment, they are the keys to a dictionary.
        The Dictionary is used to index through actions. 
        '''
        self.action_probability = dict()
        prob = [float(x) for x in probabiltyString.split(',') ]
        total = sum(prob)               # for normalizing
        prob = list(np.cumsum(prob))    # adds up values to 1    

        prob1 = list()
        for p in prob:
            prob1.append(round(p/total,3))    # rounds to ensure that there aren't floating point residuals

        for p in prob1:
            # makes the actions based off probability with the correct action 0.
            # makes the value -1, 0, 1 or -2, -1, 0, 1, 2.
            self.action_probability[p] = prob1.index(p) - int(len(prob1)/2.0)


    def display_probability_map(self):
        plt.figure(1)
        plt.imshow(self.probability_map)
        print('probmap',self.probability_map)
        plt.show()

    def read_map(self, map_path):
        '''
        Read in a specified map file of the format described in the class doc string.

        map_path - a string of the path to the file on disk
        '''
        map_file = open(map_path,'r')
        lines = [l.rstrip().lower() for l in map_file.readlines()]
        map_file.close()
        self.rows = len(lines)
        self.cols = max([len(l) for l in lines])
        if _DEBUG:
            print('rows', self.rows)
            print('cols', self.cols)
            print(lines)
        self.occupancy_grid = np.zeros((self.rows, self.cols), dtype=np.bool)
        for r in range(self.rows):
            for c in range(self.cols):
                if lines[r][c] == 'x':
                    self.occupancy_grid[r][c] = True
                if lines[r][c] == 'g':
                    self.goal = (r,c)
                elif lines[r][c] == 'i':
                    self.init_pos = (r,c,0)

    def getObstacles(self):
        return np.argwhere(self.occupancy_grid)

    def is_goal(self,s):
        '''
        Test if a specifid state is the goal state

        s - tuple describing the state as (row, col) position on the grid.

        Returns - True if s is the goal. False otherwise.
        '''
        # print(s[_X], self.goal[_X],s[_Y], self.goal[_Y])
        return (s[_X] == self.goal[_X] and
                s[_Y] == self.goal[_Y])

    def uncertainty_transition(self, s, a):
        '''
        Returns a tuple like (0.1, (x',y',t')),(0.8, (x',y',t')),(0.1, (x',y',t'))
        '''
        distribution = list()
        last = 0    # makes the values begin from 0<x<p1. Also incraments up.
        for k in self.action_probability.keys():
            # makes action spread over, probability.
            # if a = u, and probability 80%, then when 10% a = l and right.
            actionIndex = (self.action_set.index(a) + self.action_probability[k])%len(self.action_set)

            distribution.append((round(k-last,3), self.transition(s, self.action_set[actionIndex])))
            last = k    # Increments up

        return distribution


    def transition(self, s, a):
        '''
        Transition function for the current grid map.

        s - tuple describing the state as (row, col) position on the grid.
        a - the action to be performed from state s

        returns - s_prime, the state transitioned to by taking action a in state s.
        If the action is not valid (e.g. moves off the grid or into an obstacle)
        returns the current state.
        '''
        new_pos = list(s[:])

        '''
        Translate non-holonomic commands (forward, turn) to the holonomic
        commands below
        '''
        theta = s[_T] % 360
        translations = {0:'u', 45:'nw', 90:'l', 135:'sw', 180:'d', 225:'se', 270:'r', 315:'ne'}
        if a == 'f':
            a = translations[theta]



        if(new_pos[_T] < 0):
            new_pos[_T] += 360
        elif(new_pos[_T] > 360):
            new_pos[_T] -= 360

        # Ensure action stays on the board
        if a == 'u':
            if s[_Y] > 0:
                new_pos[_Y] -= 1
        elif a == 'd':
            if s[_Y] < self.rows - 1:
                new_pos[_Y] += 1
        elif a == 'l':
            if s[_X] > 0:
                new_pos[_X] -= 1
        elif a == 'r':
            if s[_X] < self.cols - 1:
                new_pos[_X] += 1
        elif a=='ne':
            if s[_Y] > 0 and s[_X] < self.cols - 1:
                new_pos[_X] += 1
                new_pos[_Y] -= 1
        elif a=='nw':
            if s[_Y] > 0 and s[_X] > 0:
                new_pos[_X] -= 1
                new_pos[_Y] -= 1
        elif a=='sw':
            if s[_Y] < self.rows - 1 and s[_X] > 0:
                new_pos[_X] -= 1
                new_pos[_Y] += 1
        elif a=='se':
            if s[_Y] < self.rows - 1 and s[_X] < self.cols - 1:
                new_pos[_X] += 1
                new_pos[_Y] += 1
        elif a == "tl":
            new_pos[_T] = s[_T] + _TURN_ANGLE
        elif a == "tr":
            new_pos[_T] = s[_T] - _TURN_ANGLE
        else:
            print('Unknown action:', str(a))

        if(new_pos[_T] < 0):
            new_pos[_T] += 360
        elif(new_pos[_T] >= 360):
            new_pos[_T] -= 360

        # Test if new position is clear
        if self.occupancy_grid[new_pos[0], new_pos[1]]:
            s_prime = tuple(s)
        else:
            s_prime = tuple(new_pos)
        return s_prime

    def display_map(self, path=[], visited={}, curr_pos =None, filename=None):
        '''
        Visualize the map read in. Optionally display the resulting plan and visisted nodes

        path - a list of tuples describing the path take from init to goal
        visited - a set of tuples describing the states visited during a search
        filename - relative path to file where image will be saved
        '''
        fig = plt.figure(2)
        fig.suptitle("Possible States from Sensor Measurement", fontsize=16)
        display_grid = np.array(self.occupancy_grid, dtype=np.float32)

        # Color all visited nodes if requested
        for v in visited:
            display_grid[v[0],v[1]] = _VISITED_COLOR
        if(curr_pos):
            display_grid[curr_pos[0],curr_pos[1]] = _CURR_POS_COLOR
        # Color path in increasing color from init to goal
        for i, p in enumerate(path):
            disp_col = _INIT_COLOR + _PATH_COLOR_RANGE*(i+1)/len(path)
            display_grid[p[0],p[1]] = disp_col

        display_grid[self.init_pos[0],self.init_pos[1]] = _INIT_COLOR
        display_grid[self.goal] = _GOAL_COLOR

        # Plot display grid for visualization
        imgplot = plt.imshow(display_grid)
        # Set interpolation to nearest to create sharp boundaries
        imgplot.set_interpolation('nearest')
        # Set color map to diverging style for contrast
        imgplot.set_cmap('Spectral')
        if filename is not None:
            plt.savefig(filename)
        plt.draw()
        plt.waitforbuttonpress(0) # this will wait for indefinite time
        plt.close(fig)

    def heuristic(self, s):
        '''
        Example of how a heuristic may be provided. This one is admissable, but dumb.

        s - tuple describing the state as (row, col) position on the grid.

        returns - floating point estimate of the cost to the goal from state s
        '''
        if(self.c_heuristic == 'euclid'):
            # print("euclid")
            # print(s, self.goal)
            x_dist = abs(s[_X] - self.goal[_X])
            y_dist = abs(s[_Y] - self.goal[_Y])
            g_cost = sqrt(x_dist**2 + y_dist**2)

        elif(self.c_heuristic == 'manhat'):
            # print("manhattan")
            x_dist = s[_X] - self.goal[_X]
            y_dist = s[_Y] - self.goal[_Y]
            g_cost = abs(x_dist) + abs(y_dist)

        return g_cost


class SearchNode:
    def __init__(self, s, A, parent=None, parent_action=None):
        '''
        s - the state defining the search node
        A - list of actions
        parent - the parent search node
        parent_action - the action taken from parent to get to s
        '''

        self.parent = parent
        self.parent_action = parent_action
        self.state = s[:]
        self.actions = A[:]
        # self.g_cost = g_cost

        alternate = ['tl', 'tr']
        ortho = ['u', 'd', 'l', 'r']
        diag = ['ne', 'nw', 'sw', 'se']
        if parent_action in ortho:
            action_cost = 1
        elif parent_action in diag:
            action_cost = 1.5
        elif parent_action == 'f':
            if s[2]%90 == 0:
                action_cost = 1
            else:
                action_cost = 1.5
        elif parent_action in alternate:
            action_cost = 0.25
        else:
            action_cost = 0

        if parent is not None:
            self.cost = parent.cost + action_cost
            self.depth = parent.depth + 1
        else:
            self.cost = 0
            self.depth = 0

    def __str__(self):
        '''
        Return a human readable description of the node
        '''
        return str(self.state) + ' ' + str(self.actions)+' '+str(self.parent)+' '+str(self.parent_action)
    def __lt__(self, other):
        return self.cost < other.cost

    def __contains__(self, x):
        '''
        Test if x is in the queue
        '''
        return x in self.s

class PriorityQ:
    '''
    Priority queue implementation with quick access for membership testing
    Setup currently to only with the SearchNode class
    '''
    def __init__(self):
        '''
        Initialize an empty priority queue
        '''
        self.l = [] # list storing the priority q
        self.s = set() # set for fast membership testing

    def __contains__(self, x):
        '''
        Test if x is in the queue
        '''
        return x in self.s

    def push(self, x, cost):
        '''
        Adds an element to the priority queue.
        If the state already exists, we update the cost
        '''
        if x.state in self.s:
            return self.replace(x, cost)
        heapq.heappush(self.l, (cost, x))
        self.s.add(x.state)

    def pop(self):
        '''
        Get the value and remove the lowest cost element from the queue
        '''
        x = heapq.heappop(self.l)
        self.s.remove(x[1].state)
        return x[1]

    def peak(self):
        '''
        Get the value of the lowest cost element in the priority queue
        '''
        x = self.l[0]
        return x[1]

    def __len__(self):
        '''
        Return the number of elements in the queue
        '''
        return len(self.l)

    def replace(self, x, new_cost):
        '''
        Removes element x from the q and replaces it with x with the new_cost
        '''
        for y in self.l:
            if x.state == y[1].state:
                if(x.cost >= y[1].cost): #Only replace if the cost is lower or equal!
                    return
                self.l.remove(y)
                self.s.remove(y[1].state)
        heapq.heapify(self.l)
        self.push(x, new_cost)

    def get_cost(self, x):
        '''
        Return the cost for the search node with state x.state
        '''
        for y in self.l:
            if x.state == y[1].state:
                return y[0]

    def __str__(self):
        '''
        Return a string of the contents of the list
        '''
        return str(self.l)

def iterative_deepening(init_state, f, is_goal, actions, search_depth_limit = 1e12):

    for depth_limit in range(search_depth_limit):
        frontier = [] #Search stack
        visited = set() #Set of visited nodes
        visited_visualization_set = set()
        n0 = SearchNode(init_state, actions)
        frontier.append(n0)
        while frontier:
            n_curr = frontier.pop()
            if (n_curr.state, n_curr.depth) not in visited:
                visited.add((n_curr.state,n_curr.depth))
                visited_visualization_set.add(n_curr.state)
                if is_goal(n_curr.state):
                    if(_DEBUG):
                        print("arrived at goal!")
                    return backpath(n_curr), visited_visualization_set, True
                else:
                    for a in actions:
                        s_prime = f(n_curr.state, a)
                        n_prime = SearchNode(s_prime, actions, n_curr, a)
                        if(n_prime.depth <= depth_limit):
                            frontier.append(n_prime)

    return [], visited_visualization_set, False

def dfs(init_state, f, is_goal, actions):
    '''
    Perform depth first search on a grid map.

    init_state - the intial state on the map
    f - transition function of the form s_prime = f(s,a)
    is_goal - function taking as input a state s and returning True if its a goal state
    actions - set of actions which can be taken by the agent

    returns - ((path, action_path), visited) or None if no path can be found
    path - a list of tuples. The first element is the initial state followed by all states
        traversed until the final goal state
    action_path - the actions taken to transition from the initial state to goal state
    '''
    frontier = [] #Search stack
    visited = set() #Set of visited nodes
    n0 = SearchNode(init_state, actions)
    frontier.append(n0)
    while frontier:
        n_curr = frontier.pop()
        if n_curr.state not in visited:
            visited.add(n_curr.state)
            if is_goal(n_curr.state):
                if(_DEBUG):
                    print("arrived at goal!")
                return backpath(n_curr), visited, True
            else:
                for a in actions:
                    s_prime = f(n_curr.state, a)
                    n_prime = SearchNode(s_prime, actions, n_curr, a)
                    frontier.append(n_prime)

    return [], visited, False

def bfs(init_state, f, is_goal, actions):
    '''
    Perform breadth first search on a grid map.

    init_state - the intial state on the map
    f - transition function of the form s_prime = f(s,a)
    is_goal - function taking as input a state s and returning True if its a goal state
    actions - set of actions which can be taken by the agent

    returns - ((path, action_path), visited) or None if no path can be found
    path - a list of tuples. The first element is the initial state followed by all states
        traversed until the final goal state
    action_path - the actions taken to transition from the initial state to goal state
    '''
    frontier = [] #Search stack
    visited = set() #Set of visited nodes
    n0 = SearchNode(init_state, actions)
    frontier.append(n0)
    while frontier:
        n_curr = frontier.pop(0)
        if n_curr.state not in visited:
            visited.add(n_curr.state)
            if is_goal(n_curr.state):
                if(_DEBUG):
                    print("arrived at goal!")
                return backpath(n_curr), visited, True
            else:
                for a in actions:
                    s_prime = f(n_curr.state, a)
                    n_prime = SearchNode(s_prime, actions, n_curr, a)
                    frontier.append(n_prime)

    return [], visited, False

def uniform_cost_search(init_state, f, is_goal, actions):
    frontier = PriorityQ() #Priority queue so lowest cost options are explored
    visited = set() #Set of visited nodes
    n0 = SearchNode(init_state, actions)
    frontier.push(n0, 1)
    while frontier:
        n_curr = frontier.pop()
        if n_curr.state not in visited:
            visited.add(n_curr.state)
            if is_goal(n_curr.state):
                if(_DEBUG):
                    print("arrived at goal!")
                    print(n_curr.cost)
                return backpath(n_curr), visited, True
            else:
                for a in actions:
                    s_prime = f(n_curr.state, a)
                    n_prime = SearchNode(s_prime, actions, n_curr, a)
                    frontier.push(n_prime, n_prime.cost)

    return [], visited, False


def a_star_search(init_state, f, is_goal, actions, h):
    '''
    init_state - value of the initial state
    f - transition function takes input state (s), action (a), returns s_prime = f(s, a)
    returns s if action is not valid
    is_goal - takes state as input returns true if it is a goal state
    actions - list of actions available
    h - heuristic function, takes input s and returns estimated cost to goal
    (note h will also need access to the map, so should be a member function of GridMap)
    '''
    frontier = PriorityQ() #Priority queue so lowest cost options are explored
    visited = set() #Set of visited nodes
    n0 = SearchNode(init_state, actions)
    frontier.push(n0, 1)
    while frontier:
        n_curr = frontier.pop()
        if n_curr.state not in visited:
            visited.add(n_curr.state)
            if is_goal(n_curr.state):
                if(_DEBUG):
                    print("arrived at goal!")
                    print(n_curr.cost)
                return backpath(n_curr), visited, True
            else:
                for a in actions:
                    s_prime = f(n_curr.state, a)
                    g_cost = h(s_prime)
                    n_prime = SearchNode(s_prime, actions, n_curr, a)
                    frontier.push(n_prime, n_prime.cost + g_cost)
    # Fill me in!
    return [], visited, False

def backpath(node):
    '''
    Function to determine the path that lead to the specified search node

    node - the SearchNode that is the end of the path

    returns - a tuple containing (path, action_path) which are lists respectively of the states
    visited from init to goal (inclusive) and the actions taken to make those transitions.
    '''
    path = []
    action_path = []

    path.append(node.state)
    action_path.append(node.parent_action)
    while node.parent is not None:
            node = node.parent
            path.append(node.state)
            action_path.append(node.parent_action)

    #Change the lists from last to first to first to last
    path.reverse()
    action_path.reverse()

    return path, action_path
