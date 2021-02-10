__author__ = 'sudab'
""" Generate a grid world """
import os, sys, getopt, pdb, string
import random
import numpy as np
import pygame
from skimage import io
import cv2
import pygame.locals as pgl
import copy
from collections import OrderedDict


class Gridworld():
    # a gridworld with uneven terrain
    def __init__(self, filename=None, initial=0, nrows=8, ncols=8, nagents=1, targets=[], obstacles=[], moveobstacles = [], regions=dict()):
        # walls are the obstacles. The edges of the gridworld will be included into the walls.
        # region is a string and can be one of: ['pavement','gravel', 'grass', 'sand']
        if filename[0] != None:
            data = io.imread(filename[0])
            data = cv2.resize(data, filename[1], interpolation=filename[2])
            regionkeys = {'deterministic'}
            (nrows,ncols) = data.shape[:2]
            data = data.flatten()
            obstacles = list(np.where(data<127)[0])
            # NEW: separate list classifying certain things as "resolvable" by the robot but these are still considered 'obstacles' by the MO
            resolvable = list(np.where((data<254) & (data>127))[0])
            #### END NEW ####
            regions = dict.fromkeys(regionkeys, {-1})
            regions['deterministic'] = range(nrows * ncols)

        def powerset(s):
            x = len(s)
            a = []
            for i in range(1,1<<x):
                a.append(list({s[j] for j in range(x) if (i &(1<<j))}))
            return a

        self.current = initial
        self.nrows = nrows
        self.ncols = ncols
        self.obstacles = obstacles
        self.resolvable = resolvable #NEW: resolvable obstacles
        self.regions = regions
        self.nagents = nagents
        self.nstates = nrows * ncols
        self.actlistMO = ['R','N', 'S', 'W', 'E']
        self.actlistR = ['forward','turnLeft','turnRight','stepL0','stepL1','stepL2','stepL3','stop']
        self.robotState = ['forward','turnLeft','turnRight','stepL0','stepL1','stepL2','stepL3','stop', 'O0', 'O1', 'O2', 'O3', 'O4', 'O5', 'O6', 'O7', 'O8', 'O9', 'O10', 'O11']
        # ['requestPending1','requestPending2']
        self.RobotTurn = ['none','turnLeft','turnRight']
        self.RobotTurnLast = ['none','turnLeft','turnRight']
        # self.RobotStepL = ['short', 'medium', 'long', 'special']
        self.RobotStepL = [0,1,2,3]
        self.RobotForward = ['forward', 'notForward']
        self.robotStop = ['stop', 'notStop']
        self.orientation = [0,1,2,3,4,5,6,7,8,9,10,11]
        self.norientations = len((self.orientation))
        # self.actioncomb = powerset(self.actlistR)


    
        self.nactionsMO = len(self.actlistMO)
        self.nactionsR = len(self.actlistR)
        self.targets = targets
        self.left_edge = []
        self.right_edge = []
        self.top_edge = []
        self.bottom_edge = []
        
        self.left_edge2 = []
        self.right_edge2 = []
        self.top_edge2 = []
        self.bottom_edge2 = []

        self.left_edge3 = []
        self.right_edge3 = []
        self.top_edge3 = []
        self.bottom_edge3 = []

        self.regions = regions
        self.moveobstacles = moveobstacles
        self.states = range(nrows*ncols)
        self.colorstates = set()
        for x in range(self.nstates):
            # note that edges are not disjoint, so we cannot use elif
            if x % self.ncols == 0:
                self.left_edge.append(x)
                self.left_edge2.append(x+1)
                self.left_edge3.append(x+2)
            if 0 <= x < self.ncols:
                self.top_edge.append(x)
                self.top_edge2.append(x+self.ncols)
                self.top_edge3.append(x+2*self.ncols)
            if x % self.ncols == self.ncols - 1:
                self.right_edge.append(x)
                self.right_edge2.append(x-1)
                self.right_edge3.append(x-2)
            if (self.nrows - 1) * self.ncols <= x <= self.nstates:
                self.bottom_edge.append(x)
                self.bottom_edge2.append(x-self.ncols)
                self.bottom_edge3.append(x-2*self.ncols)
        self.edges = self.left_edge + self.top_edge + self.right_edge + self.bottom_edge
        self.walls = self.edges + obstacles

        edges2_temp = self.left_edge2 + self.top_edge2 + self.right_edge2 + self.bottom_edge2
        edges2_temp = [x for x in edges2_temp if x not in self.edges]
        self.edges2 = list( OrderedDict.fromkeys(edges2_temp) )
        edges3_temp = self.left_edge3 + self.top_edge3 + self.right_edge3 + self.bottom_edge3
        edges3_temp = [x for x in edges3_temp if x not in self.edges and x not in self.edges2]
        self.edges3 = list( OrderedDict.fromkeys(edges3_temp) )

        self.obsborder = set()
        for obs in obstacles:
            (row,col)=self.coords(obs)
            tempobs = []
            # obsN = gw.coords2state_works(row-1,col)
            # obsNE = gw.coords2state_works(row-1,col+1)
            # obsE = gw.coords2state_works(row,col+1)
            # obsSE = gw.coords2state_works(row+1,col+1)
            # obsS = gw.coords2state_works(row+1,col)
            # obsSW = gw.coords2state_works(row+1,col-1)
            # obsW = gw.coords2state_works(row,col-1)
            # obsNW = gw.coords2state_works(row-1,col-1)
            tempobs.append(self.coords2state_works(row-1,col))
            tempobs.append(self.coords2state_works(row-1,col+1))
            tempobs.append(self.coords2state_works(row,col+1))
            tempobs.append(self.coords2state_works(row+1,col+1))
            tempobs.append(self.coords2state_works(row+1,col))
            tempobs.append(self.coords2state_works(row+1,col-1))
            tempobs.append(self.coords2state_works(row,col-1))
            tempobs.append(self.coords2state_works(row-1,col-1))

            tempobs.append(self.coords2state_works(row-2,col))
            tempobs.append(self.coords2state_works(row-2,col+1))
            # tempobs.append(self.coords2state_works(row-2,col+2))
            tempobs.append(self.coords2state_works(row-1,col+2))
            tempobs.append(self.coords2state_works(row,col+2))
            tempobs.append(self.coords2state_works(row+1,col+2))
            # tempobs.append(self.coords2state_works(row+2,col+2))
            tempobs.append(self.coords2state_works(row+2,col+1))
            tempobs.append(self.coords2state_works(row+2,col))
            tempobs.append(self.coords2state_works(row+2,col-1))
            # tempobs.append(self.coords2state_works(row+2,col-2))
            tempobs.append(self.coords2state_works(row+1,col-2))
            tempobs.append(self.coords2state_works(row,col-2))
            tempobs.append(self.coords2state_works(row-1,col-2))
            # tempobs.append(self.coords2state_works(row-2,col-2))
            tempobs.append(self.coords2state_works(row-2,col-1))

            tempobs.append(self.coords2state_works(row-2,col+2))
            tempobs.append(self.coords2state_works(row+2,col+2))
            tempobs.append(self.coords2state_works(row+2,col-2))
            tempobs.append(self.coords2state_works(row-2,col-2))

            for location in tempobs:
                if location in self.states:
                    self.obsborder.add(location)

        self.level1states = [291,292,293,294,295,296,329,334,367,372,405,410,443,448,481,482,483,484,485,486]
        self.level2states = [330,331,332,333,368,369,370,371,406,407,408,409,444,445,446,447]
        self.level0states = set(self.states) - set(self.level1states) - set(self.level2states)


        # self.stair_states = [47]
        # self.no_robot = []
        # self.no_obs = [22,23,24,35,36,37,48,49,50,61,62,63,74,75,76,46]
        # self.stair_states = [18]
        # self.no_robot = []
        # self.no_obs = [19,20,30,31,41,42,52,53,62,63,64,17]

        # self.stair_states = [30,31]
        # self.no_robot = []
        # self.no_obs = [20,21,22,32,33,34,44,45,46,56,57,58,29]

        self.stair_states = [0]
        self.no_robot = []
        self.no_obs = []


        self.probMO = {a: np.zeros((self.nstates, self.nstates)) for a in self.actlistMO}
        self.probR = {a: np.zeros((self.nstates, self.nstates)) for a in self.actlistR}

  

        for s in self.states:
            for a in self.actlistMO:
                self.getProbsMO(s, a)

        # need to add actlistR version
        # see below
        for s in self.states:
            # for a in self.actioncomb:
            for a in self.actlistR:
                for o in self.orientation:
                    self.getProbsR(s, a, o)


    def coords(self, s):
        return (s / self.ncols, s % self.ncols)  # the coordinate for state s.

    def coords2state_works(self,row,col):
        return ((row)*self.ncols + col)

    def isAllowed(self, (row,col)):
        if col not in range(self.ncols) or row not in range(self.nrows):
            return False
        return True

    def isAllowedState(self,(row,col),returnState):
        if self.isAllowed((row,col)):
            return self.rcoords((row,col))
        return returnState

    def physicallyValid(self):
        # check that all MOs are in valid spaces
        for o in self.moveobstacles:
            if (o in self.obstacles) or (o in self.resolvable):
                return False
        # check that agent is in valid space
        if self.current in self.obstacles:
            return False

        # if both valid, return True
        return True


    def rcoords(self, coords):
        s = coords[0] * self.ncols + coords[1]
        return s



    def getProbsMO(self, state, action):
        successors = []

        if state in self.obstacles or state in self.resolvable: ### NEW: MO (quadcopter) cannot go through obstacles, but robot can resolve them:
            successors = [(state, 1)]
            for (next_state, p) in successors:
                self.probMO[action][state, next_state] = p
                return
        row,col = self.coords(state)
        northState = self.isAllowedState((row-1,col),state)
        # isAllowedState returns first input if it is allowed, otherwise returns second input
        southState = self.isAllowedState((row+1,col),state)
        westState = self.isAllowedState((row,col-1),state)
        eastState = self.isAllowedState((row,col+1),state)

        reg = 'deterministic'
        if action == 'N':
            successors.append((northState, 1))
            # state, 1 (1 = probability of going to that state if intending to, got rid of probability of going to other states)

        if action == 'S':
            successors.append((southState, 1))

        if action == 'W':
            successors.append((westState, 1))

        if action == 'E':
            successors.append((eastState, 1))

        if action == 'R':
            successors.append((state,1))

        for (next_state, p) in successors:
            self.probMO[action][state, next_state] += p
        
    def getProbsR(self, state, action, orientation):
        successorsR = []

        if state in self.obstacles:
            successorsR = [(state, 1)]
            for (next_state, p) in successorsR:
                self.probR[action][state, next_state] = p
                return
        row,col = self.coords(state)
        northState = self.isAllowedState((row-1,col),state)
        northwestState = self.isAllowedState((row-1,col-1),state)
        northeastState = self.isAllowedState((row-1,col+1),state)
        southState = self.isAllowedState((row+1,col),state)
        southeastState = self.isAllowedState((row+1,col+1),state)
        southwestState = self.isAllowedState((row+1,col-1),state)
        westState = self.isAllowedState((row,col-1),state)
        eastState = self.isAllowedState((row,col+1),state)

        reg = self.getStateRegion(state)
        if action == 'N':
            successorsR.append((northState, 1))

        if action == 'S':
            successorsR.append((southState, 1))

        if action == 'W':
            successorsR.append((westState, 1))

        if action == 'E':
            successorsR.append((eastState, 1))

        if action == 'R':
            successorsR.append((state,1))

        for (next_state, p) in successorsR:
            self.probR[action][state, next_state] += p  

    def getStateRegion(self, state):
        if state in self.regions['deterministic']:
            return 'deterministic'

    ## NEW: resolving a blockage
    def resolveBlockage(self, state):
        if state in self.resolvable:
            self.resolvable.remove(state)
            self.bg_rendered = False

    ## Everything from here onwards is for creating the image

    def render(self, size=60):
        self.height = self.nrows * size + self.nrows + 1
        self.width = self.ncols * size + self.ncols + 1
        self.size = size

        #       # initialize pygame ( SDL extensions )
        pygame.init()
        pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption('Gridworld')
        self.screen = pygame.display.get_surface()
        # gets info on what is currently displayed
        self.surface = pygame.Surface(self.screen.get_size())
        self.bg = pygame.Surface(self.screen.get_size())
        self.bg_rendered = False  # optimize background render

        self.background()
        self.screen.blit(self.surface, (0, 0))
        pygame.display.flip()

        self.build_templates()
        self.updategui = True  # switch to stop updating gui if you want to collect a trace quickly

        self.state2circle(self.current)

    def getkeyinput(self):
        events = pygame.event.get()
        for event in events:
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_LEFT:
                    return 'W'
                elif event.key == pygame.K_RIGHT:
                    return 'E'
                if event.key == pygame.K_UP:
                    return 'N'
                elif event.key == pygame.K_DOWN:
                    return 'S'
                elif event.key == pygame.K_SPACE:
                    return 'R'

    def build_templates(self):

        # Note: template already in "graphics" coordinates
        template = np.array([(-1, 0), (0, 0), (1, 0), (0, 1), (1, 0), (0, -1)])
        template = self.size / 3 * template  # scale template

        v = 1.0 / np.sqrt(2)
        rot90 = np.array([(0, 1), (-1, 0)])
        rot45 = np.array([(v, -v), (v, v)])  # neg


        #
        # align the template with the first action.
        t0 = np.dot(template, rot90)
        t0 = np.dot(t0, rot90)
        t0 = np.dot(t0, rot90)

        t1 = np.dot(t0, rot45)
        t2 = np.dot(t1, rot45)
        t3 = np.dot(t2, rot45)
        t4 = np.dot(t3, rot45)
        t5 = np.dot(t4, rot45)
        t6 = np.dot(t5, rot45)
        t7 = np.dot(t6, rot45)

        self.t = [t0, t1, t2, t3, t4, t5, t6, t7]

    def indx2coord(self, s, center=False):
        # the +1 indexing business is to ensure that the grid cells
        # have borders of width 1px
        i, j = self.coords(s)
        if center:
            return i * (self.size + 1) + 1 + self.size / 2, \
                   j * (self.size + 1) + 1 + self.size / 2
        else:
            return i * (self.size + 1) + 1, j * (self.size + 1) + 1

    def accessible_blocks(self, s):
        """
        For a give state s, generate the list of walls around it.
        """
        W = []
        if s in self.walls:
            return W
        if s - self.ncols < 0 or s - self.ncols in self.walls:
            pass
        else:
            W.append(s - self.ncols)
        if s - 1 < 0 or s - 1 in self.walls:
            pass
        else:
            W.append(s - 1)
        if s + 1 in self.walls:
            pass
        else:
            W.append(s + 1)
        if s + self.ncols in self.walls:
            pass
        else:
            W.append(s + self.ncols)
        return W

    def coord2indx(self, (x, y)):
        return self.rcoords((x / (self.size + 1), y / (self.size + 1)))

    def draw_state_labels(self):
        font = pygame.font.SysFont("FreeSans", 10)
        for s in range(self.nstates):
            x, y = self.indx2coord(s, False)
            txt = font.render("%d" % s, True, (0, 0, 0))
            self.surface.blit(txt, (y, x))

        self.screen.blit(self.surface, (0, 0))
        pygame.display.flip()

    def coord2state(self, coord):
        s = self.coord2indx((coord[0], coord[1]))
        return s

    def state2circle(self, state, bg=True, blit=True):
        if bg:
            self.background()

        for n in range(self.nagents):
            x, y = self.indx2coord(state[n], center=True)
            pygame.draw.circle(self.surface, (0+(50*n), 0+(20*n), 255.0/(n+1)), (y, x), (self.size / 2)-2)
        if len(self.moveobstacles) > 0:
            for s in self.moveobstacles:
                x, y = self.indx2coord(s, center=True)
                pygame.draw.circle(self.surface, (205, 92, 0), (y, x), (self.size / 2)-2)
        if blit:
            self.screen.blit(self.surface, (0, 0))
            pygame.display.flip()

    def draw_values(self, vals):
        """
        vals: a dict with state labels as the key
        """
        font = pygame.font.SysFont("FreeSans", 10)

        for s in range(self.nstates):
            x, y = self.indx2coord(s, False)
            v = vals[s]
            txt = font.render("%.1f" % v, True, (0, 0, 0))
            self.surface.blit(txt, (y, x))

        self.screen.blit(self.surface, (0, 0))
        pygame.display.flip()

    #
    def save(self, filename):
        pygame.image.save(self.surface, filename)

    def redraw(self):
        self.screen.blit(self.surface, (0, 0))
        pygame.display.flip()

    def move_obj(self, s, bg=True, blit=True):

        """Including A moving object into the gridworld, which moves uniformly at
        random in all accessible directions (including idle), without
        hitting the wall or another other statitic obstacle.  Input: a
        gridworld gui, the current state index for the obstacle and the
        number of steps.

        """
        if bg:
            self.background()
        x, y = self.indx2coord(s, center=True)
        pygame.draw.circle(self.surface, (205, 92, 0), (y, x), self.size / 2)

        if blit:
            self.screen.blit(self.surface, (0, 0))
            pygame.display.flip()

        return

    def move_deter(self, next_state):
        self.current = next_state

        return

    def background(self):

        if self.bg_rendered:
            self.surface.blit(self.bg, (0, 0))
        else:
            self.bg.fill((84, 84, 84))
            font = pygame.font.SysFont("FreeSans", 10)

            for s in range(self.nstates):
                x, y = self.indx2coord(s, False)
                coords = pygame.Rect(y, x, self.size-1, self.size-1)
                pygame.draw.rect(self.bg, ((250, 250, 250)), coords)
            for n in range(self.nagents):

                for t in self.targets[n]:
                    x, y = self.indx2coord(t, center=True)
                    coords = pygame.Rect(y - self.size / 2, x - self.size / 2, self.size, self.size)
                    pygame.draw.rect(self.bg, (0+(50*n), 204.0/(n+1), 102.0+(50*n)/(n+1)), coords)

            for s in self.obstacles:
                (x, y) = self.indx2coord(s)
                coords = pygame.Rect(y, x, self.size-1, self.size-1)
                pygame.draw.rect(self.bg, (255, 0, 0), coords)  # the obstacles are in color red

            for r in self.resolvable:
                (x, y) = self.indx2coord(r)
                coords = pygame.Rect(y, x, self.size, self.size)
                pygame.draw.rect(self.bg, (0, 165, 0), coords)  # the resolvable obstacles are in color green

            color = {'sand': (223, 225, 179), 'gravel': (255, 255, 255), 'grass': (211, 255, 192),
                     'pavement': (192, 255, 253),'deterministic': (255,255,255)}
            for s in range(self.nstates):
                if s not in self.edges and not any(s in x for x in self.targets) and s not in self.obstacles and s not in self.resolvable and not any(s in x for x in self.colorstates):
                    (x, y) = self.indx2coord(s)
                    coords = pygame.Rect(y - self.size / 2, x - self.size / 2, self.size, self.size)
                    coords = pygame.Rect(y, x, self.size-1, self.size-1)
                    pygame.draw.rect(self.bg, color[self.getStateRegion(s)], coords)  # the obstacles are in color grey
            statecols = [(0,0,0),(190,150,150),(30,177,237),(51,118,179),(114,192,77),(253,191,145),(121,63,158)]
            for i in range(len(self.colorstates)):
                for s in self.colorstates[i]:
                    if not any(s in x for x in self.targets) and s not in self.obstacles and s not in self.resolvable:
                        (x, y) = self.indx2coord(s)
                        coords = pygame.Rect(y, x, self.size-1, self.size-1)
                        pygame.draw.rect(self.bg, statecols[i], coords)  # the obstacles are in color grey

        self.bg_rendered = True  # don't render again unless flag is set
        self.surface.blit(self.bg, (0, 0))