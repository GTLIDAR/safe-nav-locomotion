__author__ = 'sudab'
import random
import simplejson as json
import time
import copy
import itertools
import Control_Parser
import numpy as np
from beliefIOParser import BeliefIOParser


def powerset(s):
    x = len(s)
    a = []
    for i in range(1,1<<x):
        a.append({s[j] for j in range(x) if (i &(1<<j))})
    return a

def dict_compare(d1, d2):
    d1_keys = set(d1.keys())
    d2_keys = set(d2.keys())
    intersect_keys = d1_keys.intersection(d2_keys)
    added = d1_keys - d2_keys
    removed = d2_keys - d1_keys
    modified = {o : (d1[o], d2[o]) for o in intersect_keys if d1[o] != d2[o]}
    same = set(o for o in intersect_keys if d1[o] == d2[o])
    return added, removed, modified, same

def parseJson(filename):
    automaton = dict()
    file = open(filename)
    data = json.load(file)
    file.close()
    variables = dict()
    for var in data['variables']:
            v = var.split('@')[0]
            if v not in variables.keys():
                for var2ind in range(data['variables'].index(var),len(data['variables'])):
                    var2 = data['variables'][var2ind]
                    if v != var2.split('@')[0]:
                        variables[v] = [data['variables'].index(var), data['variables'].index(var2)]
                        break
                    if data['variables'].index(var2) == len(data['variables'])-1:
                        variables[v] = [data['variables'].index(var), data['variables'].index(var2)+1]

    for s in data['nodes'].keys():
        automaton[int(s)] = dict.fromkeys(['State','Successors'])
        automaton[int(s)]['State'] = dict()
        automaton[int(s)]['Successors'] = []
        for v in variables.keys():
            if variables[v][0] == variables[v][1]:
                bin  = [data['nodes'][s]['state'][variables[v][0]]]
            else:
                bin = data['nodes'][s]['state'][variables[v][0]:variables[v][1]]
            automaton[int(s)]['State'][v] = int(''.join(str(e) for e in bin)[::-1], 2)
            automaton[int(s)]['Successors'] = data['nodes'][s]['trans']
    return automaton

def findstate(automaton,statedict):
    states = set()
    for s in automaton:
        added, removed, modified, same = dict_compare(automaton[s]['State'], statedict)
        if len(modified) == 0:
            states.add(s)
    return states

def getGridstate(gwg,currstate,dirn):
    if dirn == 'W':
        return currstate - 1
    elif dirn == 'E':
        return currstate + 1
    elif dirn == 'S':
        return currstate + gwg.ncols
    elif dirn == 'N':
        return currstate - gwg.ncols


def userControlled_partition(filename,gwg,partitionGrid,moveobstacles_f,invisibilityset,jsonfile):
    automaton = parseJson(filename)
    # for s in automaton:
    #     print automaton[s]['State']['s']

    automaton_state = 0
    xstates = list(set(gwg.states))
    allstates = copy.deepcopy(xstates)
    beliefcombs = powerset(partitionGrid.keys())
    for i in range(gwg.nstates,gwg.nstates+ len(beliefcombs)):
        allstates.append(i)
    gwg.colorstates = [set(), set()]
    gridstate = copy.deepcopy(moveobstacles_f[0])
    output = BeliefIOParser(jsonfile)
    while True:
        # output.saveState(gwg, automaton, automaton_state,gridstate,moveobstacles_f)
        # try:
        #     output.saveState(gwg, automaton, automaton_state,gridstate,moveobstacles_f)
        # except:
        #     print "broken Output"
        envstate = automaton[automaton_state]['State']['st']
        # try:
        #     print 'Agent state is J ', agentstate
        # except:
        #     print 'nothingyet'

        
        gwg.moveobstacles_f[0] = copy.deepcopy(gridstate)
        gwg.render()
        # -----------------Jonas Robot Waits For Key Stroke To Take Action-----------------
        # while True:
        #             arrow = gwg.getkeyinput()
        #             if arrow != None:
        #                 break
        # ---------------------------------------------------------------------------------
        
        agentstate = automaton[automaton_state]['State']['s']
        print 'Agent state is ', agentstate
        gwg.render()
        # gwg.moveobstacles_f[0] = copy.deepcopy(gridstate)

        gwg.render()
        gwg.current = [copy.deepcopy(agentstate)]

        gwg.colorstates[0] = set()
        gwg.colorstates[0].update(invisibilityset[0][agentstate])
        gwg.render()
        # gwg.draw_state_labels()
        
        nextstates = automaton[automaton_state]['Successors']
        nextstatedirn = {'W':None,'E':None,'S':None,'N':None,'R':None, 'Belief':set()}
        # Need to Change this to add multiple next automaton states for one moving obstacle action (based on delivery request only I think, in this case only consider delivery request to be true). Need to add correct obstacle transition to specifications
        for n in nextstates:
            nenvstate = automaton[n]['State']['directionrequest']
            if automaton[n]['State']['stair'] == 0:
                if nenvstate == 4:
                    nextstatedirn['W'] = n
                if nenvstate == 2:
                    nextstatedirn['E'] = n
                if nenvstate == 3:
                    nextstatedirn['S'] = n
                if nenvstate == 1:
                    nextstatedirn['N'] = n
                if nenvstate == 0:
                    nextstatedirn['R'] = n
                if nenvstate not in [0,1,2,3,4]:
                    print "error"

        while True:
            nextstate = None
            while nextstate == None:
                while True:
                    arrow = gwg.getkeyinput()
                    if arrow != None:
                        break
                nextstate = nextstatedirn[arrow]
                if nextstate == None:
                    if arrow == 'W':
                        gridstate = gwg.moveobstacles_f[0] - 1
                    elif arrow == 'E':
                        gridstate = gwg.moveobstacles_f[0] + 1
                    elif arrow == 'S':
                        gridstate = gwg.moveobstacles_f[0] + gwg.ncols
                    elif arrow == 'N':
                        gridstate = gwg.moveobstacles_f[0] - gwg.ncols
                    elif arrow == 'R':
                        gridstate = gwg.moveobstacles_f[0]
                   
                else:
                    nenvstate = automaton[nextstate]['State']['st']
                    print 'Environment state in automaton is', allstates.index(nenvstate)
                    print 'Environment state in grid is', nenvstate
                    gridstate = copy.deepcopy(nenvstate)
                    gwg.colorstates[1] = set()
                    gwg.render()


            if len(automaton[nextstate]['Successors']) > 0:
                break

        print 'Automaton state is ', nextstate
        print 'stepL: ' + str(automaton[nextstate]['State']['stepL'])
        automaton_state = copy.deepcopy(nextstate)



                    for n in nextstatedirn['Belief']:
                        nenvstate = automaton[n]['State']['st']
                        nextbeliefs = beliefcombs[len(beliefcombs) - (len(allstates) - allstates.index(nenvstate))]
                        if any(gridstate in partitionGrid[x] for x in nextbeliefs):
                            nextstate = copy.deepcopy(n)
                            print 'Environment state in automaton is', allstates.index(nenvstate)
                            print 'Belief state is', beliefcombs[allstates.index(nenvstate) - len(xstates)]
                            nextagentstate = automaton[n]['State']['s']
                            # Jonas
                            invisstates = invisibilityset[0][nextagentstate]
                            # visstates = set(xstates) - invisstates
                            # Jonas
                            visstates = set(xstates) - set(invisstates)
                            if nenvstate not in xstates:
                                beliefcombstate = beliefcombs[allstates.index(nenvstate) - len(xstates)]
                                beliefstates = set()