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


def userControlled_partition(filename,gwg,partitionGrid,moveobstacles,invisibilityset,jsonfile):
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
    gridstate = copy.deepcopy(moveobstacles)
    output = BeliefIOParser(jsonfile)
    while True:
        output.saveState(gwg, automaton, automaton_state,gridstate,moveobstacles)
        envstate = [0 for i in range(len(moveobstacles))]
        for n in range(0,len(moveobstacles)):
            envstate[n] = automaton[automaton_state]['State']['st{}'.format(n)]
        # try:
        #     print 'Agent state is J ', agentstate
        # except:
        #     print 'nothingyet'

        
        gwg.moveobstacles = copy.deepcopy(gridstate)
        gwg.render()
        # ----------------- Robot Waits For Key Stroke To Take Action-----------------
        # while True:
        #             arrow = gwg.getkeyinput()
        #             if arrow != None:
        #                 break
        # ---------------------------------------------------------------------------------
        
        agentstate = automaton[automaton_state]['State']['s_c']
        print 'Agent state is ', agentstate
        gwg.render()
        # gwg.moveobstacles[0] = copy.deepcopy(gridstate)

        gwg.render()
        gwg.current = [copy.deepcopy(agentstate)]

        gwg.colorstates[0] = set()
        gwg.colorstates[0].update(invisibilityset[0][agentstate])
        gwg.render()
        # gwg.draw_state_labels()
        
        nextstates = automaton[automaton_state]['Successors']
        nextstatedirn = {'W':{'W':None,'E':None,'S':None,'N':None,'R':None, 'Belief':set()},'E':{'W':None,'E':None,'S':None,'N':None,'R':None, 'Belief':set()},'S':{'W':None,'E':None,'S':None,'N':None,'R':None, 'Belief':set()},'N':{'W':None,'E':None,'S':None,'N':None,'R':None, 'Belief':set()},'R':{'W':None,'E':None,'S':None,'N':None,'R':None, 'Belief':set()}, 'Belief':{'W':set(),'E':set(),'S':set(),'N':set(),'R':set(), 'Belief':set()}}
        # Need to Change this to add multiple next automaton states for one moving obstacle action (based on delivery request only I think, in this case only consider delivery request to be true). Need to add correct obstacle transition to specifications
        
        for n in nextstates:
            nenvstate = [automaton[n]['State']['st0'],automaton[n]['State']['st1']]
            if nenvstate[0] == (gwg.moveobstacles[0] - 1):
                if nenvstate[1] == gwg.moveobstacles[1] - 1:
                    nextstatedirn['W']['W'] = n
                if nenvstate[1] == gwg.moveobstacles[1] + 1:
                    nextstatedirn['W']['E'] = n
                if nenvstate[1] == gwg.moveobstacles[1] + gwg.ncols:
                    nextstatedirn['W']['S'] = n
                if nenvstate[1] == gwg.moveobstacles[1] - gwg.ncols:
                    nextstatedirn['W']['N'] = n
                if nenvstate[1] == gwg.moveobstacles[1]:
                    nextstatedirn['W']['R'] = n
                if nenvstate[1] not in xstates:
                    nextstatedirn['W']['Belief'].add(n)
            if nenvstate[0] == gwg.moveobstacles[0] + 1:
                if nenvstate[1] == gwg.moveobstacles[1] - 1:
                    nextstatedirn['E']['W'] = n
                if nenvstate[1] == gwg.moveobstacles[1] + 1:
                    nextstatedirn['E']['E'] = n
                if nenvstate[1] == gwg.moveobstacles[1] + gwg.ncols:
                    nextstatedirn['E']['S'] = n
                if nenvstate[1] == gwg.moveobstacles[1] - gwg.ncols:
                    nextstatedirn['E']['N'] = n
                if nenvstate[1] == gwg.moveobstacles[1]:
                    nextstatedirn['E']['R'] = n
                if nenvstate[1] not in xstates:
                    nextstatedirn['E']['Belief'].add(n)
            if nenvstate[0] == gwg.moveobstacles[0] + gwg.ncols:
                if nenvstate[1] == gwg.moveobstacles[1] - 1:
                    nextstatedirn['S']['W'] = n
                if nenvstate[1] == gwg.moveobstacles[1] + 1:
                    nextstatedirn['S']['E'] = n
                if nenvstate[1] == gwg.moveobstacles[1] + gwg.ncols:
                    nextstatedirn['S']['S'] = n
                if nenvstate[1] == gwg.moveobstacles[1] - gwg.ncols:
                    nextstatedirn['S']['N'] = n
                if nenvstate[1] == gwg.moveobstacles[1]:
                    nextstatedirn['S']['R'] = n
                if nenvstate[1] not in xstates:
                    nextstatedirn['S']['Belief'].add(n)
            if nenvstate[0] == gwg.moveobstacles[0] - gwg.ncols:
                if nenvstate[1] == gwg.moveobstacles[1] - 1:
                    nextstatedirn['N']['W'] = n
                if nenvstate[1] == gwg.moveobstacles[1] + 1:
                    nextstatedirn['N']['E'] = n
                if nenvstate[1] == gwg.moveobstacles[1] + gwg.ncols:
                    nextstatedirn['N']['S'] = n
                if nenvstate[1] == gwg.moveobstacles[1] - gwg.ncols:
                    nextstatedirn['N']['N'] = n
                if nenvstate[1] == gwg.moveobstacles[1]:
                    nextstatedirn['N']['R'] = n
                if nenvstate[1] not in xstates:
                    nextstatedirn['N']['Belief'].add(n)
            if nenvstate[0] == gwg.moveobstacles[0]:
                if nenvstate[1] == gwg.moveobstacles[1] - 1:
                    nextstatedirn['R']['W'] = n
                if nenvstate[1] == gwg.moveobstacles[1] + 1:
                    nextstatedirn['R']['E'] = n
                if nenvstate[1] == gwg.moveobstacles[1] + gwg.ncols:
                    nextstatedirn['R']['S'] = n
                if nenvstate[1] == gwg.moveobstacles[1] - gwg.ncols:
                    nextstatedirn['R']['N'] = n
                if nenvstate[1] == gwg.moveobstacles[1]:
                    nextstatedirn['R']['R'] = n
                if nenvstate[1] not in xstates:
                    nextstatedirn['R']['Belief'].add(n)
            if nenvstate[0] not in xstates:
                if nenvstate[1] == gwg.moveobstacles[1] - 1:
                    nextstatedirn['Belief']['W'].add(n)
                if nenvstate[1] == gwg.moveobstacles[1] + 1:
                    nextstatedirn['Belief']['E'].add(n)
                if nenvstate[1] == gwg.moveobstacles[1] + gwg.ncols:
                    nextstatedirn['Belief']['S'].add(n)
                if nenvstate[1] == gwg.moveobstacles[1] - gwg.ncols:
                    nextstatedirn['Belief']['N'].add(n)
                if nenvstate[1] == gwg.moveobstacles[1]:
                    nextstatedirn['Belief']['R'].add(n)
                if nenvstate[1] not in xstates:
                    nextstatedirn['Belief']['Belief'].add(n)
            # if nenvstate[0] not in xstates:
            #     if nenvstate[1] == gwg.moveobstacles[1] - 1:
            #         nextstatedirn['Belief']['W'] = n
            #     if nenvstate[1] == gwg.moveobstacles[1] + 1:
            #         nextstatedirn['Belief']['E'] = n
            #     if nenvstate[1] == gwg.moveobstacles[1] + gwg.ncols:
            #         nextstatedirn['Belief']['S'] = n
            #     if nenvstate[1] == gwg.moveobstacles[1] - gwg.ncols:
            #         nextstatedirn['Belief']['N'] = n
            #     if nenvstate[1] == gwg.moveobstacles[1]:
            #         nextstatedirn['Belief']['R'] = n
            #     if nenvstate[1] not in xstates:
            #         nextstatedirn['Belief']['Belief'].add(n)
                
        while True:
            nextstate = None
            while nextstate == None:
                while True:
                    arrow1 = gwg.getkeyinput()
                    if arrow1 != None:
                        break
                while True:
                    arrow2 = gwg.getkeyinput()
                    if arrow2 != None:
                        break
                        
                
                # nextstate = nextstatedirn[arrow1][arrow2]
                case = 0
                if nextstatedirn[arrow1][arrow2] != None:
                    nextstate = nextstatedirn[arrow1][arrow2]
                    case = 1
                elif len(nextstatedirn[arrow1]['Belief']) != 0:
                    print "next states if only obstacle 1 is visible" + str(nextstatedirn[arrow1]['Belief'])
                    # nextstate = nextstatedirn[arrow1]['Belief']
                    case = 2
                elif len(nextstatedirn['Belief'][arrow2]) != 0:
                    print "next states if only obstacle 2 is visible" + str(nextstatedirn['Belief'][arrow2])
                    case = 3
                elif len(nextstatedirn['Belief']['Belief']) != 0:
                    print "next states if no obstacles are visible" + str(nextstatedirn['Belief']['Belief'])
                    case = 4
                else:
                    print "HOUSTON WE HAVE A PROBLEM"
                    case = 5

                print "case =" + str(case)

                if nextstate == None:
                    if arrow1 == 'W':
                        gridstate[0] = gwg.moveobstacles[0] - 1
                    elif arrow1 == 'E':
                        gridstate[0] = gwg.moveobstacles[0] + 1
                    elif arrow1 == 'S':
                        gridstate[0] = gwg.moveobstacles[0] + gwg.ncols
                    elif arrow1 == 'N':
                        gridstate[0] = gwg.moveobstacles[0] - gwg.ncols
                    elif arrow1 == 'R':
                        gridstate[0] = gwg.moveobstacles[0]
                    
                    if arrow2 == 'W':
                        gridstate[1] = gwg.moveobstacles[1] - 1
                    elif arrow2 == 'E':
                        gridstate[1] = gwg.moveobstacles[1] + 1
                    elif arrow2 == 'S':
                        gridstate[1] = gwg.moveobstacles[1] + gwg.ncols
                    elif arrow2 == 'N':
                        gridstate[1] = gwg.moveobstacles[1] - gwg.ncols
                    elif arrow2 == 'R':
                        gridstate[1] = gwg.moveobstacles[1]
                    
                    
                    
                    if case == 2:
                        for n in nextstatedirn[arrow1]['Belief']:
                            nenvstate = [automaton[n]['State']['st0'],automaton[n]['State']['st1']]
                            nextbeliefs = beliefcombs[len(beliefcombs) - (len(allstates) - allstates.index(nenvstate[1]))]
                            # try:
                            #     nextbeliefs = beliefcombs[len(beliefcombs) - (len(allstates) - allstates.index(nenvstate[0]))]
                            # except:
                            #     nextstate = 124
                            #     continue
                            if any(gridstate[1] in partitionGrid[x] for x in nextbeliefs):
                                nextstate = copy.deepcopy(n)
                                # print 'Environment state in automaton is', allstates.index(nenvstate[0])
                                # print 'Belief state is', beliefcombs[allstates.index(nenvstate[0]) - len(xstates)]
                                nextagentstate = automaton[n]['State']['s_c']
                                invisstates = invisibilityset[0][nextagentstate]
                        
                                visstates = set(xstates) - set(invisstates)
                                if nenvstate[1] not in xstates:
                                    beliefcombstate = beliefcombs[allstates.index(nenvstate[1]) - len(xstates)]
                                    beliefstates = set()
                                    for b in beliefcombstate:
                                        beliefstates = beliefstates.union(partitionGrid[b])
                                    truebeliefstates = beliefstates - beliefstates.intersection(visstates)
                                    gwg.colorstates[1] = copy.deepcopy(truebeliefstates)
                                    gwg.render()
                                    print 'True belief set is ', truebeliefstates
                                    print 'Size of true belief set is ', len(truebeliefstates)
                    elif case == 3:
                        for n in nextstatedirn['Belief'][arrow2]:
                            nenvstate = [automaton[n]['State']['st0'],automaton[n]['State']['st1']]
                            nextbeliefs = beliefcombs[len(beliefcombs) - (len(allstates) - allstates.index(nenvstate[0]))]
                            # try:
                            #     nextbeliefs = beliefcombs[len(beliefcombs) - (len(allstates) - allstates.index(nenvstate[0]))]
                            # except:
                            #     nextstate = 124
                            #     continue
                            if any(gridstate[0] in partitionGrid[x] for x in nextbeliefs):
                                nextstate = copy.deepcopy(n)
                                # print 'Environment state in automaton is', allstates.index(nenvstate[0])
                                # print 'Belief state is', beliefcombs[allstates.index(nenvstate[0]) - len(xstates)]
                                nextagentstate = automaton[n]['State']['s_c']
                                invisstates = invisibilityset[0][nextagentstate]
                        
                                visstates = set(xstates) - set(invisstates)
                                if nenvstate[0] not in xstates:
                                    beliefcombstate = beliefcombs[allstates.index(nenvstate[0]) - len(xstates)]
                                    beliefstates = set()
                                    for b in beliefcombstate:
                                        beliefstates = beliefstates.union(partitionGrid[b])
                                    truebeliefstates = beliefstates - beliefstates.intersection(visstates)
                                    gwg.colorstates[1] = copy.deepcopy(truebeliefstates)
                                    gwg.render()
                                    print 'True belief set is ', truebeliefstates
                                    print 'Size of true belief set is ', len(truebeliefstates)
                    elif case == 4:
                        for n in nextstatedirn['Belief']['Belief']:
                            nenvstate = [automaton[n]['State']['st0'],automaton[n]['State']['st1']]
                            nextbeliefs = [beliefcombs[len(beliefcombs) - (len(allstates) - allstates.index(nenvstate[0]))],beliefcombs[len(beliefcombs) - (len(allstates) - allstates.index(nenvstate[1]))]]
                            # try:
                            #     nextbeliefs = beliefcombs[len(beliefcombs) - (len(allstates) - allstates.index(nenvstate[0]))]
                            # except:
                            #     nextstate = 124
                            #     continue
                            if any(gridstate[0] in partitionGrid[x] for x in nextbeliefs[0]):
                                nextstate = copy.deepcopy(n)
                                # print 'Environment state in automaton is', allstates.index(nenvstate[0])
                                # print 'Belief state is', beliefcombs[allstates.index(nenvstate[0]) - len(xstates)]
                                nextagentstate = automaton[n]['State']['s_c']
                                invisstates = invisibilityset[0][nextagentstate]
                        
                                visstates = set(xstates) - set(invisstates)
                                if nenvstate[0] not in xstates:
                                    beliefcombstate = beliefcombs[allstates.index(nenvstate[0]) - len(xstates)]
                                    beliefstates = set()
                                    for b in beliefcombstate:
                                        beliefstates = beliefstates.union(partitionGrid[b])
                                    truebeliefstates = beliefstates - beliefstates.intersection(visstates)
                                    gwg.colorstates[1] = copy.deepcopy(truebeliefstates)
                                    gwg.render()
                                    print 'True belief set is ', truebeliefstates
                                    print 'Size of true belief set is ', len(truebeliefstates)


                    # for n in nextstatedirn['Belief']['Belief']:
                    #         nenvstate = [automaton[n]['State']['st0'],automaton[n]['State']['st0']]
                    #         nextbeliefs = [beliefcombs[len(beliefcombs) - (len(allstates) - allstates.index(nenvstate[0]))],beliefcombs[len(beliefcombs) - (len(allstates) - allstates.index(nenvstate[1]))]]
                    #         # try:
                    #         #     nextbeliefs = beliefcombs[len(beliefcombs) - (len(allstates) - allstates.index(nenvstate[0]))]
                    #         # except:
                    #         #     nextstate = 124
                    #         #     continue
                    #         if any(gridstate[0] in partitionGrid[x] for x in nextbeliefs[0]):
                    #             nextstate = copy.deepcopy(n)
                    #             # print 'Environment state in automaton is', allstates.index(nenvstate[0])
                    #             # print 'Belief state is', beliefcombs[allstates.index(nenvstate[0]) - len(xstates)]
                    #             nextagentstate = automaton[n]['State']['s_c']
                    #             invisstates = invisibilityset[0][nextagentstate]
                        
                    #             visstates = set(xstates) - set(invisstates)
                    #             if nenvstate[0] not in xstates:
                    #                 beliefcombstate = beliefcombs[allstates.index(nenvstate[0]) - len(xstates)]
                    #                 beliefstates = set()
                    #                 for b in beliefcombstate:
                    #                     beliefstates = beliefstates.union(partitionGrid[b])
                    #                 truebeliefstates = beliefstates - beliefstates.intersection(visstates)
                    #                 gwg.colorstates[1] = copy.deepcopy(truebeliefstates)
                    #                 gwg.render()
                    #                 print 'True belief set is ', truebeliefstates
                    #                 print 'Size of true belief set is ', len(truebeliefstates)
                else:
                    nenvstate = [automaton[nextstate]['State']['st0'],automaton[nextstate]['State']['st1']]
                    print 'Environment state in automaton is', allstates.index(nenvstate[0])
                    print 'Environment state in grid is', nenvstate
                    gridstate = copy.deepcopy(nenvstate)
                    gwg.colorstates[1] = set()
                    gwg.render()

            try:
                print 'orientation is', automaton[automaton_state]['State']['orientation']
                print 'directionrequest is ', automaton[automaton_state]['State']['directionrequest']
            except:
                break

            if len(automaton[nextstate]['Successors']) > 0:
                break

        print 'Automaton state is ', nextstate
        # print 'actions: \norientation: ' + str(automaton[nextstate]['State']['orientation']) + '\nstop: ' + str(automaton[nextstate]['State']['stop']) + '\nturn Left: ' + str(automaton[nextstate]['State']['turnLeft']) + '\nturn Right: ' + str(automaton[nextstate]['State']['turnRight']) + '\nforward: ' + str(automaton[nextstate]['State']['forward'])  + '\nstepL: ' + str(automaton[nextstate]['State']['stepL'])  + '\nstanceFoot: ' + str(automaton[nextstate]['State']['stanceFoot'])
        # print 'sOld: ' + str(automaton[nextstate]['State']['sOld'])
        # print 'stanceFoot: ' + str(automaton[nextstate]['State']['stanceFoot'])
        # print 'pastTurnStanceMatchFoot: ' + str(automaton[nextstate]['State']['pastTurnStanceMatchFoot'])
        # print str(automaton[nextstate]['State'])
        automaton_state = copy.deepcopy(nextstate)


# 