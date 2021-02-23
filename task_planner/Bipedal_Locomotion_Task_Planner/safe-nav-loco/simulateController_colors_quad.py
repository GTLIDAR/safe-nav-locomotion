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


def userControlled_partition(filename,gwg,partitionGrid,moveobstacles,invisibilityset,jsonfile,jsonfile_obs):
    automaton = parseJson(filename)
    automaton_obs = parseJson(jsonfile_obs)
    # for s in automaton:
    #     print automaton[s]['State']['s']

    automaton_state = 0
    automaton_state_obs = 0
    xstates = list(set(gwg.states))
    allstates = copy.deepcopy(xstates)
    beliefcombs = powerset(partitionGrid.keys())
    for i in range(gwg.nstates,gwg.nstates+ len(beliefcombs)):
        allstates.append(i)
    gwg.colorstates = [set(), set(), set(), set(), set(), set(), set()]
    gridstate = copy.deepcopy(moveobstacles[0])
    output = BeliefIOParser(jsonfile)
    states_of_interest = {}
    for o in gwg.resolvable:
        for s in gwg.resolution[o]['state']:
            states_of_interest[s] = {'action': gwg.resolution[o]['action'],'resolves': o}
    j = 0

    prev_cassie = gwg.current[0]
    prev_quad = gwg.moveobstacles[0]
    while True:
        j +=1
        output.saveState(gwg, automaton, automaton_state,gridstate,moveobstacles,gwg)
        envstate = automaton_obs[automaton_state_obs]['State']['st']
      
        
        gwg.moveobstacles[0] = copy.deepcopy(gridstate)
        gwg.render()
        if gwg.physicalViolation() != -1:
            print("SYSTEM ENTERED PHYSICALLY INVALID STATE")
            break;
        elif gwg.current[0] in states_of_interest and states_of_interest[gwg.current[0]]['action'] == 'push':
            print("CASSIE RESOLVED AN OBSTACLE")
            gwg.resolveObstacle(gwg.current[0])
            break;
        elif gwg.moveobstacles[0] in states_of_interest and states_of_interest[gwg.moveobstacles[0]]['action'] == 'sense':
            print("QUADCOPTER RESOLVED AN OBSTACLE")
            gwg.resolveObstacle(states_of_interest[gwg.moveobstacles[0]]['resolves'])
            break;

        if j == 1:
            while True:
                arrow = gwg.getkeyinput()
                if arrow != None:
                    break
        # ----------------- Robot Waits For Key Stroke To Take Action-----------------
        # while True:
        #             arrow = gwg.getkeyinput()
        #             if arrow != None:
        #                 break
        # ---------------------------------------------------------------------------------
        time.sleep(0.2)
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
        if gwg.physicalViolation() != -1:
            print("SYSTEM ENTERED PHYSICALLY INVALID STATE")
            break;
        elif gwg.current[0] in states_of_interest and states_of_interest[gwg.current[0]]['action'] == 'push':
            print("CASSIE RESOLVED AN OBSTACLE")
            gwg.resolveObstacle(gwg.current[0])
            break;
        elif gwg.moveobstacles[0] in states_of_interest and states_of_interest[gwg.moveobstacles[0]]['action'] == 'sense':
            print("QUADCOPTER RESOLVED AN OBSTACLE")
            gwg.resolveObstacle(states_of_interest[gwg.moveobstacles[0]]['resolves'])
            break;

        prev_cassie = gwg.current[0]
        prev_quad = gwg.moveobstacles[0]
        
        nextstates = automaton[automaton_state]['Successors']
        nextstatedirn = {'W':None,'E':None,'S':None,'N':None,'R':None, 'Belief':set()}
        # Need to Change this to add multiple next automaton states for one moving obstacle action (based on delivery request only I think, in this case only consider delivery request to be true). Need to add correct obstacle transition to specifications
        for n in nextstates:
            nenvstate = automaton[n]['State']['st']
            if nenvstate == gwg.moveobstacles[0] - 1:
                nextstatedirn['W'] = n
            if nenvstate == gwg.moveobstacles[0] + 1:
                nextstatedirn['E'] = n
            if nenvstate == gwg.moveobstacles[0] + gwg.ncols:
                nextstatedirn['S'] = n
            if nenvstate == gwg.moveobstacles[0] - gwg.ncols:
                nextstatedirn['N'] = n
            if nenvstate == gwg.moveobstacles[0]:
                nextstatedirn['R'] = n
            if nenvstate not in xstates:
                nextstatedirn['Belief'].add(n)
        while True:
            nextstate = None
            while nextstate == None:


                # while True:
                #     arrow = gwg.getkeyinput()
                #     if arrow != None:
                #         break
                # nextstate = nextstatedirn[arrow]
                nextstates_obs = automaton_obs[automaton_state_obs]['Successors']
                nextstatedirn_obs = {'W':None,'E':None,'S':None,'N':None,'R':None}
                for n in nextstates_obs:
                    if automaton_obs[n]['State']['deliveryrequest'] == 1:
                        if automaton_obs[n]['State']['stop'] == automaton[automaton_state]['State']['stop']:
                            if automaton_obs[n]['State']['orientation'] == automaton[automaton_state]['State']['orientation']:
                                nenvstate_obs = automaton_obs[n]['State']['s_c']
                                # if nenvstate_obs == gwg.current[0] - 1:
                                #     nextstatedirn_obs['W'] = n
                                # if nenvstate_obs == gwg.current[0] + 1:
                                #     nextstatedirn_obs['E'] = n
                                # if nenvstate_obs == gwg.current[0] + gwg.ncols:
                                #     nextstatedirn_obs['S'] = n
                                # if nenvstate_obs == gwg.current[0] - gwg.ncols:
                                #     nextstatedirn_obs['N'] = n
                                # if nenvstate_obs == gwg.current[0]:
                                #     nextstatedirn_obs['R'] = n
                                if nenvstate_obs == automaton_obs[automaton_state_obs]['State']['s_c'] - 1:
                                    nextstatedirn_obs['W'] = n
                                if nenvstate_obs == automaton_obs[automaton_state_obs]['State']['s_c'] + 1:
                                    nextstatedirn_obs['E'] = n
                                if nenvstate_obs == automaton_obs[automaton_state_obs]['State']['s_c'] + gwg.ncols:
                                    nextstatedirn_obs['S'] = n
                                if nenvstate_obs == automaton_obs[automaton_state_obs]['State']['s_c'] - gwg.ncols:
                                    nextstatedirn_obs['N'] = n
                                if nenvstate_obs == automaton_obs[automaton_state_obs]['State']['s_c']:
                                    nextstatedirn_obs['R'] = n
                
                arrow_biped = None
                if gwg.current[0] == automaton_obs[automaton_state_obs]['State']['s_c']:
                    arrow_biped = 'R'
                if gwg.current[0] == automaton_obs[automaton_state_obs]['State']['s_c']-1:
                    arrow_biped = 'W'
                if gwg.current[0] == automaton_obs[automaton_state_obs]['State']['s_c']+1:
                    arrow_biped = 'E'
                if gwg.current[0] == automaton_obs[automaton_state_obs]['State']['s_c'] + gwg.ncols:
                    arrow_biped = 'S'
                if gwg.current[0] == automaton_obs[automaton_state_obs]['State']['s_c'] - gwg.ncols:
                    arrow_biped = 'N'
                    
                if arrow_biped == None:
                    print 'Error: Invalid robot move according to quadcopter automaton'
                nextstate_obs = nextstatedirn_obs[arrow_biped]
                gridstate = automaton_obs[nextstate_obs]['State']['st']
                #print 'Arrow_biped: ' + str(arrow_biped)
                
                arrow = None
                if gridstate == gwg.moveobstacles[0] - 1:
                    arrow = 'W'
                if gridstate == gwg.moveobstacles[0] + 1:
                    arrow = 'E'
                if gridstate == gwg.moveobstacles[0] + gwg.ncols:
                    arrow = 'S'
                if gridstate == gwg.moveobstacles[0] - gwg.ncols:
                    arrow = 'N'
                if gridstate == gwg.moveobstacles[0]:
                    arrow = 'R'

                nextstate = nextstatedirn[arrow]
                if nextstate == None:
                    if arrow == 'W':
                        gridstate = gwg.moveobstacles[0] - 1
                    elif arrow == 'E':
                        gridstate = gwg.moveobstacles[0] + 1
                    elif arrow == 'S':
                        gridstate = gwg.moveobstacles[0] + gwg.ncols
                    elif arrow == 'N':
                        gridstate = gwg.moveobstacles[0] - gwg.ncols
                    elif arrow == 'R':
                        gridstate = gwg.moveobstacles[0]
                    
                    for n in nextstatedirn['Belief']:
                        nenvstate = automaton[n]['State']['st']
                        nextbeliefs = beliefcombs[len(beliefcombs) - (len(allstates) - allstates.index(nenvstate))]
                        if any(gridstate in partitionGrid[x] for x in nextbeliefs):
                            nextstate = copy.deepcopy(n)
                            print 'Environment state in automaton is', allstates.index(nenvstate)
                            print 'Belief state is', beliefcombs[allstates.index(nenvstate) - len(xstates)]
                            nextagentstate = automaton[n]['State']['s_c']
                            invisstates = invisibilityset[0][nextagentstate]
                    
                            visstates = set(xstates) - set(invisstates)
                            if nenvstate not in xstates:
                                beliefcombstate = beliefcombs[allstates.index(nenvstate) - len(xstates)]
                                beliefstates = set()
                                for b in beliefcombstate:
                                    beliefstates = beliefstates.union(partitionGrid[b]) 
                                    gwg.colorstates[b+1] = copy.deepcopy(partitionGrid[b]) - partitionGrid[b].intersection(visstates)
                                truebeliefstates = beliefstates - beliefstates.intersection(visstates)
                                # gwg.colorstates[1] = copy.deepcopy(truebeliefstates)
                                gwg.render()
                                print 'True belief set is ', truebeliefstates
                                print 'Size of true belief set is ', len(truebeliefstates)
                else:
                    nenvstate = automaton[nextstate]['State']['st']
                    print 'Environment state in automaton is', allstates.index(nenvstate)
                    print 'Environment state in grid is', nenvstate
                    # gridstate = copy.deepcopy(nenvstate)
                    gwg.colorstates[1] = set()
                    gwg.colorstates[2] = set()
                    gwg.colorstates[3] = set()
                    gwg.colorstates[4] = set()
                    gwg.colorstates[5] = set()
                    gwg.colorstates[6] = set()
                    gwg.render()
            
            try:
                print 'orientation is', automaton[automaton_state]['State']['orientation']
                print 'directionrequest is ', automaton[automaton_state]['State']['directionrequest']
                print 'Automaton state is', str(automaton_state)
            except:
                break

            if len(automaton[nextstate]['Successors']) > 0:
                break
        # if nenvstate != gridstate:
        #     print 'Error: agent\'s automaton states are out of sync'
        print 'Automaton state is ', nextstate
        # print 'actions: \norientation: ' + str(automaton[nextstate]['State']['orientation']) + '\nstop: ' + str(automaton[nextstate]['State']['stop']) + '\nturn Left: ' + str(automaton[nextstate]['State']['turnLeft']) + '\nturn Right: ' + str(automaton[nextstate]['State']['turnRight']) + '\nforward: ' + str(automaton[nextstate]['State']['forward'])  + '\nstepL: ' + str(automaton[nextstate]['State']['stepL'])  + '\nstanceFoot: ' + str(automaton[nextstate]['State']['stanceFoot'])
        # print 'sOld: ' + str(automaton[nextstate]['State']['sOld'])
        # print 'stanceFoot: ' + str(automaton[nextstate]['State']['stanceFoot'])
        # print 'pastTurnStanceMatchFoot: ' + str(automaton[nextstate]['State']['pastTurnStanceMatchFoot'])
        # print str(automaton[nextstate]['State'])
        automaton_state = copy.deepcopy(nextstate)
        automaton_state_obs = copy.deepcopy(nextstate_obs)

    # return cassie orientation and direction request upon completion
    return (automaton[automaton_state]['State']['orientation'], automaton[automaton_state]['State']['directionrequest'], 
        automaton_obs[automaton_state_obs]['State']['orientation'], automaton_obs[automaton_state_obs]['State']['directionrequest'],
        prev_cassie, prev_quad)


