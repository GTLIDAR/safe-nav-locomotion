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

def getGridstate(gwg_c,currstate,dirn):
    if dirn == 'W':
        return currstate - 1
    elif dirn == 'E':
        return currstate + 1
    elif dirn == 'S':
        return currstate + gwg_c.ncols
    elif dirn == 'N':
        return currstate - gwg_c.ncols


def userControlled_partition(filename_c,gwg_c,partitionGrid_c,moveobstacles_c,invisibilityset_c,jsonfile_c,filename_f,gwg_f,partitionGrid_f,moveobstacles_f,invisibilityset_f,jsonfile_f):
    automaton = parseJson(filename_c)
    automaton_f = parseJson(filename_f)
    # for s in automaton:
    #     print automaton[s]['State']['s']

    automaton_state = 0
    automaton_state_f = 0
    xstates = list(set(gwg_c.states))
    xstates_f = list(set(gwg_f.states))
    allstates = copy.deepcopy(xstates)
    allstates_f = copy.deepcopy(xstates_f)
    beliefcombs = powerset(partitionGrid_c.keys())
    beliefcombs_f = powerset(partitionGrid_f.keys())

    for i in range(gwg_c.nstates,gwg_c.nstates+ len(beliefcombs)):
        allstates.append(i)
    for i in range(gwg_f.nstates,gwg_f.nstates+ len(beliefcombs_f)):
        allstates_f.append(i)
    gwg_c.colorstates = [set(), set()]
    gwg_f.colorstates = [set(), set()]
    gridstate = copy.deepcopy(moveobstacles_c[0])
    gridstate_f = copy.deepcopy(moveobstacles_f[0])
    output = BeliefIOParser(jsonfile_f)
    output_c = BeliefIOParser(jsonfile_c)
    gwg_c.render()
    time.sleep(0.3)
    while True:
        
        envstate = automaton[automaton_state]['State']['st']
        envstate_f = automaton_f[automaton_state_f]['State']['st']
        # try:
        #     print 'Agent state is J ', agentstate
        # except:
        #     print 'nothingyet'

        
        gwg_c.moveobstacles[0] = copy.deepcopy(gridstate)
        gwg_c.render()

        gwg_f.moveobstacles_f[0] = copy.deepcopy(gridstate_f)
        # gwg_f.render()
        # -----------------Robot Waits For Key Stroke To Take Action-----------------
        # while True:
        #             arrow = gwg_c.getkeyinput()
        #             if arrow != None:
        #                 break
        # ---------------------------------------------------------------------------------
        
        agentstate = automaton[automaton_state]['State']['s_c']
        nav_req = automaton[automaton_state]['State']['directionrequest']
        # print 'Agent state is ', agentstate
        # gwg_c.render()

        agentstate_f = automaton_f[automaton_state_f]['State']['s']
        # gwg_f.render()

        gwg_c.moveobstacles[0] = copy.deepcopy(gridstate)

        gwg_c.render()
        gwg_c.current = [copy.deepcopy(agentstate)]
        gwg_c.render()

        time.sleep(0.3)
        
        while True:
            output.saveState(gwg_f, automaton_f, automaton_state_f,gridstate_f,moveobstacles_f)
            agentstate_f = automaton_f[automaton_state_f]['State']['s']
          

            nextstates_f = automaton_f[automaton_state_f]['Successors']
            nextstatedirn_f = {"1":{'4':None,'2':None,'3':None,'1':None,'0':None, 'Belief':set()}, "0":{'4':None,'2':None,'3':None,'1':None,'0':None, 'Belief':set()}}
            ### nextstatedirn_f first number is stair boolean, second number is direcion request
            for n in nextstates_f:
                nenv_dir_req = automaton_f[n]['State']['directionrequest']
                nenv_stair_req = automaton_f[n]['State']['stair']
                if nenv_stair_req ==1:
                    if nenv_dir_req == 4:
                        nextstatedirn_f["1"]['4'] = n
                    if nenv_dir_req == 2:
                        nextstatedirn_f["1"]['2'] = n
                    if nenv_dir_req == 3:
                        nextstatedirn_f["1"]['3'] = n
                    if nenv_dir_req == 1:
                        nextstatedirn_f["1"]['1'] = n
                    if nenv_dir_req == 0:
                        nextstatedirn_f["1"]['0'] = n
                    if nenv_dir_req not in xstates:
                        nextstatedirn_f["1"]['Belief'].add(n)
                else:
                    if nenv_dir_req == 4:
                        nextstatedirn_f["0"]['4'] = n
                    if nenv_dir_req == 2:
                        nextstatedirn_f["0"]['2'] = n
                    if nenv_dir_req == 3:
                        nextstatedirn_f["0"]['3'] = n
                    if nenv_dir_req == 1:
                        nextstatedirn_f["0"]['1'] = n
                    if nenv_dir_req == 0:
                        nextstatedirn_f["0"]['0'] = n
                    if nenv_dir_req not in xstates:
                        nextstatedirn_f["0"]['Belief'].add(n)
              
            while True:
                nextstate_f = None
                while nextstate_f == None:
                    time.sleep(0.2)
                    nextstate_f = nextstatedirn_f[str(automaton[automaton_state]['State']['stairs'])][str(automaton[automaton_state]['State']['directionrequest'])]
                    nenv_dir_req = automaton_f[nextstate_f]['State']['st']
                    print 'Environment state in fine automaton is', allstates.index(nenv_dir_req)
                    print 'Environment state in fine grid is', nenv_dir_req
                    print 'Stair Boolean is ', automaton_f[nextstate_f]['State']['stair']
                    print 'Step Length is ', automaton_f[nextstate_f]['State']['stepL']
                    print 'turn is ', automaton_f[nextstate_f]['State']['turn']
                    gridstate_f = copy.deepcopy(nenv_dir_req)
                    gwg_f.colorstates[1] = set()
                    # gwg_f.render()


                if len(automaton_f[nextstate_f]['Successors']) > 0:
                    break

            print 'Automaton fine state is ', nextstate_f
            # print 'actions: \norientation: ' + str(automaton[nextstate]['State']['orientation']) + '\nstop: ' + str(automaton[nextstate]['State']['stop']) + '\nturn Left: ' + str(automaton[nextstate]['State']['turnLeft']) + '\nturn Right: ' + str(automaton[nextstate]['State']['turnRight']) + '\nforward: ' + str(automaton[nextstate]['State']['forward'])  + '\nstepL: ' + str(automaton[nextstate]['State']['stepL'])  + '\nstanceFoot: ' + str(automaton[nextstate]['State']['stanceFoot'])
            # print 'sOld: ' + str(automaton[nextstate]['State']['sOld'])
            # print 'stanceFoot: ' + str(automaton[nextstate]['State']['stanceFoot'])
            # print 'pastTurnStanceMatchFoot: ' + str(automaton[nextstate]['State']['pastTurnStanceMatchFoot'])
            # print str(automaton[nextstate]['State'])

            # if automaton_f[nextstate_f]['State']['requestPending1'] == 5:
            #     # if automaton_f[automaton_state_f]['State']['requestPending1'] != 5:
            #     break

            automaton_state_f = copy.deepcopy(nextstate_f)
            agentstate_f = automaton_f[automaton_state_f]['State']['s']
            gwg_f.render()
            gwg_f.current = [copy.deepcopy(agentstate_f)]
            gwg_f.colorstates[0] = set()
            gwg_f.colorstates[0].update(invisibilityset_f[0][agentstate_f])
            gwg_f.render()

            if automaton_f[automaton_state_f]['State']['requestPending1'] == 5:
                break
        
        time.sleep(0.5)
        output_c.saveState(gwg_c, automaton, automaton_state,gridstate,moveobstacles_c)
        
        gwg_c.moveobstacles[0] = copy.deepcopy(gridstate)
        gwg_c.render()

        agentstate = automaton[automaton_state]['State']['s_c']
        nav_req = automaton[automaton_state]['State']['directionrequest']
        print 'Agent state is ', agentstate
        gwg_c.render()

        gwg_c.render()
        gwg_c.current = [copy.deepcopy(agentstate)]

        gwg_c.colorstates[0] = set()
        gwg_c.colorstates[0].update(invisibilityset_c[0][agentstate])
        gwg_c.render()


        nextstates = automaton[automaton_state]['Successors']
        nextstatedirn = {'W':None,'E':None,'S':None,'N':None,'R':None, 'Belief':set()}


        # Need to Change this to add multiple next automaton states for one moving obstacle action (based on delivery request only I think, in this case only consider delivery request to be true). Need to add correct obstacle transition to specifications
        for n in nextstates:
            nenvstate = automaton[n]['State']['st']
            if nenvstate == gwg_c.moveobstacles[0] - 1:
                nextstatedirn['W'] = n
            if nenvstate == gwg_c.moveobstacles[0] + 1:
                nextstatedirn['E'] = n
            if nenvstate == gwg_c.moveobstacles[0] + gwg_c.ncols:
                nextstatedirn['S'] = n
            if nenvstate == gwg_c.moveobstacles[0] - gwg_c.ncols:
                nextstatedirn['N'] = n
            if nenvstate == gwg_c.moveobstacles[0]:
                nextstatedirn['R'] = n
            if nenvstate not in xstates:
                nextstatedirn['Belief'].add(n)
        while True:
            nextstate = None
            while nextstate == None:
                while True:
                    arrow = gwg_c.getkeyinput()
                    if arrow != None:
                        break
                nextstate = nextstatedirn[arrow]
                if nextstate == None:
                    if arrow == 'W':
                        gridstate = gwg_c.moveobstacles[0] - 1
                    elif arrow == 'E':
                        gridstate = gwg_c.moveobstacles[0] + 1
                    elif arrow == 'S':
                        gridstate = gwg_c.moveobstacles[0] + gwg_c.ncols
                    elif arrow == 'N':
                        gridstate = gwg_c.moveobstacles[0] - gwg_c.ncols
                    elif arrow == 'R':
                        gridstate = gwg_c.moveobstacles[0]
                    
                    for n in nextstatedirn['Belief']:
                        nenvstate = automaton[n]['State']['st']
                        nextbeliefs = beliefcombs[len(beliefcombs) - (len(allstates) - allstates.index(nenvstate))]
                        if any(gridstate in partitionGrid_c[x] for x in nextbeliefs):
                            nextstate = copy.deepcopy(n)
                            print 'Environment state in automaton is', allstates.index(nenvstate)
                            print 'Belief state is', beliefcombs[allstates.index(nenvstate) - len(xstates)]
                            nextagentstate = automaton[n]['State']['s_c']
                            invisstates = invisibilityset_c[0][nextagentstate]
                            visstates = set(xstates) - set(invisstates)
                            if nenvstate not in xstates:
                                beliefcombstate = beliefcombs[allstates.index(nenvstate) - len(xstates)]
                                beliefstates = set()
                                for b in beliefcombstate:
                                    beliefstates = beliefstates.union(partitionGrid_c[b])
                                truebeliefstates = beliefstates - beliefstates.intersection(visstates)
                                gwg_c.colorstates[1] = copy.deepcopy(truebeliefstates)
                                gwg_c.render()
                                print 'True belief set is ', truebeliefstates
                                print 'Size of true belief set is ', len(truebeliefstates)
                else:
                    nenvstate = automaton[nextstate]['State']['st']
                    print 'Environment state in automaton is', allstates.index(nenvstate)
                    print 'Environment state in grid is', nenvstate
                    gridstate = copy.deepcopy(nenvstate)
                    gwg_c.colorstates[1] = set()
                    gwg_c.render()


            if len(automaton[nextstate]['Successors']) > 0:
                break

        print 'Automaton state is ', nextstate
        # print 'actions: \norientation: ' + str(automaton[nextstate]['State']['orientation']) + '\nstop: ' + str(automaton[nextstate]['State']['stop']) + '\nturn Left: ' + str(automaton[nextstate]['State']['turnLeft']) + '\nturn Right: ' + str(automaton[nextstate]['State']['turnRight']) + '\nforward: ' + str(automaton[nextstate]['State']['forward'])  + '\nstepL: ' + str(automaton[nextstate]['State']['stepL'])  + '\nstanceFoot: ' + str(automaton[nextstate]['State']['stanceFoot'])
        # print 'sOld: ' + str(automaton[nextstate]['State']['sOld'])
        # print 'stanceFoot: ' + str(automaton[nextstate]['State']['stanceFoot'])
        # print 'pastTurnStanceMatchFoot: ' + str(automaton[nextstate]['State']['pastTurnStanceMatchFoot'])
        # print str(automaton[nextstate]['State'])
        automaton_state = copy.deepcopy(nextstate)


