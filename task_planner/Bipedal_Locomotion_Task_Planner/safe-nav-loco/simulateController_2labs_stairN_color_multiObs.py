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
    # gwg_c.colorstates = [set(), set()]
    gwg_c.colorstates = [set(), set(), set(), set(), set(), set(), set()]
    gwg_f.colorstates = [set(), set()]
    gridstate = copy.deepcopy(moveobstacles_c)
    gridstate_f = copy.deepcopy(moveobstacles_f[0])
    output = BeliefIOParser(jsonfile_f)
    output_c = BeliefIOParser(jsonfile_c)
    gwg_c.render()
    time.sleep(0.3)
    while True:
        envstate = [0 for i in range(len(moveobstacles_c))]
        for n in range(0,len(moveobstacles_c)):
            envstate[n] = automaton[automaton_state]['State']['st{}'.format(n)]
        # envstate = automaton[automaton_state]['State']['st']
        envstate_f = automaton_f[automaton_state_f]['State']['st']
        # try:
        #     print 'Agent state is J ', agentstate
        # except:
        #     print 'nothingyet'

        
        gwg_c.moveobstacles = copy.deepcopy(gridstate)
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

        # gwg_c.moveobstacles[0] = copy.deepcopy(gridstate)
        gwg_c.moveobstacles = copy.deepcopy(gridstate)

        gwg_c.render()
        gwg_c.current = [copy.deepcopy(agentstate)]
        gwg_c.render()

        time.sleep(0.3)
        
        while True:
            # output.saveState(gwg_f, automaton_f, automaton_state_f,gridstate_f,moveobstacles_f)
            output.saveState(gwg_f, automaton_f, automaton_state_f,gridstate,moveobstacles_c, gwg_c)
            agentstate_f = automaton_f[automaton_state_f]['State']['s']
          

            nextstates_f = automaton_f[automaton_state_f]['Successors']
            nextstatedirn_f = {"1":{"1":{'4':None,'2':None,'3':None,'1':None,'0':None, 'Belief':set()}, "0":{'4':None,'2':None,'3':None,'1':None,'0':None, 'Belief':set()}},"0":{"1":{'4':None,'2':None,'3':None,'1':None,'0':None, 'Belief':set()}, "0":{'4':None,'2':None,'3':None,'1':None,'0':None, 'Belief':set()}}}
            ### nextstatedirn_f first number is stair boolean, second number is direcion request
            for n in nextstates_f:
                nenv_dir_req = automaton_f[n]['State']['directionrequest']
                nenv_stair_req = automaton_f[n]['State']['stair']
                nenv_stairN_req = automaton_f[n]['State']['stairN']
                if nenv_stair_req ==1:
                    if nenv_stairN_req ==1:
                        if nenv_dir_req == 4:
                            nextstatedirn_f["1"]["1"]['4'] = n
                        if nenv_dir_req == 2:
                            nextstatedirn_f["1"]["1"]['2'] = n
                        if nenv_dir_req == 3:
                            nextstatedirn_f["1"]["1"]['3'] = n
                        if nenv_dir_req == 1:
                            nextstatedirn_f["1"]["1"]['1'] = n
                        if nenv_dir_req == 0:
                            nextstatedirn_f["1"]["1"]['0'] = n
                        if nenv_dir_req not in xstates:
                            nextstatedirn_f["1"]['Belief'].add(n)
                    else:
                        if nenv_dir_req == 4:
                            nextstatedirn_f["1"]["0"]['4'] = n
                        if nenv_dir_req == 2:
                            nextstatedirn_f["1"]["0"]['2'] = n
                        if nenv_dir_req == 3:
                            nextstatedirn_f["1"]["0"]['3'] = n
                        if nenv_dir_req == 1:
                            nextstatedirn_f["1"]["0"]['1'] = n
                        if nenv_dir_req == 0:
                            nextstatedirn_f["1"]["0"]['0'] = n
                        if nenv_dir_req not in xstates:
                            nextstatedirn_f["1"]["0"]['Belief'].add(n)
                else:
                    if nenv_stairN_req ==1:
                        if nenv_dir_req == 4:
                            nextstatedirn_f["0"]["1"]['4'] = n
                        if nenv_dir_req == 2:
                            nextstatedirn_f["0"]["1"]['2'] = n
                        if nenv_dir_req == 3:
                            nextstatedirn_f["0"]["1"]['3'] = n
                        if nenv_dir_req == 1:
                            nextstatedirn_f["0"]["1"]['1'] = n
                        if nenv_dir_req == 0:
                            nextstatedirn_f["0"]["1"]['0'] = n
                        if nenv_dir_req not in xstates:
                            nextstatedirn_f["0"]['Belief'].add(n)
                    else:
                        if nenv_dir_req == 4:
                            nextstatedirn_f["0"]["0"]['4'] = n
                        if nenv_dir_req == 2:
                            nextstatedirn_f["0"]["0"]['2'] = n
                        if nenv_dir_req == 3:
                            nextstatedirn_f["0"]["0"]['3'] = n
                        if nenv_dir_req == 1:
                            nextstatedirn_f["0"]["0"]['1'] = n
                        if nenv_dir_req == 0:
                            nextstatedirn_f["0"]["0"]['0'] = n
                        if nenv_dir_req not in xstates:
                            nextstatedirn_f["0"]["0"]['Belief'].add(n)
              
            while True:
                nextstate_f = None
                while nextstate_f == None:
                    time.sleep(0.2)
                    nextstate_f = nextstatedirn_f[str(automaton[automaton_state]['State']['stairs'])][str(automaton[automaton_state]['State']['stairsN'])][str(automaton[automaton_state]['State']['directionrequest'])]
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

            # if automaton_f[automaton_state_f]['State']['requestPending1'] == 5:
            #     break
            if automaton_f[automaton_state_f]['State']['TaskAchieved'] == 1:
                break
        
        time.sleep(0.5)
        output_c.saveState(gwg_c, automaton, automaton_state,gridstate,moveobstacles_c, gwg_c)
        
        gwg_c.moveobstacles = copy.deepcopy(gridstate)
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
        # nextstatedirn = {'W':None,'E':None,'S':None,'N':None,'R':None, 'Belief':set()}
        nextstatedirn = {'W':{'W':None,'E':None,'S':None,'N':None,'R':None, 'Belief':set()},'E':{'W':None,'E':None,'S':None,'N':None,'R':None, 'Belief':set()},'S':{'W':None,'E':None,'S':None,'N':None,'R':None, 'Belief':set()},'N':{'W':None,'E':None,'S':None,'N':None,'R':None, 'Belief':set()},'R':{'W':None,'E':None,'S':None,'N':None,'R':None, 'Belief':set()}, 'Belief':{'W':set(),'E':set(),'S':set(),'N':set(),'R':set(), 'Belief':set()}}
        

        # Need to Change this to add multiple next automaton states for one moving obstacle action (based on delivery request only I think, in this case only consider delivery request to be true). Need to add correct obstacle transition to specifications
        for n in nextstates:
            nenvstate = [automaton[n]['State']['st0'],automaton[n]['State']['st1']]
            if nenvstate[0] == (gwg_c.moveobstacles[0] - 1):
                if nenvstate[1] == gwg_c.moveobstacles[1] - 1:
                    nextstatedirn['W']['W'] = n
                if nenvstate[1] == gwg_c.moveobstacles[1] + 1:
                    nextstatedirn['W']['E'] = n
                if nenvstate[1] == gwg_c.moveobstacles[1] + gwg_c.ncols:
                    nextstatedirn['W']['S'] = n
                if nenvstate[1] == gwg_c.moveobstacles[1] - gwg_c.ncols:
                    nextstatedirn['W']['N'] = n
                if nenvstate[1] == gwg_c.moveobstacles[1]:
                    nextstatedirn['W']['R'] = n
                if nenvstate[1] not in xstates:
                    nextstatedirn['W']['Belief'].add(n)
            if nenvstate[0] == gwg_c.moveobstacles[0] + 1:
                if nenvstate[1] == gwg_c.moveobstacles[1] - 1:
                    nextstatedirn['E']['W'] = n
                if nenvstate[1] == gwg_c.moveobstacles[1] + 1:
                    nextstatedirn['E']['E'] = n
                if nenvstate[1] == gwg_c.moveobstacles[1] + gwg_c.ncols:
                    nextstatedirn['E']['S'] = n
                if nenvstate[1] == gwg_c.moveobstacles[1] - gwg_c.ncols:
                    nextstatedirn['E']['N'] = n
                if nenvstate[1] == gwg_c.moveobstacles[1]:
                    nextstatedirn['E']['R'] = n
                if nenvstate[1] not in xstates:
                    nextstatedirn['E']['Belief'].add(n)
            if nenvstate[0] == gwg_c.moveobstacles[0] + gwg_c.ncols:
                if nenvstate[1] == gwg_c.moveobstacles[1] - 1:
                    nextstatedirn['S']['W'] = n
                if nenvstate[1] == gwg_c.moveobstacles[1] + 1:
                    nextstatedirn['S']['E'] = n
                if nenvstate[1] == gwg_c.moveobstacles[1] + gwg_c.ncols:
                    nextstatedirn['S']['S'] = n
                if nenvstate[1] == gwg_c.moveobstacles[1] - gwg_c.ncols:
                    nextstatedirn['S']['N'] = n
                if nenvstate[1] == gwg_c.moveobstacles[1]:
                    nextstatedirn['S']['R'] = n
                if nenvstate[1] not in xstates:
                    nextstatedirn['S']['Belief'].add(n)
            if nenvstate[0] == gwg_c.moveobstacles[0] - gwg_c.ncols:
                if nenvstate[1] == gwg_c.moveobstacles[1] - 1:
                    nextstatedirn['N']['W'] = n
                if nenvstate[1] == gwg_c.moveobstacles[1] + 1:
                    nextstatedirn['N']['E'] = n
                if nenvstate[1] == gwg_c.moveobstacles[1] + gwg_c.ncols:
                    nextstatedirn['N']['S'] = n
                if nenvstate[1] == gwg_c.moveobstacles[1] - gwg_c.ncols:
                    nextstatedirn['N']['N'] = n
                if nenvstate[1] == gwg_c.moveobstacles[1]:
                    nextstatedirn['N']['R'] = n
                if nenvstate[1] not in xstates:
                    nextstatedirn['N']['Belief'].add(n)
            if nenvstate[0] == gwg_c.moveobstacles[0]:
                if nenvstate[1] == gwg_c.moveobstacles[1] - 1:
                    nextstatedirn['R']['W'] = n
                if nenvstate[1] == gwg_c.moveobstacles[1] + 1:
                    nextstatedirn['R']['E'] = n
                if nenvstate[1] == gwg_c.moveobstacles[1] + gwg_c.ncols:
                    nextstatedirn['R']['S'] = n
                if nenvstate[1] == gwg_c.moveobstacles[1] - gwg_c.ncols:
                    nextstatedirn['R']['N'] = n
                if nenvstate[1] == gwg_c.moveobstacles[1]:
                    nextstatedirn['R']['R'] = n
                if nenvstate[1] not in xstates:
                    nextstatedirn['R']['Belief'].add(n)
            if nenvstate[0] not in xstates:
                if nenvstate[1] == gwg_c.moveobstacles[1] - 1:
                    nextstatedirn['Belief']['W'].add(n)
                if nenvstate[1] == gwg_c.moveobstacles[1] + 1:
                    nextstatedirn['Belief']['E'].add(n)
                if nenvstate[1] == gwg_c.moveobstacles[1] + gwg_c.ncols:
                    nextstatedirn['Belief']['S'].add(n)
                if nenvstate[1] == gwg_c.moveobstacles[1] - gwg_c.ncols:
                    nextstatedirn['Belief']['N'].add(n)
                if nenvstate[1] == gwg_c.moveobstacles[1]:
                    nextstatedirn['Belief']['R'].add(n)
                if nenvstate[1] not in xstates:
                    nextstatedirn['Belief']['Belief'].add(n)
        # for n in nextstates:
        #     nenvstate = automaton[n]['State']['st']
        #     if nenvstate == gwg_c.moveobstacles[0] - 1:
        #         nextstatedirn['W'] = n
        #     if nenvstate == gwg_c.moveobstacles[0] + 1:
        #         nextstatedirn['E'] = n
        #     if nenvstate == gwg_c.moveobstacles[0] + gwg_c.ncols:
        #         nextstatedirn['S'] = n
        #     if nenvstate == gwg_c.moveobstacles[0] - gwg_c.ncols:
        #         nextstatedirn['N'] = n
        #     if nenvstate == gwg_c.moveobstacles[0]:
        #         nextstatedirn['R'] = n
        #     if nenvstate not in xstates:
        #         nextstatedirn['Belief'].add(n)
        while True:
            nextstate = None
            while nextstate == None:
                while True:
                    arrow1 = gwg_c.getkeyinput()
                    if arrow1 != None:
                        break
                while True:
                    arrow2 = gwg_c.getkeyinput()
                    if arrow2 != None:
                        break
                    # arrow = gwg_c.getkeyinput()
                    # if arrow != None:
                    #     break

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
                # nextstate = nextstatedirn[arrow]

                # if nextstate == None:
                #     if arrow == 'W':
                #         gridstate = gwg_c.moveobstacles[0] - 1
                #     elif arrow == 'E':
                #         gridstate = gwg_c.moveobstacles[0] + 1
                #     elif arrow == 'S':
                #         gridstate = gwg_c.moveobstacles[0] + gwg_c.ncols
                #     elif arrow == 'N':
                #         gridstate = gwg_c.moveobstacles[0] - gwg_c.ncols
                #     elif arrow == 'R':
                #         gridstate = gwg_c.moveobstacles[0]
                if nextstate == None:
                    if arrow1 == 'W':
                        gridstate[0] = gwg_c.moveobstacles[0] - 1
                    elif arrow1 == 'E':
                        gridstate[0] = gwg_c.moveobstacles[0] + 1
                    elif arrow1 == 'S':
                        gridstate[0] = gwg_c.moveobstacles[0] + gwg_c.ncols
                    elif arrow1 == 'N':
                        gridstate[0] = gwg_c.moveobstacles[0] - gwg_c.ncols
                    elif arrow1 == 'R':
                        gridstate[0] = gwg_c.moveobstacles[0]
                    
                    if arrow2 == 'W':
                        gridstate[1] = gwg_c.moveobstacles[1] - 1
                    elif arrow2 == 'E':
                        gridstate[1] = gwg_c.moveobstacles[1] + 1
                    elif arrow2 == 'S':
                        gridstate[1] = gwg_c.moveobstacles[1] + gwg_c.ncols
                    elif arrow2 == 'N':
                        gridstate[1] = gwg_c.moveobstacles[1] - gwg_c.ncols
                    elif arrow2 == 'R':
                        gridstate[1] = gwg_c.moveobstacles[1]

                    if case == 2:
                        for n in nextstatedirn[arrow1]['Belief']:
                            nenvstate = [automaton[n]['State']['st0'],automaton[n]['State']['st1']]
                            nextbeliefs = beliefcombs[len(beliefcombs) - (len(allstates) - allstates.index(nenvstate[1]))]
                            # try:
                            #     nextbeliefs = beliefcombs[len(beliefcombs) - (len(allstates) - allstates.index(nenvstate[0]))]
                            # except:
                            #     nextstate = 124
                            #     continue
                            if any(gridstate[1] in partitionGrid_c[x] for x in nextbeliefs):
                                nextstate = copy.deepcopy(n)
                                # print 'Environment state in automaton is', allstates.index(nenvstate[0])
                                # print 'Belief state is', beliefcombs[allstates.index(nenvstate[0]) - len(xstates)]
                                nextagentstate = automaton[n]['State']['s_c']
                                invisstates = invisibilityset_c[0][nextagentstate]
                        
                                visstates = set(xstates) - set(invisstates)
                                if nenvstate[1] not in xstates:
                                    beliefcombstate = beliefcombs[allstates.index(nenvstate[1]) - len(xstates)]
                                    beliefstates = set()
                                    for b in beliefcombstate:
                                        beliefstates = beliefstates.union(partitionGrid_c[b])
                                        gwg_c.colorstates[b+1] = copy.deepcopy(partitionGrid_c[b]) - partitionGrid_c[b].intersection(visstates)
                                    truebeliefstates = beliefstates - beliefstates.intersection(visstates)
                                    # gwg_c.colorstates[1] = copy.deepcopy(truebeliefstates)
                                    gwg_c.render()
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
                            if any(gridstate[0] in partitionGrid_c[x] for x in nextbeliefs):
                                nextstate = copy.deepcopy(n)
                                # print 'Environment state in automaton is', allstates.index(nenvstate[0])
                                # print 'Belief state is', beliefcombs[allstates.index(nenvstate[0]) - len(xstates)]
                                nextagentstate = automaton[n]['State']['s_c']
                                invisstates = invisibilityset_c[0][nextagentstate]
                        
                                visstates = set(xstates) - set(invisstates)
                                if nenvstate[0] not in xstates:
                                    beliefcombstate = beliefcombs[allstates.index(nenvstate[0]) - len(xstates)]
                                    beliefstates = set()
                                    for b in beliefcombstate:
                                        beliefstates = beliefstates.union(partitionGrid_c[b])
                                        gwg_c.colorstates[b+1] = copy.deepcopy(partitionGrid_c[b]) - partitionGrid_c[b].intersection(visstates)
                                    truebeliefstates = beliefstates - beliefstates.intersection(visstates)
                                    # gwg_c.colorstates[1] = copy.deepcopy(truebeliefstates)
                                    gwg_c.render()
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
                            if any(gridstate[0] in partitionGrid_c[x] for x in nextbeliefs[0]):
                                if any(gridstate[0] in partitionGrid_c[x] for x in nextbeliefs[0]):
                                    nextstate = copy.deepcopy(n)
                                    # print 'Environment state in automaton is', allstates.index(nenvstate[0])
                                    # print 'Belief state is', beliefcombs[allstates.index(nenvstate[0]) - len(xstates)]
                                    nextagentstate = automaton[n]['State']['s_c']
                                    invisstates = invisibilityset_c[0][nextagentstate]
                            
                                    visstates = set(xstates) - set(invisstates)
                                    if nenvstate[0] not in xstates:
                                        beliefcombstate = beliefcombs[allstates.index(nenvstate[0]) - len(xstates)]
                                        beliefstates = set()
                                        for b in beliefcombstate:
                                            beliefstates = beliefstates.union(partitionGrid_c[b])
                                            gwg_c.colorstates[b+1] = copy.deepcopy(partitionGrid_c[b]) - partitionGrid_c[b].intersection(visstates)
                                        truebeliefstates = beliefstates - beliefstates.intersection(visstates)
                                        tmp = copy.deepcopy(truebeliefstates)
                                        # gwg_c.colorstates[1] = copy.deepcopy(truebeliefstates)
                                        gwg_c.render()
                                        print 'True belief set is ', truebeliefstates
                                        print 'Size of true belief set is ', len(truebeliefstates)
                                    if nenvstate[1] not in xstates:
                                        beliefcombstate = beliefcombs[allstates.index(nenvstate[1]) - len(xstates)]
                                        beliefstates = set()
                                        for b in beliefcombstate:
                                            beliefstates = beliefstates.union(partitionGrid_c[b])
                                            # gwg_c.colorstates[b+1] = copy.deepcopy(partitionGrid_c[b]) - partitionGrid_c[b].intersection(visstates)
                                            tmp3 = copy.deepcopy(partitionGrid_c[b]) - partitionGrid_c[b].intersection(visstates)
                                            gwg_c.colorstates[b+1].update(tmp3)
                                        truebeliefstates = beliefstates - beliefstates.intersection(visstates)
                                        # gwg_c.colorstates[1] = copy.deepcopy(truebeliefstates)

                                        tmp2 = copy.deepcopy(truebeliefstates)
                                        tmp.update(tmp2)
                    
                                        # gwg_c.colorstates[1] = copy.deepcopy(tmp)
                                        gwg_c.render()
                                        print 'True belief set is ', truebeliefstates
                                        print 'Size of true belief set is ', len(truebeliefstates)
                    
                    # for n in nextstatedirn['Belief']:
                    #     nenvstate = automaton[n]['State']['st']
                    #     nextbeliefs = beliefcombs[len(beliefcombs) - (len(allstates) - allstates.index(nenvstate))]
                    #     if any(gridstate in partitionGrid_c[x] for x in nextbeliefs):
                    #         nextstate = copy.deepcopy(n)
                    #         print 'Environment state in automaton is', allstates.index(nenvstate)
                    #         print 'Belief state is', beliefcombs[allstates.index(nenvstate) - len(xstates)]
                    #         nextagentstate = automaton[n]['State']['s_c']
                    #         invisstates = invisibilityset_c[0][nextagentstate]
                    #         visstates = set(xstates) - set(invisstates)
                    #         if nenvstate not in xstates:
                    #             beliefcombstate = beliefcombs[allstates.index(nenvstate) - len(xstates)]
                    #             beliefstates = set()
                    #             for b in beliefcombstate:
                    #                 beliefstates = beliefstates.union(partitionGrid_c[b])
                    #                 gwg_c.colorstates[b+1] = copy.deepcopy(partitionGrid_c[b]) - partitionGrid_c[b].intersection(visstates)
                    #             truebeliefstates = beliefstates - beliefstates.intersection(visstates)
                    #             # gwg_c.colorstates[1] = copy.deepcopy(truebeliefstates)
                    #             gwg_c.render()
                    #             print 'True belief set is ', truebeliefstates
                    #             print 'Size of true belief set is ', len(truebeliefstates)
                else:
                    # nenvstate = automaton[nextstate]['State']['st']
                    nenvstate = [automaton[nextstate]['State']['st0'],automaton[nextstate]['State']['st1']]
                    print 'Environment state in automaton is', allstates.index(nenvstate[0])
                    print 'Environment state in grid is', nenvstate
                    gridstate = copy.deepcopy(nenvstate)
                    gwg_c.colorstates[1] = set()
                    gwg_c.colorstates[2] = set()
                    gwg_c.colorstates[3] = set()
                    gwg_c.colorstates[4] = set()
                    gwg_c.colorstates[5] = set()
                    gwg_c.colorstates[6] = set()
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


