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


def userControlled_partition(filename,gwg,partitionGrid,moveobstacles,invisibilityset,jsonfile,jsonfile_obs,capabilities,filename_f,gwg_f,partitionGrid_f,moveobstacles_f,invisibilityset_f,jsonfile_f):
    automaton = parseJson(filename)
    automaton_obs = parseJson(jsonfile_obs)
    automaton_f = parseJson(filename_f)
    # for s in automaton:
    #     print automaton[s]['State']['s']

    automaton_state = 0
    automaton_state_obs = 0
    automaton_state_f = 0
    xstates = list(set(gwg.states))
    xstates_f = list(set(gwg_f.states))
    allstates = copy.deepcopy(xstates)
    allstates_f = copy.deepcopy(xstates_f)
    beliefcombs = powerset(partitionGrid.keys())
    beliefcombs_f = powerset(partitionGrid_f.keys())
    for i in range(gwg.nstates,gwg.nstates+ len(beliefcombs)):
        allstates.append(i)
    for i in range(gwg_f.nstates,gwg_f.nstates+ len(beliefcombs_f)):
        allstates_f.append(i)
    gwg.colorstates = [set(), set(), set(), set(), set(), set(), set()]
    gwg_f.colorstates = [set(), set()]
    gridstate = copy.deepcopy(moveobstacles[0])
    gridstate_f = copy.deepcopy(moveobstacles_f[0])
    output = BeliefIOParser(jsonfile)
    output_f = BeliefIOParser(jsonfile_f)
    j = 0
    wait=0
    coop_location_c = 0
    cooprequest_c = 0
    coop_location_q = 0
    cooprequest_q = 0
    wait_over = 0
    resolving = 0
    bad_state = 0

    gwg.moveobstacles[0] = copy.deepcopy(gridstate)
    # gwg.render()
    agentstate = automaton[automaton_state]['State']['s_c']
    gwg.colorstates[0].update(invisibilityset[0][agentstate])
    gwg.render()

    while True:
        j +=1
        output.saveState(gwg, automaton, automaton_state,gridstate,moveobstacles,gwg)
        envstate = automaton_obs[automaton_state_obs]['State']['st']
        envstate_f = automaton_f[automaton_state_f]['State']['st']
      

        
        gwg.moveobstacles[0] = copy.deepcopy(gridstate)
        # gwg.render()

        gwg_f.moveobstacles_f[0] = copy.deepcopy(gridstate_f)


        # if gwg.physicalViolation() != -1 and wait_over == 0:
        if wait == 1 and resolving ==0:
            print("SYSTEM ENTERED PHYSICALLY INVALID STATE")
            # break;
            # wait=1
            # bad_state = gwg.physicalViolation()
            # while True:
            #     arrow = gwg.getkeyinput()
            #     if arrow != None:
            #         break
            print("Bad state: ", bad_state)
            if gwg.resolution[bad_state]['action'] in capabilities['quad']:
                print("Obstacle Resolvable by Quadcopter")
                coop_location_q = gwg.resolution[bad_state]['state']
                coop_location_c = PUDO_t_c_cassie[0]#gwg.current[0] - 1
                # gwg.current[0] = gwg.current[0] - 1
                cooprequest_c = 1
                cooprequest_q = 1
                resolving = 1
            elif gwg.resolution[bad_state]['action'] in capabilities['cassie']:
                print("Obstacle Resolvable by Cassie")
                # coop_location_q = PUDO_t_c_quad[0]#gwg.moveobstacles[0] - (2)
                coop_location_c = gwg.resolution[bad_state]['state'][0]
                # gwg.moveobstacles[0] = gwg.moveobstacles[0] - 2
                cooprequest_c = 1
                cooprequest_q = 1
                resolving = 1


        if gwg.current[0] in gwg.resolvable and gwg.resolution[gwg.current[0]]['action'] == 'push':
            print("CASSIE RESOLVED AN OBSTACLE")
            # while True:
            #     arrow = gwg.getkeyinput()
            #     if arrow != None:
            #         break
            gwg.resolveObstacle(gwg.current[0])
            # break;
            wait=0
            wait_over = 1
            cooprequest_c = 0
            resolving = 0
            # gwg.moveobstacles[0] = gwg.moveobstacles[0] + 2

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
        nav_req = automaton[automaton_state]['State']['directionrequest']

        agentstate_f = automaton_f[automaton_state_f]['State']['s']



        while True:
            output_f.saveState(gwg_f, automaton_f, automaton_state_f,gridstate,moveobstacles_f,gwg)
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
            # gwg_f.render()
            gwg_f.current = [copy.deepcopy(agentstate_f)]
            gwg_f.colorstates[0] = set()
            gwg_f.colorstates[0].update(invisibilityset_f[0][agentstate_f])
            # gwg_f.render()

            # if automaton_f[automaton_state_f]['State']['requestPending1'] == 5:
            #     break
            if automaton_f[automaton_state_f]['State']['TaskAchieved'] == 1:
                break

        time.sleep(0.5)

        gwg.moveobstacles[0] = copy.deepcopy(gridstate)
        # gwg.render()

        agentstate = automaton[automaton_state]['State']['s_c']
        nav_req = automaton[automaton_state]['State']['directionrequest']

        print 'Agent state is ', agentstate
        # gwg.render()
        # gwg.moveobstacles[0] = copy.deepcopy(gridstate)

        # gwg.render()
        gwg.current = [copy.deepcopy(agentstate)]

        gwg.colorstates[0] = set()
        gwg.colorstates[0].update(invisibilityset[0][agentstate])
        gwg.render()
        # gwg.draw_state_labels()
        # if gwg.physicalViolation() != -1:
        #     print("SYSTEM ENTERED PHYSICALLY INVALID STATE")
        #     break;
        # elif gwg.current[0] in gwg.resolvable and gwg.resolution[gwg.current[0]]['action'] == 'push':
        #     print("CASSIE RESOLVED AN OBSTACLE")
        #     gwg.resolveObstacle(gwg.current[0])
        #     break;
        
        nextstates = automaton[automaton_state]['Successors']
        nextstatedirn = {'W':None,'E':None,'S':None,'N':None,'R':None, 'Belief':set()}
        # Need to Change this to add multiple next automaton states for one moving obstacle action (based on delivery request only I think, in this case only consider delivery request to be true). Need to add correct obstacle transition to specifications
        for n in nextstates:
            if automaton[n]['State']['cooprequest'] == cooprequest_c:
                if automaton[n]['State']['coop_location'] == coop_location_c:
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
                        if automaton_obs[n]['State']['wait'] == wait:
                            if automaton_obs[n]['State']['stop'] == automaton[automaton_state]['State']['stop']:
                                if automaton_obs[n]['State']['orientation'] == automaton[automaton_state]['State']['orientation']:
                                    nenvstate_obs = automaton_obs[n]['State']['s_c']
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
                gridstate = automaton_obs[automaton_obs[nextstate_obs]['Successors'][0]]['State']['st']

                temp_moveobs = copy.deepcopy(gwg.moveobstacles[0])
                gwg.moveobstacles[0] = copy.deepcopy(gridstate)
                if gwg.physicalViolation() != -1:
                    if wait == 0:
                        if wait_over == 0:
                            bad_state = gwg.physicalViolation()
                            gwg.moveobstacles[0] = copy.deepcopy(temp_moveobs)
                            wait=1
                            continue
                gwg.moveobstacles[0] = copy.deepcopy(temp_moveobs)


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
                                # gwg.render()
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
                    # gwg.render()
            
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