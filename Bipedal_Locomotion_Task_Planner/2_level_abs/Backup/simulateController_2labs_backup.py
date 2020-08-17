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
        # -----------------Jonas Robot Waits For Key Stroke To Take Action-----------------
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

        # gwg_f.render()
        # gwg_f.current = [copy.deepcopy(agentstate_f)]

        # gwg_c.colorstates[0] = set()
        # gwg_c.colorstates[0].update(invisibilityset_c[0][agentstate])
        # gwg_c.render()

        time.sleep(0.3)

        # gwg_f.colorstates[0] = set()
        # gwg_f.colorstates[0].update(invisibilityset_f[0][agentstate_f])
        # gwg_f.render()
        # gwg_c.draw_state_labels()
        
        while True:
            output.saveState(gwg_f, automaton_f, automaton_state_f,gridstate_f)
            agentstate_f = automaton_f[automaton_state_f]['State']['s']
            # gwg_f.render()
            # gwg_f.current = [copy.deepcopy(agentstate_f)]
            # gwg_f.colorstates[0] = set()
            # gwg_f.colorstates[0].update(invisibilityset_f[0][agentstate_f])
            # gwg_f.render()

            nextstates_f = automaton_f[automaton_state_f]['Successors']
            nextstatedirn_f = {'4':None,'2':None,'3':None,'1':None,'0':None, 'Belief':set()}

            for n in nextstates_f:
                nenvstate_f = automaton_f[n]['State']['directionrequest']
                if nenvstate_f == 4:
                    nextstatedirn_f['4'] = n
                if nenvstate_f == 2:
                    nextstatedirn_f['2'] = n
                if nenvstate_f == 3:
                    nextstatedirn_f['3'] = n
                if nenvstate_f == 1:
                    nextstatedirn_f['1'] = n
                if nenvstate_f == 0:
                    nextstatedirn_f['0'] = n
                if nenvstate_f not in xstates:
                    nextstatedirn_f['Belief'].add(n)

            while True:
                nextstate_f = None
                while nextstate_f == None:
                    time.sleep(0.5)
                    nextstate_f = nextstatedirn_f[str(automaton[automaton_state]['State']['directionrequest'])]
                    # if nextstate_f == None:
                    #     if arrow == 'W':
                    #         gridstate = gwg_f.moveobstacles_f[0] - 1
                    #     elif arrow == 'E':
                    #         gridstate = gwg_f.moveobstacles_f[0] + 1
                    #     elif arrow == 'S':
                    #         gridstate = gwg_f.moveobstacles_f[0] + gwg.ncols
                    #     elif arrow == 'N':
                    #         gridstate = gwg_f.moveobstacles_f[0] - gwg.ncols
                    #     elif arrow == 'R':
                    #         gridstate = gwg_f.moveobstacles_f[0]
                        
                    #     for n in nextstatedirn_f['Belief']:
                    #         nenvstate_f = automaton_f[n]['State']['st']
                    #         nextbeliefs_f = beliefcombs_f[len(beliefcombs_f) - (len(allstates_f) - allstates_f.index(nenvstate_f))]
                    # else:
                    nenvstate_f = automaton_f[nextstate_f]['State']['st']
                    print 'Environment state in fine automaton is', allstates.index(nenvstate_f)
                    print 'Environment state in fine grid is', nenvstate_f
                    gridstate_f = copy.deepcopy(nenvstate_f)
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
            
            automaton_state_f = copy.deepcopy(nextstate_f)

            # gwg_f.colorstates[0] = set()
            # gwg_f.colorstates[0].update(invisibilityset_f[0][agentstate_f])
            # gwg_f.render()
            agentstate_f = automaton_f[automaton_state_f]['State']['s']
            gwg_f.render()
            gwg_f.current = [copy.deepcopy(agentstate_f)]
            gwg_f.colorstates[0] = set()
            gwg_f.colorstates[0].update(invisibilityset_f[0][agentstate_f])
            gwg_f.render()

            if automaton_f[automaton_state_f]['State']['requestPending1'] == 5:
                break
        
        time.sleep(0.5)
        
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
                            # Jonas
                            invisstates = invisibilityset_c[0][nextagentstate]
                            # visstates = set(xstates) - invisstates
                            # Jonas
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


# def userControlled_imperfect_sensor(filename,gwg_c,partitionGrid_c,moveobstacles,allowed_states,invisibilityset_c,belief_gridstates,sensor_uncertainty,saveImage=None):
#     automaton = [None]*gwg_c.nagents
#     automaton_state = [None]*gwg_c.nagents
#     agentstate = [None]*gwg_c.nagents
#     targetstate = [None]*gwg_c.nagents
#     truebeliefstates = [set()]*gwg_c.nagents
#     allstates = [[None]]*gwg_c.nagents
#     beliefcombs = [[None]]*gwg_c.nagents
#     xstates = list(set(gwg_c.states))
#     belief_ncols = gwg_c.ncols - sensor_uncertainty + 1
#     belief_nrows = gwg_c.nrows - sensor_uncertainty + 1
#     timestep = 0
#     for n in range(gwg_c.nagents):
#         automaton[n] = parseJson(filename[n])
#         automaton_state[n] = 0
#         beliefcombs[n] = powerset(partitionGrid_c[n].keys())
#         allstates[n] = copy.deepcopy(belief_gridstates.keys())
#         for i in range(belief_ncols * belief_nrows, belief_ncols * belief_nrows + len(beliefcombs[n])):
#             allstates[n].append(i)
#         allstates[n].append(len(allstates[n]))  # nominal state if target leaves allowed region
#     gwg_c.colorstates = [set(), set()]
#     gridstate = copy.deepcopy(moveobstacles[0])

#     while True:
#         gazeboOutput(gwg_c)
#         gwg_c.colorstates[0] = set()
#         for n in range(gwg_c.nagents):
#             agentstate[n] = automaton[n][automaton_state[n]]['State']['s']
#             targetstate[n] = automaton[n][automaton_state[n]]['State']['st']
#             print 'Agent state is ', agentstate
#             # gwg_c.colorstates[0] = gwg_c.colorstates[0].union(invisibilityset_c[n][agentstate[n]].intersection(allowed_states[n]))
#             gwg_c.colorstates[0] = gwg_c.colorstates[0].union(
#                 invisibilityset_c[n][agentstate[n]])
#         activeagents = set(range(gwg_c.nagents))
#         for n in range(gwg_c.nagents):
#             if targetstate[n] == allstates[n][-1]:
#                 activeagents.remove(n)

#         gwg_c.render()
#         gwg_c.moveobstacles[0] = copy.deepcopy(gridstate)

#         gwg_c.render()
#         gwg_c.current = copy.deepcopy(agentstate)

#         gwg_c.render()
#         # gwg_c.draw_state_labels()

#         if saveImage!=None:
#             gwg_c.save(saveImage+str(timestep)+'.png')
#             timestep+=1

#         nextstatedirn = [dict()]*gwg_c.nagents
#         nexttargetstate = [None]*gwg_c.nagents
#         nextagentstate = [None]*gwg_c.nagents
#         nextstate = [None]*gwg_c.nagents
#         targetcanbeinregion=[None]*gwg_c.nagents
#         for m in activeagents:
#             nextstatedirn[m] = {'W':set(),'E':set(),'S':set(),'N':set(),'R':set(),'Belief':set(),'Out':None,'Incoming':set()}
#             nextstates = automaton[m][automaton_state[m]]['Successors']
#             for n in nextstates:
#                 nexttargetstate[m]= automaton[m][n]['State']['st']
#                 if nexttargetstate[m] == allstates[m][-1]:
#                     nextstatedirn[m]['Out'] = n
#                 if nexttargetstate[m] >= belief_nrows*belief_ncols:
#                     nextstatedirn[m]['Belief'].add(n)
#                 else:
#                     if gwg_c.moveobstacles[0] - 1 in belief_gridstates[nexttargetstate[m]]:
#                         nextstatedirn[m]['W'].add(n)
#                     if gwg_c.moveobstacles[0] + 1 in belief_gridstates[nexttargetstate[m]]:
#                         nextstatedirn[m]['E'].add(n)
#                     if gwg_c.moveobstacles[0] + gwg_c.ncols in belief_gridstates[nexttargetstate[m]]:
#                         nextstatedirn[m]['S'].add(n)
#                     if gwg_c.moveobstacles[0] - gwg_c.ncols in belief_gridstates[nexttargetstate[m]]:
#                         nextstatedirn[m]['N'].add(n)
#                     if gwg_c.moveobstacles[0] in belief_gridstates[nexttargetstate[m]]:
#                         nextstatedirn[m]['R'].add(n)
#         for m in set(range(gwg_c.nagents)) - activeagents:
#             nextstatedirn[m] = {'W':None,'E':None,'S':None,'N':None,'R':None,'Belief':set(),'Out':None,'Incoming':set()}
#             nextstates = automaton[m][automaton_state[m]]['Successors']
#             for n in nextstates:
#                 nexttargetstate[m]= automaton[m][n]['State']['st']
#                 if nexttargetstate[m] == allstates[m][-1]:
#                     nextstatedirn[m]['Out'] = n
#                 elif nexttargetstate[m] >= belief_nrows*belief_ncols:
#                     nextstatedirn[m]['Belief'].add(n)
#                 elif nexttargetstate[m] in allowed_states[m]:
#                     nextstatedirn[m]['Incoming'].add(n)
#         while True:
#             while None in nextstate:
#                 while True:
#                     arrow = gwg_c.getkeyinput()
#                     if arrow != None:
#                         break
#                 for m in activeagents:
#                     if len(nextstatedirn[m]['Belief'])>0 and len(nextstatedirn[m][arrow])==0 and nextstatedirn[m]['Belief']!={nextstatedirn[m]['Out']}:
#                         # sensor = input('Measured by sensor? ')
#                         targetcanbeinregion[m] = True
#                         sensor = 0
#                     elif nextstatedirn[m]['Belief']=={nextstatedirn[m]['Out']} and len(nextstatedirn[m][arrow])==0:
#                         sensor = 0
#                         targetcanbeinregion[m] = False
#                     else:
#                         sensor = 1
#                         targetcanbeinregion[m] = True
#                     sensor = sensor>0
#                     nextstate[m] = nextstatedirn[m][arrow]
#                     gridstate = getGridstate(gwg_c, moveobstacles[0], arrow)
#                     if not sensor:
#                         if targetcanbeinregion[m]:
#                             for n in nextstatedirn[m]['Belief']:
#                                 nexttargetstate[m] = automaton[m][n]['State']['st']
#                                 if nexttargetstate[m] != allstates[m][-1]:
#                                     nextbeliefs = beliefcombs[m][nexttargetstate[m] - belief_ncols*belief_nrows]
#                                     if any(gridstate in partitionGrid_c[m][x] for x in nextbeliefs) or gridstate not in allowed_states[m]:
#                                         nextstate[m] = copy.deepcopy(n)
#                                         nextagentstate[m] = automaton[m][n]['State']['s']
#                                         invisstates = invisibilityset_c[m][nextagentstate[m]]
#                                         visstates = set(xstates) - invisstates
#                                         if nexttargetstate[m] >= belief_ncols*belief_nrows:
#                                             beliefcombstate = beliefcombs[m][nexttargetstate[m] - belief_ncols*belief_nrows]
#                                             beliefstates = set()
#                                             for b in beliefcombstate:
#                                                 beliefstates = beliefstates.union(partitionGrid_c[m][b])
#                                             truebeliefstates[m] = beliefstates - beliefstates.intersection(visstates)
#                                     # elif nexttargetstate[m]!=allstates[m][-1]: #Target has left region
#                                     #     nextstate[m] = nextstatedirn[m]['Out']
#                                         # print 'True belief set is ', truebeliefstates
#                                         # print 'Size of true belief set is ', len(truebeliefstates)
#                         else:
#                             nextstate[m] = nextstatedirn[m]['Out']
#                             nexttargetstate[m] = automaton[m][nextstate[m]]['State']['st']
#                             print 'Environment state in automaton is', allstates[m].index(nexttargetstate[m])
#                             print 'Environment state in grid is', nexttargetstate[m]
#                             truebeliefstates[m] = set()
#                             gwg_c.render()
#                     else:
#                         nextstate[m] = random.choice(list(nextstate[m]))
#                         nexttargetstate[m] = automaton[m][nextstate[m]]['State']['st']
#                         nextagentstate[m] = automaton[m][nextstate[m]]['State']['s']
#                         print 'Environment state in belief grid is', nexttargetstate[m]
#                         invisstates = invisibilityset_c[m][nextagentstate[m]]
#                         visstates = set(xstates) - invisstates
#                         truebeliefstates[m] = belief_gridstates[nexttargetstate[m]].intersection(visstates)
#                         gwg_c.render()

#                 for m in set(range(gwg_c.nagents)) - activeagents:
#                     if len(nextstatedirn[m]['Belief'])>0:
#                         targetcanbeinregion[m] = True
#                         sensor = 0
#                     elif len(nextstatedirn[m]['Incoming'])>0 and gridstate in allowed_states[m]:
#                         targetcanbeinregion[m] = True
#                         sensor = 1
#                     else:
#                         targetcanbeinregion[m] = False
#                         sensor = 0
#                     sensor = sensor>0
#                     if targetcanbeinregion[m]:
#                         if sensor:
#                             nextstate[m] = set()
#                             for ns in nextstatedirn[m]['Incoming']:
#                                 if gridstate in belief_gridstates[automaton[m][ns]['State']['st']]:  # Fix for imperfect sensing!
#                                     nextstate[m].add(ns)
#                             nextstate[m] = random.choice(list(nextstate[m]))
#                             nexttargetstate[m] = automaton[m][nextstate[m]]['State']['st']
#                             nextagentstate[m] = automaton[m][nextstate[m]]['State']['s']
#                             invisstates = invisibilityset_c[m][nextagentstate[m]]
#                             visstates = set(xstates) - invisstates
#                             truebeliefstates[m] = belief_gridstates[nexttargetstate[m]].intersection(visstates)
#                             gwg_c.render()
#                         else:
#                             for n in nextstatedirn[m]['Belief']:
#                                 nexttargetstate[m] = automaton[m][n]['State']['st']
#                                 if nexttargetstate[m] != allstates[m][-1]:
#                                     nextbeliefs = beliefcombs[m][nexttargetstate[m] - belief_ncols*belief_nrows]
#                                     if any(gridstate in partitionGrid_c[m][x] for x in nextbeliefs):
#                                         nextstate[m] = copy.deepcopy(n)
#                                         nextagentstate[m] = automaton[m][n]['State']['s']
#                                         invisstates = invisibilityset_c[m][nextagentstate[m]]
#                                         visstates = set(xstates) - invisstates
#                                         if nexttargetstate[m] >= belief_ncols*belief_nrows:
#                                             beliefcombstate = beliefcombs[m][nexttargetstate[m] - belief_ncols*belief_nrows]
#                                             beliefstates = set()
#                                             for b in beliefcombstate:
#                                                 beliefstates = beliefstates.union(partitionGrid_c[m][b])
#                                             truebeliefstates[m] = beliefstates - beliefstates.intersection(visstates)

#                         print 'Environment state in automaton is', allstates[m].index(nexttargetstate[m])
#                         print 'Environment state in grid is', nexttargetstate[m]
#                         gwg_c.colorstates[1] = set()
#                         gwg_c.render()
#                         truebeliefstates[m] = set()
#                     else:
#                         nextstate[m] = nextstatedirn[m]['Out']
#                         nexttargetstate[m] = automaton[m][nextstate[m]]['State']['st']
#                         truebeliefstates[m] = set()

#                 gwg_c.colorstates[1] = set()
#                 for m in range(gwg_c.nagents):
#                     gwg_c.colorstates[1] = gwg_c.colorstates[1].union(truebeliefstates[m])



#             if len(automaton[0][nextstate[0]]['Successors']) > 0: #Fix this
#                 break

#         print 'Automaton state is ', nextstate
#         automaton_state = copy.deepcopy(nextstate)


# def userControlled_imperfect_sensor_Permissive(automaton,gwg_c,partitionGrid_c,moveobstacles,allowed_states,invisibilityset_c,belief_gridstates,sensor_uncertainty,agentpath,folderlocn):
#     automaton_state = [None] * gwg_c.nagents
#     agentstate = [None] * gwg_c.nagents
#     targetstate = [None] * gwg_c.nagents
#     truebeliefstates = [set()] * gwg_c.nagents
#     allstates = [[None]] * gwg_c.nagents
#     beliefcombs = [[None]] * gwg_c.nagents
#     xstates = list(set(gwg_c.states))
#     belief_ncols = gwg_c.ncols - sensor_uncertainty + 1
#     belief_nrows = gwg_c.nrows - sensor_uncertainty + 1
#     initial = {'s': gwg_c.current[0], 'st': moveobstacles[0]}
#     for n in range(gwg_c.nagents):
#         automaton_state[n] = findstate(automaton[n], initial).pop()
#         beliefcombs[n] = powerset(partitionGrid_c[n].keys())
#         allstates[n] = copy.deepcopy(belief_gridstates.keys())
#         for i in range(belief_ncols * belief_nrows, belief_ncols * belief_nrows + len(beliefcombs)):
#             allstates[n].append(i)
#         allstates.append(len(allstates))  # nominal state if target leaves allowed region
#     gwg_c.colorstates = [set(), set()]
#     gwg_c.targets = [[int(state) for state in agentpath]]
#     gridstate = copy.deepcopy(moveobstacles[0])
#     allow = True
#     tstep = folderlocn[1]
#     for desiredstate in agentpath:
#         gazeboOutput(gwg_c)
#         if not allow:
#             return gwg_c,tstep
#         gwg_c.colorstates[0] = set()
#         for n in range(gwg_c.nagents):
#             agentstate[n] = automaton[n][automaton_state[n]]['State']['s']
#             targetstate[n] = automaton[n][automaton_state[n]]['State']['st']
#             print 'Agent state is ', agentstate
#             gwg_c.colorstates[0] = gwg_c.colorstates[0].union(invisibilityset_c[n][agentstate[n]])

#         activeagents = set(range(gwg_c.nagents))
#         for n in range(gwg_c.nagents):
#             if targetstate[n] == allstates[n][-1]:
#                 activeagents.remove(n)

#         gwg_c.render()
#         gwg_c.moveobstacles[0] = copy.deepcopy(gridstate)

#         gwg_c.render()
#         gwg_c.current = copy.deepcopy(agentstate)

#         gwg_c.render()
#         # gwg_c.draw_state_labels()
#         gwfile = folderlocn[0] + 'figs/path/gridworldfig_' + str(tstep) + '.png'
#         tstep += 1
#         gwg_c.save(gwfile)
#         nextstatedirn = [dict()] * gwg_c.nagents
#         nexttargetstate = [None] * gwg_c.nagents
#         nextagentstate = [None] * gwg_c.nagents
#         nextstate = [None] * gwg_c.nagents
#         for m in activeagents:
#             nextstatedirn[m] = {'W': set(), 'E': set(), 'S': set(), 'N': set(), 'R': set(), 'Belief': set(),
#                                 'Out': None, 'Incoming': set()}
#             nextstates = Control_Parser.computeAutomatonState(automaton[m],automaton_state[m],{'s':desiredstate})
#             if len(nextstates) == 0:
#                 break
#             for n in nextstates:
#                 nexttargetstate[m] = automaton[m][n]['State']['st']
#                 if nexttargetstate[m] == allstates[m][-1]:
#                     nextstatedirn[m]['Out'] = n
#                 if nexttargetstate[m] >= belief_nrows * belief_ncols:
#                     nextstatedirn[m]['Belief'].add(n)
#                 else:
#                     if gwg_c.moveobstacles[0] - 1 in belief_gridstates[nexttargetstate[m]]:
#                         nextstatedirn[m]['W'].add(n)
#                     if gwg_c.moveobstacles[0] + 1 in belief_gridstates[nexttargetstate[m]]:
#                         nextstatedirn[m]['E'].add(n)
#                     if gwg_c.moveobstacles[0] + gwg_c.ncols in belief_gridstates[nexttargetstate[m]]:
#                         nextstatedirn[m]['S'].add(n)
#                     if gwg_c.moveobstacles[0] - gwg_c.ncols in belief_gridstates[nexttargetstate[m]]:
#                         nextstatedirn[m]['N'].add(n)
#                     if gwg_c.moveobstacles[0] in belief_gridstates[nexttargetstate[m]]:
#                         nextstatedirn[m]['R'].add(n)

#         while True:
#             while None in nextstate:
#                 while True:
#                     arrow = gwg_c.getkeyinput()
#                     if arrow != None:
#                         break
#                 for m in activeagents:
#                     if len(nextstatedirn[m]['Belief']) > 0:
#                         sensor = input('Measured by sensor? ')
#                     else:
#                         sensor = 1
#                     sensor = sensor > 0
#                     nextstate[m] = nextstatedirn[m][arrow]
#                     gridstate = getGridstate(gwg_c, moveobstacles[0], arrow)
#                     if not sensor:
#                         if gridstate in allowed_states[m]:
#                             for n in nextstatedirn[m]['Belief']:
#                                 nexttargetstate[m] = automaton[m][n]['State']['st']
#                                 nextbeliefs = beliefcombs[m][nexttargetstate[m] - belief_ncols * belief_nrows]
#                                 if any(gridstate in partitionGrid_c[m][x] for x in nextbeliefs):
#                                     nextstate[m] = copy.deepcopy(n)
#                                     nextagentstate[m] = automaton[m][n]['State']['s']
#                                     if nextagentstate[m] == desiredstate:
#                                         continue
#                                     allow = True
#                                     invisstates = invisibilityset_c[m][nextagentstate[m]]
#                                     visstates = set(xstates) - invisstates
#                                     if nexttargetstate[m] >= belief_ncols * belief_nrows:
#                                         beliefcombstate = beliefcombs[m][
#                                             nexttargetstate[m] - belief_ncols * belief_nrows]
#                                         beliefstates = set()
#                                         for b in beliefcombstate:
#                                             beliefstates = beliefstates.union(partitionGrid_c[m][b])
#                                         truebeliefstates[m] = beliefstates - beliefstates.intersection(visstates)
#                             if not allow:
#                                 break
#                                         # print 'True belief set is ', truebeliefstates
#                                         # print 'Size of true belief set is ', len(truebeliefstates)
#                         else:
#                             nextstate[m] = nextstatedirn[m]['Out']
#                             nexttargetstate[m] = automaton[m][nextstate[m]]['State']['st']
#                             print 'Environment state in automaton is', allstates[m].index(nexttargetstate[m])
#                             print 'Environment state in grid is', nexttargetstate[m]
#                             truebeliefstates[m] = set()
#                             gwg_c.render()
#                     else:
#                         # nextstate[m] = random.choice(list(nextstate[m]))
#                         nextstate[m] = Control_Parser.computeAutomatonState(automaton[m], automaton_state[m],
#                                                                             {'st':gridstate,'s': desiredstate})
#                         if len(nextstate[m])==0:
#                             allow = False
#                             print('Desired action not allowed! Compute new path')
#                             break

#                         else:
#                             nextstate[m] = nextstate[m].pop()
#                         nexttargetstate[m] = automaton[m][nextstate[m]]['State']['st']
#                         nextagentstate[m] = automaton[m][nextstate[m]]['State']['s']
#                         print 'Environment state in belief grid is', nexttargetstate[m]
#                         invisstates = invisibilityset_c[m][nextagentstate[m]]
#                         visstates = set(xstates) - invisstates
#                         truebeliefstates[m] = belief_gridstates[nexttargetstate[m]].intersection(visstates)
#                         gwg_c.render()
#                 # for m in set(range(gwg_c.nagents)) - activeagents:
#                 #     if gridstate in allowed_states[m]:
#                 #         for ns in nextstatedirn[m]['Incoming']:
#                 #             if gridstate == automaton[m][ns]['State']['st']:
#                 #                 nextstate[m] = ns
#                 #         nexttargetstate[m] = automaton[m][nextstate[m]]['State']['st']
#                 #         print 'Environment state in automaton is', allstates[m].index(nexttargetstate[m])
#                 #         print 'Environment state in grid is', nexttargetstate[m]
#                 #         gwg_c.colorstates[1] = set()
#                 #         gwg_c.render()
#                 #         truebeliefstates[m] = set()
#                 #     else:
#                 #         nextstate[m] = nextstatedirn[m]['Out']
#                 #         nexttargetstate[m] = automaton[m][nextstate[m]]['State']['st']
#                 #         truebeliefstates[m] = set()

#                 gwg_c.colorstates[1] = set()
#                 for m in range(gwg_c.nagents):
#                     gwg_c.colorstates[1] = gwg_c.colorstates[1].union(truebeliefstates[m])
#             if allow:
#                 if len(automaton[0][nextstate[0]]['Successors']) > 0:  # Fix this
#                     allow = True
#                     break
#             else:
#                 break
#         print 'Automaton state is ', nextstate
#         automaton_state = copy.deepcopy(nextstate)
#     if allow:
#         print('Reached vantage point! Compute new path')
#         for n in range(gwg_c.nagents):
#             print automaton_state[n]
#             agentstate[n] = automaton[n][automaton_state[n]]['State']['s']
#             targetstate[n] = automaton[n][automaton_state[n]]['State']['st']
#             print 'Agent state is ', agentstate
#             gwg_c.colorstates[0] = gwg_c.colorstates[0].union(invisibilityset_c[n][agentstate[n]])

#             gwg_c.render()
#             gwg_c.moveobstacles[0] = copy.deepcopy(gridstate)

#             gwg_c.render()
#             gwg_c.current = copy.deepcopy(agentstate)

#             gwg_c.render()
#     else:
#         print('Desired action not allowed! Compute new path')
#     # gwg_c.draw_state_labels()
#     return gwg_c,tstep


# def gazeboOutput(gwg_c):
#     for n in range(len(gwg_c.current)):
#         filename = 'statehistorypatrol{}.txt'.format(n)
#         with open(filename,'a') as file:
#             s = gwg_c.current[n]
#             file.write('{},{}\n'.format(gwg_c.current[n],gwg_c.moveobstacles[0]))
#             file.close()
#     # for n in range(len(gwg_c.moveobstacles)):
#     #     filename = 'statehistory_target{}.txt'.format(n)
#     #     y_obs,x_obs = gwg_c.coords(gwg_c.moveobstacles[n])
#     #     with open(filename,'a') as file:
#     #         file.write('{},{}\n'.format(x_obs,y_obs))
#     #         file.close()


