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
    gridstate = copy.deepcopy(moveobstacles[0])
    output = BeliefIOParser(jsonfile)
    while True:
        output.saveState(gwg, automaton, automaton_state,gridstate)
        envstate = automaton[automaton_state]['State']['st']
        # try:
        #     print 'Agent state is J ', agentstate
        # except:
        #     print 'nothingyet'

        
        gwg.moveobstacles[0] = copy.deepcopy(gridstate)
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
                while True:
                    arrow = gwg.getkeyinput()
                    if arrow != None:
                        break
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
                                truebeliefstates = beliefstates - beliefstates.intersection(visstates)
                                gwg.colorstates[1] = copy.deepcopy(truebeliefstates)
                                gwg.render()
                                print 'True belief set is ', truebeliefstates
                                print 'Size of true belief set is ', len(truebeliefstates)
                else:
                    nenvstate = automaton[nextstate]['State']['st']
                    print 'Environment state in automaton is', allstates.index(nenvstate)
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


# def userControlled_imperfect_sensor(filename,gwg,partitionGrid,moveobstacles,allowed_states,invisibilityset,belief_gridstates,sensor_uncertainty,saveImage=None):
#     automaton = [None]*gwg.nagents
#     automaton_state = [None]*gwg.nagents
#     agentstate = [None]*gwg.nagents
#     targetstate = [None]*gwg.nagents
#     truebeliefstates = [set()]*gwg.nagents
#     allstates = [[None]]*gwg.nagents
#     beliefcombs = [[None]]*gwg.nagents
#     xstates = list(set(gwg.states))
#     belief_ncols = gwg.ncols - sensor_uncertainty + 1
#     belief_nrows = gwg.nrows - sensor_uncertainty + 1
#     timestep = 0
#     for n in range(gwg.nagents):
#         automaton[n] = parseJson(filename[n])
#         automaton_state[n] = 0
#         beliefcombs[n] = powerset(partitionGrid[n].keys())
#         allstates[n] = copy.deepcopy(belief_gridstates.keys())
#         for i in range(belief_ncols * belief_nrows, belief_ncols * belief_nrows + len(beliefcombs[n])):
#             allstates[n].append(i)
#         allstates[n].append(len(allstates[n]))  # nominal state if target leaves allowed region
#     gwg.colorstates = [set(), set()]
#     gridstate = copy.deepcopy(moveobstacles[0])

#     while True:
#         gazeboOutput(gwg)
#         gwg.colorstates[0] = set()
#         for n in range(gwg.nagents):
#             agentstate[n] = automaton[n][automaton_state[n]]['State']['s']
#             targetstate[n] = automaton[n][automaton_state[n]]['State']['st']
#             print 'Agent state is ', agentstate
#             # gwg.colorstates[0] = gwg.colorstates[0].union(invisibilityset[n][agentstate[n]].intersection(allowed_states[n]))
#             gwg.colorstates[0] = gwg.colorstates[0].union(
#                 invisibilityset[n][agentstate[n]])
#         activeagents = set(range(gwg.nagents))
#         for n in range(gwg.nagents):
#             if targetstate[n] == allstates[n][-1]:
#                 activeagents.remove(n)

#         gwg.render()
#         gwg.moveobstacles[0] = copy.deepcopy(gridstate)

#         gwg.render()
#         gwg.current = copy.deepcopy(agentstate)

#         gwg.render()
#         # gwg.draw_state_labels()

#         if saveImage!=None:
#             gwg.save(saveImage+str(timestep)+'.png')
#             timestep+=1

#         nextstatedirn = [dict()]*gwg.nagents
#         nexttargetstate = [None]*gwg.nagents
#         nextagentstate = [None]*gwg.nagents
#         nextstate = [None]*gwg.nagents
#         targetcanbeinregion=[None]*gwg.nagents
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
#                     if gwg.moveobstacles[0] - 1 in belief_gridstates[nexttargetstate[m]]:
#                         nextstatedirn[m]['W'].add(n)
#                     if gwg.moveobstacles[0] + 1 in belief_gridstates[nexttargetstate[m]]:
#                         nextstatedirn[m]['E'].add(n)
#                     if gwg.moveobstacles[0] + gwg.ncols in belief_gridstates[nexttargetstate[m]]:
#                         nextstatedirn[m]['S'].add(n)
#                     if gwg.moveobstacles[0] - gwg.ncols in belief_gridstates[nexttargetstate[m]]:
#                         nextstatedirn[m]['N'].add(n)
#                     if gwg.moveobstacles[0] in belief_gridstates[nexttargetstate[m]]:
#                         nextstatedirn[m]['R'].add(n)
#         for m in set(range(gwg.nagents)) - activeagents:
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
#                     arrow = gwg.getkeyinput()
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
#                     gridstate = getGridstate(gwg, moveobstacles[0], arrow)
#                     if not sensor:
#                         if targetcanbeinregion[m]:
#                             for n in nextstatedirn[m]['Belief']:
#                                 nexttargetstate[m] = automaton[m][n]['State']['st']
#                                 if nexttargetstate[m] != allstates[m][-1]:
#                                     nextbeliefs = beliefcombs[m][nexttargetstate[m] - belief_ncols*belief_nrows]
#                                     if any(gridstate in partitionGrid[m][x] for x in nextbeliefs) or gridstate not in allowed_states[m]:
#                                         nextstate[m] = copy.deepcopy(n)
#                                         nextagentstate[m] = automaton[m][n]['State']['s']
#                                         invisstates = invisibilityset[m][nextagentstate[m]]
#                                         visstates = set(xstates) - invisstates
#                                         if nexttargetstate[m] >= belief_ncols*belief_nrows:
#                                             beliefcombstate = beliefcombs[m][nexttargetstate[m] - belief_ncols*belief_nrows]
#                                             beliefstates = set()
#                                             for b in beliefcombstate:
#                                                 beliefstates = beliefstates.union(partitionGrid[m][b])
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
#                             gwg.render()
#                     else:
#                         nextstate[m] = random.choice(list(nextstate[m]))
#                         nexttargetstate[m] = automaton[m][nextstate[m]]['State']['st']
#                         nextagentstate[m] = automaton[m][nextstate[m]]['State']['s']
#                         print 'Environment state in belief grid is', nexttargetstate[m]
#                         invisstates = invisibilityset[m][nextagentstate[m]]
#                         visstates = set(xstates) - invisstates
#                         truebeliefstates[m] = belief_gridstates[nexttargetstate[m]].intersection(visstates)
#                         gwg.render()

#                 for m in set(range(gwg.nagents)) - activeagents:
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
#                             invisstates = invisibilityset[m][nextagentstate[m]]
#                             visstates = set(xstates) - invisstates
#                             truebeliefstates[m] = belief_gridstates[nexttargetstate[m]].intersection(visstates)
#                             gwg.render()
#                         else:
#                             for n in nextstatedirn[m]['Belief']:
#                                 nexttargetstate[m] = automaton[m][n]['State']['st']
#                                 if nexttargetstate[m] != allstates[m][-1]:
#                                     nextbeliefs = beliefcombs[m][nexttargetstate[m] - belief_ncols*belief_nrows]
#                                     if any(gridstate in partitionGrid[m][x] for x in nextbeliefs):
#                                         nextstate[m] = copy.deepcopy(n)
#                                         nextagentstate[m] = automaton[m][n]['State']['s']
#                                         invisstates = invisibilityset[m][nextagentstate[m]]
#                                         visstates = set(xstates) - invisstates
#                                         if nexttargetstate[m] >= belief_ncols*belief_nrows:
#                                             beliefcombstate = beliefcombs[m][nexttargetstate[m] - belief_ncols*belief_nrows]
#                                             beliefstates = set()
#                                             for b in beliefcombstate:
#                                                 beliefstates = beliefstates.union(partitionGrid[m][b])
#                                             truebeliefstates[m] = beliefstates - beliefstates.intersection(visstates)

#                         print 'Environment state in automaton is', allstates[m].index(nexttargetstate[m])
#                         print 'Environment state in grid is', nexttargetstate[m]
#                         gwg.colorstates[1] = set()
#                         gwg.render()
#                         truebeliefstates[m] = set()
#                     else:
#                         nextstate[m] = nextstatedirn[m]['Out']
#                         nexttargetstate[m] = automaton[m][nextstate[m]]['State']['st']
#                         truebeliefstates[m] = set()

#                 gwg.colorstates[1] = set()
#                 for m in range(gwg.nagents):
#                     gwg.colorstates[1] = gwg.colorstates[1].union(truebeliefstates[m])



#             if len(automaton[0][nextstate[0]]['Successors']) > 0: #Fix this
#                 break

#         print 'Automaton state is ', nextstate
#         automaton_state = copy.deepcopy(nextstate)


# def userControlled_imperfect_sensor_Permissive(automaton,gwg,partitionGrid,moveobstacles,allowed_states,invisibilityset,belief_gridstates,sensor_uncertainty,agentpath,folderlocn):
#     automaton_state = [None] * gwg.nagents
#     agentstate = [None] * gwg.nagents
#     targetstate = [None] * gwg.nagents
#     truebeliefstates = [set()] * gwg.nagents
#     allstates = [[None]] * gwg.nagents
#     beliefcombs = [[None]] * gwg.nagents
#     xstates = list(set(gwg.states))
#     belief_ncols = gwg.ncols - sensor_uncertainty + 1
#     belief_nrows = gwg.nrows - sensor_uncertainty + 1
#     initial = {'s': gwg.current[0], 'st': moveobstacles[0]}
#     for n in range(gwg.nagents):
#         automaton_state[n] = findstate(automaton[n], initial).pop()
#         beliefcombs[n] = powerset(partitionGrid[n].keys())
#         allstates[n] = copy.deepcopy(belief_gridstates.keys())
#         for i in range(belief_ncols * belief_nrows, belief_ncols * belief_nrows + len(beliefcombs)):
#             allstates[n].append(i)
#         allstates.append(len(allstates))  # nominal state if target leaves allowed region
#     gwg.colorstates = [set(), set()]
#     gwg.targets = [[int(state) for state in agentpath]]
#     gridstate = copy.deepcopy(moveobstacles[0])
#     allow = True
#     tstep = folderlocn[1]
#     for desiredstate in agentpath:
#         gazeboOutput(gwg)
#         if not allow:
#             return gwg,tstep
#         gwg.colorstates[0] = set()
#         for n in range(gwg.nagents):
#             agentstate[n] = automaton[n][automaton_state[n]]['State']['s']
#             targetstate[n] = automaton[n][automaton_state[n]]['State']['st']
#             print 'Agent state is ', agentstate
#             gwg.colorstates[0] = gwg.colorstates[0].union(invisibilityset[n][agentstate[n]])

#         activeagents = set(range(gwg.nagents))
#         for n in range(gwg.nagents):
#             if targetstate[n] == allstates[n][-1]:
#                 activeagents.remove(n)

#         gwg.render()
#         gwg.moveobstacles[0] = copy.deepcopy(gridstate)

#         gwg.render()
#         gwg.current = copy.deepcopy(agentstate)

#         gwg.render()
#         # gwg.draw_state_labels()
#         gwfile = folderlocn[0] + 'figs/path/gridworldfig_' + str(tstep) + '.png'
#         tstep += 1
#         gwg.save(gwfile)
#         nextstatedirn = [dict()] * gwg.nagents
#         nexttargetstate = [None] * gwg.nagents
#         nextagentstate = [None] * gwg.nagents
#         nextstate = [None] * gwg.nagents
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
#                     if gwg.moveobstacles[0] - 1 in belief_gridstates[nexttargetstate[m]]:
#                         nextstatedirn[m]['W'].add(n)
#                     if gwg.moveobstacles[0] + 1 in belief_gridstates[nexttargetstate[m]]:
#                         nextstatedirn[m]['E'].add(n)
#                     if gwg.moveobstacles[0] + gwg.ncols in belief_gridstates[nexttargetstate[m]]:
#                         nextstatedirn[m]['S'].add(n)
#                     if gwg.moveobstacles[0] - gwg.ncols in belief_gridstates[nexttargetstate[m]]:
#                         nextstatedirn[m]['N'].add(n)
#                     if gwg.moveobstacles[0] in belief_gridstates[nexttargetstate[m]]:
#                         nextstatedirn[m]['R'].add(n)

#         while True:
#             while None in nextstate:
#                 while True:
#                     arrow = gwg.getkeyinput()
#                     if arrow != None:
#                         break
#                 for m in activeagents:
#                     if len(nextstatedirn[m]['Belief']) > 0:
#                         sensor = input('Measured by sensor? ')
#                     else:
#                         sensor = 1
#                     sensor = sensor > 0
#                     nextstate[m] = nextstatedirn[m][arrow]
#                     gridstate = getGridstate(gwg, moveobstacles[0], arrow)
#                     if not sensor:
#                         if gridstate in allowed_states[m]:
#                             for n in nextstatedirn[m]['Belief']:
#                                 nexttargetstate[m] = automaton[m][n]['State']['st']
#                                 nextbeliefs = beliefcombs[m][nexttargetstate[m] - belief_ncols * belief_nrows]
#                                 if any(gridstate in partitionGrid[m][x] for x in nextbeliefs):
#                                     nextstate[m] = copy.deepcopy(n)
#                                     nextagentstate[m] = automaton[m][n]['State']['s']
#                                     if nextagentstate[m] == desiredstate:
#                                         continue
#                                     allow = True
#                                     invisstates = invisibilityset[m][nextagentstate[m]]
#                                     visstates = set(xstates) - invisstates
#                                     if nexttargetstate[m] >= belief_ncols * belief_nrows:
#                                         beliefcombstate = beliefcombs[m][
#                                             nexttargetstate[m] - belief_ncols * belief_nrows]
#                                         beliefstates = set()
#                                         for b in beliefcombstate:
#                                             beliefstates = beliefstates.union(partitionGrid[m][b])
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
#                             gwg.render()
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
#                         invisstates = invisibilityset[m][nextagentstate[m]]
#                         visstates = set(xstates) - invisstates
#                         truebeliefstates[m] = belief_gridstates[nexttargetstate[m]].intersection(visstates)
#                         gwg.render()
#                 # for m in set(range(gwg.nagents)) - activeagents:
#                 #     if gridstate in allowed_states[m]:
#                 #         for ns in nextstatedirn[m]['Incoming']:
#                 #             if gridstate == automaton[m][ns]['State']['st']:
#                 #                 nextstate[m] = ns
#                 #         nexttargetstate[m] = automaton[m][nextstate[m]]['State']['st']
#                 #         print 'Environment state in automaton is', allstates[m].index(nexttargetstate[m])
#                 #         print 'Environment state in grid is', nexttargetstate[m]
#                 #         gwg.colorstates[1] = set()
#                 #         gwg.render()
#                 #         truebeliefstates[m] = set()
#                 #     else:
#                 #         nextstate[m] = nextstatedirn[m]['Out']
#                 #         nexttargetstate[m] = automaton[m][nextstate[m]]['State']['st']
#                 #         truebeliefstates[m] = set()

#                 gwg.colorstates[1] = set()
#                 for m in range(gwg.nagents):
#                     gwg.colorstates[1] = gwg.colorstates[1].union(truebeliefstates[m])
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
#         for n in range(gwg.nagents):
#             print automaton_state[n]
#             agentstate[n] = automaton[n][automaton_state[n]]['State']['s']
#             targetstate[n] = automaton[n][automaton_state[n]]['State']['st']
#             print 'Agent state is ', agentstate
#             gwg.colorstates[0] = gwg.colorstates[0].union(invisibilityset[n][agentstate[n]])

#             gwg.render()
#             gwg.moveobstacles[0] = copy.deepcopy(gridstate)

#             gwg.render()
#             gwg.current = copy.deepcopy(agentstate)

#             gwg.render()
#     else:
#         print('Desired action not allowed! Compute new path')
#     # gwg.draw_state_labels()
#     return gwg,tstep


# def gazeboOutput(gwg):
#     for n in range(len(gwg.current)):
#         filename = 'statehistorypatrol{}.txt'.format(n)
#         with open(filename,'a') as file:
#             s = gwg.current[n]
#             file.write('{},{}\n'.format(gwg.current[n],gwg.moveobstacles[0]))
#             file.close()
#     # for n in range(len(gwg.moveobstacles)):
#     #     filename = 'statehistory_target{}.txt'.format(n)
#     #     y_obs,x_obs = gwg.coords(gwg.moveobstacles[n])
#     #     with open(filename,'a') as file:
#     #         file.write('{},{}\n'.format(x_obs,y_obs))
#     #         file.close()


