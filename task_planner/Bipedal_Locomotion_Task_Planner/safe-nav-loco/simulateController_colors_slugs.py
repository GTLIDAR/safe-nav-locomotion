__author__ = 'sudab'
import random
import simplejson as json
import time
import copy
import itertools
import Control_Parser
import numpy as np
from beliefIOParser import BeliefIOParser
import subprocess
import os



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

def parseSlugs(bits, inputAPs, outputAPs = []):
    state = dict
    variables = dict()
    # APs = []
    APs = inputAPs + outputAPs

    Bits = list(bits)
    Bits.pop(len(Bits)-1)
    Bits.pop(len(Bits)-1)
    Bits = [int(i) for i in Bits]

    for var in APs:
            v = var.split('@')[0]
            if v not in variables.keys():
                for var2ind in range(APs.index(var),len(APs)):
                    var2 = APs[var2ind]
                    if v != var2.split('@')[0]:
                        variables[v] = [APs.index(var), APs.index(var2)]
                        break
                    if APs.index(var2) == len(APs)-1:
                        variables[v] = [APs.index(var), APs.index(var2)+1]

    
    state = dict.fromkeys(['State'])
    state['State'] = dict()
    for v in variables.keys():
        if variables[v][0] == variables[v][1]:
            bin  = [Bits[variables[v][0]]]
        else:
            bin = Bits[variables[v][0]:variables[v][1]]
        state['State'][v] = int(''.join(str(e) for e in bin)[::-1], 2)
      
    return state


def userControlled_partition(slugsLink,filename,gwg,partitionGrid,moveobstacles,invisibilityset,jsonfile,allowed_states = [],targets = []):
    # Open Slugs
    slugsProcess = subprocess.Popen(slugsLink+" --interactiveStrategy "+" --biasForAction"+" "+filename, shell=True, bufsize=32000, stdin=subprocess.PIPE, stdout=subprocess.PIPE)

    # Get input APs
    slugsProcess.stdin.write("XPRINTINPUTS\n")
    slugsProcess.stdin.flush()
    slugsProcess.stdout.readline() # Skip the prompt
    lastLine = " "
    inputAPs = []
    while (lastLine!=""):
        lastLine = slugsProcess.stdout.readline().strip()
        if lastLine!="":
            inputAPs.append(lastLine)

    # Get output APs
    slugsProcess.stdin.write("XPRINTOUTPUTS\n")
    slugsProcess.stdin.flush()
    slugsProcess.stdout.readline() # Skip the prompt
    lastLine = " "
    outputAPs = []
    while (lastLine!=""):
        lastLine = slugsProcess.stdout.readline().strip()
        if lastLine!="":
            outputAPs.append(lastLine)

    # Get initial state
    slugsProcess.stdin.write("XGETINIT\n")
    slugsProcess.stdin.flush()
    slugsProcess.stdout.readline() # Skip the prompt
    currentState = slugsProcess.stdout.readline().strip()

    state = parseSlugs(currentState, inputAPs, outputAPs)


    # automaton = parseJson(filename)
    # # for s in automaton:
    # #     print automaton[s]['State']['s']

    # automaton_state = 0
    xstates = list(set(gwg.states))
    allstates = copy.deepcopy(xstates)
    beliefcombs = powerset(partitionGrid.keys())
    for i in range(gwg.nstates,gwg.nstates+ len(beliefcombs)):
        allstates.append(i)
    gwg.colorstates = [set(), set(), set(), set(), set(), set(), set()]
    gridstate = copy.deepcopy(moveobstacles[0])
    output = BeliefIOParser(jsonfile)
    nonbeliefstates = gwg.states
    
    while True:
        # output.saveState(gwg, automaton, automaton_state,gridstate,moveobstacles,gwg)

        envstate = state['State']['st']
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
        time.sleep(0.2)
        agentstate = state['State']['s_c']
        print 'Agent state is ', agentstate
        gwg.render()
        # gwg.moveobstacles[0] = copy.deepcopy(gridstate)

        gwg.render()
        gwg.current = [copy.deepcopy(agentstate)]

        gwg.colorstates[0] = set()
        gwg.colorstates[0].update(invisibilityset[0][agentstate])
        gwg.render()
        # gwg.draw_state_labels()




        
        # nextstates = automaton[automaton_state]['Successors']
        nextstatedirn = {'W':None,'E':None,'S':None,'N':None,'R':None}
        # Need to Change this to add multiple next automaton states for one moving obstacle action (based on delivery request only I think, in this case only consider delivery request to be true). Need to add correct obstacle transition to specifications
        beliefset = set()
        repeat = set()
        b_acts = []
        for a in range(gwg.nactionsMO):  
            for t in np.nonzero(gwg.probMO[gwg.actlistMO[a]][gridstate])[0]:
                if t in allowed_states[0] and t not in repeat:
                    if t not in invisibilityset[0][agentstate]:
                        nextstatedirn[gwg.actlistMO[a]]=t
                        # stri += 'st\' = {} \\/'.format(t)
                        repeat.add(t)
                    else:
                        if not t == agentstate and t not in targets: # not allowed to move on agent's position
                            try: # here we evaluate which grid partitions the robot enters with action a
                                partgridkeyind = [inv for inv in range(len(partitionGrid.values())) if t in partitionGrid.values()[inv]][0]
                                t2 = partitionGrid.keys()[partgridkeyind]
                                beliefset.add(t2)
                                b_acts.append(gwg.actlistMO[a])
                            except:
                                print t
                                print 'tests'
        for act in b_acts:
            if len(beliefset) > 0: #here we write the possible next belief state if the obstacle was at the edge of the visible range at the current step
                b2 = allstates[len(nonbeliefstates) + beliefcombs.index(beliefset)]
                nextstatedirn[act]=b2      
            else:
                print 'Issue: action without next state'


        # for n in gwg.actlistMO:
        #     nenvstate = automaton[n]['State']['st']
        #     if nenvstate == gwg.moveobstacles[0] - 1:
        #         nextstatedirn['W'] = n
        #     if nenvstate == gwg.moveobstacles[0] + 1:
        #         nextstatedirn['E'] = n
        #     if nenvstate == gwg.moveobstacles[0] + gwg.ncols:
        #         nextstatedirn['S'] = n
        #     if nenvstate == gwg.moveobstacles[0] - gwg.ncols:
        #         nextstatedirn['N'] = n
        #     if nenvstate == gwg.moveobstacles[0]:
        #         nextstatedirn['R'] = n
        #     if nenvstate not in xstates:
        #         nextstatedirn['Belief'].add(n)

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
                    gridstate = copy.deepcopy(nenvstate)
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

        print 'Automaton state is ', nextstate
        # print 'actions: \norientation: ' + str(automaton[nextstate]['State']['orientation']) + '\nstop: ' + str(automaton[nextstate]['State']['stop']) + '\nturn Left: ' + str(automaton[nextstate]['State']['turnLeft']) + '\nturn Right: ' + str(automaton[nextstate]['State']['turnRight']) + '\nforward: ' + str(automaton[nextstate]['State']['forward'])  + '\nstepL: ' + str(automaton[nextstate]['State']['stepL'])  + '\nstanceFoot: ' + str(automaton[nextstate]['State']['stanceFoot'])
        # print 'sOld: ' + str(automaton[nextstate]['State']['sOld'])
        # print 'stanceFoot: ' + str(automaton[nextstate]['State']['stanceFoot'])
        # print 'pastTurnStanceMatchFoot: ' + str(automaton[nextstate]['State']['pastTurnStanceMatchFoot'])
        # print str(automaton[nextstate]['State'])
        automaton_state = copy.deepcopy(nextstate)

