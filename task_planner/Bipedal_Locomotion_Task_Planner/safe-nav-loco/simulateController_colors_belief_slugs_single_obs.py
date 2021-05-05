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
    inputVariables = dict()
    # APs = []
    APs = inputAPs + outputAPs
    inVars = []

    inbits = len(inputAPs)

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
    
    for var in inputAPs:
            v = var.split('@')[0]
            if v not in inputVariables.keys():
                # cnt+=1
                for var2ind in range(inputAPs.index(var),len(inputAPs)):
                    var2 = inputAPs[var2ind]
                    if v != var2.split('@')[0]:
                        inputVariables[v] = [inputAPs.index(var), inputAPs.index(var2)]
                        # inVars.append(v[[inputAPs.index(var), inputAPs.index(var2)]])
                        break
                    if inputAPs.index(var2) == len(inputAPs)-1:
                        inputVariables[v] = [inputAPs.index(var), inputAPs.index(var2)+1]
                        # inVars.append([inputAPs.index(var), inputAPs.index(var2)+1])

    
    state = dict.fromkeys(['State'])
    state['State'] = dict()
    for v in variables.keys():
        if variables[v][0] == variables[v][1]:
            bin  = [Bits[variables[v][0]]]
        else:
            bin = Bits[variables[v][0]:variables[v][1]]
        state['State'][v] = int(''.join(str(e) for e in bin)[::-1], 2)
      
    return state,inputVariables,inbits

def SlugsInput(inputVariables, state_next, inbits):
    inVars = inputVariables.keys()
    bits_l = [None]*inbits
    for var in inVars:
        bits_v = list(bin(state_next['State'][var])[2:].zfill(inputVariables[var][1]-inputVariables[var][0]))
        # bits_v_r = bits_v[::-1]
        for i in range(len(bits_v)):
            bits_l[inputVariables[var][0]+i] = bits_v[-1-i]
        
    bits = ''

    return bits.join(bits_l)


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

    stateSL, inputVars, inbits = parseSlugs(currentState, inputAPs, outputAPs)


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
    Bstates = []
    for i in range(len(beliefcombs)+1):
        Bstates.append(i)
    # Bstates.append(len(Bstates))
    # beliefcombs.append(set())
    
    while True:
        # output.saveState(gwg, automaton, automaton_state,gridstate,moveobstacles,gwg)
        state_next = copy.deepcopy(stateSL)

        envstate = stateSL['State']['st0']
        # try:
        #     print 'Agent state is J ', agentstate
        # except:
        #     print 'nothingyet'
        belief = stateSL['State']['belief']
        orientation = stateSL['State']['orientation']
        directionrequest = stateSL['State']['directionrequest']
        agentstate = stateSL['State']['s_c']

        if directionrequest == 0:
            # orientation_next = orientation
            state_next['State']['orientation'] = orientation
            state_next['State']['s_c'] = agentstate
        elif directionrequest == 1:
            state_next['State']['orientation'] = 0
            state_next['State']['s_c'] = agentstate-gwg.ncols
            # orientation_next = 0
        elif directionrequest == 2:
            state_next['State']['orientation'] = 1
            state_next['State']['s_c'] = agentstate+1
            # orientation_next = 1
        elif directionrequest == 3:
            state_next['State']['orientation'] = 2
            state_next['State']['s_c'] = agentstate+gwg.ncols
            # orientation_next = 2
        elif directionrequest == 4:
            state_next['State']['orientation'] = 3
            state_next['State']['s_c'] = agentstate-1
            # orientation_next = 3
        else:
            print 'orientation or state issue'

        state_next['State']['deliveryrequest']=1
        state_next['State']['sOld']=agentstate

        if ((agentstate ==28 or agentstate ==29 or agentstate == 30) and belief == beliefcombs.index(set([4])) and envstate == len(gwg.states)):
            state_next['State']['cond1'] = 1
        else:
            state_next['State']['cond1'] = 0

        if ((agentstate !=28 and agentstate !=29 and agentstate != 30) and envstate == len(gwg.states)):
            state_next['State']['cond2'] = 1
        else:
            state_next['State']['cond2'] = 0

        if stateSL['State']['cond1'] ==1 or stateSL['State']['cond2']==1:
            state_next['State']['test'] = 1
        else:
            state_next['State']['test'] = 0



        
        gwg.moveobstacles[0] = copy.deepcopy(gridstate)
        gwg.render()
        # ----------------- Robot Waits For Key Stroke To Take Action-----------------
        # while True:
        #             arrow = gwg.getkeyinput()
        #             if arrow != None:
        #                 break
        # ---------------------------------------------------------------------------------
        time.sleep(0.2)
        # agentstate = state['State']['s_c']
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
        nextbelief = {'W':None,'E':None,'S':None,'N':None,'R':None}
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
                        nextbelief[gwg.actlistMO[a]] = len(Bstates)-1
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
        
        # need code so if currently belief is non zero add all new possible beliefs to beliefset
        if belief != len(Bstates)-1:
            s = agentstate
            (row,col)=gwg.coords(s)
            closestates = []
            # coordcombs = [[-3,0],[-2,0],[-1,0],[0,0],[1,0],[2,0],[3,0],[0,-3],[0,-2],[0,-1],[0,1],[0,2],[0,3],[1,1],[1,-1],[-1,-1],[-1,1]]
            coordcombs = [[-1,0],[0,0],[1,0],[0,-1],[0,1],[1,1],[1,-1],[-1,-1],[-1,1]]
            
            for coordspecific in coordcombs:
                if (row + coordspecific[0]<gwg.nrows) and (row + coordspecific[0]>0):
                    if (col+coordspecific[1]<gwg.ncols) and (col+coordspecific[1]>0):
                        state = gwg.coords2state_works(row+coordspecific[0],col+coordspecific[1])
                        closestates.append(state)

            invisstates = invisibilityset[0][s]
            visstates = set(nonbeliefstates) - invisstates
            # beliefcombstate = beliefcombs[st - len(nonbeliefstates)]
            beliefcombstate = beliefcombs[belief]
            beliefstates = set()
            for currbeliefstate in beliefcombstate:
                beliefstates = beliefstates.union(partitionGrid[currbeliefstate])
                # beliefstates is the combination of actual states that the target can be in based on your current state st
            beliefstates = beliefstates - set(targets) # remove target positions (no transitions from target positions)
            beliefstates_vis = beliefstates.intersection(visstates)

            for sOld in closestates:
                invisstatesOld = invisibilityset[0][sOld]
                visstatesOld = set(nonbeliefstates) - invisstatesOld
                Newvisstates = visstates - visstatesOld
                beliefstates_invis_and_new = beliefstates - (beliefstates_vis - Newvisstates)

            for b in beliefstates_invis_and_new:
                for a in range(gwg.nactionsMO):
                    for t in np.nonzero(gwg.probMO[gwg.actlistMO[a]][b])[0]:
                        if t not in invisibilityset[0][s]:
                            if t in allowed_states[0] and t not in repeat:
                                # nothing here for now, captured by gridstate based code
                                repeat.add(t)
                        else:
                            if t in gwg.targets[0]:
                                continue
                            if t in allowed_states[0]:
                                t2 = partitionGrid.keys()[[inv for inv in range(len(partitionGrid.values())) if t in partitionGrid.values()[inv]][0]]
                                beliefset.add(t2)
                                # b_acts.append(gwg.actlistMO[a])



        for act in b_acts:
            if len(beliefset) > 0: #here we write the possible next belief state if the obstacle was at the edge of the visible range at the current step
                b2 = beliefcombs.index(beliefset)
                nextstatedirn[act]=len(gwg.states)
                nextbelief[act] = b2      
            else:
                print 'Issue: action without next state'


        while True:
            nextstate = None
            while nextstate == None:
                while True:
                    arrow = gwg.getkeyinput()
                    if arrow != None:
                        break
                nextstate = nextstatedirn[arrow]

                if nextstate == None:
                    break

                state_next['State']['st0'] = nextstate
                # if nextstate
                state_next['State']['belief'] = nextbelief[arrow]

                if state_next['State']['st0']==len(gwg.states):
                    nbelief = nextbelief[arrow]
                    beliefcombstate = beliefcombs[nbelief]
                    beliefstates = set()
                    invisstates = invisibilityset[0][state_next['State']['s_c']]
                    visstates = set(xstates) - set(invisstates)
                    for b in beliefcombstate:
                        beliefstates = beliefstates.union(partitionGrid[b]) 
                        gwg.colorstates[b+1] = copy.deepcopy(partitionGrid[b]) - partitionGrid[b].intersection(visstates)
                    truebeliefstates = beliefstates - beliefstates.intersection(visstates)
                    # gwg.colorstates[1] = copy.deepcopy(truebeliefstates)
                    gwg.render()
                    print 'True belief set is ', truebeliefstates
                    print 'Size of true belief set is ', len(truebeliefstates)

                else:
                    # nenvstate = gridstate
                    # print 'Environment state in automaton is', allstates.index(nenvstate)
                    # print 'Environment state in grid is', nenvstate
                    # gridstate = copy.deepcopy(nenvstate)
                    gwg.colorstates[1] = set()
                    gwg.colorstates[2] = set()
                    gwg.colorstates[3] = set()
                    gwg.colorstates[4] = set()
                    gwg.colorstates[5] = set()
                    gwg.colorstates[6] = set()
                    gwg.render()
                

                if nextstate == None:
                    break
                else:
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

                    nextInput = SlugsInput(inputVars, state_next, inbits)
                    slugsProcess.stdin.write("XMAKETRANS\n"+nextInput)
                    print "XMAKETRANS: "+nextInput
                    slugsProcess.stdin.flush()
                    slugsProcess.stdout.readline() # Skip the prompt
                    nextLine = slugsProcess.stdout.readline().strip()
                    print "NextLine:   "+nextLine
                    if nextLine!="ERROR":
                        stateSL, inputVars, inbits = parseSlugs(nextLine, inputAPs, outputAPs)
                        print "belief:" +str(stateSL['State']['belief'])
                        print "s_c:" + str(stateSL['State']['s_c'])
                        print "st0:" + str(stateSL['State']['st0'])
                    else:
                        print "ERROR"
                    # print 'test'
                    break
            break

        # if state['State']['st0']==len(gwg.states):
        #     beliefcombstate = beliefcombs[nbelief]
        #     beliefstates = set()
        #     for b in beliefcombstate:
        #         beliefstates = beliefstates.union(partitionGrid[b]) 
        #         gwg.colorstates[b+1] = copy.deepcopy(partitionGrid[b]) - partitionGrid[b].intersection(visstates)
        #     truebeliefstates = beliefstates - beliefstates.intersection(visstates)
        #     # gwg.colorstates[1] = copy.deepcopy(truebeliefstates)
        #     gwg.render()
        #     print 'True belief set is ', truebeliefstates
        #     print 'Size of true belief set is ', len(truebeliefstates)

        # else:
        #     # nenvstate = gridstate
        #     # print 'Environment state in automaton is', allstates.index(nenvstate)
        #     # print 'Environment state in grid is', nenvstate
        #     # gridstate = copy.deepcopy(nenvstate)
        #     gwg.colorstates[1] = set()
        #     gwg.colorstates[2] = set()
        #     gwg.colorstates[3] = set()
        #     gwg.colorstates[4] = set()
        #     gwg.colorstates[5] = set()
        #     gwg.colorstates[6] = set()
        #     gwg.render()
                    
                    # for n in nextstatedirn['Belief']:
                    #     nenvstate = automaton[n]['State']['st0']
                    #     nbelief = automaton[n]['State']['belief']
                    #     # nextbeliefs = beliefcombs[len(beliefcombs) - (len(allstates) - allstates.index(nenvstate))]
                    #     nextbeliefs = beliefcombs[nbelief]
                    #     if any(gridstate in partitionGrid[x] for x in nextbeliefs):
                    #         nextstate = copy.deepcopy(n)
                    #         print 'Environment state in automaton is', allstates.index(nenvstate)
                    #         print 'Belief state is', beliefcombs[allstates.index(nenvstate) - len(xstates)]
                    #         nextagentstate = automaton[n]['State']['s_c']
                    #         invisstates = invisibilityset[0][nextagentstate]
                    
                    #         visstates = set(xstates) - set(invisstates)
                    #         if nenvstate not in xstates:
                    #             # beliefcombstate = beliefcombs[allstates.index(nenvstate) - len(xstates)]
                    #             beliefcombstate = beliefcombs[nbelief]
                    #             beliefstates = set()
                    #             for b in beliefcombstate:
                    #                 beliefstates = beliefstates.union(partitionGrid[b]) 
                    #                 gwg.colorstates[b+1] = copy.deepcopy(partitionGrid[b]) - partitionGrid[b].intersection(visstates)
                    #             truebeliefstates = beliefstates - beliefstates.intersection(visstates)
                    #             # gwg.colorstates[1] = copy.deepcopy(truebeliefstates)
                    #             gwg.render()
                    #             print 'True belief set is ', truebeliefstates
                    #             print 'Size of true belief set is ', len(truebeliefstates)
                # else:
                #     nenvstate = automaton[nextstate]['State']['st0']
                #     print 'Environment state in automaton is', allstates.index(nenvstate)
                #     print 'Environment state in grid is', nenvstate
                #     gridstate = copy.deepcopy(nenvstate)
                #     gwg.colorstates[1] = set()
                #     gwg.colorstates[2] = set()
                #     gwg.colorstates[3] = set()
                #     gwg.colorstates[4] = set()
                #     gwg.colorstates[5] = set()
                #     gwg.colorstates[6] = set()
                #     gwg.render()

        #     try:
        #         print 'orientation is', automaton[automaton_state]['State']['orientation']
        #         print 'directionrequest is ', automaton[automaton_state]['State']['directionrequest']
        #         print 'Automaton state is', str(automaton_state)
        #     except:
        #         break

        #     if len(automaton[nextstate]['Successors']) > 0:
        #         break

        # print 'Automaton state is ', nextstate
        # # print 'actions: \norientation: ' + str(automaton[nextstate]['State']['orientation']) + '\nstop: ' + str(automaton[nextstate]['State']['stop']) + '\nturn Left: ' + str(automaton[nextstate]['State']['turnLeft']) + '\nturn Right: ' + str(automaton[nextstate]['State']['turnRight']) + '\nforward: ' + str(automaton[nextstate]['State']['forward'])  + '\nstepL: ' + str(automaton[nextstate]['State']['stepL'])  + '\nstanceFoot: ' + str(automaton[nextstate]['State']['stanceFoot'])
        # # print 'sOld: ' + str(automaton[nextstate]['State']['sOld'])
        # # print 'stanceFoot: ' + str(automaton[nextstate]['State']['stanceFoot'])
        # # print 'pastTurnStanceMatchFoot: ' + str(automaton[nextstate]['State']['pastTurnStanceMatchFoot'])
        # # print str(automaton[nextstate]['State'])
        # automaton_state = copy.deepcopy(nextstate)


