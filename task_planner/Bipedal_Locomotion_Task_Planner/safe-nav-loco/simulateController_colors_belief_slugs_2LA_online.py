__author__ = 'Jonas'
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
from itertools import repeat, permutations
import lcm
from drake import lcmt_TAMP_waypoint
import datetime
# from statistics import mean

# global msg1
lc = lcm.LCM()
msg = lcmt_TAMP_waypoint()
msg1 = lcmt_TAMP_waypoint()


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

def my_handler(channel, data):
    global msg1
    msg1 = lcmt_TAMP_waypoint.decode(data)
    print "received N: " + str(msg1.N)
    print "received E: " + str(msg1.E)
    print "received S: " + str(msg1.S)
    print "received W: " + str(msg1.W)
    # print "test"
    # return msg1
    return


def userControlled_partition(slugsLink,filename,gwg,partitionGrid,moveobstacles,invisibilityset,jsonfile,filename_f,gwg_f,partitionGrid_f,moveobstacles_f,invisibilityset_f,jsonfile_f,allowed_states = [],targets = []):
    # lc = lcm.LCM()
    then = time.time()
    # msg = lcmt_TAMP_waypoint()
    # msg1 = lcmt_TAMP_waypoint()
    subscription = lc.subscribe("new_waypoint", my_handler)

    # print "Start time is: " + str(then)
    print 'Start time is: ' + str(datetime.datetime.now().time())
    # Open Slugs
    slugsProcess = subprocess.Popen(slugsLink+" --interactiveStrategy "+" --biasForAction"+" "+filename, shell=True, bufsize=32000, stdin=subprocess.PIPE, stdout=subprocess.PIPE)
    print ' 3.1 '
    # Get input APs
    slugsProcess.stdin.write("XPRINTINPUTS\n")
    slugsProcess.stdin.flush()
    slugsProcess.stdout.readline() # Skip the prompt
    lastLine = " "
    inputAPs = []
    print ' 3.5 '
    while (lastLine!=""):
        lastLine = slugsProcess.stdout.readline().strip()
        if lastLine!="":
            inputAPs.append(lastLine)
    
    now = time.time()
    print('Synthesis took ', now - then, ' seconds')

    print ' 4 '
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
    gridstate = copy.deepcopy(moveobstacles)
    output = BeliefIOParser(jsonfile)
    nonbeliefstates = gwg.states
    Bstates = []
    for i in range(len(beliefcombs)+1):
        Bstates.append(i)
    # Bstates.append(len(Bstates))
    # beliefcombs.append(set())

    automaton_f = parseJson(filename_f)
    automaton_state_f = 0
    xstates_f = list(set(gwg_f.states))
    allstates_f = copy.deepcopy(xstates_f)


    gwg_f.colorstates = [set(), set()]
    gridstate_f = copy.deepcopy(moveobstacles_f[0])


    while True:
        # output.saveState(gwg, automaton, automaton_state,gridstate,moveobstacles,gwg)
        state_next = copy.deepcopy(stateSL)

        envstate0 = stateSL['State']['st0']
        envstate1 = stateSL['State']['st1']
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

        if ((agentstate ==28 or agentstate ==29 or agentstate == 30) and belief == beliefcombs.index(set([4])) and envstate0 == len(gwg.states) and envstate1 == len(gwg.states)):
            state_next['State']['cond1'] = 1
        else:
            state_next['State']['cond1'] = 0

        if ((agentstate !=28 and agentstate !=29 and agentstate != 30) and envstate0 == len(gwg.states) and envstate1 == len(gwg.states)):
            state_next['State']['cond2'] = 1
        else:
            state_next['State']['cond2'] = 0

        if stateSL['State']['cond1'] ==1 or stateSL['State']['cond2']==1:
            state_next['State']['test'] = 1
        else:
            state_next['State']['test'] = 0



        
        gwg.moveobstacles = copy.deepcopy(gridstate)
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

        envstate_f = automaton_f[automaton_state_f]['State']['st']
        gwg_f.moveobstacles_f[0] = copy.deepcopy(gridstate_f)
        agentstate_f = automaton_f[automaton_state_f]['State']['s']







        # insert fine level controller here >>>>>>

        while True:
            # output.saveState(gwg_f, automaton_f, automaton_state_f,gridstate_f,moveobstacles_f)
            # output.saveState(gwg_f, automaton_f, automaton_state_f,gridstate,moveobstacles_c, gwg_c)
            agentstate_f = automaton_f[automaton_state_f]['State']['s']
          

            nextstates_f = automaton_f[automaton_state_f]['Successors']
            # nextstatedirn_f = {"1":{"1":{'4':None,'2':None,'3':None,'1':None,'0':None}, "0":{'4':None,'2':None,'3':None,'1':None,'0':None}},"0":{"1":{'4':None,'2':None,'3':None,'1':None,'0':None}, "0":{'4':None,'2':None,'3':None,'1':None,'0':None}}}
            
            nextstatedirn_f ={}

            ### nextstatedirn_f first number is stair boolean, second number is direcion request
            for n in nextstates_f:
                # nenv_dir_req = automaton_f[n]['State']['directionrequest']
                # nenv_stair_req = automaton_f[n]['State']['stair']
                # nenv_stairN_req = automaton_f[n]['State']['stairN']
                if stateSL['State']['stairs'] != automaton_f[n]['State']['stair']:
                    continue
                if stateSL['State']['stairsN'] != automaton_f[n]['State']['stairN']:
                    continue
                if stateSL['State']['directionrequest'] != automaton_f[n]['State']['directionrequest']:
                    continue

                nextstatedirn_f[automaton_f[n]['State']['s']] = n


                
              
            while True:

                msg.stepL = automaton_f[automaton_state_f]['State']['stepL']
                msg.stepH = automaton_f[automaton_state_f]['State']['stepH']
                msg.turn = automaton_f[automaton_state_f]['State']['turn']
                msg.stop =automaton_f[automaton_state_f]['State']['stop']
                msg.forward = automaton_f[automaton_state_f]['State']['forward']
                msg.stanceFoot = automaton_f[automaton_state_f]['State']['stanceFoot']
                msg.orientation = automaton_f[automaton_state_f]['State']['orientation']
                msg.s = gridstate[0] # automaton_f[automaton_state_f]['State']['s']
                msg.TaskAchieved = gridstate[1] # automaton_f[automaton_state_f]['State']['TaskAchieved']
                # msg.obs1 = automaton_f[automaton_state_f]['State']['turn'] #gridstate[0]
                # msg.obs2 = automaton_f[automaton_state_f]['State']['turn'] # gridstate[1]
                print 'sent TaskAchieved is ',msg.TaskAchieved
                lc.publish("new_waypoint", msg.encode())

                nextstate_f = None
                while nextstate_f == None:
                    time.sleep(0.2)

                    # subscription = lc.subscribe("new_waypoint", my_handler)

                    # msg.N = 1
                    # lc.publish("new_waypoint", msg.encode())

                    # subscription = lc.subscribe("new_waypoint", my_handler)
                    try:
                        # while True:
                        lc.handle()
                        lc.handle()
                        print "Test LC Handle"
                    except:
                        pass


                    # lc.unsubscribe(subscription)


                    # row,col = gwg_f.coords(agentstate_f)
                    # nextstate_f = nextstatedirn_f[str(automaton[automaton_state]['State']['stairs'])][str(automaton[automaton_state]['State']['stairsN'])][str(automaton[automaton_state]['State']['directionrequest'])]
                    # nextstate_f = nextstatedirn_f[str(stateSL['State']['stairs'])][str(stateSL['State']['stairsN'])][str(stateSL['State']['directionrequest'])]
                    if len(nextstatedirn_f) == 1:
                        nextstate_f = nextstatedirn_f[nextstatedirn_f.keys()[0]]
                    else:
                        # ave = np.mean(nextstatedirn_f.keys())
                        # nstate = ave
                        # if msg1.N == 1:
                        # nstate = gw.transR[gw.transR[nstate]['W5'][0]]['S1'][0]
                        row,col = gwg_f.coords(agentstate_f)
                        nextgrids = gwg_f.MapState((row + msg1.S - msg1.N, col + msg1.E - msg1.W),agentstate_f)[0]
                        nextstate_f = nextstatedirn_f[nextgrids]



                    nenv_dir_req = automaton_f[nextstate_f]['State']['st']
                    print 'Environment state in fine automaton is', allstates.index(nenv_dir_req)
                    print 'Environment state in fine grid is', nenv_dir_req
                    print 'Stair Boolean is ', automaton_f[nextstate_f]['State']['stair']
                    print 'Step Length is ', automaton_f[nextstate_f]['State']['stepL']
                    print 'turn is ', automaton_f[nextstate_f]['State']['turn']
                    print 'Agent location in action planner is ', str(nextstate_f)
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

            # msg.stepL = automaton_f[automaton_state_f]['State']['stepL']
            # msg.stepH = automaton_f[automaton_state_f]['State']['stepH']
            # msg.turn = automaton_f[automaton_state_f]['State']['turn']
            # msg.stop =automaton_f[automaton_state_f]['State']['stop']
            # msg.forward = automaton_f[automaton_state_f]['State']['forward']
            # msg.stanceFoot = automaton_f[automaton_state_f]['State']['stanceFoot']
            # msg.orientation = automaton_f[automaton_state_f]['State']['orientation']
            # msg.s = automaton_f[automaton_state_f]['State']['s']
            # lc.publish("new_waypoint", msg.encode())

            # if automaton_f[automaton_state_f]['State']['requestPending1'] == 5:
            #     break
            if automaton_f[automaton_state_f]['State']['TaskAchieved'] == 1:
                break



        # <<<<<<  insert fine level controller above


        time.sleep(0.5)
        gwg.render()










        
        # nextstates = automaton[automaton_state]['Successors']
        nextstatedirn0 = {'W':None,'E':None,'S':None,'N':None,'R':None}
        nextstatedirn1 = {'W':None,'E':None,'S':None,'N':None,'R':None}

        # nextbelief = {'W':None,'E':None,'S':None,'N':None,'R':None}
        
        # nextstatedirn = {'W':{'W':None,'E':None,'S':None,'N':None,'R':None},'E':{'W':None,'E':None,'S':None,'N':None,'R':None},'S':{'W':None,'E':None,'S':None,'N':None,'R':None},'N':{'W':None,'E':None,'S':None,'N':None,'R':None},'R':{'W':None,'E':None,'S':None,'N':None,'R':None}}
        nextbelief = {'W':{'W':None,'E':None,'S':None,'N':None,'R':None},'E':{'W':None,'E':None,'S':None,'N':None,'R':None},'S':{'W':None,'E':None,'S':None,'N':None,'R':None},'N':{'W':None,'E':None,'S':None,'N':None,'R':None},'R':{'W':None,'E':None,'S':None,'N':None,'R':None}}
        
        
        # Need to Change this to add multiple next automaton states for one moving obstacle action (based on delivery request only I think, in this case only consider delivery request to be true). Need to add correct obstacle transition to specifications
        beliefset0 = set()
        beliefset1 = set()
        repeat0 = set()
        repeat1 = set()
        b_acts0 = []
        b_acts1 = []
        actions0 = []
        actions1 = []
        for a in range(gwg.nactionsMO):  
            for t in np.nonzero(gwg.probMO[gwg.actlistMO[a]][gridstate[0]])[0]:
                if t in allowed_states[0] and t not in repeat0:
                    if t not in invisibilityset[0][agentstate]:
                        nextstatedirn0[gwg.actlistMO[a]]=t
                        actions0.append(gwg.actlistMO[a])
                        # stri += 'st\' = {} \\/'.format(t)
                        # nextbelief[gwg.actlistMO[a]] = len(Bstates)-1
                        repeat0.add(t)
                    else:
                        if not t == agentstate and t not in targets: # not allowed to move on agent's position
                            try: # here we evaluate which grid partitions the robot enters with action a
                                partgridkeyind = [inv for inv in range(len(partitionGrid.values())) if t in partitionGrid.values()[inv]][0]
                                t2 = partitionGrid.keys()[partgridkeyind]
                                beliefset0.add(t2)
                                b_acts0.append(gwg.actlistMO[a])
                                nextstatedirn0[gwg.actlistMO[a]]=len(gwg.states)
                                actions0.append(gwg.actlistMO[a])
                            except:
                                print t
                                print 'tests'

        for a in range(gwg.nactionsMO):  
            for t in np.nonzero(gwg.probMO[gwg.actlistMO[a]][gridstate[1]])[0]:
                if t in allowed_states[0] and t not in repeat1:
                    if t not in invisibilityset[0][agentstate]:
                        nextstatedirn1[gwg.actlistMO[a]]=t
                        actions1.append(gwg.actlistMO[a])
                        # stri += 'st\' = {} \\/'.format(t)
                        # nextbelief[gwg.actlistMO[a]] = len(Bstates)-1
                        repeat1.add(t)
                    else:
                        if not t == agentstate and t not in targets: # not allowed to move on agent's position
                            try: # here we evaluate which grid partitions the robot enters with action a
                                partgridkeyind = [inv for inv in range(len(partitionGrid.values())) if t in partitionGrid.values()[inv]][0]
                                t2 = partitionGrid.keys()[partgridkeyind]
                                beliefset1.add(t2)
                                b_acts1.append(gwg.actlistMO[a])
                                nextstatedirn1[gwg.actlistMO[a]]=len(gwg.states)
                                actions1.append(gwg.actlistMO[a])
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



            if envstate0 == len(gwg.states):
                for b in beliefstates_invis_and_new:
                    for a in range(gwg.nactionsMO):
                        for t in np.nonzero(gwg.probMO[gwg.actlistMO[a]][b])[0]:
                            if t not in invisibilityset[0][s]:
                                if t in allowed_states[0] and t not in repeat0:
                                    # nothing here for now, captured by gridstate based code
                                    repeat0.add(t)
                            else:
                                if t in gwg.targets[0]:
                                    continue
                                if t in allowed_states[0]:
                                    t2 = partitionGrid.keys()[[inv for inv in range(len(partitionGrid.values())) if t in partitionGrid.values()[inv]][0]]
                                    beliefset0.add(t2)
                                    # nextstatedirn0[gwg.actlistMO[a]]=len(gwg.states)
                                    # b_acts.append(gwg.actlistMO[a])

            if envstate1 == len(gwg.states):
                for b in beliefstates_invis_and_new:
                    for a in range(gwg.nactionsMO):
                        for t in np.nonzero(gwg.probMO[gwg.actlistMO[a]][b])[0]:
                            if t not in invisibilityset[0][s]:
                                if t in allowed_states[0] and t not in repeat1:
                                    # nothing here for now, captured by gridstate based code
                                    repeat1.add(t)
                            else:
                                if t in gwg.targets[0]:
                                    continue
                                if t in allowed_states[0]:
                                    t2 = partitionGrid.keys()[[inv for inv in range(len(partitionGrid.values())) if t in partitionGrid.values()[inv]][0]]
                                    beliefset1.add(t2)
                                    # nextstatedirn1[gwg.actlistMO[a]]=len(gwg.states)
                                    # b_acts.append(gwg.actlistMO[a])

        for acts in list(itertools.product(actions0, actions1)):
            if nextstatedirn0[acts[0]] == len(gwg.states) and nextstatedirn1[acts[1]] != len(gwg.states):
                b2 = beliefcombs.index(beliefset0)
                nextbelief[acts[0]][acts[1]] = b2 
            if nextstatedirn0[acts[0]] != len(gwg.states) and nextstatedirn1[acts[1]] == len(gwg.states):
                b2 = beliefcombs.index(beliefset1)
                nextbelief[acts[0]][acts[1]] = b2
            if nextstatedirn0[acts[0]] == len(gwg.states) and nextstatedirn1[acts[1]] == len(gwg.states):
                b2 = beliefcombs.index(beliefset0.union(beliefset1))
                nextbelief[acts[0]][acts[1]] = b2
            if nextstatedirn0[acts[0]] != len(gwg.states) and nextstatedirn1[acts[1]] != len(gwg.states):
                # b2 = beliefcombs.index(beliefset0)
                nextbelief[acts[0]][acts[1]] = len(Bstates)-1 


        


        # for acts in list(itertools.product(b_acts0, b_acts1)):
        #     if nextstatedirn0[acts[0]] == len(gwg.states)


        # for act in b_acts:
        #     if len(beliefset) > 0: #here we write the possible next belief state if the obstacle was at the edge of the visible range at the current step
        #         b2 = beliefcombs.index(beliefset)
        #         nextstatedirn[act]=len(gwg.states)
        #         nextbelief[act] = b2      
        #     else:
        #         print 'Issue: action without next state'




        while True:
            nextstate0 = None
            nextstate1 = None
            while nextstate0 == None or nextstate1 == None:
                while True:
                    arrow0 = gwg.getkeyinput()
                    if arrow0 != None:
                        break
                while True:
                    arrow1 = gwg.getkeyinput()
                    if arrow1 != None:
                        break
                nextstate0 = nextstatedirn0[arrow0]
                nextstate1 = nextstatedirn1[arrow1]

                if nextstate0 == None or nextstate1 == None:
                    break

                state_next['State']['st0'] = nextstate0
                state_next['State']['st1'] = nextstate1
                # if nextstate
                state_next['State']['belief'] = nextbelief[arrow0][arrow1]

                if state_next['State']['belief'] != len(Bstates)-1 :
                    nbelief = nextbelief[arrow0][arrow1]
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
                    # nenvstate0 = gridstate
                    # print 'Environment state in automaton is', allstates.index(nenvstate)
                    # print 'Environment state in grid is', nenvstate
                    # gridstate = copy.deepcopy(nenvstate0)
                    gwg.colorstates[1] = set()
                    gwg.colorstates[2] = set()
                    gwg.colorstates[3] = set()
                    gwg.colorstates[4] = set()
                    gwg.colorstates[5] = set()
                    gwg.colorstates[6] = set()
                    gwg.render()
                

                if nextstate0 == None or nextstate1 == None:
                    break
                else:
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
                        print "st1:" + str(stateSL['State']['st1'])
                    else:
                        print "ERROR"
                        nextstate0 = None
                        nextstate1 = None
                        break

                    if arrow0 == 'W':
                        gridstate[0] = gwg.moveobstacles[0] - 1
                    elif arrow0 == 'E':
                        gridstate[0] = gwg.moveobstacles[0] + 1
                    elif arrow0 == 'S':
                        gridstate[0] = gwg.moveobstacles[0] + gwg.ncols
                    elif arrow0 == 'N':
                        gridstate[0] = gwg.moveobstacles[0] - gwg.ncols
                    elif arrow0 == 'R':
                        gridstate[0] = gwg.moveobstacles[0]

                    if arrow1 == 'W':
                        gridstate[1] = gwg.moveobstacles[1] - 1
                    elif arrow1 == 'E':
                        gridstate[1] = gwg.moveobstacles[1] + 1
                    elif arrow1 == 'S':
                        gridstate[1] = gwg.moveobstacles[1] + gwg.ncols
                    elif arrow1 == 'N':
                        gridstate[1] = gwg.moveobstacles[1] - gwg.ncols
                    elif arrow1 == 'R':
                        gridstate[1] = gwg.moveobstacles[1]

                    # nextInput = SlugsInput(inputVars, state_next, inbits)
                    # slugsProcess.stdin.write("XMAKETRANS\n"+nextInput)
                    # print "XMAKETRANS: "+nextInput
                    # slugsProcess.stdin.flush()
                    # slugsProcess.stdout.readline() # Skip the prompt
                    # nextLine = slugsProcess.stdout.readline().strip()
                    # print "NextLine:   "+nextLine
                    # if nextLine!="ERROR":
                    #     stateSL, inputVars, inbits = parseSlugs(nextLine, inputAPs, outputAPs)
                    #     print "belief:" +str(stateSL['State']['belief'])
                    #     print "s_c:" + str(stateSL['State']['s_c'])
                    #     print "st0:" + str(stateSL['State']['st0'])
                    #     print "st1:" + str(stateSL['State']['st1'])
                    # else:
                    #     print "ERROR"
                    # print 'test'


                    if nextstate0 != None and nextstate1 != None:
                        break
                    # break
            # break
            if nextstate0 != None and nextstate1 != None:
                break
