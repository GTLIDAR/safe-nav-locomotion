
import numpy as np
import copy
import itertools
from tqdm import *
import simplejson as json
import math
from gridworld_multi_obs import *
import time


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
                        variables[v] = [data['variables'].index(var), data['variables'].index(var2) + 1]

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

def reach_statesMO(gw,states):
    t =set()
    for state in states:
        for action in gw.actlistMO:
            t.update(set(np.nonzero(gw.probMO[action][state])[0]))
    return t

def reach_statesR(gw,states):
    t =set()
    for state in states:
        for action in gw.actlistR:
            t.update(set(np.nonzero(gw.probR[action][state])[0]))
    return t


def powerset(s):
    x = len(s)
    a = []
    then_PS = time.time()
    for i in range(1,1<<x):
        a.append({s[j] for j in range(x) if (i &(1<<j))})
    now_PS = time.time()
    print 'Power set took ', now_PS - then_PS, ' seconds'
    return a

def cartesian (lists):
    if lists == []: return [()]
    return [x + (y,) for x in cartesian(lists[:-1]) for y in lists[-1]]

def target_visibility(gw):
    return

##################################################################################
def card_to_slugs_int(card):
    if card == 'N':
        slugs_int = 0
    elif card == 'S':
        slugs_int = 1
    elif card == 'W':
        slugs_int = 2
    elif card == 'E':
        slugs_int = 3
    else:
        print('___-------> ERROR. The only inputs accepted by "card_to_slugs_int" are {N,S,W,E} <-------___')
        slugs_int = 4
    return slugs_int
##################################################################################
def slugs_int_to_card(slugs_int):
    if slugs_int == 0:
        card = 'N'
    elif slugs_int == 1:
        card = 'S'
    elif slugs_int == 2:
        card = 'W'
    elif slugs_int == 3:
        card = 'E'
    else:
        print('___-------> ERROR. The only inputs accepted by "slugs_int_to_card" are {0,1,2,3} <-------___')
        card = None
    return card
##################################################################################
def target_visibility(gw, invisibilityset, partitionGrid, target_vision_dist, filename_target_vis, allowed_states, visset_target):
    nonbeliefstates = gw.states
    beliefcombs = powerset(partitionGrid.keys())
    allstates = copy.deepcopy(nonbeliefstates)
    for i in range(gw.nstates,gw.nstates + len(beliefcombs)):
        allstates.append(i)
    allstates.append(len(allstates)) # nominal state if target leaves allowed region
    # Open a new file
    f = open(filename_target_vis,'w+')
    # Start the loop for determining vision
    counter1 = 0
    counter2 = 0
    print '(1) Looping Through Belief States'
    print '(2) Looping Through Physical States'
    print '(3) Looping Through Invisible Belief States to Determine Target Vision'
    for st in tqdm(set(allstates) - (set(nonbeliefstates) - set(allowed_states))):
        if (st in allowed_states) or (st == allstates[-1]):
            continue
        else:
            for s in tqdm(allowed_states):
                invisstates = invisibilityset[s]
                visstates = set(nonbeliefstates) - invisstates
                beliefcombstate = beliefcombs[st - len(nonbeliefstates)]
                beliefstates = set()
                for currbeliefstate in beliefcombstate:
                    beliefstates = beliefstates.union(partitionGrid[currbeliefstate])
                # beliefstates = beliefstates - set(targets) # remove target positions (no transitions from target positions)
                beliefstates_vis = beliefstates.intersection(visstates)
                beliefstates_invis = beliefstates - beliefstates_vis
                if not bool(beliefstates_invis):
                    continue
                string = '{}'.format(st) + ';' + '{}'.format(s) + ':'
                # # Determine which states are visible from the states (in the belief set) that cannot be seen by agent
                repeat = set()
                for iState in tqdm(beliefstates_invis):
                    for key,value in visset_target[iState].items():
                        if not bool(value):
                            continue
                        for item_2 in value:
                            if item_2 not in repeat:
                                string += ('{}'.format(item_2) + ',')
                                repeat.add(item_2)
                string = string[:-1] + '\n'
                f.write(string)
    f.close()
    return filename_target_vis
##################################################################################
def vis_parser(file_target_vis):
    vis_dict = dict()
    f2parse = open(file_target_vis, 'r')
    for lines in f2parse:
        # First split apart the belief state from the rest of the string
        array1 = lines.split(';')
        st_key = int(array1[0])
        # Next split the agent state from the rest of the string
        array2 = array1[1].split(':')
        s_key = int(array2[0])
        value = list(map(int, array2[1].split(',')))
        if st_key not in vis_dict:
            vis_dict[st_key] = dict()
        vis_dict[st_key][s_key] = value
    f2parse.close()
    return vis_dict
##################################################################################
def write_to_slugs_part_dist(infile,gw,init,initmovetarget,invisibilityset,PUDO_targets,visset_target = [],targets = [],vel=1,visdist = 5,allowed_states = [],
    fullvis_states = [],partitionGrid =dict(), belief_safety = 0, belief_liveness = 0, target_reachability = False,
    target_has_vision = False, target_vision_dist = 1.1, filename_target_vis = None, compute_vis_flag = False):
    nonbeliefstates = gw.states
    beliefcombs = powerset(partitionGrid.keys())
    #beliefcombs is all possible combination of belief states defined in main file
    allstates = copy.deepcopy(nonbeliefstates)
    for i in range(gw.nstates,gw.nstates + len(beliefcombs)):
        allstates.append(i)
    allstates.append(len(allstates)) # nominal state if target leaves allowed region
    num_of_obs = len(initmovetarget)


    filename = infile+'.structuredslugs'
    file = open(filename,'w')
    file.write('[INPUT]\n')
    stri = ""
    for n in range(0,len(initmovetarget)):
        stri += 'st{}:0...{}\n'.format(n,(len(gw.states)))
    file.write(stri)
    file.write('belief:0...{}\n'.format(len(beliefcombs)))
    # file.write('st2:0...{}\n'.format(len(allstates) -1))
    file.write('orientation:0...3\n')
    file.write('s_c:0...{}\n'.format(len(gw.states)-1))
    file.write('deliveryrequest\n')
    file.write('sOld:0...{}\n'.format(len(gw.states)-1))
    file.write('test\n')
    file.write('cond1\n')
    file.write('cond2\n')

    file.write('\n[OUTPUT]\n')
    file.write('directionrequest:0...4\n')
    file.write('stop\n')
    file.write('requestPending1\n')
    file.write('requestPending2\n')
    file.write('stairs\n')
    file.write('stairsN\n')
    # file.write('stairs_n\n')

    file.write('\n[ENV_INIT]\n')
    file.write('s_c = {}\n'.format(init))
    file.write('orientation = 1\n')
    file.write('deliveryrequest\n')

    # if initmovetarget in allowed_states:
    #     file.write('st = {}\n'.format(initmovetarget))
    # else:
    #     file.write('st = {}\n'.format(allstates[-1]))
    stri = ""
    for n in range(0,len(initmovetarget)):
        stri += 'st{} = {}\n'.format(n,initmovetarget[n])
    file.write(stri)

    file.write('sOld = {}\n'.format(init))
    

    file.write('\n[SYS_INIT]\n')
    file.write('stop\n')
    file.write('directionrequest = 2\n')

    # writing env_trans
    file.write('\n[ENV_TRANS]\n') #write specifications on how the environment state can transition at each step with "'" indicating the next state
    print 'Writing ENV_TRANS'
    for n in range(0,len(initmovetarget)):
        for st in tqdm(set(allstates) - (set(nonbeliefstates) - set(allowed_states)) - set(gw.stair_states)): #iterate through allowed states and belief states (tqdm displays a progress bar)
            if st in allowed_states: #write transitions if the dynamic obstacle (st) is visible
                for s in allowed_states:
                    repeat = set()
                    stri = "(s_c = {} /\\ st{} = {}) -> ".format(s,n,st)
                    beliefset = set()
                    for a in range(gw.nactionsMO):  
                        for t in np.nonzero(gw.probMO[gw.actlistMO[a]][st])[0]:
                            if t in allowed_states and t not in repeat:
                                if t not in invisibilityset[s]:
                                    stri += 'st{}\' = {} \\/'.format(n,t)
                                    repeat.add(t)
                                else:
                                    if not t == s and t not in targets: # not allowed to move on agent's position
                                        try: # here we evaluate which grid partitions the robot enters with action a
                                            partgridkeyind = [inv for inv in range(len(partitionGrid.values())) if t in partitionGrid.values()[inv]][0]
                                            t2 = partitionGrid.keys()[partgridkeyind]
                                            beliefset.add(t2) 
                                        except:
                                            print t
                                            print 'tests'
                            elif t not in allowed_states and t not in gw.obstacles and allstates[-1] not in repeat: # Error state????
                                stri += 'st{}\' = {} \\/'.format(n,allstates[-1])
                                # t should always be in allowed state or in obstacle state
                                repeat.add(allstates[-1])
                    if len(beliefset) > 0: #here we write the possible next belief state if the obstacle was at the edge of the visible range at the current step
                        b2 = allstates[len(nonbeliefstates) + beliefcombs.index(beliefset)]
                        if b2 not in repeat:
                            stri += ' st{}\' = {} \\/'.format(n,b2)
                            repeat.add(b2)
                    stri = stri[:-3]
                    stri += '\n'
                    file.write(stri)

            elif st == allstates[-1]: # Error state
                stri = "st{} = {} -> ".format(n,st)
                for t in fullvis_states:
                    stri += "st{}' = {} \\/ ".format(n,t)
                stri += "st{}' = {}".format(n,st)
                stri += '\n'
                file.write(stri)
            else: #write transitions if the dynamic obstacle (st) is not visible ransitions between belief states and from belief state to visible state
                for s in tqdm(allowed_states):
                    
                    (row,col)=gw.coords(s)
                    closestates = []
                    coordcombs = [[-3,0],[-2,0],[-1,0],[0,0],[1,0],[2,0],[3,0],[0,-3],[0,-2],[0,-1],[0,1],[0,2],[0,3],[1,1],[1,-1],[-1,-1],[-1,1]]

                    for coordspecific in coordcombs:
                        if (row + coordspecific[0]<gw.nrows) and (row + coordspecific[0]>0):
                            if (col+coordspecific[1]<gw.ncols) and (col+coordspecific[1]>0):
                                state = gw.coords2state_works(row+coordspecific[0],col+coordspecific[1])
                                closestates.append(state)

                    invisstates = invisibilityset[s]
                    visstates = set(nonbeliefstates) - invisstates
                    beliefcombstate = beliefcombs[st - len(nonbeliefstates)]
                    beliefstates = set()
                    for currbeliefstate in beliefcombstate:
                        beliefstates = beliefstates.union(partitionGrid[currbeliefstate])
                        # beliefstates is the combination of actual states that the target can be in based on your current state st
                    beliefstates = beliefstates - set(targets) # remove target positions (no transitions from target positions)
                    beliefstates_vis = beliefstates.intersection(visstates)

                    for sOld in closestates:
                        invisstatesOld = invisibilityset[sOld]
                        visstatesOld = set(nonbeliefstates) - invisstatesOld
                        Newvisstates = visstates - visstatesOld
                        beliefstates_invis_and_new = beliefstates - (beliefstates_vis - Newvisstates)
                        
                        if len(beliefstates_invis_and_new) > 0:
                            stri = "(s_c = {} /\\ st{} = {} /\\ sOld = {}) -> ".format(s,n,st,sOld)
                            repeat = set()
                            beliefset = set()
                            for b in beliefstates_invis_and_new:
                            # for b in beliefstates:
                                for a in range(gw.nactionsMO):
                                    for t in np.nonzero(gw.probMO[gw.actlistMO[a]][b])[0]:
                                        if t not in invisibilityset[s]:
                                            if t in allowed_states and t not in repeat:
                                                stri += ' st{}\' = {} \\/'.format(n,t)
                                                repeat.add(t)
                                        else:
                                            if t in gw.targets[0]:
                                                continue
                                            if t in allowed_states:
                                                t2 = partitionGrid.keys()[[inv for inv in range(len(partitionGrid.values())) if t in partitionGrid.values()[inv]][0]]
                                                beliefset.add(t2)
                            if len(beliefset) > 0:
                                b2 = allstates[len(nonbeliefstates) + beliefcombs.index(beliefset)]
                                if b2 not in repeat:
                                    stri += ' st{}\' = {} \\/'.format(n,b2)
                                    repeat.add(b2)

                            stri = stri[:-3]
                            stri += '\n'
                            file.write(stri)

    for n in range(0,len(initmovetarget)):
        for stair in gw.stair_states:
            file.write("st{}' != {}\n".format(n,stair))
        for obs in gw.no_obs:
            file.write("st{}' != {}\n".format(n,obs))

    print 'Writing Action Based Environment Transitions'
    stri = ''
    stri += '\n'
    stri += '\n'
    file.write(stri)

    for n in range(0,len(initmovetarget)):
        stri =""
        for edgeS in gw.edges:
            stri += "st{}' != {}\n".format(n,edgeS)
        stri += "\n"
        file.write(stri)


    file.write("sOld' = s_c\n\n")


    stri = "directionrequest = 0 -> orientation' = orientation & s_c' = s_c\n"
    stri += "directionrequest = 1 -> orientation' = 0 & s_c'+{} = s_c\n".format(gw.ncols)
    stri += "directionrequest = 2 -> orientation' = 1 & s_c' = s_c+1\n"
    stri += "directionrequest = 3 -> orientation' = 2 & s_c' = s_c+{}\n".format(gw.ncols)
    stri += "directionrequest = 4 -> orientation' = 3 & s_c'+1= s_c\n\n"
    file.write(stri)
    

    for n in range(0,len(initmovetarget)):
        file.write('directionrequest = 0 -> st{}\' != s_c\n'.format(n))




    stri = "((s_c =28 | s_c =29 | s_c =30) & "
    for n in range(0,len(initmovetarget)):
        stri += "st{} = {} & ".format(n,str(len(nonbeliefstates) + beliefcombs.index(set([4]))))
    stri = stri[:-3]
    stri += ") -> cond1'\n"
    file.write(stri)

    stri = "((s_c !=28 & s_c !=29 & s_c !=30) | "
    for n in range(0,len(initmovetarget)):
        stri += "st{}! = {} | ".format(n,str(len(nonbeliefstates) + beliefcombs.index(set([4]))))
    stri = stri[:-3]
    stri += ") -> !cond1'\n".format(str(len(nonbeliefstates) + beliefcombs.index(set([4]))))
    file.write(stri)


    stri = "((s_c !=28 & s_c !=29 & s_c !=30) & "
    for n in range(0,len(initmovetarget)):
        for visstate in set(nonbeliefstates):
            stri += "st{} != {} /\\ ".format(n,visstate)
    stri = stri[:-4]
    stri += ") -> cond2'\n"
    file.write(stri)

    stri = "((s_c =28 | s_c =29 | s_c =30) | "
    for n in range(0,len(initmovetarget)):
        for visstate in set(nonbeliefstates):
            stri += "st{} = {} | ".format(n,visstate)
    stri = stri[:-2]
    stri += ") -> !cond2'\n"
    file.write(stri)

    file.write("cond1 | cond2 -> test'\n")
    file.write("!cond1 & !cond2 -> !test'\n")


    # Writing env_safety
    print 'Writing ENV_SAFETY'
    for n in range(0,len(initmovetarget)):
        for obs in tqdm(gw.obstacles):
            if obs in allowed_states:
                file.write('!st = {}\n'.format(obs))






    # writing sys_trans
    file.write('\n[SYS_TRANS]\n')

    print 'Writing SYS_TRANS'
   

    stri = "(s_c = {}) -> ! requestPending1'\n".format(PUDO_targets[0])
    stri += "!((s_c = {})) ->(requestPending1' <-> (requestPending1 | deliveryrequest))\n\n".format(PUDO_targets[0])
    stri += "(s_c = {}) -> !requestPending2'\n".format(PUDO_targets[1])
    stri += "!((s_c = {})) -> (requestPending2' <-> ((s_c = {} & requestPending1) | requestPending2))\n\n".format(PUDO_targets[1],PUDO_targets[0])
    file.write(stri)


    stri = ""
    for row in range(gw.nrows-1):
        stri += "s_c = {} \\/ s_c = {} \\/ s_c = {} -> s_c' != {} & s_c' != {} & s_c' != {}\n".format(((row+1)*gw.ncols -3),((row+1)*gw.ncols -2),((row+1)*gw.ncols -1),((row+1)*gw.ncols),((row+1)*gw.ncols +1),((row+1)*gw.ncols +2))


    file.write(stri)

    file.write("orientation' = 0 -> directionrequest' !=3\n")
    file.write("orientation' = 1 -> directionrequest' !=4\n")
    file.write("orientation' = 2 -> directionrequest' !=1\n")
    file.write("orientation' = 3 -> directionrequest' !=2\n")

    file.write("orientation = 0 & directionrequest = 0 -> directionrequest' = 0 \/ directionrequest' = 1\n")
    file.write("orientation = 1 & directionrequest = 0 -> directionrequest' = 0 \/ directionrequest' = 2\n")
    file.write("orientation = 2 & directionrequest = 0 -> directionrequest' = 0 \/ directionrequest' = 3\n")
    file.write("orientation = 3 & directionrequest = 0 -> directionrequest' = 0 \/ directionrequest' = 4\n")


    stri =""
    for edgeS in gw.edges:
        stri += "s_c' != {}\n".format(edgeS)
    stri += "\n"
    file.write(stri)

    for stair in gw.stair_states:
        file.write("s_c' = {} -> stairs'\n".format(stair))

    stri = ""
    for stair in gw.stair_states:
        stri +="s_c' != {} & ".format(stair)
    stri = stri[:-3]
    stri +="-> !stairs'\n"
    file.write(stri)

    file.write("stairs' -> directionrequest' = 2 \/ directionrequest' = 4\n")
    # file.write("stairs_n -> stairs'\n")
    # file.write("!stairs_n -> !stairs'\n")
    file.write("stairs' -> directionrequest' = directionrequest\n")
    
    file.write("stairsN -> stairs'\n")
    file.write("!stairsN -> !stairs'\n")


    for obs in gw.obstacles:
        file.write('s_c != {}\n'.format(obs))


    for n in range(0,len(initmovetarget)):
        for s in set(allowed_states):
            stri = 'st{}\' = {} -> !s_c\' = {}\n'.format(n,s,s)
            file.write(stri)
            stri = 'st{}\' = {} -> !s_c = {}\n'.format(n,s,s)
            file.write(stri)





   

    # Writing sys_liveness
    file.write('\n[SYS_LIVENESS]\n')
    file.write("!requestPending1\n")
    file.write("!requestPending2\n")

    



    file.write('\n[ENV_LIVENESS]\n')
   
    # for n in range(0,len(initmovetarget)):
    #     stri = ""
    #     for visstate in set(nonbeliefstates):
    #         stri += "st{} != {} /\\ ".format(n,visstate)
    #     stri = stri[:-4]
    #     stri += "\n"
    #     file.write(stri)

    # stri = ""
    # for n in range(0,len(initmovetarget)):
    #     for visstate in set(nonbeliefstates):
    #         stri += "st{} != {} /\\ ".format(n,visstate)
    # stri = stri[:-4]
    # stri += "\n"
    # file.write(stri)

    file.write("test'\n")

    # stri = ""
    # for Bstate in (set(allstates) - set(nonbeliefstates)):
    #     stri += "st = {} \/ ".format(Bstate)
    # stri = stri[:-4]
    # file.write(stri)


    # stri = ""
    # for n in range(0,len(initmovetarget)):
    #     stri += "st{}' = {} & ".format(n,allstates[-2])
    # stri = stri[:-3]
    # file.write(stri)
    
    # file.write("st' = {} \\/ st' = {} \\/ st' = {}".format(68,80,allstates[-2]))
    # file.write("st' = {}".format(80))