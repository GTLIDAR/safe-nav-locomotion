
import numpy as np
import copy
import itertools
from tqdm import *
import simplejson as json
import math
from gridworld_fine_auto_spec import *
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


    filename = infile+'.structuredslugs'
    file = open(filename,'w')
    file.write('[INPUT]\n')

    # file.write('st:0...{}\n'.format(len(allstates) -1))
    file.write('st:0...1\n')
    file.write('orientation:0...15\n')
    file.write('s:0...{}\n'.format(len(gw.states)-1))
    file.write('directionrequest:0...4\n')
    file.write('stair\n')
    file.write('crossV:0...2\n')
    file.write('crossH:0...2\n')


    file.write('\n[OUTPUT]\n')
    file.write('forward\n')
    file.write('stepL:0...4\n')
    file.write('stop\n')
    file.write('requestPending1:0...5\n')
    file.write('stepH:0...6\n')
    file.write('turn:0...4\n')
    file.write('stanceFoot:0...1\n')
    file.write('fail:0...1\n')



    if target_reachability:
        file.write('c:0...1\n')

    file.write('\n[ENV_INIT]\n')
    file.write('s = {}\n'.format(init))
    file.write('orientation = 4\n')
    file.write('directionrequest = 2\n')
    file.write('!stair\n')
    file.write('crossV=0\n')
    file.write('crossH=0\n')


    if initmovetarget in allowed_states:
        file.write('st = {}\n'.format(initmovetarget))
    else:
        file.write('st = {}\n'.format(allstates[-1]))

    

    file.write('\n[SYS_INIT]\n')
    if target_reachability:
        file.write('c = 0\n')

    file.write('!forward\n')

    file.write('turn = 2\n')
    file.write('stop\n')
    file.write('stepH = 3\n')
    file.write('requestPending1=2\n')
    file.write('stepL=0\n')
    file.write('stanceFoot=0\n')

    # writing env_trans
    file.write('\n[ENV_TRANS]\n')
    print 'Writing ENV_TRANS'
    file.write("st' = {}\n".format(initmovetarget))

    print 'Writing Action Based Environment Transitions'
    stri = ''
    stri += '\n'
    stri += '\n'
    file.write(stri)


   

    #### Walking Straight ####

    stri = ""
    for s in gw.states:
        stri += "((orientation=0 & turn=2)) & s={} & forward & (stepL=0 | stepL=1) -> s' = {}\n".format(s,gw.transR[s]['N4'][0])
        stri += "((orientation=4 & turn=2)) & s={} & forward & (stepL=0 | stepL=1) -> s' = {}\n".format(s,gw.transR[s]['E4'][0])
        stri += "((orientation=8 & turn=2)) & s={} & forward & (stepL=0 | stepL=1) -> s' = {}\n".format(s,gw.transR[s]['S4'][0])
        stri += "((orientation=12 & turn=2)) & s={} & forward & (stepL=0 | stepL=1) -> s' = {}\n\n".format(s,gw.transR[s]['W4'][0])
    file.write(stri)

    stri = "\nforward & turn=1 & orientation>0 -> orientation'+1 = orientation\n"
    stri += "forward & turn=3 & orientation<15 -> orientation' = orientation+1\n"
    stri += "forward & turn=1 & orientation=0 -> orientation' = 15\n"
    stri += "forward & turn=3 & orientation=15 -> orientation' = 0\n"
    stri += "\n"
    file.write(stri)

    #### First Step of Turn ####
    file.write("\n\n\n #First Step of Turn\n\n")
    stri = ""
    for s in gw.states:
        stri += "s={} & orientation=0 & turn=3 & stepL=2 -> s'={}\n".format(s,gw.transR[s]['N3E2'][0])
        stri += "s={} & orientation=4 & turn=3 & stepL=2 -> s'={}\n".format(s,gw.transR[s]['S2E3'][0])
        stri += "s={} & orientation=8 & turn=3 & stepL=2 -> s'={}\n".format(s,gw.transR[s]['S3W2'][0])
        stri += "s={} & orientation=12 & turn=3 & stepL=2 -> s'={}\n".format(s,gw.transR[s]['N2W3'][0])
        stri += "s={} & orientation=0 & turn=1 & stepL=2 -> s'={}\n".format(s,gw.transR[s]['N3W2'][0])
        stri += "s={} & orientation=4 & turn=1 & stepL=2 -> s'={}\n".format(s,gw.transR[s]['N2E3'][0])
        stri += "s={} & orientation=8 & turn=1 & stepL=2 -> s'={}\n".format(s,gw.transR[s]['S3E2'][0])
        stri += "s={} & orientation=12 & turn=1 & stepL=2 -> s'={}\n\n".format(s,gw.transR[s]['S2W3'][0])
    file.write(stri)

    #### Second Step of Turn ####
    file.write("\n\n\n #Second Step of Turn\n\n")
    stri = ""
    for s in gw.states:
        stri += "s={} & orientation=1 & turn=3 & stepL=1 -> s'={}\n".format(s,gw.transR[s]['N2E2'][0])
        stri += "s={} & orientation=5 & turn=3 & stepL=1 -> s'={}\n".format(s,gw.transR[s]['S2E2'][0])
        stri += "s={} & orientation=9 & turn=3 & stepL=1 -> s'={}\n".format(s,gw.transR[s]['S2W2'][0])
        stri += "s={} & orientation=13 & turn=3 & stepL=1 -> s'={}\n".format(s,gw.transR[s]['N2W2'][0])
        stri += "s={} & orientation=15 & turn=1 & stepL=1 -> s'={}\n".format(s,gw.transR[s]['N2W2'][0])
        stri += "s={} & orientation=3 & turn=1 & stepL=1 -> s'={}\n".format(s,gw.transR[s]['N2E2'][0])
        stri += "s={} & orientation=7 & turn=1 & stepL=1 -> s'={}\n".format(s,gw.transR[s]['S2E2'][0])
        stri += "s={} & orientation=11 & turn=1 & stepL=1 -> s'={}\n".format(s,gw.transR[s]['S2W2'][0])
    file.write(stri)

    #### Third Step of Turn ####
    file.write("\n\n\n #Third Step of Turn\n\n")
    stri = ""
    for s in gw.states:
        stri += "s={} & orientation=2 & turn=3 & stepL=2 -> s'={}\n".format(s,gw.transR[s]['N2E3'][0])
        stri += "s={} & orientation=6 & turn=3 & stepL=2 -> s'={}\n".format(s,gw.transR[s]['S3E2'][0])
        stri += "s={} & orientation=10 & turn=3 & stepL=2 -> s'={}\n".format(s,gw.transR[s]['S2W3'][0])
        stri += "s={} & orientation=14 & turn=3 & stepL=2 -> s'={}\n".format(s,gw.transR[s]['N3W2'][0])
        stri += "s={} & orientation=14 & turn=1 & stepL=2 -> s'={}\n".format(s,gw.transR[s]['N2W3'][0])
        stri += "s={} & orientation=2 & turn=1 & stepL=2 -> s'={}\n".format(s,gw.transR[s]['N3E2'][0])
        stri += "s={} & orientation=6 & turn=1 & stepL=2 -> s'={}\n".format(s,gw.transR[s]['S2E3'][0])
        stri += "s={} & orientation=10 & turn=1 & stepL=2 -> s'={}\n\n".format(s,gw.transR[s]['S3W2'][0])
    file.write(stri)

     #### Last Step of Turn ####
    file.write("\n\n\n #Last Step of Turn\n\n")
    stri = ""
    for s in gw.states:
        stri += "((orientation=15 & turn=3) | (orientation=1 & turn=1))  & s={} & forward & (stepL=0 | stepL=1) -> s'={}\n".format(s,gw.transR[s]['N3'][0])
        stri += "((orientation=3 & turn=3) | (orientation=5 & turn=1))  & s={} & forward & (stepL=0 | stepL=1) -> s'={}\n".format(s,gw.transR[s]['E3'][0])
        stri += "((orientation=7 & turn=3) | (orientation=9 & turn=1))  & s={} & forward & (stepL=0 | stepL=1) -> s'={}\n".format(s,gw.transR[s]['S3'][0])
        stri += "((orientation=11 & turn=3) | (orientation=13 & turn=1))  & s={} & forward & (stepL=0 | stepL=1) -> s'={}\n\n".format(s,gw.transR[s]['W3'][0])
    file.write(stri)












    ##### Specs associated with turning in direction of stance foot #####
    for s in gw.states:
        stri += "((orientation=0 & turn=2)) & s={} & forward & (stepL=3) -> s' = {}\n".format(s,gw.transR[s]['N3'][0])
        stri += "((orientation=4 & turn=2)) & s={} & forward & (stepL=3) -> s' = {}\n".format(s,gw.transR[s]['E3'][0])
        stri += "((orientation=8 & turn=2)) & s={} & forward & (stepL=3) -> s' = {}\n".format(s,gw.transR[s]['S3'][0])
        stri += "((orientation=12 & turn=2)) & s={} & forward & (stepL=3) -> s' = {}\n\n".format(s,gw.transR[s]['W3'][0])
    file.write(stri)

    stri = "\nforward & turn=0 & orientation>0 -> orientation'+1 = orientation\n"
    stri += "forward & turn=4 & orientation<15 -> orientation' = orientation+1\n"
    stri += "forward & turn=0 & orientation=0 -> orientation' = 15\n"
    stri += "forward & turn=4 & orientation=15 -> orientation' = 0\n"
    stri += "\n"
    file.write(stri)

    #### First Step of Turn ####
    file.write("\n\n\n #First Step of Turn\n\n")
    stri = ""
    for s in gw.states:
        stri += "s={} & orientation=0 & turn=4 & stepL=1 -> s'={}\n".format(s,gw.transR[s]['N2E1'][0])
        stri += "s={} & orientation=4 & turn=4 & stepL=1 -> s'={}\n".format(s,gw.transR[s]['S1E2'][0])
        stri += "s={} & orientation=8 & turn=4 & stepL=1 -> s'={}\n".format(s,gw.transR[s]['S2W1'][0])
        stri += "s={} & orientation=12 & turn=4 & stepL=1 -> s'={}\n".format(s,gw.transR[s]['N1W2'][0])
        stri += "s={} & orientation=0 & turn=0 & stepL=1 -> s'={}\n".format(s,gw.transR[s]['N2W1'][0])
        stri += "s={} & orientation=4 & turn=0 & stepL=1 -> s'={}\n".format(s,gw.transR[s]['N1E2'][0])
        stri += "s={} & orientation=8 & turn=0 & stepL=1 -> s'={}\n".format(s,gw.transR[s]['S2E1'][0])
        stri += "s={} & orientation=12 & turn=0 & stepL=1 -> s'={}\n\n".format(s,gw.transR[s]['S1W2'][0])
    file.write(stri)

    #### Second Step of Turn ####
    file.write("\n\n\n #Second Step of Turn\n\n")
    stri = ""
    for s in gw.states:
        stri += "s={} & orientation=1 & turn=4 & stepL=2 -> s'={}\n".format(s,gw.transR[s]['N3E3'][0])
        stri += "s={} & orientation=5 & turn=4 & stepL=2 -> s'={}\n".format(s,gw.transR[s]['S3E3'][0])
        stri += "s={} & orientation=9 & turn=4 & stepL=2 -> s'={}\n".format(s,gw.transR[s]['S3W3'][0])
        stri += "s={} & orientation=13 & turn=4 & stepL=2 -> s'={}\n".format(s,gw.transR[s]['N3W3'][0])
        stri += "s={} & orientation=15 & turn=0 & stepL=2 -> s'={}\n".format(s,gw.transR[s]['N3W3'][0])
        stri += "s={} & orientation=3 & turn=0 & stepL=2 -> s'={}\n".format(s,gw.transR[s]['N3E3'][0])
        stri += "s={} & orientation=7 & turn=0 & stepL=2 -> s'={}\n".format(s,gw.transR[s]['S3E3'][0])
        stri += "s={} & orientation=11 & turn=0 & stepL=2 -> s'={}\n".format(s,gw.transR[s]['S3W3'][0])
    file.write(stri)

    #### Third Step of Turn ####
    file.write("\n\n\n #Third Step of Turn\n\n")
    stri = ""
    for s in gw.states:
        stri += "s={} & orientation=2 & turn=4 & stepL=1-> s'={}\n".format(s,gw.transR[s]['N1E2'][0])
        stri += "s={} & orientation=6 & turn=4 & stepL=1 -> s'={}\n".format(s,gw.transR[s]['S2E1'][0])
        stri += "s={} & orientation=10 & turn=4 & stepL=1 -> s'={}\n".format(s,gw.transR[s]['S1W2'][0])
        stri += "s={} & orientation=14 & turn=4 & stepL=1 -> s'={}\n".format(s,gw.transR[s]['N2W1'][0])
        stri += "s={} & orientation=14 & turn=0 & stepL=1 -> s'={}\n".format(s,gw.transR[s]['N1W2'][0])
        stri += "s={} & orientation=2 & turn=0 & stepL=1 -> s'={}\n".format(s,gw.transR[s]['N2E1'][0])
        stri += "s={} & orientation=6 & turn=0 & stepL=1 -> s'={}\n".format(s,gw.transR[s]['S1E2'][0])
        stri += "s={} & orientation=10 & turn=0 & stepL=1 -> s'={}\n\n".format(s,gw.transR[s]['S2W1'][0])
    file.write(stri)

     #### Last Step of Turn ####
    file.write("\n\n\n #Last Step of Turn\n\n")
    stri = ""
    for s in gw.states:
        stri += "((orientation=15 & turn=4) | (orientation=1 & turn=0))  & s={} & forward & (stepL=4) -> s'={}\n".format(s,gw.transR[s]['N5'][0])
        stri += "((orientation=3 & turn=4) | (orientation=5 & turn=0))  & s={} & forward & (stepL=4) -> s'={}\n".format(s,gw.transR[s]['E5'][0])
        stri += "((orientation=7 & turn=4) | (orientation=9 & turn=0))  & s={} & forward & (stepL=4) -> s'={}\n".format(s,gw.transR[s]['S5'][0])
        stri += "((orientation=11 & turn=4) | (orientation=13 & turn=0))  & s={} & forward & (stepL=4) -> s'={}\n\n".format(s,gw.transR[s]['W5'][0])
    file.write(stri)













    # file.write("\n\n\n #specs_for_turning\n\n")
    # top3_edge = gw.top_edge+gw.top_edge2+gw.top_edge3
    # top2_edge = gw.top_edge+gw.top_edge2
    # right3_edge = gw.right_edge+gw.right_edge2+gw.right_edge3
    # right2_edge = gw.right_edge+gw.right_edge2
    # bottom3_edge = gw.bottom_edge+gw.bottom_edge2+gw.bottom_edge3
    # bottom2_edge = gw.bottom_edge+gw.bottom_edge2
    # left3_edge = gw.left_edge+gw.left_edge2+gw.left_edge3
    # left2_edge = gw.left_edge+gw.left_edge2

    # top4_edge = gw.top_edge+gw.top_edge2+gw.top_edge3+gw.top_edge4
    # right4_edge = gw.right_edge+gw.right_edge2+gw.right_edge3+gw.right_edge4
    # bottom4_edge = gw.bottom_edge+gw.bottom_edge2+gw.bottom_edge3+gw.bottom_edge4
    # left4_edge = gw.left_edge+gw.left_edge2+gw.left_edge3+gw.left_edge4

    top5_edge = gw.top_edge+gw.top_edge2+gw.top_edge3+gw.top_edge4+gw.top_edge5
    right5_edge = gw.right_edge+gw.right_edge2+gw.right_edge3+gw.right_edge4+gw.right_edge5
    bottom5_edge = gw.bottom_edge+gw.bottom_edge2+gw.bottom_edge3+gw.bottom_edge4+gw.bottom_edge5
    left5_edge = gw.left_edge+gw.left_edge2+gw.left_edge3+gw.left_edge4+gw.left_edge5


     #### cross specs ####
    file.write("\n\n\n #cross specs\n\n")

    stri = "("
    for s in top5_edge:
        stri+="s={} | ".format(s)
    stri = stri[:-3] 
    stri += ") & ("
    for s in bottom5_edge:
        stri+="s'={} | ".format(s)
    stri = stri[:-3]
    stri += ") -> crossV' = 1\n\n"  
    file.write(stri)

    stri = "("
    for s in bottom5_edge:
        stri+="s={} | ".format(s)
    stri = stri[:-3] 
    stri += ") & ("
    for s in top5_edge:
        stri+="s'={} | ".format(s)
    stri = stri[:-3]
    stri += ") -> crossV' = 2\n\n"  
    file.write(stri)

    stri = "("
    for s in top5_edge:
        stri+="s!={} & ".format(s)
    stri = stri[:-3] 
    stri += ") | ("
    for s in bottom5_edge:
        stri+="s'!={} & ".format(s)
    stri = stri[:-3]
    stri += ") -> crossV' != 1\n\n"  
    file.write(stri)

    stri = "("
    for s in bottom5_edge:
        stri+="s!={} & ".format(s)
    stri = stri[:-3] 
    stri += ") | ("
    for s in top5_edge:
        stri+="s'!={} & ".format(s)
    stri = stri[:-3]
    stri += ") -> crossV' != 2\n\n"  
    file.write(stri)

    



    stri = "("
    for s in right5_edge:
        stri+="s={} | ".format(s)
    stri = stri[:-3] 
    stri += ") & ("
    for s in left5_edge:
        stri+="s'={} | ".format(s)
    stri = stri[:-3]
    stri += ") -> crossH' = 1\n\n"  
    file.write(stri)

    stri = "("
    for s in left5_edge:
        stri+="s={} | ".format(s)
    stri = stri[:-3] 
    stri += ") & ("
    for s in right5_edge:
        stri+="s'={} | ".format(s)
    stri = stri[:-3]
    stri += ") -> crossH' = 2\n\n"  
    file.write(stri)

    stri = "("
    for s in right5_edge:
        stri+="s!={} & ".format(s)
    stri = stri[:-3] 
    stri += ") | ("
    for s in left5_edge:
        stri+="s'!={} & ".format(s)
    stri = stri[:-3]
    stri += ") -> crossH' != 1\n\n"  
    file.write(stri)

    stri = "("
    for s in left5_edge:
        stri+="s!={} & ".format(s)
    stri = stri[:-3] 
    stri += ") | ("
    for s in right5_edge:
        stri+="s'!={} & ".format(s)
    stri = stri[:-3]
    stri += ") -> crossH' != 2\n\n"  
    file.write(stri)










    stri = "!forward -> s' = s\n"
    stri += "turn=2 -> orientation' = orientation"
    stri += "\n"
    file.write(stri)

    
    # stri = "st' != s\n"
    stri += "\n"
    file.write(stri)
    file.write(" (directionrequest' != 4 & directionrequest' != 2) | (directionrequest != 4 & directionrequest != 2)-> !stair'\n")



    # Writing env_safety
    print 'Writing ENV_SAFETY'
    for obs in tqdm(gw.obstacles):
        if obs in allowed_states:
            file.write('!st = {}\n'.format(obs))

    ##### Navigation goal tracking
    file.write("directionrequest = 0 & orientation = 0 & requestPending1 = 5 -> directionrequest' =1 \/ directionrequest' =0\n")
    file.write("directionrequest = 0 & orientation = 4 & requestPending1 = 5 -> directionrequest' =2 \/ directionrequest' =0\n")
    file.write("directionrequest = 0 & orientation = 8 & requestPending1 = 5 -> directionrequest' =3 \/ directionrequest' =0\n")
    file.write("directionrequest = 0 & orientation = 12 & requestPending1 = 5 -> directionrequest' =4 \/ directionrequest' =0\n")

    file.write("directionrequest = 1 -> directionrequest' !=3\n")
    file.write("directionrequest = 2 -> directionrequest' !=4\n")
    file.write("directionrequest = 3 -> directionrequest' !=1\n")
    file.write("directionrequest = 4 -> directionrequest' !=2\n")

    file.write("requestPending1 != 5 -> directionrequest' = directionrequest\n")
    # file.write("crossH' = 0 & crossV'=0 -> directionrequest' = directionrequest\n")

    file.write("requestPending1 != 5 & stair-> stair' \n")
    file.write("requestPending1 != 5 & !stair-> !stair' \n")
    


    # writing sys_trans
    file.write('\n[SYS_TRANS]\n')
    print 'Writing SYS_TRANS'
    
    stri = "!forward' -> turn'=2\n\n"
    stri += "stop -> stepL=0\n"
    stri += "!forward -> (stepL=0 & turn=2)\n\n"
    stri += "stop <-> !forward'\n\n"




    stri += "(turn=3 & (orientation'!=0 & orientation'!=4  & orientation'!=8 & orientation'!=12)) -> turn'=3\n"
    stri += "(turn=1 & (orientation'!=0 & orientation'!=4  & orientation'!=8 & orientation'!=12)) -> turn'=1\n"

    stri += "(turn=4 & (orientation'!=0 & orientation'!=4  & orientation'!=8 & orientation'!=12)) -> turn'=4\n"
    stri += "(turn=0 & (orientation'!=0 & orientation'!=4  & orientation'!=8 & orientation'!=12)) -> turn'=0\n"
    stri += "\n"
    file.write(stri)

    # file.write("turn = 2 & turn' != 2 -> stepL' = 2\n")
    # file.write("stepL = 2 -> stepL' = 1\n")
    # file.write("turn' != 2 -> stepL' != 0\n")
    # file.write("stepL = 0 -> turn' = 2\n")
    # file.write("turn = 2 & stepL = 1 -> turn' != 2\n")

    file.write("turn = 2 & (turn' = 1 \/ turn' = 3) -> stepL' = 2\n")
    file.write("turn = 2 & (turn' = 0 \/ turn' = 4) -> stepL' = 1\n")
    file.write("stepL = 2 -> stepL' = 1\n")
    file.write("turn' != 2 -> stepL' != 0\n")
    file.write("stepL = 0 -> turn' = 2\n")

    file.write("turn = 2 & stepL = 1 -> (turn' = 1 \/ turn' = 3)\n")
    file.write("turn = 2 & stepL = 3 -> (turn' = 0 \/ turn' = 4)\n")
    file.write("(orientation'=15 & turn'=4) | (orientation'=1 & turn'=0)|(orientation'=3 & turn'=4) | (orientation'=5 & turn'=0)| (orientation'=7 & turn'=4) | (orientation'=9 & turn'=0) | (orientation'=11 & turn'=4) | (orientation'=13 & turn'=0) -> stepL' = 4\n")
    
    file.write("turn = 2 & stepL != 3 -> turn'!=0 & turn' !=4\n")
    # file.write("(orientation'=0 & orientation'=4  & orientation'=8 & orientation'=12) -> stepL' != 4\n")


    file.write("(turn' != 0 & turn' != 4 -> stepL' !=4)\n")
    file.write("(orientation'!=0 & orientation'!=4  & orientation'!=8 & orientation'!=12) -> stepL' != 3\n")
    # file.write("stepL = 3 -> (turn' =0 | turn' = 4)\n")
    # file.write("stepL = 4 -> stepL' != 0\n")
    file.write("(orientation'=0 | orientation'=4  | orientation'=8 | orientation'=12) -> stepL' != 4\n")
    file.write("turn != 2 & (orientation'=0 | orientation'=4  | orientation'=8 | orientation'=12) -> turn' = 2\n")

    

    # file.write("crossH != 0 -> crossV = 0\n")
    # file.write("crossV != 0 -> crossH = 0\n")

    # file.write("turn != 1\n")
    # file.write("turn != 3\n")

    stri = ""
    stri += ""

    # footstance based navigation:
    file.write('\n')
    # file.write("stanceFoot !=2\n")
    file.write("forward' & stanceFoot=0 -> stanceFoot'=1\n")
    file.write("forward' & stanceFoot=1 -> stanceFoot'=0\n")
    file.write("!forward' -> stanceFoot' =stanceFoot\n")
    file.write("(orientation = 0 | orientation = 4 | orientation = 8 | orientation = 12) & stanceFoot = 0 -> turn !=1\n")
    file.write("(orientation = 0 | orientation = 4 | orientation = 8 | orientation = 12) & stanceFoot = 1 -> turn !=3\n")

    file.write("(orientation = 0 | orientation = 4 | orientation = 8 | orientation = 12) & stanceFoot = 0 -> turn !=4\n")
    file.write("(orientation = 0 | orientation = 4 | orientation = 8 | orientation = 12) & stanceFoot = 1 -> turn !=0\n")




    ##### Specs that govern how liveness specs can be met #####
    file.write("\n\n #goal tracking specs\n\n")

    file.write("requestPending1 = 1 -> (crossV' = 1 & requestPending1' = 5)|(crossV' != 1 & requestPending1' = 1)\n")
    file.write("requestPending1 = 2 -> (crossH' = 1 & requestPending1' = 5)|(crossH' != 1 & requestPending1' = 2)\n")
    file.write("requestPending1 = 3 -> (crossV' = 2 & requestPending1' = 5)|(crossV' != 2 & requestPending1' = 3)\n")
    file.write("requestPending1 = 4 -> (crossH' = 2 & requestPending1' = 5)|(crossH' != 2 & requestPending1' = 4)\n")
    file.write("requestPending1 = 0 -> ((!forward'  & requestPending1' = 5) \/ (forward' & requestPending1' = 0))\n\n")
    # file.write("requestPending1 = 0 -> ((!forward' & requestPending1' = 5) \/ (forward' & turn'!=2 & requestPending1' = 0))\n\n")

    file.write("requestPending1 = 1 & (crossV =2 | crossH !=0) -> fail' = 1\n")
    file.write("requestPending1 = 2 & (crossV !=0 | crossH =2) -> fail' = 1\n")
    file.write("requestPending1 = 3 & (crossV =1 | crossH !=0) -> fail' = 1\n")
    file.write("requestPending1 = 4 & (crossV !=0 | crossH =1) -> fail' = 1\n")
    file.write("requestPending1 = 0 & (crossV !=0 | crossH !=0) -> fail' = 1\n")
    file.write("(crossV !=0 & crossH !=0) -> fail' = 1\n")

    file.write("fail = 1 -> fail' = 1\n")
    # file.write("stanceFoot' =2 -> !forward'\n")
    file.write("fail' = 1 -> stop'\n")



    file.write("stop & (requestPending1' = 0 | requestPending1' = 5) -> stop'\n")

    file.write("requestPending1 = 5 -> requestPending1' = directionrequest'\n")

    
   
    file.write("\nstair'  & directionrequest' = 2 -> (stepL' = 0 & stepH' = 4) \/ ((stepL' = 1 & stepH' = 4)) \/ ((stepL' = 2 & stepH' = 6))\n")
    file.write("stair' & directionrequest' = 4 -> (stepL' = 0 & stepH' = 2) \/ ((stepL' = 1 & stepH' = 1)) \/ ((stepL' = 2 & stepH' = 0))\n\n")
    
    file.write("!stair' -> stepH' = 3\n\n")




##################################################################################

    # Writing sys_liveness
    file.write('\n[SYS_LIVENESS]\n')
    file.write("requestPending1 = 5\n")






    file.write('\n[ENV_LIVENESS]\n')
   