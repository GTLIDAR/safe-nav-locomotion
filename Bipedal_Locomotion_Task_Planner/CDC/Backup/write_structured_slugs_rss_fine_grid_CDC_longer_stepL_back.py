
import numpy as np
import copy
import itertools
from tqdm import *
import simplejson as json
import math
from gridworld import *
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
            ########## Jonas ##########
            t.update(set(np.nonzero(gw.probMO[action][state])[0]))
    return t

########## Jonas ##########
def reach_statesR(gw,states):
    t =set()
    for state in states:
        for action in gw.actlistR:
            t.update(set(np.nonzero(gw.probR[action][state])[0]))
    return t
########## Jonas ##########

def powerset(s):
    x = len(s)
    a = []
    then_PS = time.time()
    for i in range(1,1<<x):
        a.append({s[j] for j in range(x) if (i &(1<<j))})
    now_PS = time.time()
    print 'Power set took ', now_PS - then_PS, ' seconds'
    return a
    #then_PS = time.time()
    #pow_set_lists = reduce(lambda result, x: result + [subset + [x] for subset in result],s, [[]])
    #pow_set_lists.remove([])
    #i = 0
    #for specific_list in pow_set_lists:
    #    pow_set_lists[i] = set(specific_list)
    #    i +=1
    #now_PS = time.time()
   # print 'Power set took ', now_PS - then_PS, ' seconds'
    #return pow_set_lists

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

    # target_total_vis = target_visibility(gw)

    filename = infile+'.structuredslugs'
    file = open(filename,'w')
    file.write('[INPUT]\n')
    # print('nonbeliefstates: ' + str(nonbeliefstates))
    # print('allstates: ' + str(allstates))
    # print('len(allstates): ' + str(len(allstates)))
    file.write('st:0...{}\n'.format(len(allstates) -1))
    file.write('orientation:0...15\n')
    file.write('s:0...{}\n'.format(len(gw.states)-1))
    file.write('directionrequest:0...4\n')
    file.write('stair\n')
    # file.write('border_trans\n')
    # file.write('sOld:0...{}\n'.format(len(gw.states)-1))
    # file.write('pastTurnStanceMatchFoot:0...2\n')

    file.write('\n[OUTPUT]\n')
    file.write('forward\n')
    # file.write('turnLeft\n')
    # file.write('turnRight\n')
    file.write('stepL:0...3\n')
    file.write('stop\n')
    file.write('requestPending1:0...5\n')
    # file.write('requestPending2\n')
    file.write('stepH:0...6\n')
    file.write('turn:1...3\n')
    file.write('stanceFoot:0...2\n')
    # file.write('s:0...{}\n'.format(len(gw.states)-1))
    if target_reachability:
        file.write('c:0...1\n')

    file.write('\n[ENV_INIT]\n')
    file.write('s = {}\n'.format(init))
    file.write('orientation = 4\n')
    file.write('directionrequest = 2\n')
    file.write('!stair\n')
    # file.write('pastTurnStanceMatchFoot = 2\n')

    if initmovetarget in allowed_states:
        file.write('st = {}\n'.format(initmovetarget))
    else:
        file.write('st = {}\n'.format(allstates[-1]))

    # file.write('sOld = {}\n'.format(init))
    

    file.write('\n[SYS_INIT]\n')
    # file.write('s = {}\n'.format(init))
    if target_reachability:
        file.write('c = 0\n')

    file.write('!forward\n')
    # file.write('!turnLeft\n')
    # file.write('!turnRight\n')
    file.write('turn = 2\n')
    file.write('stop\n')
    file.write('stepH = 3\n')
    # file.write('!stop\n')
    file.write('requestPending1=2\n')

    # writing env_trans
    file.write('\n[ENV_TRANS]\n')
    print 'Writing ENV_TRANS'
    file.write("st' = {}\n".format(initmovetarget))

    ##################### Jonas Action Based Specs ###################
    print 'Writing Action Based Environment Transitions'
    stri = ''
    stri += '\n'
    stri += '\n'
    file.write(stri)


   

    #### Walking North ####
    stri = "((orientation=0 & turn=2))  & s>{} & forward & (stepL=0 | stepL=1) -> s' + {} = s\n".format(4*gw.ncols-1, 4*gw.ncols)

    stri += "((orientation=0 & turn=2)) & "
    stri += "s<{} & ".format(4*gw.ncols)
    stri += "forward & (stepL=0 | stepL=1) -> s' = s + {}\n".format((gw.nrows-4)*(gw.ncols))

    #### Walking East ####
    stri += "\n((orientation=4 & turn=2)) & "
    for row in range(gw.nrows):
        stri += "s != {} & s != {} & s != {} & s != {} & ".format((row+1)*gw.ncols-3, (row+1)*gw.ncols-2, (row+1)*gw.ncols-1, (row+1)*gw.ncols-4)
    stri += "forward & (stepL=0 | stepL=1) -> s' = s+4\n"


    stri += "((orientation=4 & turn=2)) & ("
    for row in range(gw.nrows):
        stri += "s = {} \\/ s = {} \\/ s = {} \\/ s = {} \\/ ".format((row+1)*gw.ncols-3, (row+1)*gw.ncols-2, (row+1)*gw.ncols-1,(row+1)*gw.ncols-4)
    stri = stri[:-4]
    stri += ") & forward & (stepL=0 | stepL=1) -> s' + {} = s\n".format(gw.ncols-4)


    #### Walking South ####
    stri += "\n((orientation=8 & turn=2)) & s<{} & forward & (stepL=0 | stepL=1) -> s' = s + {}\n".format((gw.nrows-4)*gw.ncols, 4*gw.ncols)

    stri += "((orientation=8 & turn=2)) & "
    stri += "s>{} &".format((gw.nrows-4)*gw.ncols-1)
    stri += "forward & (stepL=0 | stepL=1) -> s' + {} = s\n".format(gw.ncols*(gw.nrows-4))

    #### Walking West ####
    stri += "\n((orientation=12 & turn=2)) & "
    for row in range(gw.nrows):
        stri += "s != {} & s != {} & s != {} & s != {} & ".format(row*gw.ncols+2 , row*gw.ncols+1 , row*gw.ncols,row*gw.ncols+3)
    stri += "forward & (stepL=0 | stepL=1) -> s' + 4 = s\n"

    stri += "((orientation=12 & turn=2)) & ("
    for row in range(gw.nrows):
        stri += "s = {} \\/ s = {} \\/ s = {} \\/ s = {} \\/ ".format(row*gw.ncols+2 , row*gw.ncols+1 , row*gw.ncols,row*gw.ncols+3)
    stri = stri[:-4]
    stri += ") & forward & (stepL=0 | stepL=1) -> s' = s + {}\n".format(gw.ncols-4)
    file.write(stri)



    ####last_step_turning
    #### Walking North ####
    stri = "((orientation=15 & turn=3) | (orientation=1 & turn=1))  & s>{} & forward & (stepL=0 | stepL=1) -> s' + {} = s\n".format(3*gw.ncols-1, 3*gw.ncols)

    stri += "((orientation=15 & turn=3) | (orientation=1 & turn=1)) & "
    stri += "s<{} & ".format(3*gw.ncols)
    stri += "forward & (stepL=0 | stepL=1) -> s' = s + {}\n".format((gw.nrows-3)*(gw.ncols))

    #### Walking East ####
    stri += "\n((orientation=3 & turn=3) | (orientation=5 & turn=1)) & "
    for row in range(gw.nrows):
        stri += "s != {} & s != {} & s != {} & ".format((row+1)*gw.ncols-3, (row+1)*gw.ncols-2, (row+1)*gw.ncols-1)
    stri += "forward & (stepL=0 | stepL=1) -> s' = s+3\n"


    stri += "((orientation=3 & turn=3) | (orientation=5 & turn=1)) & ("
    for row in range(gw.nrows):
        stri += "s = {} \\/ s = {} \\/ s = {} \\/ ".format((row+1)*gw.ncols-3, (row+1)*gw.ncols-2, (row+1)*gw.ncols-1)
    stri = stri[:-4]
    stri += ") & forward & (stepL=0 | stepL=1) -> s' + {} = s\n".format(gw.ncols-3)


    #### Walking South ####
    stri += "\n((orientation=7 & turn=3) | (orientation=9 & turn=1)) & s<{} & forward & (stepL=0 | stepL=1) -> s' = s + {}\n".format((gw.nrows-3)*gw.ncols, 3*gw.ncols)

    stri += "((orientation=7 & turn=3) | (orientation=9 & turn=1)) & "
    stri += "s>{} &".format((gw.nrows-3)*gw.ncols-1)
    stri += "forward & (stepL=0 | stepL=1) -> s' + {} = s\n".format(gw.ncols*(gw.nrows-3))

    #### Walking West ####
    stri += "\n((orientation=11 & turn=3) | (orientation=13 & turn=1)) & "
    for row in range(gw.nrows):
        stri += "s != {} & s != {} & s != {} & ".format(row*gw.ncols+2 , row*gw.ncols+1 , row*gw.ncols)
    stri += "forward & (stepL=0 | stepL=1) -> s' + 3 = s\n"

    stri += "((orientation=11 & turn=3) | (orientation=13 & turn=1)) & ("
    for row in range(gw.nrows):
        stri += "s = {} \\/ s = {} \\/ s = {} \\/ ".format(row*gw.ncols+2 , row*gw.ncols+1 , row*gw.ncols)
    stri = stri[:-4]
    stri += ") & forward & (stepL=0 | stepL=1) -> s' = s + {}\n".format(gw.ncols-3)
    file.write(stri)






    stri = "\nforward & turn=1 & orientation>0 -> orientation'+1 = orientation\n"
    stri += "forward & turn=3 & orientation<15 -> orientation' = orientation+1\n"
    stri += "forward & turn=1 & orientation=0 -> orientation' = 15\n"
    stri += "forward & turn=3 & orientation=15 -> orientation' = 0\n"
    stri += "\n"
    file.write(stri)
    ###45deg change###



    file.write("\n\n\n #specs_for_turning\n\n")
    top3_edge = gw.top_edge+gw.top_edge2+gw.top_edge3
    top2_edge = gw.top_edge+gw.top_edge2
    right3_edge = gw.right_edge+gw.right_edge2+gw.right_edge3
    right2_edge = gw.right_edge+gw.right_edge2
    bottom3_edge = gw.bottom_edge+gw.bottom_edge2+gw.bottom_edge3
    bottom2_edge = gw.bottom_edge+gw.bottom_edge2
    left3_edge = gw.left_edge+gw.left_edge2+gw.left_edge3
    left2_edge = gw.left_edge+gw.left_edge2

    top4_edge = gw.top_edge+gw.top_edge2+gw.top_edge3+gw.top_edge4
    right4_edge = gw.right_edge+gw.right_edge2+gw.right_edge3+gw.right_edge4
    bottom4_edge = gw.bottom_edge+gw.bottom_edge2+gw.bottom_edge3+gw.bottom_edge4
    left4_edge = gw.left_edge+gw.left_edge2+gw.left_edge3+gw.left_edge4

    ##### Grid transitions when turning right with orientation 0, 1, or 3
    ##### first step of turn #####
    edge_comb = top3_edge+right2_edge
    stri = "\n("
    for edgeS in edge_comb:
        stri += "s != {} & ".format(edgeS)
    stri += "orientation=0 & turn=3 & stepL=2) -> s' + {} = s\n".format(3*gw.ncols-2)
    file.write(stri)

    top_edge_minus_right = list(set(top3_edge) - set(right2_edge))
    stri = "("
    for edgeS in top_edge_minus_right:
        stri += "s = {} | ".format(edgeS)
    stri = stri[:-3]
    stri += ") & orientation=0 & turn=3 & stepL=2 -> s' = s + {}\n".format(gw.ncols*(gw.nrows-3)+2)
    file.write(stri)

    right_edge_minus_top = list(set(right2_edge) - set(top3_edge))
    stri = "("
    for edgeS in right_edge_minus_top:
        stri += "s = {} | ".format(edgeS)
    stri = stri[:-3]
    stri += ") & orientation=0 & turn=3 & stepL=2 -> s' + {} = s\n".format(4*gw.ncols-2)
    file.write(stri)


    ##### second step of turn #####
    edge_comb = top2_edge+right2_edge
    stri = "\n("
    for edgeS in edge_comb:
        stri += "s != {} & ".format(edgeS)
    stri += "orientation=1 & turn=3 & stepL=1) -> s' + {} = s\n".format(2*gw.ncols-2)
    file.write(stri)

    top_edge_minus_right = list(set(top2_edge) - set(right2_edge))
    stri = "("
    for edgeS in top_edge_minus_right:
        stri += "s = {} | ".format(edgeS)
    stri = stri[:-3]
    stri += ") & orientation=1 & turn=3 & stepL=1 -> s' = s + {}\n".format(gw.ncols*(gw.nrows-2)+2)
    file.write(stri)

    right_edge_minus_top = list(set(right2_edge) - set(top2_edge))
    stri = "("
    for edgeS in right_edge_minus_top:
        stri += "s = {} | ".format(edgeS)
    stri = stri[:-3]
    stri += ") & orientation=1 & turn=3 & stepL=1 -> s' + {} = s\n".format(3*gw.ncols-2)
    file.write(stri)


    ##### third step of turn #####
    edge_comb = top2_edge+right3_edge
    stri = "\n("
    for edgeS in edge_comb:
        stri += "s != {} & ".format(edgeS)
    stri += "orientation=2 & turn=3 & stepL=2) -> s' + {} = s\n".format(2*gw.ncols-3)
    file.write(stri)

    top_edge_minus_right = list(set(top2_edge) - set(right3_edge))
    stri = "("
    for edgeS in top_edge_minus_right:
        stri += "s = {} | ".format(edgeS)
    stri = stri[:-3]
    stri += ") & orientation=2 & turn=3 & stepL=2 -> s' = s + {}\n".format(gw.ncols*(gw.nrows-2)+3)
    file.write(stri)

    right_edge_minus_top = list(set(right3_edge) - set(top2_edge))
    stri = "("
    for edgeS in right_edge_minus_top:
        stri += "s = {} | ".format(edgeS)
    stri = stri[:-3]
    stri += ") & orientation=2 & turn=3 & stepL=2 -> s' + {} = s\n".format(3*gw.ncols-2)
    file.write(stri)

    ##### Grid transitions when turning right with orientation 4, 5, or 6
    ##### first step of turn #####
    edge_comb = right3_edge + bottom2_edge
    stri = "\n("
    for edgeS in edge_comb:
        stri += "s != {} & ".format(edgeS)
    stri += "orientation=4 & turn=3 & stepL=2) -> s' = s + {}\n".format(2*gw.ncols+3)
    file.write(stri)

    right_edge_minus_bottom = list(set(right3_edge) - set(bottom2_edge))
    stri = "("
    for edgeS in right_edge_minus_bottom:
        stri += "s = {} | ".format(edgeS)
    stri = stri[:-3]
    stri += ") & orientation=4 & turn=3 & stepL=2 -> s' = s + {}\n".format(gw.ncols+3)
    file.write(stri)

    bottom_edge_minus_right = list(set(bottom2_edge) - set(right3_edge))
    stri = "("
    for edgeS in bottom_edge_minus_right:
        stri += "s = {} | ".format(edgeS)
    stri = stri[:-3]
    stri += ") & orientation=4 & turn=3 & stepL=2 -> s' + {} = s\n".format(gw.ncols*(gw.nrows-2)-3)
    file.write(stri)


    ##### second step of turn #####
    edge_comb = right2_edge + bottom2_edge
    stri = "\n("
    for edgeS in edge_comb:
        stri += "s != {} & ".format(edgeS)
    stri += "orientation=5 & turn=3 & stepL=1) -> s' = s + {}\n".format(2*gw.ncols+2)
    file.write(stri)

    right_edge_minus_bottom = list(set(right2_edge) - set(bottom2_edge))
    stri = "("
    for edgeS in right_edge_minus_bottom:
        stri += "s = {} | ".format(edgeS)
    stri = stri[:-3]
    stri += ") & orientation=5 & turn=3 & stepL=1 -> s' = s + {}\n".format(gw.ncols+2)
    file.write(stri)

    bottom_edge_minus_right = list(set(bottom2_edge) - set(right2_edge))
    stri = "("
    for edgeS in bottom_edge_minus_right:
        stri += "s = {} | ".format(edgeS)
    stri = stri[:-3]
    stri += ") & orientation=5 & turn=3 & stepL=1 -> s' + {} = s\n".format(gw.ncols*(gw.nrows-2)-2)
    file.write(stri)


    ##### third step of turn #####
    edge_comb = right2_edge + bottom3_edge
    stri = "\n("
    for edgeS in edge_comb:
        stri += "s != {} & ".format(edgeS)
    stri += "orientation=6 & turn=3 & stepL=2) -> s' = s + {}\n".format(3*gw.ncols+2)
    file.write(stri)

    right_edge_minus_bottom = list(set(right2_edge) - set(bottom3_edge))
    stri = "("
    for edgeS in right_edge_minus_bottom:
        stri += "s = {} | ".format(edgeS)
    stri = stri[:-3]
    stri += ") & orientation=6 & turn=3 & stepL=2 -> s' = s + {}\n".format(2*gw.ncols+2)
    file.write(stri)

    bottom_edge_minus_right = list(set(bottom3_edge) - set(right2_edge))
    stri = "("
    for edgeS in bottom_edge_minus_right:
        stri += "s = {} | ".format(edgeS)
    stri = stri[:-3]
    stri += ") & orientation=6 & turn=3 & stepL=2 -> s' + {} = s\n".format(gw.ncols*(gw.nrows-3)-2)
    file.write(stri)





    ##### Grid transitions when turning right with orientation 8, 9, or 10
    ##### first step of turn #####
    edge_comb = bottom3_edge+left2_edge
    stri = "\n("
    for edgeS in edge_comb:
        stri += "s != {} & ".format(edgeS)
    stri += "orientation=8 & turn=3 & stepL=2) -> s' = s + {}\n".format(3*gw.ncols-2)
    file.write(stri)

    bottom_edge_minus_left = list(set(bottom3_edge) - set(left2_edge))
    stri = "("
    for edgeS in bottom_edge_minus_left:
        stri += "s = {} | ".format(edgeS)
    stri = stri[:-3]
    stri += ") & orientation=8 & turn=3 & stepL=2 -> s' + {} = s\n".format(gw.ncols*(gw.nrows-3)+2)
    file.write(stri)

    left_edge_minus_bottom = list(set(left2_edge) - set(bottom3_edge))
    stri = "("
    for edgeS in left_edge_minus_bottom:
        stri += "s = {} | ".format(edgeS)
    stri = stri[:-3]
    stri += ") & orientation=8 & turn=3 & stepL=2 -> s' = s + {}\n".format(4*gw.ncols-2)
    file.write(stri)


    ##### second step of turn #####
    edge_comb = bottom2_edge+left2_edge
    stri = "\n("
    for edgeS in edge_comb:
        stri += "s != {} & ".format(edgeS)
    stri += "orientation=9 & turn=3 & stepL=1) -> s' = s + {}\n".format(2*gw.ncols-2)
    file.write(stri)

    bottom_edge_minus_left = list(set(bottom2_edge) - set(left2_edge))
    stri = "("
    for edgeS in bottom_edge_minus_left:
        stri += "s = {} | ".format(edgeS)
    stri = stri[:-3]
    stri += ") & orientation=9 & turn=3 & stepL=1 -> s' + {} = s\n".format(gw.ncols*(gw.nrows-2)+2)
    file.write(stri)

    left_edge_minus_bottom = list(set(left2_edge) - set(bottom2_edge))
    stri = "("
    for edgeS in left_edge_minus_bottom:
        stri += "s = {} | ".format(edgeS)
    stri = stri[:-3]
    stri += ") & orientation=9 & turn=3 & stepL=1 -> s' = s + {}\n".format(3*gw.ncols-2)
    file.write(stri)

    ##### third step of turn #####
    edge_comb = bottom2_edge+left3_edge
    stri = "\n("
    for edgeS in edge_comb:
        stri += "s != {} & ".format(edgeS)
    stri += "orientation=10 & turn=3 & stepL=2) -> s' = s + {}\n".format(2*gw.ncols-3)
    file.write(stri)

    bottom_edge_minus_left = list(set(bottom2_edge) - set(left3_edge))
    stri = "("
    for edgeS in bottom_edge_minus_left:
        stri += "s = {} | ".format(edgeS)
    stri = stri[:-3]
    stri += ") & orientation=10 & turn=3 & stepL=2 -> s' + {} = s\n".format(gw.ncols*(gw.nrows-2)+3)
    file.write(stri)

    left_edge_minus_bottom = list(set(left3_edge) - set(bottom2_edge))
    stri = "("
    for edgeS in left_edge_minus_bottom:
        stri += "s = {} | ".format(edgeS)
    stri = stri[:-3]
    stri += ") & orientation=10 & turn=3 & stepL=2 -> s' = s + {}\n".format(3*gw.ncols-3)
    file.write(stri)


    
    ##### Grid transitions when turning right with orientation 12, 13, or 14
    ##### first step of turn #####
    left_top_edge = left3_edge + top2_edge
    stri = "\n("
    for edgeS in left_top_edge:
        stri += "s != {} & ".format(edgeS)
    stri += "orientation=12 & turn=3 & stepL=2) -> s' + {} = s\n".format(2*gw.ncols+3)
    file.write(stri)

    left_edge_minus_top = list(set(left3_edge) - set(top2_edge))
    stri = "("
    for edgeS in left_edge_minus_top:
        stri += "s = {} | ".format(edgeS)
    stri = stri[:-3]
    stri += ") & orientation=12 & turn=3 & stepL=2 -> s' + {} = s\n".format(gw.ncols+3)
    file.write(stri)

    top_edge_minus_left = list(set(top2_edge) - set(left3_edge))
    stri = "("
    for edgeS in top_edge_minus_left:
        stri += "s = {} | ".format(edgeS)
    stri = stri[:-3]
    stri += ") & orientation=12 & turn=3 & stepL=2 -> s' = s + {}\n".format((gw.nrows-2)*gw.ncols-3)
    file.write(stri)


    ##### second step of turn #####
    left_top_edge = left2_edge + top2_edge
    stri = "\n("
    for edgeS in left_top_edge:
        stri += "s != {} & ".format(edgeS)
    stri += "orientation=13 & turn=3 & stepL=1) -> s' + {} = s\n".format(2*gw.ncols+2)
    file.write(stri)

    left_edge_minus_top = list(set(left2_edge) - set(top2_edge))
    stri = "("
    for edgeS in left_edge_minus_top:
        stri += "s = {} | ".format(edgeS)
    stri = stri[:-3]
    stri += ") & orientation=13 & turn=3 & stepL=1 -> s' + {} = s\n".format(gw.ncols+2)
    file.write(stri)

    top_edge_minus_left = list(set(top2_edge) - set(left2_edge))
    stri = "("
    for edgeS in top_edge_minus_left:
        stri += "s = {} | ".format(edgeS)
    stri = stri[:-3]
    stri += ") & orientation=13 & turn=3 & stepL=1 -> s' = s + {}\n".format((gw.nrows-2)*gw.ncols-2)
    file.write(stri)

    ##### third step of turn #####
    left_top_edge = left2_edge + top3_edge
    stri = "\n("
    for edgeS in left_top_edge:
        stri += "s != {} & ".format(edgeS)
    stri += "orientation=14 & turn=3 & stepL=2) -> s' + {} = s\n".format(3*gw.ncols+2)
    file.write(stri)

    left_edge_minus_top = list(set(left2_edge) - set(top3_edge))
    stri = "("
    for edgeS in left_edge_minus_top:
        stri += "s = {} | ".format(edgeS)
    stri = stri[:-3]
    stri += ") & orientation=14 & turn=3 & stepL=2 -> s' + {} = s\n".format(2*gw.ncols+2)
    file.write(stri)

    top_edge_minus_left = list(set(top3_edge) - set(left2_edge))
    stri = "("
    for edgeS in top_edge_minus_left:
        stri += "s = {} | ".format(edgeS)
    stri = stri[:-3]
    stri += ") & orientation=14 & turn=3 & stepL=2 -> s' = s + {}\n".format((gw.nrows-3)*gw.ncols-2)
    file.write(stri)
    






    ##### Grid transitions when turning left with orientation 0, 15, or 14
    # stri += "((orientation=0 | orientation=11) & turn=1 & stepL=3) -> s' + {} = s\n".format(gw.ncols+1)
    ##### first step of turn
    edge_comb = top3_edge + left2_edge
    stri = "\n("
    for edgeS in left_top_edge:
        stri += "s != {} & ".format(edgeS)
    stri += "orientation=0 & turn=1 & stepL=2) -> s' + {} = s\n".format(3*gw.ncols+2)
    file.write(stri)

    left_edge_minus_top = list(set(left2_edge) - set(top3_edge))
    stri = "("
    for edgeS in left_edge_minus_top:
        stri += "s = {} | ".format(edgeS)
    stri = stri[:-3]
    stri += ") & orientation=0 & turn=1 & stepL=2 -> s' + {} = s\n".format(2*gw.ncols+2)
    file.write(stri)

    top_edge_minus_left = list(set(top3_edge) - set(left2_edge))
    stri = "("
    for edgeS in top_edge_minus_left:
        stri += "s = {} | ".format(edgeS)
    stri = stri[:-3]
    stri += ") & orientation=0 & turn=1 & stepL=2 -> s' = s + {}\n".format((gw.nrows-3)*gw.ncols-2)
    file.write(stri)

    ##### second step of turn
    left_top_edge = left2_edge + top2_edge
    stri = "\n("
    for edgeS in left_top_edge:
        stri += "s != {} & ".format(edgeS)
    stri += "orientation=15 & turn=1 & stepL=1) -> s' + {} = s\n".format(2*gw.ncols+2)
    file.write(stri)

    left_edge_minus_top = list(set(left2_edge) - set(top2_edge))
    stri = "("
    for edgeS in left_edge_minus_top:
        stri += "s = {} | ".format(edgeS)
    stri = stri[:-3]
    stri += ") & orientation=15 & turn=1 & stepL=1 -> s' + {} = s\n".format(gw.ncols+2)
    file.write(stri)

    top_edge_minus_left = list(set(top2_edge) - set(left2_edge))
    stri = "("
    for edgeS in top_edge_minus_left:
        stri += "s = {} | ".format(edgeS)
    stri = stri[:-3]
    stri += ") & orientation=15 & turn=1 & stepL=1 -> s' = s + {}\n".format((gw.nrows-2)*gw.ncols-2)
    file.write(stri)

    ##### third step of turn
    left_top_edge = left3_edge + top2_edge
    stri = "\n("
    for edgeS in left_top_edge:
        stri += "s != {} & ".format(edgeS)
    stri += "orientation=14 & turn=1 & stepL=2) -> s' + {} = s\n".format(2*gw.ncols+3)
    file.write(stri)

    left_edge_minus_top = list(set(left3_edge) - set(top2_edge))
    stri = "("
    for edgeS in left_edge_minus_top:
        stri += "s = {} | ".format(edgeS)
    stri = stri[:-3]
    stri += ") & orientation=14 & turn=1 & stepL=2 -> s' + {} = s\n".format(gw.ncols+3)
    file.write(stri)

    top_edge_minus_left = list(set(top2_edge) - set(left3_edge))
    stri = "("
    for edgeS in top_edge_minus_left:
        stri += "s = {} | ".format(edgeS)
    stri = stri[:-3]
    stri += ") & orientation=14 & turn=1 & stepL=2 -> s' = s + {}\n".format((gw.nrows-2)*gw.ncols-3)
    file.write(stri)


    ##### Grid transitions when turning left with orientation 4, 3, or 2
    # stri += "((orientation=3 | orientation=2) & turn=1 & stepL=3) -> s' + {} = s\n".format(gw.ncols-1)
    

    ##### first step of turn
    edge_comb = top2_edge+right3_edge
    stri = "\n("
    for edgeS in edge_comb:
        stri += "s != {} & ".format(edgeS)
    stri += "orientation=4 & turn=1 & stepL=2) -> s' + {} = s\n".format(2*gw.ncols-3)
    file.write(stri)

    top_edge_minus_right = list(set(top2_edge) - set(right3_edge))
    stri = "("
    for edgeS in top_edge_minus_right:
        stri += "s = {} | ".format(edgeS)
    stri = stri[:-3]
    stri += ") & orientation=4 & turn=1 & stepL=2 -> s' = s + {}\n".format(gw.ncols*(gw.nrows-2)+3)
    file.write(stri)

    right_edge_minus_top = list(set(right3_edge) - set(top2_edge))
    stri = "("
    for edgeS in right_edge_minus_top:
        stri += "s = {} | ".format(edgeS)
    stri = stri[:-3]
    stri += ") & orientation=4 & turn=1 & stepL=2 -> s' + {} = s\n".format(3*gw.ncols-2)
    file.write(stri)

    ##### second step of turn #####
    edge_comb = top2_edge+right2_edge
    stri = "\n("
    for edgeS in edge_comb:
        stri += "s != {} & ".format(edgeS)
    stri += "orientation=3 & turn=1 & stepL=1) -> s' + {} = s\n".format(2*gw.ncols-2)
    file.write(stri)

    top_edge_minus_right = list(set(top2_edge) - set(right2_edge))
    stri = "("
    for edgeS in top_edge_minus_right:
        stri += "s = {} | ".format(edgeS)
    stri = stri[:-3]
    stri += ") & orientation=3 & turn=1 & stepL=1 -> s' = s + {}\n".format(gw.ncols*(gw.nrows-2)+2)
    file.write(stri)

    right_edge_minus_top = list(set(right2_edge) - set(top2_edge))
    stri = "("
    for edgeS in right_edge_minus_top:
        stri += "s = {} | ".format(edgeS)
    stri = stri[:-3]
    stri += ") & orientation=3 & turn=1 & stepL=1 -> s' + {} = s\n".format(3*gw.ncols-2)
    file.write(stri)

    ##### third step of turn #####
    edge_comb = top3_edge+right2_edge
    stri = "\n("
    for edgeS in edge_comb:
        stri += "s != {} & ".format(edgeS)
    stri += "orientation=2 & turn=1 & stepL=2) -> s' + {} = s\n".format(3*gw.ncols-2)
    file.write(stri)

    top_edge_minus_right = list(set(top3_edge) - set(right2_edge))
    stri = "("
    for edgeS in top_edge_minus_right:
        stri += "s = {} | ".format(edgeS)
    stri = stri[:-3]
    stri += ") & orientation=2 & turn=1 & stepL=2 -> s' = s + {}\n".format(gw.ncols*(gw.nrows-3)+2)
    file.write(stri)

    right_edge_minus_top = list(set(right2_edge) - set(top3_edge))
    stri = "("
    for edgeS in right_edge_minus_top:
        stri += "s = {} | ".format(edgeS)
    stri = stri[:-3]
    stri += ") & orientation=2 & turn=1 & stepL=2 -> s' + {} = s\n".format(4*gw.ncols-2)
    file.write(stri)




    ##### Grid transitions when turning left with orientation 8, 7, or 6
    # stri += "((orientation=6 | orientation=5) & turn=1 & stepL=3) -> s' = s + {}\n".format(gw.ncols+1)
    ##### third step of turn #####
    edge_comb = right2_edge + bottom3_edge
    stri = "\n("
    for edgeS in edge_comb:
        stri += "s != {} & ".format(edgeS)
    stri += "orientation=8 & turn=1 & stepL=2) -> s' = s + {}\n".format(3*gw.ncols+2)
    file.write(stri)

    right_edge_minus_bottom = list(set(right2_edge) - set(bottom3_edge))
    stri = "("
    for edgeS in right_edge_minus_bottom:
        stri += "s = {} | ".format(edgeS)
    stri = stri[:-3]
    stri += ") & orientation=8 & turn=1 & stepL=2 -> s' = s + {}\n".format(2*gw.ncols+2)
    file.write(stri)

    bottom_edge_minus_right = list(set(bottom3_edge) - set(right2_edge))
    stri = "("
    for edgeS in bottom_edge_minus_right:
        stri += "s = {} | ".format(edgeS)
    stri = stri[:-3]
    stri += ") & orientation=8 & turn=1 & stepL=2 -> s' + {} = s\n".format(gw.ncols*(gw.nrows-3)-2)
    file.write(stri)

    ##### second step of turn #####
    edge_comb = right2_edge + bottom2_edge
    stri = "\n("
    for edgeS in edge_comb:
        stri += "s != {} & ".format(edgeS)
    stri += "orientation=7 & turn=1 & stepL=1) -> s' = s + {}\n".format(2*gw.ncols+2)
    file.write(stri)

    right_edge_minus_bottom = list(set(right2_edge) - set(bottom2_edge))
    stri = "("
    for edgeS in right_edge_minus_bottom:
        stri += "s = {} | ".format(edgeS)
    stri = stri[:-3]
    stri += ") & orientation=7 & turn=1 & stepL=1 -> s' = s + {}\n".format(gw.ncols+2)
    file.write(stri)

    bottom_edge_minus_right = list(set(bottom2_edge) - set(right2_edge))
    stri = "("
    for edgeS in bottom_edge_minus_right:
        stri += "s = {} | ".format(edgeS)
    stri = stri[:-3]
    stri += ") & orientation=7 & turn=1 & stepL=1 -> s' + {} = s\n".format(gw.ncols*(gw.nrows-2)-2)
    file.write(stri)
    
    ##### first step of turn #####
    edge_comb = right3_edge + bottom2_edge
    stri = "\n("
    for edgeS in edge_comb:
        stri += "s != {} & ".format(edgeS)
    stri += "orientation=6 & turn=1 & stepL=2) -> s' = s + {}\n".format(2*gw.ncols+3)
    file.write(stri)

    right_edge_minus_bottom = list(set(right3_edge) - set(bottom2_edge))
    stri = "("
    for edgeS in right_edge_minus_bottom:
        stri += "s = {} | ".format(edgeS)
    stri = stri[:-3]
    stri += ") & orientation=6 & turn=1 & stepL=2 -> s' = s + {}\n".format(gw.ncols+3)
    file.write(stri)

    bottom_edge_minus_right = list(set(bottom2_edge) - set(right3_edge))
    stri = "("
    for edgeS in bottom_edge_minus_right:
        stri += "s = {} | ".format(edgeS)
    stri = stri[:-3]
    stri += ") & orientation=6 & turn=1 & stepL=2 -> s' + {} = s\n".format(gw.ncols*(gw.nrows-2)-3)
    file.write(stri)
    
    ##### Grid transitions when turning left with orientation 12, 11, or 10
    # stri += "((orientation=9 | orientation=8) & turn=1 & stepL=3) -> s' = s + {}\n".format(gw.ncols-1)
    ##### first step of turn #####
    edge_comb = bottom2_edge+left3_edge
    stri = "\n("
    for edgeS in edge_comb:
        stri += "s != {} & ".format(edgeS)
    stri += "orientation=12 & turn=1 & stepL=2) -> s' = s + {}\n".format(2*gw.ncols-3)
    file.write(stri)

    bottom_edge_minus_left = list(set(bottom2_edge) - set(left3_edge))
    stri = "("
    for edgeS in bottom_edge_minus_left:
        stri += "s = {} | ".format(edgeS)
    stri = stri[:-3]
    stri += ") & orientation=12 & turn=1 & stepL=2 -> s' + {} = s\n".format(gw.ncols*(gw.nrows-2)+3)
    file.write(stri)

    left_edge_minus_bottom = list(set(left3_edge) - set(bottom2_edge))
    stri = "("
    for edgeS in left_edge_minus_bottom:
        stri += "s = {} | ".format(edgeS)
    stri = stri[:-3]
    stri += ") & orientation=12 & turn=1 & stepL=2 -> s' = s + {}\n".format(3*gw.ncols-3)
    file.write(stri)

    ##### second step of turn #####
    edge_comb = bottom2_edge+left2_edge
    stri = "\n("
    for edgeS in edge_comb:
        stri += "s != {} & ".format(edgeS)
    stri += "orientation=11 & turn=1 & stepL=1) -> s' = s + {}\n".format(2*gw.ncols-2)
    file.write(stri)

    bottom_edge_minus_left = list(set(bottom2_edge) - set(left2_edge))
    stri = "("
    for edgeS in bottom_edge_minus_left:
        stri += "s = {} | ".format(edgeS)
    stri = stri[:-3]
    stri += ") & orientation=11 & turn=1 & stepL=1 -> s' + {} = s\n".format(gw.ncols*(gw.nrows-2)+2)
    file.write(stri)

    left_edge_minus_bottom = list(set(left2_edge) - set(bottom2_edge))
    stri = "("
    for edgeS in left_edge_minus_bottom:
        stri += "s = {} | ".format(edgeS)
    stri = stri[:-3]
    stri += ") & orientation=11 & turn=1 & stepL=1 -> s' = s + {}\n".format(3*gw.ncols-2)
    file.write(stri)

    ##### third step of turn #####
    edge_comb = bottom3_edge+left2_edge
    stri = "\n("
    for edgeS in edge_comb:
        stri += "s != {} & ".format(edgeS)
    stri += "orientation=10 & turn=1 & stepL=2) -> s' = s + {}\n".format(3*gw.ncols-2)
    file.write(stri)

    bottom_edge_minus_left = list(set(bottom3_edge) - set(left2_edge))
    stri = "("
    for edgeS in bottom_edge_minus_left:
        stri += "s = {} | ".format(edgeS)
    stri = stri[:-3]
    stri += ") & orientation=10 & turn=1 & stepL=2 -> s' + {} = s\n".format(gw.ncols*(gw.nrows-3)+2)
    file.write(stri)

    left_edge_minus_bottom = list(set(left2_edge) - set(bottom3_edge))
    stri = "("
    for edgeS in left_edge_minus_bottom:
        stri += "s = {} | ".format(edgeS)
    stri = stri[:-3]
    stri += ") & orientation=10 & turn=1 & stepL=2 -> s' = s + {}\n".format(4*gw.ncols-2)
    file.write(stri)




    

    stri = "!forward -> s' = s\n"
    ###45deg change###
    stri += "turn=2 -> orientation' = orientation"
    ###45deg change###
    stri += "\n"
    file.write(stri)

    
    # stri = "!forward -> st' != s\n"
    stri = "st' != s\n"
    stri += "\n"
    file.write(stri)

    # file.write("(orientation != 1 & directionrequest' != 2) /\ (orientation != 3 & directionrequest' != 4) -> !stair'\n")
    # file.write("(orientation' != 3 & directionrequest' != 2) & (orientation' != 9 & directionrequest' != 4) -> !stair'\n")
    # file.write("(orientation' != 4 & orientation' != 12) \/ (directionrequest' != 4 & directionrequest' != 2) -> !stair'\n")
    # file.write(" (directionrequest' != 4 & directionrequest' != 2) -> !stair'\n")
    file.write(" (directionrequest' != 4 & directionrequest' != 2) | (directionrequest != 4 & directionrequest != 2)-> !stair'\n")

    # footstance based navigation:
    # file.write("(orientation=0 | orientation=3 |orientation=6 | orientation=9) /\\ turnLeft /\\ stanceFoot=0 -> pastTurnStanceMatchFoot' = 1\n")
    # file.write("(orientation=0 | orientation=3 |orientation=6 | orientation=9) /\\ turnLeft /\\ stanceFoot=1 -> pastTurnStanceMatchFoot' = 0\n")
    # file.write("(orientation=0 | orientation=3 |orientation=6 | orientation=9) /\\ turnRight /\\ stanceFoot=0 -> pastTurnStanceMatchFoot' = 0\n")
    # file.write("(orientation=0 | orientation=3 |orientation=6 | orientation=9) /\\ turnRight /\\ stanceFoot=1 -> pastTurnStanceMatchFoot' = 1\n")
    # file.write("forward /\\ ((orientation !=0 /\\ orientation!=3 /\\ orientation != 6 /\\ orientation != 9) \/ (!turnLeft /\\ !turnRight)) -> pastTurnStanceMatchFoot' = pastTurnStanceMatchFoot\n")
    # file.write("!forward -> pastTurnStanceMatchFoot' = 2\n")
    ##################### Jonas Action Based Specs ###################


    # Writing env_safety
    print 'Writing ENV_SAFETY'
    for obs in tqdm(gw.obstacles):
        if obs in allowed_states:
            file.write('!st = {}\n'.format(obs))

    ##### Navigation goal tracking

    ##### Attempt to elliminate stop when entering each new coarse grid #####:
    file.write("directionrequest = 0 & orientation = 0 & requestPending1 = 5 -> directionrequest' =1 \/ directionrequest' =0\n")
    file.write("directionrequest = 0 & orientation = 4 & requestPending1 = 5 -> directionrequest' =2 \/ directionrequest' =0\n")
    file.write("directionrequest = 0 & orientation = 8 & requestPending1 = 5 -> directionrequest' =3 \/ directionrequest' =0\n")
    file.write("directionrequest = 0 & orientation = 12 & requestPending1 = 5 -> directionrequest' =4 \/ directionrequest' =0\n")

    file.write("directionrequest = 1 -> directionrequest' !=3\n")
    file.write("directionrequest = 2 -> directionrequest' !=4\n")
    file.write("directionrequest = 3 -> directionrequest' !=1\n")
    file.write("directionrequest = 4 -> directionrequest' !=2\n")

    file.write("requestPending1 != 5 -> directionrequest' = directionrequest\n")
    # file.write("requestPending1 = 5 -> directionrequest' != 0\n\n")

    ###### Can't use specs below since the robot tries to brake env specs by turning 
    file.write("requestPending1 != 5 & stair-> stair' \n")
    file.write("requestPending1 != 5 & !stair-> !stair' \n")

    ##################### Some Suda Stuff ###################
    # if target_reachability:
    #     for t in targets:
    #         file.write('!st = {}\n'.format(t))
    ##################### Some Suda Stuff ###################

    # writing sys_trans
    file.write('\n[SYS_TRANS]\n')
    # all this code does is check what states there is a probability greater than 0 of getting to from the current state. Need to include orientation in that probability map
    # not sure how to deal with actions relying on previous actions. Should I just include actions in the spec file?
    #In general do not need to rely on previous actions/next actions other than to tell the system to stop ahead of time, how to deal with this?

    print 'Writing SYS_TRANS'
    

    ####################################### JONAS ############################
    stri = "!forward' -> turn'=2\n\n"
    stri += "stop -> stepL=0\n"
    stri += "!forward -> (stepL=0 & turn=2)\n\n"
    stri += "stop <-> !forward'\n\n"

    ##### NEED TO REPLACE #######
    # stri += "(orientation'!=0 & orientation'!=3  & orientation'!=6 & orientation'!=9) -> stepL=3\n"
    # stri += "(orientation'=0 | orientation'=3  | orientation'=6 | orientation'=9) -> stepL!=3\n\n"


    # stri += "(orientation'=0 | orientation'=4  | orientation'=8 | orientation'=12) -> stepL=0\n\n"
    ##### NEED TO REPLACE #######


    stri += "(turn=3 & (orientation'!=0 & orientation'!=4  & orientation'!=8 & orientation'!=12)) -> turn'=3\n"
    stri += "(turn=1 & (orientation'!=0 & orientation'!=4  & orientation'!=8 & orientation'!=12)) -> turn'=1\n"
    stri += "\n"
    file.write(stri)

    file.write("turn = 2 & turn' != 2 -> stepL' = 2\n")
    file.write("stepL = 2 -> stepL' = 1\n")
    file.write("turn' != 2 -> stepL' != 0\n")
    file.write("stepL = 0 -> turn' = 2\n")
    file.write("turn = 2 & stepL = 1 -> turn' != 2\n")

    # file.write("turn' = 2 -> stepL' !=2 \n")
    # file.write("stepL=1 -> turn' !=2 \n")
    # file.write("stepL = 1 & (orientation = 0 | orientation = 4 | orientation = 8 | orientation = 12) -> turn !=2\n")



    stri = ""
    stri += ""

    # footstance based navigation:
    file.write('\n')
    file.write("stanceFoot !=2\n")
    file.write("forward' & stanceFoot=0 -> stanceFoot'=1\n")
    file.write("forward' & stanceFoot=1 -> stanceFoot'=0\n")
    file.write("!forward' -> stanceFoot' =stanceFoot\n")
    file.write("(orientation = 0 | orientation = 4 | orientation = 8 | orientation = 12) & stanceFoot = 0 -> turn !=1\n")
    file.write("(orientation = 0 | orientation = 4 | orientation = 8 | orientation = 12) & stanceFoot = 1 -> turn !=3\n")



    # file.write("!forward' -> stanceFoot' =2\n")
    # file.write("forward' -> stanceFoot' !=2\n")

    # # file.write("forward & stanceFoot=0 -> stanceFoot'=1\n")
    # # file.write("forward & stanceFoot=1 -> stanceFoot'=0\n")

    ##### NEED TO REPLACE #######
    # file.write('\n')
    # file.write('turn !=2 -> stepL != 1 /\ stepL !=2\n')
    # file.write('\n')

    file.write('\n')
    # file.write("turn'=2 -> stepL'=0\n")
    file.write('\n')
    ##### NEED TO REPLACE   -DONE? #######

    ##### Fine Specific Specs #####
    # stri =""
    # for edgeS in gw.edges:
    #     stri += "s' = {} -> turn' = 2\n".format(edgeS)
    # stri += "\n"
    # file.write(stri)

    # stri =""
    # for edgeS in gw.edges:
    #     stri += "s' = {} & directionrequest' != 0-> !forward\n".format(edgeS)
    # stri += "\n"
    # file.write(stri)

    # file.write("requestPending1' = directionrequest\n")





    ##### Specs that govern how liveness specs can be met #####
    file.write("\n\n #goal tracking specs\n\n")

    stri = "requestPending1 = 1 & stepL!=0 -> ((("
    for edgeS in top3_edge:
        stri += "s' = {} \/ ".format(edgeS)
    stri = stri[:-4]
    stri+= ") & requestPending1' = 5) \/ (("
    for edgeS in top3_edge:
        stri += "s' != {} & ".format(edgeS)
    stri = stri[:-3]
    stri+= ") & requestPending1' = 1))\n"
    file.write(stri)

    stri = "requestPending1 = 2 & stepL!=0 -> ((("
    for edgeS in right3_edge:
        stri += "s' = {} \/ ".format(edgeS)
    stri = stri[:-4]
    stri+= ") & requestPending1' = 5) \/ (("
    for edgeS in right3_edge:
        stri += "s' != {} & ".format(edgeS)
    stri = stri[:-3]
    stri+= ") & requestPending1' = 2))\n"
    file.write(stri)

    stri = "requestPending1 = 3 & stepL!=0 -> ((("
    for edgeS in bottom3_edge:
        stri += "s' = {} \/ ".format(edgeS)
    stri = stri[:-4]
    stri+= ") & requestPending1' = 5) \/ (("
    for edgeS in bottom3_edge:
        stri += "s' != {} & ".format(edgeS)
    stri = stri[:-3]
    stri+= ") & requestPending1' = 3))\n"
    file.write(stri)

    stri = "requestPending1 = 4 & stepL!=0 -> ((("
    for edgeS in left3_edge:
        stri += "s' = {} \/ ".format(edgeS)
    stri = stri[:-4]
    stri+= ") & requestPending1' = 5) \/ (("
    for edgeS in left3_edge:
        stri += "s' != {} & ".format(edgeS)
    stri = stri[:-3]
    stri+= ") & requestPending1' = 4))\n"
    file.write(stri)

    ####4 cel step length
    stri = "requestPending1 = 1 & stepL=0 -> ((("
    for edgeS in top4_edge:
        stri += "s' = {} \/ ".format(edgeS)
    stri = stri[:-4]
    stri+= ") & requestPending1' = 5) \/ (("
    for edgeS in top4_edge:
        stri += "s' != {} & ".format(edgeS)
    stri = stri[:-3]
    stri+= ") & requestPending1' = 1))\n"
    file.write(stri)

    stri = "requestPending1 = 2 & stepL=0 -> ((("
    for edgeS in right4_edge:
        stri += "s' = {} \/ ".format(edgeS)
    stri = stri[:-4]
    stri+= ") & requestPending1' = 5) \/ (("
    for edgeS in right4_edge:
        stri += "s' != {} & ".format(edgeS)
    stri = stri[:-3]
    stri+= ") & requestPending1' = 2))\n"
    file.write(stri)

    stri = "requestPending1 = 3 & stepL=0 -> ((("
    for edgeS in bottom4_edge:
        stri += "s' = {} \/ ".format(edgeS)
    stri = stri[:-4]
    stri+= ") & requestPending1' = 5) \/ (("
    for edgeS in bottom4_edge:
        stri += "s' != {} & ".format(edgeS)
    stri = stri[:-3]
    stri+= ") & requestPending1' = 3))\n"
    file.write(stri)

    stri = "requestPending1 = 4 & stepL=0 -> ((("
    for edgeS in left4_edge:
        stri += "s' = {} \/ ".format(edgeS)
    stri = stri[:-4]
    stri+= ") & requestPending1' = 5) \/ (("
    for edgeS in left4_edge:
        stri += "s' != {} & ".format(edgeS)
    stri = stri[:-3]
    stri+= ") & requestPending1' = 4))\n"
    file.write(stri)

    # stri = "requestPending1 = 0 -> ((!forward & requestPending1' = 5) \/ (forward & requestPending1' = 0))\n"
    # stri = "requestPending1 = 0 -> ((!forward & ("
    # for edgeS in gw.edges3+gw.edges2+gw.edges:
    #     stri += "s != {} & ".format(edgeS)
    # stri = stri[:-3]
    # stri+= ") & requestPending1' = 5) \/ (forward & requestPending1' = 0))\n"
    # file.write(stri)

    stri = "requestPending1 = 0 -> ((!forward' & ("
    for edgeS in gw.edges3+gw.edges2+gw.edges+gw.edges4:
        stri += "s != {} & ".format(edgeS)
    stri = stri[:-3]
    stri+= ") & requestPending1' = 5) \/ (forward' & ("
    for edgeS in gw.edges3+gw.edges2+gw.edges+gw.edges4:
        stri += "s = {} | ".format(edgeS)
    stri = stri[:-3]
    stri+=") & requestPending1' = 0))\n"
    file.write(stri)

    stri = "directionrequest = 0 -> ((!forward' & ("
    for edgeS in gw.edges3+gw.edges2+gw.edges+gw.edges4:
        stri += "s != {} & ".format(edgeS)
    stri = stri[:-3]
    stri+= ")) \/ (forward' & ("
    for edgeS in gw.edges3+gw.edges2+gw.edges+gw.edges4:
        stri += "s = {} | ".format(edgeS)
    stri = stri[:-3]
    stri+=")))\n"
    file.write(stri)






    file.write("requestPending1 = 5 -> requestPending1' = directionrequest'\n")

    # corner_states = [0,gw.ncols-1,gw.nstates-1,gw.nstates-gw.ncols]
    def intersection(lst1, lst2): 
        return list(set(lst1) & set(lst2))
  
    TL_Corner = intersection(top3_edge,left3_edge)
    TR_Corner = intersection(top3_edge,right3_edge)
    BR_Corner = intersection(bottom3_edge,right3_edge)
    BL_Corner = intersection(bottom3_edge,left3_edge)

    corner_states = TL_Corner+TR_Corner+BR_Corner+BL_Corner

    #####Jonas Turn Back on
    # stri =""
    # for edgeS in corner_states:
    #     stri += "s' != {}\n".format(edgeS)
    # stri += "\n"
    # file.write(stri)
    #####Jonas Turn Back on

    ##### Ensure the robot doesn't leave cell before completing nav goal #####

    file.write("\n\n #specs to stay in coarse cell until goal is completed\n\n")

    ######Need to Fix
    # stri =""
    # for edgeS in gw.edges + gw.edges2+gw.edges3:
    #     stri += "s' = {} | ".format(edgeS)
    #     stri = stri[:-3]
    #     stri += " & requestPending1' != 5 -> !forward'\n"
    # stri += "\n"
    # file.write(stri)

    stri ="("
    for edgeS in gw.top_edge+gw.top_edge2+gw.top_edge3+gw.top_edge4:
        stri += "s' = {} | ".format(edgeS)
    stri = stri[:-3]
    stri += ") & (orientation = 0 | orientation = 1 | orientation = 2 | orientation = 3 | orientation = 4 | orientation = 12 | orientation = 13 | orientation = 14 | orientation = 15) & requestPending1' != 5 -> !forward'\n"
    stri += "\n"
    file.write(stri)

    stri ="("
    for edgeS in gw.right_edge+gw.right_edge2+gw.right_edge3+gw.right_edge4:
        stri += "s' = {} | ".format(edgeS)
    stri = stri[:-3]
    stri += ") & (orientation = 0 | orientation = 1 | orientation = 2 | orientation = 3 | orientation = 4 | orientation = 5 | orientation = 6 | orientation = 7 | orientation = 8) & requestPending1' != 5 -> !forward'\n"
    stri += "\n"
    file.write(stri)

    stri ="("
    for edgeS in gw.bottom_edge+gw.bottom_edge2+gw.bottom_edge3+gw.bottom_edge4:
        stri += "s' = {} | ".format(edgeS)
    stri = stri[:-3]
    stri += ") & (orientation = 4 | orientation = 5 | orientation = 6 | orientation = 7 | orientation = 8 | orientation = 9 | orientation = 10 | orientation = 11 | orientation = 12) & requestPending1' != 5 -> !forward'\n"
    stri += "\n"
    file.write(stri)

    stri ="("
    for edgeS in gw.left_edge+gw.left_edge2+gw.left_edge3+gw.left_edge4:
        stri += "s' = {} | ".format(edgeS)
    stri = stri[:-3]
    stri += ") & (orientation = 8 | orientation = 9 | orientation = 10 | orientation = 11 | orientation = 12 | orientation = 13 | orientation = 14 | orientation = 15 | orientation = 0) & requestPending1' != 5 -> !forward'\n"
    stri += "\n"
    file.write(stri)

    

    ####when turning
    #     stri ="("
    # for edgeS in gw.top_edge+gw.top_edge2+gw.top_edge3+gw.top_edge4:
    #     stri += "s' = {} | ".format(edgeS)
    # stri = stri[:-3]
    # stri += ") & (orientation = 0 | orientation = 1 | orientation = 2 | orientation = 3 | orientation = 4 | orientation = 12 | orientation = 13 | orientation = 14 | orientation = 15) & requestPending1' != 5 -> !forward'\n"
    # stri += "\n"
    # file.write(stri)

    # stri ="("
    # for edgeS in gw.right_edge+gw.right_edge2+gw.right_edge3+gw.right_edge4:
    #     stri += "s' = {} | ".format(edgeS)
    # stri = stri[:-3]
    # stri += ") & (orientation = 0 | orientation = 1 | orientation = 2 | orientation = 3 | orientation = 4 | orientation = 5 | orientation = 6 | orientation = 7 | orientation = 8) & requestPending1' != 5 -> !forward'\n"
    # stri += "\n"
    # file.write(stri)

    # stri ="("
    # for edgeS in gw.bottom_edge+gw.bottom_edge2+gw.bottom_edge3+gw.bottom_edge4:
    #     stri += "s' = {} | ".format(edgeS)
    # stri = stri[:-3]
    # stri += ") & (orientation = 4 | orientation = 5 | orientation = 6 | orientation = 7 | orientation = 8 | orientation = 9 | orientation = 10 | orientation = 11 | orientation = 12) & requestPending1' != 5 -> !forward'\n"
    # stri += "\n"
    # file.write(stri)

    # stri ="("
    # for edgeS in gw.left_edge+gw.left_edge2+gw.left_edge3+gw.left_edge4:
    #     stri += "s' = {} | ".format(edgeS)
    # stri = stri[:-3]
    # stri += ") & (orientation = 8 | orientation = 9 | orientation = 10 | orientation = 11 | orientation = 12 | orientation = 13 | orientation = 14 | orientation = 15 | orientation = 0) & requestPending1' != 5 -> !forward'\n"
    # stri += "\n"
    # file.write(stri)

    ######Need to Fix



    
    #####Jonas Turn Back on
    # file.write("\nstair' & orientation' = 4 & directionrequest' = 2 -> (stepL' = 0 & stepH' = 4) \/ ((stepL' = 1 & stepH' = 5)) \/ ((stepL' = 2 & stepH' = 6))\n")
    # file.write("stair' & orientation' = 12 & directionrequest' = 4 -> (stepL' = 0 & stepH' = 2) \/ ((stepL' = 1 & stepH' = 1)) \/ ((stepL' = 2 & stepH' = 0))\n\n")
    
    file.write("\nstair'  & directionrequest' = 2 -> (stepL' = 0 & stepH' = 4) \/ ((stepL' = 1 & stepH' = 4)) \/ ((stepL' = 2 & stepH' = 6))\n")
    file.write("stair' & directionrequest' = 4 -> (stepL' = 0 & stepH' = 2) \/ ((stepL' = 1 & stepH' = 1)) \/ ((stepL' = 2 & stepH' = 0))\n\n")
    
    file.write("!stair' -> stepH' = 3\n\n")
    #####Jonas Turn Back on




##################################################################################

    # Writing sys_liveness
    file.write('\n[SYS_LIVENESS]\n')
    file.write("requestPending1 = 5\n")
    # file.write("turn != 2\n")





    file.write('\n[ENV_LIVENESS]\n')
   
    # stri = ""
    # for visstate in set(nonbeliefstates):
    #     stri += "st != {} /\\ ".format(visstate)
    # stri = stri[:-4]
    # file.write(stri)

    # stri = ""
    # for Bstate in (set(allstates) - set(nonbeliefstates)):
    #     stri += "st = {} \/ ".format(Bstate)
    # stri = stri[:-4]
    # file.write(stri)

    # file.write("st' = {}".format(allstates[-2]))

    # file.write("st' = {}".format(115))