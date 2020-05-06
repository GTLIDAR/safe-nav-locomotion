
import numpy as np
import copy
import itertools
from tqdm import *
import simplejson as json
import math
from gridworld import *

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

def reach_states(gw,states):
    t =set()
    for state in states:
        for action in gw.actlist:
            t.update(set(np.nonzero(gw.prob[action][state])[0]))
    return t

def powerset(s):
    x = len(s)
    a = []
    for i in range(1,1<<x):
        a.append({s[j] for j in range(x) if (i &(1<<j))})
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
def write_to_slugs_part_dist(infile,gw,init,initmovetarget,invisibilityset,visset_target,targets,vel=1,visdist = 5,allowed_states = [],
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
    file.write('orientation:0...3\n') # 0 = North, 1 = South, 2 = West, 3 = East

    file.write('[OUTPUT]\n')
    file.write('s:0...{}\n'.format(len(gw.states)-1))
    if target_reachability:
        file.write('c:0...1\n')

    file.write('[ENV_INIT]\n')

    if initmovetarget in allowed_states:
        file.write('st = {}\n'.format(initmovetarget))
    else:
        file.write('st = {}\n'.format(allstates[-1]))

    file.write('[SYS_INIT]\n')
    file.write('s = {}\n'.format(init))
    if target_reachability:
        file.write('c = 0\n')

    # writing env_trans
    file.write('\n[ENV_TRANS]\n')
    print 'Writing ENV_TRANS'
    for st in tqdm(set(allstates) - (set(nonbeliefstates) - set(allowed_states))): #Only allowed states and belief states
        if st in allowed_states:
            for s in allowed_states:
                repeat = set()
                stri = "(s = {} /\\\\ st = {}) -> ".format(s,st)
                beliefset = set()
                for a in range(gw.nactions):
                    for t in np.nonzero(gw.prob[gw.actlist[a]][st])[0]:
                        if t in allowed_states and t not in repeat:
                            if t not in invisibilityset[s]:
                                stri += 'st\' = {} \\\\/'.format(t)
                                repeat.add(t)
                            else:
                                if not t == s and t not in targets: # not allowed to move on agent's position
                                    try:
                                        partgridkeyind = [inv for inv in range(len(partitionGrid.values())) if t in partitionGrid.values()[inv]][0]
                                        t2 = partitionGrid.keys()[partgridkeyind]
                                        beliefset.add(t2)
                                    except:
                                        print t
                                        # print('test print t')
                        elif t not in allowed_states and t not in gw.obstacles and allstates[-1] not in repeat: # Error state????
                            stri += 'st\' = {} \\\\/'.format(allstates[-1])
                            repeat.add(allstates[-1])
                if len(beliefset) > 0:
                    b2 = allstates[len(nonbeliefstates) + beliefcombs.index(beliefset)]
                    if b2 not in repeat:
                        stri += ' st\' = {} \\\\/'.format(b2)
                        repeat.add(b2)
                stri = stri[:-3]
                stri += '\n'
                file.write(stri)
                file.write("s = {} -> !st' = {}\n".format(s,s))
                # .format() fills {} with whats in ()
                # file.write("s = {} -> !st = {}\n".format(s,s))
        elif st == allstates[-1]: # Error state?????
            stri = "st = {} -> ".format(st)
            for t in fullvis_states:
                stri += "st' = {} \\\\/ ".format(t)
            stri += "st' = {}".format(st)
            stri += '\n'
            file.write(stri)
        else: # Belief states
            for s in allowed_states:
                invisstates = invisibilityset[s]
                visstates = set(nonbeliefstates) - invisstates

                beliefcombstate = beliefcombs[st - len(nonbeliefstates)]
                beliefstates = set()
                for currbeliefstate in beliefcombstate:
                    beliefstates = beliefstates.union(partitionGrid[currbeliefstate])
                beliefstates = beliefstates - set(targets) # remove target positions (no transitions from target positions)
                beliefstates_vis = beliefstates.intersection(visstates)
                beliefstates_invis = beliefstates - beliefstates_vis

                if belief_safety > 0 and len(beliefstates_invis) > belief_safety:
                    continue # no transitions from error states

                # what does this (^) even do, just continues?

                if len(beliefstates) > 0:
                    stri = "(s = {} /\\\\ st = {}) -> ".format(s,st)
                    repeat = set()
                    beliefset = set()
                    for b in beliefstates:
                        for a in range(gw.nactions):
                            for t in np.nonzero(gw.prob[gw.actlist[a]][b])[0]:
                                if t not in invisibilityset[s]:
                                    if t in allowed_states and t not in repeat:
                                        stri += ' st\' = {} \\\\/'.format(t)
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
                            stri += ' st\' = {} \\\\/'.format(b2)
                            repeat.add(b2)

                    stri = stri[:-3]
                    stri += '\n'
                    file.write(stri)

    # Writing env_safety
    print 'Writing ENV_SAFETY'
    for obs in tqdm(gw.obstacles):
        if obs in allowed_states:
            file.write('!st = {}\n'.format(obs))

    ##################### Some Suda Stuff ###################
    # if target_reachability:
    #     for t in targets:
    #         file.write('!st = {}\n'.format(t))
    ##################### Some Suda Stuff ###################

    # writing sys_trans
    file.write('\n[SYS_TRANS]\n')
    print 'Writing SYS_TRANS'
    for s in tqdm(nonbeliefstates):
        if s in allowed_states:
            repeat = set()
            uset = list(itertools.product(range(len(gw.actlist)),repeat=vel))
            stri = "s = {} -> ".format(s)
            for u in uset:
                snext = copy.deepcopy(s)
                for v in range(vel):
                    act = gw.actlist[u[v]]
                    snext = np.nonzero(gw.prob[act][snext])[0][0]
                if snext not in repeat:
                    stri += '(s\' = {}) \\\\/'.format(snext)
                    repeat.add(snext)
            stri = stri[:-3]
            stri += '\n'
            file.write(stri)
        else:
            file.write("!s = {}\n".format(s))
    # Writing sys_safety
    for obs in gw.obstacles:
        if obs in allowed_states:
            file.write('!s = {}\n'.format(obs))

    for s in set(allowed_states):
        stri = 'st = {} -> !s = {}\n'.format(s,s)
        file.write(stri)
        stri = 'st = {} -> !s\' = {}\n'.format(s,s)
        file.write(stri)



    if belief_safety > 0:
        #beliefcombs is all possible combination of belief states defined in main file
        for b in beliefcombs:
            beliefset = set()
            for beliefstate in b:
                beliefset = beliefset.union(partitionGrid[beliefstate])
            beliefset =  beliefset -set(gw.targets[0])
            if len(beliefset) > belief_safety:
                stri = 'st = {} -> '.format(len(nonbeliefstates)+beliefcombs.index(b))
                counter = 0
                stri += '('
                for x in allowed_states:
                    invisstates = invisibilityset[x]
                    beliefset_invis = beliefset.intersection(invisstates)
                    if len(beliefset_invis) > belief_safety:
                        stri += '!s = {} /\\\\ '.format(nonbeliefstates.index(x)) # <--- why??
                        counter += 1
                stri = stri[:-3]
                stri += ')\n'
                if counter > 0:
                    file.write(stri)

    if target_reachability:
        stri = 'c = 0 /\\\\ ('
        for t in targets:
            stri += ('s = {} \\\\/ '.format(nonbeliefstates.index(t)))
        stri = stri[:-3]
        stri += ') -> c\' = 1'
        stri += '\n'
        stri += '!('
        for t in targets:
            stri += ('s = {} \\\\/ '.format(nonbeliefstates.index(t)))
        stri = stri[:-3]
        stri += ') -> c\' = 0\n'
        stri += 'c = 1 -> c\' = 0 \n'
        file.write(stri)

#################################### -- Orientation stuff -- ##############################################
    if target_has_vision:
        # Check to see if a file was provided
        if compute_vis_flag is True:
            file_target_vis = target_visibility(gw, invisibilityset, partitionGrid, target_vision_dist, filename_target_vis, allowed_states, visset_target)
        else:
            # If no file was provided, go make one that includes the vision information of the target in
            # in belief states
            file_target_vis = filename_target_vis
        # Convert the target vision file into a dictionary
        target_vision_dict = vis_parser(file_target_vis)

        # Write the specs using the dictionary
        print 'Writing belief state vision specs'
        for st,dict_of_states in target_vision_dict.items():
            for s,vis_list in dict_of_states.items():
                if not bool(vis_list):
                    continue
                Q_string = ('st = {}'.format(st) + ' /\\\\ s = {}'.format(s) + ' -> ')
                for item in vis_list:
                    Q_string += ('!s = {}'.format(item) + ' /\\\\')
                Q_string = Q_string[:-3] + '\n'
                file.write(Q_string)

                Q_next_string = ('st = {}'.format(st) + ' /\\\\ s = {}'.format(s) + ' -> ')
                for item in vis_list:
                    Q_next_string += ('!s\' = {}'.format(item) + ' /\\\\')
                Q_next_string = Q_next_string[:-3] + '\n'
                file.write(Q_next_string)

        for state_key,state_value in visset_target.items():
            if isinstance(state_value, set): # Check to see if the state_value is of type "set" instead of type "dict". "set" means that state_key corresponds to an obstacle.
                continue
            for state_card,state_vis_set in state_value.items():
                if not bool(state_vis_set): # Check to see if the set is empty
                    continue
                slugs_card = card_to_slugs_int(state_card)
                Q_string = ('st = {}'.format(state_key) + ' /\\\\ orientation = {}'.format(slugs_card) + ' -> ')
                for vis_state in state_vis_set:
                    Q_string += ('!s = {}'.format(vis_state) + ' /\\\\')
                Q_string = Q_string[:-3] + '\n'
                file.write(Q_string)

                Q_next_string = ('st = {}'.format(state_key) + ' /\\\\ orientation = {}'.format(slugs_card) + ' -> ')
                for vis_state in state_vis_set:
                    Q_next_string += ('!s\' = {}'.format(vis_state) + ' /\\\\')
                Q_next_string = Q_next_string[:-3] + '\n'
                file.write(Q_next_string)
##################################################################################

    # Writing sys_liveness
    file.write('\n[SYS_LIVENESS]\n')
    if target_reachability:
        file.write('c = 1\n')

    stri  = ''
    if belief_liveness >0:
        for st in allowed_states:
            stri+='st = {}'.format(st)
            if st != allowed_states[-1]:
                stri+=' \\\\/ '
        for b in beliefcombs:
            beliefset = set()
            for beliefstate in b:
                beliefset = beliefset.union(partitionGrid[beliefstate])
            beliefset =  beliefset -set(gw.targets[0])
            stri1 = ' \\\\/ (st = {} /\\\\ ('.format(len(nonbeliefstates)+beliefcombs.index(b))
            count = 0
            for x in allowed_states:
                truebelief = beliefset.intersection(invisibilityset[x])
                if len(truebelief) <= belief_liveness:
                    if count > 0:
                        stri1 += ' \\\\/ '
                    stri1 += ' s = {} '.format(nonbeliefstates.index(x))
                    count+=1
            stri1+='))'
            if count > 0 and count < len(allowed_states):
                stri+=stri1
            if count == len(allowed_states):
                stri+= ' \\\\/ st = {}'.format(len(nonbeliefstates)+beliefcombs.index(b))
        stri += ' \\\\/ st = {}'.format(allstates[-1])
        # This some how adds the location of the obstacle????
        for st in set(nonbeliefstates)- set(allowed_states):
            stri += ' \\\\/ st = {}'.format(st)
        stri += '\n'
        file.write(stri)

    ##################### Some Suda Stuff ###################
    # file.write('\n[ENV_LIVENESS]\n')
    # for t in targets:
    #     file.write('st = {}\n'.format(t+1))
    ##################### Some Suda Stuff ###################


def write_to_slugs_imperfect_sensor(infile,gw,init,initmovetarget,invisibilityset,targets,vel=1,visdist = 5,allowed_states = [],
    fullvis_states = [],partitionGrid =dict(), belief_safety = 0, belief_liveness = 0, target_reachability = False,sensor_uncertainty=1,sensor_uncertain_dict = dict()):



    nonbeliefstates = gw.states
    beliefcombs = powerset(partitionGrid.keys())

    belief_ncols = gw.ncols - sensor_uncertainty + 1
    belief_nrows = gw.nrows - sensor_uncertainty + 1

    allstates = copy.deepcopy(sensor_uncertain_dict.keys())
    for i in range(belief_ncols*belief_nrows,belief_ncols*belief_nrows + len(beliefcombs)):
        allstates.append(i)
    allstates.append(len(allstates)) # nominal state if target leaves allowed region

    filename = infile + '.structuredslugs'
    file = open(filename, 'w')
    file.write('[INPUT]\n')
    file.write('st:0...{}\n'.format(len(allstates) - 1))

    file.write('[OUTPUT]\n')
    file.write('s:0...{}\n'.format(len(gw.states) - 1))
    if target_reachability:
        file.write('c:0...1\n')

    file.write('[ENV_INIT]\n')
    initbelief = sensor_uncertain_dict.values().index(
        [i for i in sensor_uncertain_dict.values() if initmovetarget in i][0])
    if initmovetarget in allowed_states:
        file.write('st = {}\n'.format(initbelief))
    else:
        file.write('st = {}\n'.format(allstates[-1]))

    file.write('[SYS_INIT]\n')
    file.write('s = {}\n'.format(init))
    if target_reachability:
        file.write('c = 0\n')


    # for s in gw.states:
    #     old_row_col = gw.coords(s)
    #     new_row_col = (old_row_col[0]/sensor_uncertainty,old_row_col[1]/sensor_uncertainty)
    #     new_state = new_row_col[0] * belief_ncols + new_row_col[1]
    #     if not sensor_uncertain_dict[new_state]:
    #         sensor_uncertain_dict[new_state] = set()
    #     sensor_uncertain_dict[new_state].add(s)
    # index = 0
    # initbelief = sensor_uncertain_dict.values().index([i for i in sensor_uncertain_dict.values() if initmovetarget in i][0])
    # belief_partitionGrid = dict()
    # for bs in partitionGrid:
    #     belief_partitionGrid[index] = set()
    #     for s in partitionGrid[bs]:
    #         beliefgridstate = sensor_uncertain_dict.values().index([i for i in sensor_uncertain_dict.values() if s in i][0])
    #         belief_partitionGrid[index].add(beliefgridstate)

    file.write('\n[ENV_TRANS]\n')
    print 'Writing ENV_TRANS'
    borderstates = set()
    for belief_st in tqdm(set(allstates) - (set(nonbeliefstates) - set(allowed_states))): #Only allowed states and belief states <- not correct if sensing uncertainty is more than 1 - Fix!
        if belief_st < belief_nrows*belief_ncols:
            for s in allowed_states:
                repeat = set()
                stri = "(s = {} /\\\\ st = {}) -> ".format(s, belief_st)
                beliefset = set()
                for st in sensor_uncertain_dict[belief_st]:
                    for a in range(gw.nactions):
                        for t in np.nonzero(gw.prob[gw.actlist[a]][st])[0]:
                            if t in allowed_states:
                                if t not in invisibilityset[s]:
                                    all_belief_gridstate_sets = [i for i in sensor_uncertain_dict.values() if t in i]
                                    for belief_gridstates in all_belief_gridstate_sets:
                                        next_belief_gridstate = sensor_uncertain_dict.keys()[sensor_uncertain_dict.values().index(belief_gridstates)]
                                        if next_belief_gridstate not in repeat:
                                            stri += 'st\' = {} \\\\/'.format(next_belief_gridstate)
                                            repeat.add(next_belief_gridstate)
                                else:
                                    if not t == s and t not in targets:  # not allowed to move on agent's position
                                        try:
                                            partgridkeyind = [inv for inv in range(len(partitionGrid.values())) if
                                                              t in partitionGrid.values()[inv]][0]
                                            t2 = partitionGrid.keys()[partgridkeyind]
                                            beliefset.add(t2)
                                        except:
                                            print t

                            elif t not in allowed_states and t not in gw.obstacles:  # Target is outside the agent's allowed region
                                if allstates[-1] not in repeat:
                                    stri += 'st\' = {} \\\\/'.format(allstates[-1])
                                    repeat.add(allstates[-1])
                                borderstates.add(belief_st)

                if len(beliefset) > 0:
                    b2 = allstates[belief_nrows*belief_ncols + beliefcombs.index(beliefset)]
                    if b2 not in repeat:
                        stri += ' st\' = {} \\\\/'.format(b2)
                        repeat.add(b2)
                stri = stri[:-3]
                stri += '\n'
                if len(repeat) > 0:
                    file.write(stri)
                # file.write("s = {} -> !st' = {}\n".format(s, s))
                # file.write("s = {} -> !st = {}\n".format(s,s))
        elif belief_st != allstates[-1]: # Belief states
            for s in allowed_states:
                invisstates = invisibilityset[s]
                visstates = set(nonbeliefstates) - invisstates

                beliefcombstate = beliefcombs[belief_st - belief_ncols*belief_nrows]
                beliefstates = set()
                for currbeliefstate in beliefcombstate:
                    beliefstates = beliefstates.union(partitionGrid[currbeliefstate])
                beliefstates = beliefstates - set(targets) # remove target positions (no transitions from target positions)
                beliefstates_vis = beliefstates.intersection(visstates)
                beliefstates_invis = beliefstates - beliefstates_vis

                if belief_safety > 0 and len(beliefstates_invis) > belief_safety:
                    continue # no transitions from error states

                if len(beliefstates) > 0:
                    stri = "(s = {} /\\\\ st = {}) -> ".format(s,belief_st)
                    repeat = set()
                    beliefset = set()
                    for b in beliefstates:
                        for a in range(gw.nactions):
                            for t in np.nonzero(gw.prob[gw.actlist[a]][b])[0]:
                                if t in allowed_states:
                                    if t not in invisibilityset[s]:
                                        all_belief_gridstate_sets = [i for i in sensor_uncertain_dict.values() if t in i]
                                        for belief_gridstates in all_belief_gridstate_sets:
                                            next_belief_gridstate = sensor_uncertain_dict.keys()[
                                                sensor_uncertain_dict.values().index(belief_gridstates)]
                                            if next_belief_gridstate not in repeat:
                                                stri += 'st\' = {} \\\\/'.format(next_belief_gridstate)
                                                repeat.add(next_belief_gridstate)
                                    else:
                                        if t in gw.targets[0]:
                                            continue
                                        if t in allowed_states:
                                            t2 = partitionGrid.keys()[[inv for inv in range(len(partitionGrid.values())) if t in partitionGrid.values()[inv]][0]]
                                            beliefset.add(t2)
                                elif t not in allowed_states and t not in gw.obstacles:  # Target is outside the agent's allowed region
                                    if allstates[-1] not in repeat:
                                        stri += 'st\' = {} \\\\/'.format(allstates[-1])
                                        repeat.add(allstates[-1])

                    if len(beliefset) > 0:
                        b2 = allstates[belief_nrows*belief_ncols + beliefcombs.index(beliefset)]
                        if b2 not in repeat:
                            stri += ' st\' = {} \\\\/'.format(b2)
                            repeat.add(b2)

                    stri = stri[:-3]
                    stri += '\n'
                    file.write(stri)
    belief_st = allstates[-1] # Target is outside the agent's allowed region
    for s in allowed_states:
        invisstates = invisibilityset[s]
        beliefset = set()
        stri = "(s = {} /\\\\ st = {} -> st' = {} \\\\/ ".format(s, belief_st,belief_st)
        repeat = set()
        for t in borderstates:
            if not t in invisstates:
                all_belief_gridstate_sets = [i for i in sensor_uncertain_dict.values() if t in i]
                for belief_gridstates in all_belief_gridstate_sets:
                    next_belief_gridstate = sensor_uncertain_dict.keys()[
                        sensor_uncertain_dict.values().index(belief_gridstates)]
                    if next_belief_gridstate not in repeat:
                        stri += 'st\' = {} \\\\/'.format(next_belief_gridstate)
                        repeat.add(next_belief_gridstate)
            else:
                t2 = partitionGrid.keys()[
                    [inv for inv in range(len(partitionGrid.values())) if t in partitionGrid.values()[inv]][
                        0]]
                beliefset.add(t2)
        if len(beliefset) > 0:
            b2 = allstates[belief_nrows * belief_ncols + beliefcombs.index(beliefset)]
            if b2 not in repeat:
                stri += ' st\' = {} \\\\/'.format(b2)
                repeat.add(b2)

        stri = stri[:-3]
        stri += ')\n'
        file.write(stri)

            #
            #
            # stri = "st = {} -> ".format(belief_st)
            # for t in fullvis_states:
            #     stri += "st' = {} \\\\/ ".format(t)
            # stri += "st' = {}".format(belief_st)
            # stri += '\n'
            # file.write(stri)
            #

    # writing sys_trans
    print('Border states are: ', borderstates)
    file.write('\n[SYS_TRANS]\n')
    print 'Writing SYS_TRANS'
    for s in tqdm(nonbeliefstates):
        if s in allowed_states:
            repeat = set()
            uset = list(itertools.product(range(len(gw.actlist)), repeat=vel))
            stri = "s = {} -> ".format(s)
            for u in uset:
                snext = copy.deepcopy(s)
                for v in range(vel):
                    act = gw.actlist[u[v]]
                    snext = np.nonzero(gw.prob[act][snext])[0][0]
                if snext not in repeat:
                    stri += '(s\' = {}) \\\\/'.format(snext)
                    repeat.add(snext)
            stri = stri[:-3]
            stri += '\n'
            file.write(stri)
        else:
            file.write("!s = {}\n".format(s))
    # Writing sys_safety
    for obs in gw.obstacles:
        if obs in allowed_states:
            file.write('!s = {}\n'.format(obs))

    # for s in set(allowed_states):
    #     stri = 'st = {} -> !s = {}\n'.format(s, s)
    #     file.write(stri)
    #     stri = 'st = {} -> !s\' = {}\n'.format(s, s)
    #     file.write(stri)

    if belief_safety > 0:
        for b in beliefcombs:
            beliefset = set()
            for beliefstate in b:
                beliefset = beliefset.union(partitionGrid[beliefstate])
            beliefset = beliefset - set(gw.targets[0])
            beliefset = beliefset.intersection(set(allowed_states))
            if len(beliefset) > belief_safety:
                stri = 'st = {} -> '.format(belief_ncols*belief_nrows + beliefcombs.index(b))
                counter = 0
                stri += '('
                for x in allowed_states:
                    invisstates = invisibilityset[x]
                    beliefset_invis = beliefset.intersection(invisstates)
                    if len(beliefset_invis) > belief_safety:
                        stri += '!s = {} /\\\\ '.format(nonbeliefstates.index(x))  # <--- why??
                        counter += 1
                stri = stri[:-3]
                stri += ')\n'
                if counter > 0:
                    file.write(stri)

    if target_reachability:
        stri = 'c = 0 /\\\\ ('
        for t in targets:
            stri += ('s = {} \\\\/ '.format(nonbeliefstates.index(t)))
        stri = stri[:-3]
        stri += ') -> c\' = 1'
        stri += '\n'
        stri += '!('
        for t in targets:
            stri += ('s = {} \\\\/ '.format(nonbeliefstates.index(t)))
        stri = stri[:-3]
        stri += ') -> c\' = 0\n'
        # stri += 'c = 1 -> c\' = 1 \n'
        file.write(stri)


    # Writing sys_liveness
    file.write('\n[SYS_LIVENESS]\n')
    if target_reachability:
        file.write('c = 1\n')

    stri = ''
    if belief_liveness >= sensor_uncertainty:
        for st in range(belief_nrows*belief_ncols):
            stri += 'st = {}'.format(st)
            if st != range(belief_nrows*belief_ncols)[-1]:
                stri += ' \\\\/ '
        for b in beliefcombs:
            beliefset = set()
            for beliefstate in b:
                beliefset = beliefset.union(partitionGrid[beliefstate])
            beliefset = beliefset - set(gw.targets[0])
            stri1 = ' \\\\/ (st = {} /\\\\ ('.format(belief_ncols*belief_nrows + beliefcombs.index(b))
            count = 0
            for x in allowed_states:
                truebelief = beliefset.intersection(invisibilityset[x])
                if len(truebelief) <= belief_liveness:
                    if count > 0:
                        stri1 += ' \\\\/ '
                    stri1 += ' s = {} '.format(nonbeliefstates.index(x))
                    count += 1
            stri1 += '))'
            if count > 0 and count < len(allowed_states):
                stri += stri1
            if count == len(allowed_states):
                stri += ' \\\\/ st = {}'.format(belief_ncols*belief_nrows + beliefcombs.index(b))
        stri += ' \\\\/ st = {}'.format(allstates[-1])
        # Target can leave the region.
        repeat =set()
        for t in set(nonbeliefstates) - set(allowed_states):
            all_belief_gridstate_sets = [i for i in sensor_uncertain_dict.values() if t in i]
            for belief_gridstates in all_belief_gridstate_sets:
                next_belief_gridstate = sensor_uncertain_dict.keys()[
                    sensor_uncertain_dict.values().index(belief_gridstates)]
                if next_belief_gridstate not in repeat:
                    stri += '\\\\/ st\' = {}'.format(next_belief_gridstate)
                    repeat.add(next_belief_gridstate)
        # stri = stri[:-3]
        stri += '\n'
        file.write(stri)

    ##################### Some Suda Stuff ###################
    # file.write('\n[ENV_LIVENESS]\n')
    # for t in targets:
    #     file.write('st = {}\n'.format(t+1))
    ##################### Some Suda Stuff ###################