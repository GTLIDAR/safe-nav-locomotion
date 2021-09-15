from gridworld_fine_auto_spec import *
# import write_structured_slugs_rss_fine_auto_spec_more_turns_no_cross
# import write_structured_slugs_JRNL_boundary
# import write_structured_slugs_JRNL_boundary_stair_mod
# import write_structured_slugs_fine_staight
# import write_structured_slugs_fine_staight_nondeterministic
# import write_structured_slugs_fine_staight_nondeterministic_every_step
# import write_structured_slugs_fine_staight_nondeterministic_sagital
import write_structured_slugs_fine_staight_nondeterministic_sagital_once
import compute_all_vis
import cv2
# import visibility
import os
import subprocess
import time
import copy
import cPickle as pickle
from tqdm import *
import simulateController_fine_abs as Simulator
import itertools
import Control_Parser
import json
import datetime

if __name__ == '__main__':
    then = time.time()
    print 'time: ' + str(datetime.datetime.now().time())
    mapname_fine = 'fine_abstraction'
    scale = (int(40*2.8),40)
    rownum_f = 26
    rownum_f = 26

    filename_f = 'figures/'+mapname_fine+'.png'
    image_f = cv2.imread(filename_f, cv2.IMREAD_GRAYSCALE)
    image_f = cv2.resize(image_f,dsize=(rownum_f,rownum_f),interpolation=cv2.INTER_AREA)
    h_f, w_f = image_f.shape[:2]
    
    folder_locn = 'Examples/'
    example_name = 'Belief_Evasion_fine_abstraction_straight'
    example_name = 'Belief_Evasion_fine_abstraction_nondeterministic_last_step_of_turn_no_short'
    example_name = 'Belief_Evasion_fine_abstraction_nondeterministic_sagital_once'
    jsonfile_name = folder_locn + "Integration/" + example_name + ".json"
    trial_name = folder_locn + example_name
    version = '01'
    slugs = '../../slugs-master/src/slugs' # Path to slugs
    save_to_Gazebo = False
    outfile = trial_name + '.json'
    infile = copy.deepcopy(trial_name)
    gwfile = folder_locn + '/figs/gridworldfig_' + example_name + '.png'
    target_vis_file = trial_name + '.txt'
    nagents = 1
    targets = [[]]

    initial_f = [351]
    # initial_f = [264]
    # initial_f = [210]
    moveobstacles_f = [0]
    PUDO_t = [3,27,45,21,24]

    filename_f = [filename_f,(rownum_f,rownum_f),cv2.INTER_AREA]
    gwg_f = Gridworld(filename_f,nagents=nagents, targets=targets, initial_f=initial_f, moveobstacles_f=moveobstacles_f)
    gwg_f.colorstates = [set(), set()]
    gwg_f.render()
    gwg_f.save(gwfile)
    partition = dict()
    allowed_states = [[None]] * nagents
    pg = [[None]]*nagents
   
    allowed_states[0] = list(set(gwg_f.states) - set(gwg_f.obstacles))

    pg[0] = {0:allowed_states[0]}
 
        
    #########################################################
    # print('pg: ' + str(pg[0]))
    visdist = [12,20,3500,3500]
    target_vis_dist = 2
    vel = [1,2,2,2]
    invisibilityset = []
    sensor_uncertainty = 1
    filename_f = []

    for n in [0]:
        # obj is a list of states that are static obstacle cells
        obj = compute_all_vis.img2obj(image_f)
        # compute visibility for each state
        # iset is a dict that has all the states listed that are not visible from the current state
        if len(gwg_f.obstacles)>0:
            iset = compute_all_vis.compute_visibility_for_all(obj, h_f, w_f, radius=visdist[n])
        else:
            iset = {}
            for state in gwg_f.states:
                iset[state] = set()
        invisibilityset.append(iset)
        outfile = trial_name+'agent'+str(n) +'.json'
        filename_f.append(outfile)
        print 'output file: ', outfile
        print 'input file name:', infile

        # write_structured_slugs_fine_staight_nondeterministic_every_step.write_to_slugs_part_dist(infile, gwg_f, initial_f[n], moveobstacles_f[0], iset, PUDO_targets = PUDO_t,
        #                                                            visdist =  visdist[n], allowed_states = allowed_states[n],
        #                                                            partitionGrid = pg[n])
        # write_structured_slugs_JRNL_cross_causal.write_to_slugs_part_dist(infile, gwg_f, initial_f[n], moveobstacles_f[0], iset, PUDO_targets = PUDO_t,
        #                                                            visdist =  visdist[n], allowed_states = allowed_states[n],
        #                                                            partitionGrid = pg[n])
        # write_structured_slugs_fine_staight_nondeterministic_sagital.write_to_slugs_part_dist(infile, gwg_f, initial_f[n], moveobstacles_f[0], iset, PUDO_targets = PUDO_t,
        #                                                            visdist =  visdist[n], allowed_states = allowed_states[n],
        #                                                            partitionGrid = pg[n])

        write_structured_slugs_fine_staight_nondeterministic_sagital_once.write_to_slugs_part_dist(infile, gwg_f, initial_f[n], moveobstacles_f[0], iset, PUDO_targets = PUDO_t,
                                                                   visdist =  visdist[n], allowed_states = allowed_states[n],
                                                                   partitionGrid = pg[n])
        
        bf_conv = time.time()
        print('Writing specifications took ', bf_conv - then, ' seconds')

        print ('Converting input file...')
        os.system('python compiler.py ' + infile + '.structuredslugs > ' + infile + '.slugsin')
        bf_syn = time.time()
        print('Converting to .slugsin took ', bf_syn - bf_conv, ' seconds')
        print('Computing controller...')
        sp = subprocess.Popen(slugs + ' --explicitStrategy --jsonOutput ' + infile + '.slugsin > ' + outfile,
                                  shell=True, stdout=subprocess.PIPE)
        sp.wait()

    now = time.time()
    print('Total synthesis took ', now - then, ' seconds')
    print('Actual synthesis took ', now - bf_syn, ' seconds')

    Simulator.userControlled_partition(filename_f[0], gwg_f, pg[0], moveobstacles_f, invisibilityset, jsonfile_name)