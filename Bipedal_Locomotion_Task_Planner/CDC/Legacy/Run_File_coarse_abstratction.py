from gridworld import *
import argparse
# import write_structured_slugs_copy_2
# import write_structured_slugs_past_action_foot_stance_MP_specs_step_height_23_38_no_step_over
import write_structured_slugs_rss_coarse_grid_stair
import compute_all_vis
import cv2
# import visibility
import os
import subprocess
import time
import copy
import cPickle as pickle
from tqdm import *
import simulateController as Simulator
import itertools
import Control_Parser
import json
import datetime
def parseArguments():
    #### From --> https://stackoverflow.com/questions/28479543/run-python-script-with-some-of-the-argument-that-are-optional
    #### EVEN BETTER --> https://pymotw.com/2/argparse/
    # # Create argument parser
    parser = argparse.ArgumentParser()

    # # Positional mandatory arguments
    # parser.add_argument("SynthesisFlag", help="Include this boolean to run the synthesis", type=bool)

    #  # Optional arguments
    parser.add_argument("-synF", action='store_true', default=False, dest='synFlag',
                        help="Include this boolean to run the synthesis")
    parser.add_argument("-cvF", action='store_true', default=False, dest='visFlag',
                        help="Include this boolean to compute belief visibility of the target")
    parser.add_argument("-noVisF", action='store_false', default=True, dest='noVisFlag',
                        help="Include this boolean to run synthesis without any target vision")

    # # Parse arguments
    args = parser.parse_args()

    return args


if __name__ == '__main__':

    args = parseArguments()
    print 'Synthesis Flag is: ', args.synFlag
    if not args.noVisFlag:
        print '--> Target has No vision'
    if args.noVisFlag and args.synFlag:
        print 'Compute Vision Flag is: ', args.visFlag

    then = time.time()
    print 'time: ' + str(datetime.datetime.now().time())
    ######     1) Choose Environment input figure name:     #####
    mapname_fine = 'fine_abstraction'
    scale = (int(40*2.8),40)
    rownum_f = 7
    colnum_f = 7

    # mapname_coarse = 'BeliefEvasion_coarse_14_10'
    # mapname_coarse = 'BelieEvasion_38_23_Extra_obs'
    mapname_coarse = 'BeliefEvasion_CRT'
    # mapname_coarse = 'BelieEvasionFifteen_w'
    rownum_c = 28
    colnum_c = 42
    rownum_c = 23
    colnum_c = 38
    rownum_c = 8
    colnum_c = 12
    # rownum_c = 10
    # colnum_c = 14
    rownum_c = 5
    colnum_c = 8



    filename_f = 'figures/'+mapname_fine+'.png'
    image_f = cv2.imread(filename_f, cv2.IMREAD_GRAYSCALE)
    image_f = cv2.resize(image_f,dsize=(colnum_f,rownum_f),interpolation=cv2.INTER_AREA)
    h_f, w_f = image_f.shape[:2]

    filename_c = 'figures/'+mapname_coarse+'.png'
    image_c = cv2.imread(filename_c, cv2.IMREAD_GRAYSCALE)
    image_c = cv2.resize(image_c,dsize=(colnum_c,rownum_c),interpolation=cv2.INTER_AREA)
    h_c, w_c = image_c.shape[:2]
    
    folder_locn = 'Examples/'
    example_name = 'Belief_Evasion_coarse_crt_test'
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

    #####     4) pick initial location for robot and dynamic obstacle, pick goal locations     #####

    # 7_7
    initial_fine = [24]
    moveobstacles_fine = [0]
    PUDO_t = [3,27,45,21]

    # initial_c = [812]
    # moveobstacles_c = [191]
    # PUDO_t_c = [408,611]
    initial_c = [74]
    moveobstacles_c = [15]
    PUDO_t_c = [34,61]

    # initial_c = [114]
    # moveobstacles_c = [17]
    # PUDO_t_c = [79,85]

    #CRT
    initial_c = [17]
    moveobstacles_c = [11]
    PUDO_t_c = [13,9]

    filename_c = [filename_c,(colnum_c,rownum_c),cv2.INTER_AREA]
    gwg_c = Gridworld(filename_c,nagents=nagents, targets=targets, initial=initial_c, moveobstacles=moveobstacles_c)
    gwg_c.colorstates = [set(), set()]
    gwg_c.render()
    gwg_c.save(gwfile)
    partition = dict()
    allowed_states = [[None]] * nagents
    pg = [[None]]*nagents
    # allowed_states[0] = list(set(gwgfine.states) - set(gwgfine.obstacles)-set(gwgfine.obsborder))
    allowed_states[0] = list(set(gwg_c.states) - set(gwg_c.obstacles))

    #####     5) Pick belief state partitions     #####
    pg[0] = {0:allowed_states[0]}
 
        
    #########################################################
    # print('pg: ' + str(pg[0]))
    visdist = [12,20,3500,3500]
    target_vis_dist = 2
    vel = [1,2,2,2]
    invisibilityset = []
    sensor_uncertainty = 1
    filename_c = []

    for n in [0]:
        # obj is a list of states that are static obstacle cells
        obj = compute_all_vis.img2obj(image_c)
        # compute visibility for each state
        # iset is a dict that has all the states listed that are not visible from the current state
        if len(gwg_c.obstacles)>0:
            iset = compute_all_vis.compute_visibility_for_all(obj, h_c, w_c, radius=visdist[n])
        else:
            iset = {}
            for state in gwg_c.states:
                iset[state] = set()
        invisibilityset.append(iset)
        outfile = trial_name+'agent'+str(n) +'.json'
        filename_c.append(outfile)
        print 'output file: ', outfile
        print 'input file name:', infile

        #####     6) chose specification writing file     #####

        # write_structured_slugs_past_action_foot_stance_MP_specs_step_height_23_38_no_step_over.write_to_slugs_part_dist(infile, gwg, initial[n], moveobstacles[0], iset, PUDO_targets = PUDO_t,
        #                                                            visdist =  visdist[n], allowed_states = allowed_states[n],
        #                                                            partitionGrid = pg[n])

        write_structured_slugs_rss_coarse_grid_stair.write_to_slugs_part_dist(infile, gwg_c, initial_c[n], moveobstacles_c[0], iset, PUDO_targets = PUDO_t_c,
                                                                   visdist =  visdist[n], allowed_states = allowed_states[n],
                                                                   partitionGrid = pg[n])
        
        noww = time.time()
        print('Writing specifications took ', noww - then, ' seconds')

        print ('Converting input file...')
        os.system('python compiler.py ' + infile + '.structuredslugs > ' + infile + '.slugsin')
        print('Computing controller...')
        sp = subprocess.Popen(slugs + ' --explicitStrategy --jsonOutput ' + infile + '.slugsin > ' + outfile,
                                  shell=True, stdout=subprocess.PIPE)
        sp.wait()

    now = time.time()
    print('Synthesis took ', now - then, ' seconds')
    print('Actual synthesis took ', now - noww, ' seconds')

    Simulator.userControlled_partition(filename_c[0], gwg_c, pg[0], moveobstacles_c, invisibilityset, jsonfile_name)