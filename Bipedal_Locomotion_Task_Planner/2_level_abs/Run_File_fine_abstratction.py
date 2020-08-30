from gridworld_fine import *
import argparse
# import write_structured_slugs_copy_2
# import write_structured_slugs_past_action_foot_stance_MP_specs_step_height_23_38_no_step_over
import write_structured_slugs_rss_fine_grid_turn_across_border
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
    rownum_f = 7

    mapname_coarse = 'BelieEvasion_38_23_Extra_obs'
    rownum_c = 7
    colnum_c = 7



    filename_f = 'figures/'+mapname_fine+'.png'
    image_f = cv2.imread(filename_f, cv2.IMREAD_GRAYSCALE)
    image_f = cv2.resize(image_f,dsize=(rownum_f,rownum_f),interpolation=cv2.INTER_AREA)
    h_f, w_f = image_f.shape[:2]

    filename_c = 'figures/'+mapname_coarse+'.png'
    image_c = cv2.imread(filename_c, cv2.IMREAD_GRAYSCALE)
    image_c = cv2.resize(image_c,dsize=(rownum_c,rownum_c),interpolation=cv2.INTER_AREA)
    h_c, w_c = image_c.shape[:2]
    
    folder_locn = 'Examples/'
    example_name = 'Belief_Evasion_fine_abstraction_stair_test_size_25'
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
    initial_f = [24]
    moveobstacles_f = [0]
    PUDO_t = [3,27,45,21,24]

    initial_c = [812]
    moveobstacles_c = [191]
    PUDO_t_c = [408,611]

    filename_f = [filename_f,(rownum_f,rownum_f),cv2.INTER_AREA]
    filename_c = [filename_c,(rownum_c,rownum_c),cv2.INTER_AREA]
    gwg_f = Gridworld(filename_f,nagents=nagents, targets=targets, initial_f=initial_f, moveobstacles_f=moveobstacles_f, filename_c = filename_c, initial_c=initial_c, moveobstacles_c=moveobstacles_c)
    gwg_f.colorstates = [set(), set()]
    gwg_f.render()
    gwg_f.save(gwfile)
    partition = dict()
    allowed_states = [[None]] * nagents
    pg = [[None]]*nagents
    # allowed_states[0] = list(set(gwg_f.states) - set(gwg_f.obstacles)-set(gwg_f.obsborder))
    allowed_states[0] = list(set(gwg_f.states) - set(gwg_f.obstacles))

    #####     5) Pick belief state partitions     #####
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

        #####     6) chose specification writing file     #####

        # write_structured_slugs_past_action_foot_stance_MP_specs_step_height_23_38_no_step_over.write_to_slugs_part_dist(infile, gwg, initial[n], moveobstacles[0], iset, PUDO_targets = PUDO_t,
        #                                                            visdist =  visdist[n], allowed_states = allowed_states[n],
        #                                                            partitionGrid = pg[n])

        write_structured_slugs_rss_fine_grid_turn_across_border.write_to_slugs_part_dist(infile, gwg_f, initial_f[n], moveobstacles_f[0], iset, PUDO_targets = PUDO_t,
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

    Simulator.userControlled_partition(filename_f[0], gwg_f, pg[0], moveobstacles_f, invisibilityset, jsonfile_name)