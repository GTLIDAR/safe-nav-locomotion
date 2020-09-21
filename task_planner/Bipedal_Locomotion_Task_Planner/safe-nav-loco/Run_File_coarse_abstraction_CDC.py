from gridworld import *
import write_structured_slugs_rss_coarse_grid_stair_cdc
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

if __name__ == '__main__':
    then = time.time()
    print 'time: ' + str(datetime.datetime.now().time())
    ######     1) Choose Environment input figure name:     #####
    mapname_coarse = 'BeliefEvasion_CDC'
    rownum_c = 7
    colnum_c = 13


    #####     2) pick initial location for robot and dynamic obstacle, pick goal locations     #####
    initial_c = [62]
    moveobstacles_c = [28]
    PUDO_t_c = [37,53]


    filename_c = 'figures/'+mapname_coarse+'.png'
    image_c = cv2.imread(filename_c, cv2.IMREAD_GRAYSCALE)
    image_c = cv2.resize(image_c,dsize=(colnum_c,rownum_c),interpolation=cv2.INTER_AREA)
    h_c, w_c = image_c.shape[:2]
    
    folder_locn = 'Examples/'
    example_name = 'Belief_Evasion_coarse_cdc_map'
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

    #####     3) Pick belief state partitions     #####
    pg[0] = {0:allowed_states[0]}

    # pg[0] = {0: set.union(*[set(range(0,24))])  - set(gwg_c.obstacles), 1: set.union(*[set(range(27,37))])  - set(gwg_c.obstacles), 2: set.union(*[set(range(40,50))])  - set(gwg_c.obstacles),
    		#  3: set.union(*[set(range(53,63))])  - set(gwg_c.obstacles), 4: set.union(*[set(range(66,76))])  - set(gwg_c.obstacles)}
    
 
    visdist = [4,20,3500,3500]
    target_vis_dist = 2
    vel = [1,2,2,2]
    invisibilityset = []
    sensor_uncertainty = 1
    filename_c = []

    for n in [0]:
        # obj is a list of states that are static obstacle cells
        obj = compute_all_vis.img2obj(image_c)
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

        write_structured_slugs_rss_coarse_grid_stair_cdc.write_to_slugs_part_dist(infile, gwg_c, initial_c[n], moveobstacles_c[0], iset, PUDO_targets = PUDO_t_c,
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
    print('Total synthesis took ', now - then, ' seconds')
    print('Actual synthesis took ', now - noww, ' seconds')

    Simulator.userControlled_partition(filename_c[0], gwg_c, pg[0], moveobstacles_c, invisibilityset, jsonfile_name)