from gridworld_JRNL_color import *
# import write_structured_slugs_rss_coarse_quad_assignable
import write_structured_slugs_rss_coarse_quad_wait
import compute_all_vis
import cv2
# import visibility
import os
import subprocess
import time
import copy
import cPickle as pickle
from tqdm import *
import simulateController_colors as Simulator
import itertools
import Control_Parser
import json
import datetime

if __name__ == '__main__':
    then = time.time()
    print 'time: ' + str(datetime.datetime.now().time())
    ######     1) Choose Environment input figure name:     #####
    mapname_coarse = 'BeliefEvasion_jrnl_CRT3'
    rownum_c = 7
    colnum_c = 12

    mapname_coarse = 'Cooperation'
    rownum_c = 7
    colnum_c = 13

    #####     2) pick initial location for robot and dynamic obstacle, pick goal locations     #####
    # initial_c = [52]
    # moveobstacles_c = [38]
    # PUDO_t_c = [53,45]

    # initial_c = [56]
    # moveobstacles_c = [61]
    # PUDO_t_c = [54,49]

    initial_c = [71]
    moveobstacles_c = [14]
    PUDO_t_c = [41, 49]

    filename_c = 'figures/'+mapname_coarse+'.png'
    image_c = cv2.imread(filename_c, cv2.IMREAD_GRAYSCALE)
    image_c = cv2.resize(image_c,dsize=(colnum_c,rownum_c),interpolation=cv2.INTER_AREA)
    h_c, w_c = image_c.shape[:2]
    
    folder_locn = 'Examples/'
    example_name = 'Cooperation_quad_wait'#'Belief_Evasion_coarse_quad'
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
    
    # pg[0] = {0:(set(allowed_states[0])-set([12,23,24,25,34,35,37,38,39,45,46,47,48,49,50,56,57,61])),1:set([12,23,24,34,35]),2:set([45,46,56,57]),3:set([25]),4:set([47]),5:set([37,38,39,48,49,50,61])}
    # pg[0] = {0:(set(allowed_states[0])-set([12,23,24,25,34,35,37,38,39,45,46,47,48,49,50,56,57,61])),1:set([12,23,24,34,35]),2:set([45,46,56,57]),3:set([25]),4:set([47]),5:set([38,39,48,49,50,61])}

    # pg[0] = {0:(set(allowed_states[0])-set([12,23,24,25,34,35,37,38,39,45,46,47,48,49,50,56,57,61])),1:set([12,23,24,34,35]),2:set([45,46,56,57]),3:set([25]),4:set([47,48]),5:set([38,39,49,50,61])}
    
    # pg[0] = {0:(set(allowed_states[0])-set([34,35,36,45,56,48,49,56,57,58,59,60])),1:set([34,35,45,46]),2:set([56,57]),3:set([36]),4:set([58]),5:set([48,49,59,60])}
    #for crt hyb env 2
    # pg[0] = {0:(set(allowed_states[0])-set([34,35,36,45,46,56,48,49,56,57,58,59,60])),1:set([34,35,45,46]),2:set([56,57]),3:set([36]),4:set([58,59]),5:set([49,60,60,61])}
    #for crt hyb env 3:
    #pg[0] = {0:(set(allowed_states[0])-set([37,38,39,49,50,53,54,61,62,63,64,65,66])),1:set([37,38,49,50]),2:set([61,62]),3:set([39]),4:set([63,64]),5:set([53,54,65,66])}
    #for coop
    #pg[0] = {0:(set(allowed_states[0])-set([53, 54, 66, 67, 27, 28, 29, 40, 41, 42, 55, 56, 68, 69, 14, 15, 16, 57, 58, 70, 71])),1:set([53, 54, 66, 67]),2:set([27, 28, 29, 40, 41, 42]),3:set([55, 56, 68, 69]),4:set([14, 15, 16]),5:set([57, 58, 70, 71])}

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

        write_structured_slugs_rss_coarse_quad_wait.write_to_slugs_part_dist(infile, gwg_c, initial_c[n], moveobstacles_c[0], iset, PUDO_targets = PUDO_t_c,
                                                                   visdist =  visdist[n], allowed_states = allowed_states[n],
                                                                   partitionGrid = pg[n])
        
        noww = time.time()
        print('Writing specifications took ', noww - then, ' seconds')

        print ('Converting input file...')
        os.system('python compiler.py ' + infile + '.structuredslugs > ' + infile + '.slugsin')
        print('Computing controller...')
        # sp = subprocess.Popen(slugs + ' --explicitStrategy --jsonOutput ' + infile + '.slugsin > ' + outfile,
        #                           shell=True, stdout=subprocess.PIPE)
        # sp = subprocess.Popen(slugs + ' --explicitStrategy --fixedPointRecycling --jsonOutput ' + infile + '.slugsin > ' + outfile,
        #                           shell=True, stdout=subprocess.PIPE)
        sp = subprocess.Popen(slugs + ' --biasForAction --explicitStrategy --jsonOutput ' + infile + '.slugsin > ' + outfile,
                                  shell=True, stdout=subprocess.PIPE)
                                  
        # sp = subprocess.Popen(slugs + ' --counterStrategy ',
        #                           shell=True, stdout=subprocess.PIPE)

        # --computeInterestingRunOfTheSystem

                                  
        sp.wait()

    now = time.time()
    print('Total synthesis took ', now - then, ' seconds')
    print('Actual synthesis took ', now - noww, ' seconds')

    #Simulator.userControlled_partition(filename_c[0], gwg_c, pg[0], moveobstacles_c, invisibilityset, jsonfile_name)