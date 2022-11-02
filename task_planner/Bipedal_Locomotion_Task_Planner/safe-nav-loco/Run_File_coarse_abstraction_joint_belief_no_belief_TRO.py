from gridworld_multi_obs_TRO import *
# from gridworld_multi_obs import *
# import write_structured_slugs_rss_coarse_grid_multi_obs
import write_structured_slugs_no_belief_TRO
# import write_structured_slugs_belief_TRO
# import write_structured_slugs_rss_coarse_grid_multi_obs_joint_belief_purturb
import compute_all_vis
import cv2
# import visibility
import os
import subprocess
import time
import copy
import cPickle as pickle
from tqdm import *
# import simulateController_multi_obs as Simulator
# import simulateController_2labs_stairN_color_multiObs as Simulator
import simulateController_colors_belief as Simulator
import itertools
import Control_Parser
import json
import datetime

if __name__ == '__main__':
    then = time.time()
    print 'time: ' + str(datetime.datetime.now().time())
    ######     1) Choose Environment input figure name:     #####

    # mapname_coarse = 'MAP30'
    # rownum_c = 10
    # colnum_c = 5

    # mapname_coarse = 'BeliefEvasion_jrnl_CRT3'
    # rownum_c = 7
    # colnum_c = 12

    mapname_coarse = 'TRO'
    rownum_c = 5
    colnum_c = 6

    # mapname_coarse = 'KLAUS'
    # rownum_c = 6
    # colnum_c = 8


    # mapname_coarse = 'CCB'
    # rownum_c = 8
    # colnum_c = 8


    #####     2) pick initial location for robot and dynamic obstacle, pick goal locations     #####
    # initial_c = [63]
    # moveobstacles_c = [28,27,27]
    # PUDO_t_c = [31,45]

    # initial_c = [12]
    # moveobstacles_c = [38,38]
    # # moveobstacles_c = [61]
    # PUDO_t_c = [37,12]

    initial_c = [56]
    moveobstacles_c = [61,62]
    # moveobstacles_c = [61]
    PUDO_t_c = [57,49]

    #initial_c = [21]
    #moveobstacles_c = [19,20]
    # moveobstacles_c = [61]
    #PUDO_t_c = [16,13]

    
    # moveobstacles_c = [19,20]
    # # moveobstacles_c = [61]
    # PUDO_t_c = [30,25]
    # initial_c = [37]
    # moveobstacles_c = [33,34]

    moveobstacles_c = [9,10]
    # moveobstacles_c = [61]
    PUDO_t_c = [30,25]
    initial_c = [21]
    



    filename_c = 'figures/'+mapname_coarse+'.png'
    image_c = cv2.imread(filename_c, cv2.IMREAD_GRAYSCALE)
    image_c = cv2.resize(image_c,dsize=(colnum_c,rownum_c),interpolation=cv2.INTER_AREA)
    h_c, w_c = image_c.shape[:2]
    
    folder_locn = 'Examples/'
    example_name = 'TRO_ENV_New_GW2'
    example_name = 'TRO_ENV_New_GW_KLAUS'
    
    example_name = 'TRO_ENV_New_GW_CCB'
    
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


   # pg[0] = {0:(set(allowed_states[0])-set([37,38,39,49,50,53,54,61,62,63,64,65,66])),1:set([37,38,49,50]),2:set([61,62]),3:set([39]),4:set([63,64]),5:set([53,54,65,66])}
    #pg[0] = {0:(set(allowed_states[0])-set([17,18,19,25,26,28,29,30,33,34,35,36,37,38])),1:set([17,18,25,26]),2:set([33,34]),3:set([19]),4:set([35]),5:set([28,29,30,36,37,38])}

 
    visdist = [20,20,3500,3500]
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

        write_structured_slugs_no_belief_TRO.write_to_slugs_part_dist(infile, gwg_c, initial_c[n], moveobstacles_c, iset, PUDO_targets = PUDO_t_c,
                                                                   visdist =  visdist[n], allowed_states = allowed_states[n],
                                                                   partitionGrid = pg[n])
        
        noww = time.time()
        print('Writing specifications took ', noww - then, ' seconds')

        print ('Converting input file...')
        os.system('python compiler.py ' + infile + '.structuredslugs > ' + infile + '.slugsin')
        print('Computing controller...')

        nowww = time.time()
        print('Conversion took ', nowww - noww, ' seconds')
        # sp = subprocess.Popen(slugs + ' --explicitStrategy --jsonOutput ' + infile + '.slugsin > ' + outfile,
        #                           shell=True, stdout=subprocess.PIPE)
        # sp = subprocess.Popen(slugs + ' --explicitStrategy --fixedPointRecycling --jsonOutput ' + infile + '.slugsin > ' + outfile,
        #                           shell=True, stdout=subprocess.PIPE)
        sp = subprocess.Popen(slugs + ' --biasForAction --explicitStrategy --jsonOutput ' + infile + '.slugsin > ' + outfile,
                                  shell=True, stdout=subprocess.PIPE)
        sp.wait()

    now = time.time()
    print('Total synthesis took ', now - then, ' seconds')
    print('Actual synthesis + conversion took ', now - noww, ' seconds')
    print('Actual synthesis took ', now - nowww, ' seconds')

    Simulator.userControlled_partition(filename_c[0], gwg_c, pg[0], moveobstacles_c, invisibilityset, jsonfile_name)