from gridworld_JRNL_color import *
import write_structured_slugs_rss_coarse_quad
import write_structured_slugs_rss_coarse_grid_stair_cdc_JRNL_CRT
import compute_all_vis
import cv2
# import visibility
import os
import subprocess
import time
import copy
import cPickle as pickle
from tqdm import *
import simulateController_colors_quad as Simulator
import itertools
import Control_Parser
import json
import datetime
from numpy import random

if __name__ == '__main__':


    
    print 'time: ' + str(datetime.datetime.now().time())
    ######     1) Choose Environment input figure name:     #####
    # mapname_coarse = 'BeliefEvasion_jrnl_CRT3'
    # rownum_c = 7
    # colnum_c = 12

    mapname_coarse = 'Cooperation' # also 'Cooperation_flipped' and 'Cooperation_even'
    rownum_c = 7
    colnum_c = 13
    

    mapname_coarse = 'Cooperation_sense' # ['Cooperation', 'Cooperation_sense','Cooperation_chain'] 
    rownum_c = 7
    colnum_c = 13

    #####     2) pick initial location for robot and dynamic obstacle, pick goal locations     #####
    # NOTE: for now we do assume that the first target is always reachable, if any are blocked it is the second target
    # this is because when resynthesizing, we keep the first target and change the second
    # TODO is to make this more robust (perhaps setting both to prev?)

    # initial_c = [52]
    # moveobstacles_c = [38]
    # PUDO_t_c = [53,45]

    # initial_c = [56]
    # moveobstacles_c = [61]
    # PUDO_t_c = [54,49]


    #coop original
    # initial_c_orig = [71]
    # moveobstacles_c_orig = [14]
    # PUDO_t_c_quad_orig = [54, 49]
    # PUDO_t_c_cassie_orig = [58, 28]

    # sense
    initial_c_orig = [71]
    moveobstacles_c_orig = [14]
    PUDO_t_c_quad_orig = [28, 58]
    PUDO_t_c_cassie_orig = [41, 49]
    random.seed(427052) # this seeds to specifically get the behavior for the 'sense' case study shown in the paper, uncomment for true rng in obstacle sensing

    # chain
    # initial_c_orig = [58]
    # moveobstacles_c_orig = [49]
    # PUDO_t_c_quad_orig = [22, 76]
    # PUDO_t_c_cassie_orig = [41, 45]
    # random.seed(420) # this seeds to specifically get the behavior for the 'chain' case study shown in the paper, uncomment for true rng in obstacle sensing


    # coop, quad resolves
    # initial_c_orig = [71]
    # moveobstacles_c_orig = [14]
    # PUDO_t_c_quad_orig = [58, 28]
    # PUDO_t_c_cassie_orig = [54, 49]

    # #coop troubleshooting
    # initial_c_orig = [47]
    # moveobstacles_c_orig = [54]
    # PUDO_t_c_quad_orig = [54, 49]
    # PUDO_t_c_cassie_orig = [58, 28]

    # # flipped
    # initial_c_orig = [71]
    # moveobstacles_c_orig = [66]
    # PUDO_t_c_quad_orig = [40, 49]
    # PUDO_t_c_cassie_orig = [32, 62]

    # # flipped troubleshooting
    # initial_c = [32]
    # moveobstacles_c = [42]
    # PUDO_t_c_quad = [28, 54]
    # PUDO_t_c_cassie = [32, 43]

    # even
    # initial_c_orig = [72]
    # moveobstacles_c_orig = [66]
    # PUDO_t_c_quad_orig = [28, 48]
    # PUDO_t_c_cassie_orig = [60, 36]

    # even troubleshooting
    # initial_c = [47]
    # moveobstacles_c = [44]
    # PUDO_t_c_quad = [28, 29]
    # PUDO_t_c_cassie = [60, 45]



    need_synthesis = []#[0, 1, 2, 3, 4, 5, 6, 7]
    run_mode = 0 # 0 = original, 1 = obstacle resolution, -1 = other error
    capabilities = {
        'quad':['fly', 'sense'],
        'cassie':['push', 'grab']
    }
    pending_cassie_PUDO = []
    pending_quad_PUDO = []

    initial_c = copy.deepcopy(initial_c_orig)
    moveobstacles_c = copy.deepcopy(moveobstacles_c_orig)
    PUDO_t_c_quad = copy.deepcopy(PUDO_t_c_quad_orig)
    PUDO_t_c_cassie = copy.deepcopy(PUDO_t_c_cassie_orig)

    # initial cassie orientation/direction for automaton
    cassie_orientation = 1
    cassie_request = 2
    quad_orientation = 1
    quad_request = 2

    # gridworld that synthesis sees
    filename_c = 'figures/'+mapname_coarse+'.png'
    image_c = cv2.imread(filename_c, cv2.IMREAD_GRAYSCALE)
    image_c = cv2.resize(image_c,dsize=(colnum_c,rownum_c),interpolation=cv2.INTER_AREA)
    h_c, w_c = image_c.shape[:2]

    # actual gridworld environment
    filename_a = 'figures/'+mapname_coarse+'_actual.png'
    image_a = cv2.imread(filename_a, cv2.IMREAD_GRAYSCALE)
    image_a = cv2.resize(image_a,dsize=(colnum_c,rownum_c),interpolation=cv2.INTER_AREA)
    h_a, w_a = image_a.shape[:2]
    
    folder_locn = 'Examples/'
    example_name_quad = mapname_coarse + '_quad'#'Belief_Evasion_coarse_quad'
    example_name_cassie = mapname_coarse + '_cassie'#'Belief_Evasion_coarse_quad'
    jsonfile_name_quad = folder_locn + "Integration/" + example_name_quad + ".json"
    jsonfile_name_cassie = folder_locn + "Integration/" + example_name_cassie + ".json"
    trial_name_quad = folder_locn + example_name_quad
    trial_name_cassie = folder_locn + example_name_cassie
    version = '01'
    slugs = '../../slugs-master/src/slugs' # Path to slugs
    save_to_Gazebo = False
    outfile_quad = trial_name_quad + '.json'
    outfile_cassie = trial_name_cassie + '.json'
    
    gwfile_quad = folder_locn + '/figs/gridworldfig_' + example_name_quad + '.png'
    gwfile_cassie = folder_locn + '/figs/gridworldfig_' + example_name_cassie + '.png'
    target_vis_file_quad = trial_name_quad + '.txt'
    target_vis_file_cassie = trial_name_cassie + '.txt'
    nagents = 1
    targets = [[]]

    filename_c = [filename_c,(colnum_c,rownum_c),cv2.INTER_AREA]
    known_additional_obs = []
     
    # actual gridworld environment
    filename_a = [filename_a,(colnum_c,rownum_c),cv2.INTER_AREA]
    gwg_a = Gridworld(filename_a,nagents=nagents, targets=targets, initial=initial_c, moveobstacles=moveobstacles_c)
    gwg_a.colorstates = [set(), set()]
    f = 0

    while run_mode != -1:
        # gridworld that synthesis sees
        gwg_c = Gridworld(filename_c,nagents=nagents, targets=targets, initial=initial_c, moveobstacles=moveobstacles_c, obstacles=known_additional_obs)
        
        gwg_c.colorstates = [set(), set()]
        gwg_c.render()
        gwg_c.save(gwfile_quad)
        gwg_c.save(gwfile_cassie)
        partition = dict()
        allowed_states = [[None]] * nagents
        pg = [[None]]*nagents
        # allowed_states[0] = list(set(gwgfine.states) - set(gwgfine.obstacles)-set(gwgfine.obsborder))
        allowed_states[0] = list(set(gwg_c.states) - set(gwg_c.obstacles))

        #####     3) Pick belief state partitions     #####
        pg[0] = {0:set(allowed_states[0])}
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
        #filename_c = []


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
            infile_quad = copy.deepcopy(trial_name_quad) + '_' + str(f)
            infile_cassie = copy.deepcopy(trial_name_cassie) + '_' + str(f)
            outfile_quad = trial_name_quad+'agent'+str(n) +'_' + str(f)+'.json'
            outfile_cassie = trial_name_cassie+'agent'+str(n) +'_' + str(f)+'.json'
            print 'output files: ', outfile_quad, ', ', outfile_cassie
            print 'input file names:', infile_quad, ', ', infile_cassie

            if f in need_synthesis:
                then = time.time()

                write_structured_slugs_rss_coarse_quad.write_to_slugs_part_dist(infile_quad, gwg_c, initial_c[n], moveobstacles_c[0], iset, PUDO_targets = PUDO_t_c_quad,
                                                                           visdist =  visdist[n], allowed_states = allowed_states[n],
                                                                           partitionGrid = pg[n], orientation = quad_orientation, direction_request = quad_request)

                write_structured_slugs_rss_coarse_grid_stair_cdc_JRNL_CRT.write_to_slugs_part_dist(infile_cassie, gwg_c, initial_c[n], moveobstacles_c[0], iset, PUDO_targets = PUDO_t_c_cassie,
                                                                           visdist =  visdist[n], allowed_states = allowed_states[n],
                                                                           partitionGrid = pg[n], orientation = cassie_orientation, direction_request = cassie_request)
                
                noww = time.time()
                print('Writing specifications took ', noww - then, ' seconds')

                print ('Converting input files...')
                os.system('python compiler.py ' + infile_quad + '.structuredslugs > ' + infile_quad + '.slugsin')
                os.system('python compiler.py ' + infile_cassie + '.structuredslugs > ' + infile_cassie + '.slugsin')
                print('Computing controllers...')
                # sp = subprocess.Popen(slugs + ' --explicitStrategy --jsonOutput ' + infile + '.slugsin > ' + outfile,
                #                           shell=True, stdout=subprocess.PIPE)
                # sp = subprocess.Popen(slugs + ' --explicitStrategy --fixedPointRecycling --jsonOutput ' + infile + '.slugsin > ' + outfile,
                #                           shell=True, stdout=subprocess.PIPE)
                sp_quad = subprocess.Popen(slugs + ' --biasForAction --explicitStrategy --jsonOutput ' + infile_quad + '.slugsin > ' + outfile_quad,
                                          shell=True, stdout=subprocess.PIPE)

                sp_cassie = subprocess.Popen(slugs + ' --biasForAction --explicitStrategy --jsonOutput ' + infile_cassie + '.slugsin > ' + outfile_cassie,
                                          shell=True, stdout=subprocess.PIPE)
                                          
                # sp = subprocess.Popen(slugs + ' --counterStrategy ',
                #                           shell=True, stdout=subprocess.PIPE)

                # --computeInterestingRunOfTheSystem

                                          
                sp_quad.wait()
                sp_cassie.wait()
                


        
        now = time.time()
        if f in need_synthesis:
            print('Total synthesis took ', now - then, ' seconds')
            print('Actual synthesis took ', now - noww, ' seconds')

        f = f + 1

        cassie_orientation, cassie_request, quad_orientation, quad_request, prev_cassie, prev_quad, new_obs = Simulator.userControlled_partition(outfile_cassie, gwg_a, pg[0], moveobstacles_c, invisibilityset, example_name_cassie + '.json', outfile_quad)
        known_additional_obs.extend(new_obs)

        bad_state = gwg_a.physicalViolation()
        print("Bad state: ", bad_state)
        if bad_state != -1:
            # if it's a physical Violation, resolve
            # first, save what the previous objective was, will pop later once current violation is resolved
            pending_cassie_PUDO.append(copy.deepcopy(PUDO_t_c_cassie))
            pending_quad_PUDO.append(copy.deepcopy(PUDO_t_c_quad))
            #run_mode = 1 this no longer necessary with push/pop method
            print(gwg_a.current)
            # change PUDO targets to resolve obstacles
            # for now, move quad/cassie one space if it's the one causing the violation (hand tuned atm)
            if gwg_a.resolution[bad_state]['action'] in capabilities['quad']:
                print("Obstacle Resolvable by Quadcopter")
                PUDO_t_c_quad[1] = gwg_a.resolution[bad_state]['state'][0]
                #PUDO_t_c_cassie[1] = PUDO_t_c_cassie[0]+1
                PUDO_t_c_cassie = [prev_cassie, prev_cassie]
                gwg_a.current[0] = prev_cassie
                print("old cassie positions", cassie_orientation, cassie_request)
                cassie_orientation = (cassie_orientation + 1) % 4
                if cassie_request != 0:
                    cassie_request = ((cassie_request) % 4) + 1
                print("new cassie positions", cassie_orientation, cassie_request)

            elif gwg_a.resolution[bad_state]['action'] in capabilities['cassie']:
                print("Obstacle Resolvable by Cassie")
                #PUDO_t_c_quad[1] = PUDO_t_c_quad[0]+1#gwg_a.moveobstacles[0] - (2)
                PUDO_t_c_quad = [prev_quad, prev_quad]
                PUDO_t_c_cassie[1] = gwg_a.resolution[bad_state]['state'][0]
                gwg_a.moveobstacles[0] = prev_quad
                print("old quad positions", quad_orientation, quad_request)
                quad_orientation = (quad_orientation + 1) % 4
                if quad_request != 0:
                    quad_request = ((quad_request) % 4) +1
                print("new quad positions", quad_orientation, quad_request)
            else:
                run_mode = -1
                print("ERROR: NO AGENT HAS CAPABILITY OF RESOLVING OBSTACLE")


        else:
            # if not, either a separate error occurred (in which case the code has failed elsewhere) or an obstacle has been resolved, return to previous goal
            # run_mode = 0 this no longer necessary with push/pop method

            PUDO_t_c_quad = pending_quad_PUDO.pop()
            PUDO_t_c_cassie = pending_cassie_PUDO.pop()

        # update initial states to current states
        initial_c = gwg_a.current
        moveobstacles_c = gwg_a.moveobstacles
        #then go back to beginning of loop and synthesize new solution

        print("Cassie is at ", initial_c)
        print("Quadcopter is at ", moveobstacles_c)
        print("Cassie's targets are now ", PUDO_t_c_cassie)
        print("Quad's targets are now ", PUDO_t_c_quad)

