from gridworld import *
import argparse
# import write_structured_slugs_copy_2
import write_structured_slugs_past_action_foot_stance_MP_specs_step_height_23_38_no_step_over
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
    # mapname = '3ne'
    # mapname = 'BeliefTestEvasion'
    # mapname = 'BelieEvasionTwenty'
    # mapname = 'BelieEvasionFifteen_w'
    # mapname = 'chicago4_45_2454_5673_map'
    mapname = 'BelieEvasion_64_30'
    # mapname = 'BelieEvasion_fifteen'
    # mapname = 'BelieEvasion_15_20_sparse_obs'
    scale = (int(40*2.8),40)
    # rownum = 15
    # colnum = 20
    rownum = 30
    colnum = 64
    # rownum = 15
    # colnum = 15
    filename = ['figures/' + mapname + '.pgm',scale,cv2.INTER_LINEAR_EXACT]

    filename = 'figures/'+mapname+'.png'
    image = cv2.imread(filename, cv2.IMREAD_GRAYSCALE)
    image = cv2.resize(image,dsize=(colnum,rownum),interpolation=cv2.INTER_AREA)
    h, w = image.shape[:2]
    folder_locn = 'Examples/'
    example_name = 'Jonas_Belief_Evasion_Terminal_foot_stance_vis_10_yingke_30_64_vis_12'
    jsonfile_name = example_name + ".json"
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
    # initial = [54]
    # moveobstacles = [47]
    # PUDO_t = [84,143]

    initial = [1076]
    moveobstacles = [75]
    # PUDO_t = [1162,818]
    PUDO_t = [842,818]

    # PUDO_t = [84,147]
    # PUDO_t = [189,53]

    # initial = [54]
    # moveobstacles = [47]
    # PUDO_t = [92,100]

    # initial = [42]
    # moveobstacles = [278]
    # PUDO_t = [115,203]

    filename = [filename,(colnum,rownum),cv2.INTER_AREA]
    gwg = Gridworld(filename,nagents=nagents, targets=targets, initial=initial, moveobstacles=moveobstacles)
    gwg.colorstates = [set(), set()]
    gwg.render()
    gwg.save(gwfile)
    partition = dict()
    allowed_states = [[None]] * nagents
    pg = [[None]]*nagents
    # allowed_states[0] = list(set(gwg.states) - set(gwg.obstacles)-set(gwg.obsborder))
    allowed_states[0] = list(set(gwg.states) - set(gwg.obstacles))

    #####     5) Pick belief state partitions     #####
    pg[0] = {0:allowed_states[0]}
 
    # pg[0] = {0: set.union(*[set(range(0,10))])  - set(gwg.obstacles), 1: set.union(*[set(range(10,20))])  - set(gwg.obstacles), 2: set.union(*[set(range(20,30))])  - set(gwg.obstacles),
    # 		 3: set.union(*[set(range(30,40))])  - set(gwg.obstacles), 4: set.union(*[set(range(40,50))])  - set(gwg.obstacles), 5: set.union(*[set(range(50,60))])  - set(gwg.obstacles),
    # 		 6: set.union(*[set(range(60,70))])  - set(gwg.obstacles), 7: set.union(*[set(range(70,80))])  - set(gwg.obstacles), 8: set.union(*[set(range(80,90))])  - set(gwg.obstacles),
    # 		 9: set.union(*[set(range(90,100))])  - set(gwg.obstacles)}
             
    # pg[0] = {0: set.union(*[set(range(0,30))])  - set(gwg.obstacles), 1: set.union(*[set(range(30,60))])  - set(gwg.obstacles), 2: set.union(*[set(range(60,90))])  - set(gwg.obstacles),
    # 		 3: set.union(*[set(range(90,120))])  - set(gwg.obstacles), 4: set.union(*[set(range(120,150))])  - set(gwg.obstacles), 5: set.union(*[set(range(150,180))])  - set(gwg.obstacles),
    # 		 6: set.union(*[set(range(180,210))])  - set(gwg.obstacles), 7: set.union(*[set(range(210,225))])  - set(gwg.obstacles)}
    
    
    # pg[0] = {0: set.union(*[set(range(0,15))])  - set(gwg.obstacles), 1: set.union(*[set(range(15,30))])  - set(gwg.obstacles), 2: set.union(*[set(range(30,45))])  - set(gwg.obstacles),
    # 		 3: set.union(*[set(range(45,60))])  - set(gwg.obstacles), 4: set.union(*[set(range(60,75))])  - set(gwg.obstacles), 5: set.union(*[set(range(75,90))])  - set(gwg.obstacles),
    # 		 6: set.union(*[set(range(90,105))])  - set(gwg.obstacles), 7: set.union(*[set(range(105,120))])  - set(gwg.obstacles), 8: set.union(*[set(range(120,135))])  - set(gwg.obstacles),
    #           9: set.union(*[set(range(135,150))])  - set(gwg.obstacles), 10: set.union(*[set(range(150,165))])  - set(gwg.obstacles), 11: set.union(*[set(range(165,180))])  - set(gwg.obstacles),
    #            12: set.union(*[set(range(180,195))])  - set(gwg.obstacles), 13: set.union(*[set(range(195,210))])  - set(gwg.obstacles), 14: set.union(*[set(range(210,225))])  - set(gwg.obstacles)}



    # block1 = []
    # block2 = []
    # block3 = []
    # block4 = []
    # block5 = []
    # block6 = []
    # block7 = []
    # block8 = []
    # block9 = []

    # for s in gwg.states:
    #     (row,col)=gwg.coords(s)
    #     if row<5:
    #         if col<5:
    #             block1.append(s)
    #         elif col<10:
    #             block2.append(s)
    #         else:
    #             block3.append(s)
    #     elif row<10:
    #         if col<5:
    #             block4.append(s)
    #         elif col<10:
    #             block5.append(s)
    #         else:
    #             block6.append(s)
    #     else:
    #         if col<5:
    #             block7.append(s)
    #         elif col<10:
    #             block8.append(s)
    #         else:
    #             block9.append(s)

    # pg[0] = {0: set.union(*[set(block1)])  - set(gwg.obstacles), 1: set.union(*[set(block2)])  - set(gwg.obstacles), 2: set.union(*[set(block3)])  - set(gwg.obstacles),
    # 		 3: set.union(*[set(block4)])  - set(gwg.obstacles), 4: set.union(*[set(block5)])  - set(gwg.obstacles), 5: set.union(*[set(block6)])  - set(gwg.obstacles),
    # 		 6: set.union(*[set(block7)])  - set(gwg.obstacles), 7: set.union(*[set(block8)])  - set(gwg.obstacles), 8: set.union(*[set(block9)])  - set(gwg.obstacles)}


   
    #pg[0] = dict()
    #i = 0
    # #26 seems to be the max
    #b = range(1,25)
    #for state in b:
    #    pg[0][i]=set([state])
    #    i += 1

    # for state in allowed_states[0]:
    #     pg[0][i]=set([state])
    #     i += 1
        
    #########################################################
    # print('pg: ' + str(pg[0]))
    visdist = [12,20,3500,3500]
    target_vis_dist = 2
    vel = [1,2,2,2]
    invisibilityset = []
    sensor_uncertainty = 1
    filename = []

    for n in [0]:
        # obj is a list of states that are static obstacle cells
        obj = compute_all_vis.img2obj(image)
        # compute visibility for each state
        # iset is a dict that has all the states listed that are not visible from the current state
        iset = compute_all_vis.compute_visibility_for_all(obj, h, w, radius=visdist[n])
        invisibilityset.append(iset)
        outfile = trial_name+'agent'+str(n) +'.json'
        filename.append(outfile)
        print 'output file: ', outfile
        print 'input file name:', infile

        #####     6) chose specification writing file     #####

        write_structured_slugs_past_action_foot_stance_MP_specs_step_height_23_38_no_step_over.write_to_slugs_part_dist(infile, gwg, initial[n], moveobstacles[0], iset, PUDO_targets = PUDO_t,
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

    Simulator.userControlled_partition(filename[0], gwg, pg[0], moveobstacles, invisibilityset, jsonfile_name)