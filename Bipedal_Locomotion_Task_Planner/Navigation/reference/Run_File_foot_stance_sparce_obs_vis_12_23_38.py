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
    # Make sure to state the agent and the target far enough from each other such that the games initial conditions do not violate safety.
    # mapname = '3ne'
    # mapname = 'BeliefTestEvasion'
    # mapname = 'BelieEvasionTwenty'
    # mapname = 'BelieEvasionFifteen_w'
    # mapname = 'chicago4_45_2454_5673_map'
    # mapname = 'BelieEvasion_64_30'
    # mapname = 'BelieEvasion_fifteen'
    # mapname = 'BelieEvasion_15_20_sparse_obs'
    mapname = 'BelieEvasion_38_23_height'
    scale = (int(40*2.8),40)
    # rownum = 15
    # colnum = 20
    # rownum = 30
    # colnum = 64
    # rownum = 15
    # colnum = 15
    rownum = 23
    colnum = 38
    filename = ['figures/' + mapname + '.pgm',scale,cv2.INTER_LINEAR_EXACT]

    ########## Jonas ##########
    filename = 'figures/'+mapname+'.png'
    ########## Jonas ##########

    # image = cv2.imread(filename[0], cv2.IMREAD_GRAYSCALE)  # 0 if obstacle, 255 if free space
    # image = cv2.resize(image, dsize=scale, interpolation=filename[2])
    ########## Jonas ##########
    image = cv2.imread(filename, cv2.IMREAD_GRAYSCALE)
    image = cv2.resize(image,dsize=(colnum,rownum),interpolation=cv2.INTER_AREA)
    ########## Jonas ##########
    # image[image<220] = 0
    # image[image >= 254] = 255
    h, w = image.shape[:2]
    # print('image.shape[:2]' + str(image.shape[:2]))
    # h, w are both 10
    ######################################
    folder_locn = 'Examples/'
    example_name = 'Jonas_Belief_Evasion_Terminal_foot_stance_yingke_23_38_vis_12'
    trial_name = folder_locn + example_name
    version = '01'
    # slugs = '/home/sudab/Applications/slugs/src/slugs'
    ########## Jonas ##########
    slugs = '/home/sa-zhao/Documents/Jonas/Slugs/slugs-master/src/slugs' # Path to slugs executable
    # print('test1')
    ########## Jonas ##########
    save_to_Gazebo = False
    outfile = trial_name + '.json'
    infile = copy.deepcopy(trial_name)
    gwfile = folder_locn + '/figs/gridworldfig_' + example_name + '.png'
    target_vis_file = trial_name + '.txt'
    nagents = 1
    # targets = [[],[],[],[],[]]
    targets = [[]]
    # initial = [16]
    # moveobstacles = [41]
    # initial = [54]
    # moveobstacles = [47]
    # PUDO_t = [84,143]

    # initial = [1076]
    # moveobstacles = [75]
    # # PUDO_t = [1162,818]
    # PUDO_t = [842,818]

    # 38_23
    initial = [812]
    moveobstacles = [90]
    PUDO_t = [408,611]

    # PUDO_t = [84,147]
    # PUDO_t = [189,53]

    # initial = [54]
    # moveobstacles = [47]
    # PUDO_t = [92,100]

    # initial = [42]
    # moveobstacles = [278]
    # PUDO_t = [115,203]

    ########## Jonas ##########
    # print('filename_main: ' + str(filename))
    # filename = [filename,(8,8),cv2.INTER_AREA]
    filename = [filename,(colnum,rownum),cv2.INTER_AREA]
    ########## Jonas ##########

    
    gwg = Gridworld(filename,nagents=nagents, targets=targets, initial=initial, moveobstacles=moveobstacles)
    gwg.colorstates = [set(), set()]
    gwg.render()
    # gwg.draw_state_labels()
    gwg.save(gwfile)
    partition = dict()
    allowed_states = [[None]] * nagents
    pg = [[None]]*nagents
    # print('pg: ' + str(pg))
    #################### Agent allowed states ####################
    # allowed_states[0] = set(gwg.states) - set(gwg.obstacles)
    # allowed_states[0] = list(set(gwg.states) - set(gwg.obstacles)-set(gwg.obsborder))
    allowed_states[0] = list(set(gwg.states) - set(gwg.obstacles))
    ################### Single Partition ################### abstract belief state
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
     ######################### Create sensor uncertainty dictionary #############################
    # belief_ncols = gwg.ncols - sensor_uncertainty + 1
    # belief_nrows = gwg.nrows - sensor_uncertainty + 1
    # sensor_uncertain_dict = dict.fromkeys(range(belief_ncols * belief_nrows))
    # for i in range(belief_nrows):
    #     for j in range(belief_ncols):
    #         belief_gridstate = i * belief_ncols + j
    #         sensor_uncertain_dict[belief_gridstate] = set()
    #         for srow in range(i, i + sensor_uncertainty):
    #             for scol in range(j, j + sensor_uncertainty):
    #                 gridstate = gwg.rcoords((srow, scol))
    #                 uset = list(itertools.product(['E', 'S', 'R'], repeat=sensor_uncertainty - 1))
    #                 for u in uset:
    #                     snext = copy.deepcopy(i * gwg.ncols + j)
    #                     for v in range(sensor_uncertainty - 1):
    #                         act = u[v]
    #                         snext = np.nonzero(gwg.prob[act][snext])[0][0]
    #                     # if gridstate not in iset[belief_gridstate]:
    #                     sensor_uncertain_dict[belief_gridstate].add(snext)
    #                 # sensor_uncertain_dict[belief_gridstate].add(gridstate)
    ##############################################################################################

    for n in [0]:
        # obj is a list of states that are static obstacle cells
        obj = compute_all_vis.img2obj(image)
        # print('obj: ' + str(obj))

        # compute visibility for each state
        # iset is a dict? that has all the states listed that are not visible from the current state
        iset = compute_all_vis.compute_visibility_for_all(obj, h, w, radius=visdist[n])
        # print('iset: ' + str(iset))\\\
        #     iset[s] = visibility.invis(gwg, s, visdist[n])
        #     if s in gwg.obstacles:
        #         iset[s] = {-1}
        # pickle_out = open("Examples/iset_gazeboexample_2.pickle", "wb")
        # pickle.dump(iset, pickle_out)
        # pickle_in = open("Examples/iset_gazeboexample.pickle", "rb")
        # iset = pickle.load(pickle_in)
        invisibilityset.append(iset)
        # at this point invisibilityset = iset


        outfile = trial_name+'agent'+str(n) +'.json'
        filename.append(outfile)
        print 'output file: ', outfile
        print 'input file name:', infile
        # Do the synthesis if asked
        # if args.synFlag:
        #     print('yay')
        #     write_structured_slugs.write_to_slugs_part_dist(infile, gwg, initial[n], moveobstacles[0], iset,
        #                                                            targets[n], vel[n], visdist[n], allowed_states[n],
        #                                                            [],
        #                                                            pg[n], belief_safety=1, belief_liveness=0,
        #                                                            target_reachability=False)

        #     # write_structured_slugs.write_to_slugs_part_dist(infile, gwg, initial[n], moveobstacles[0], iset, [], targets[n], vel[n],
        #     #                          visdist[n], allowed_states[n],[], pg[n], belief_safety=1, belief_liveness=0,
        #     #                          target_reachability=False,
        #     #                          target_has_vision=False, target_vision_dist=1.1, filename_target_vis=None,
        #     #                          compute_vis_flag=False)

        #     print ('Converting input file...')
        #     os.system('python compiler.py ' + infile + '.structuredslugs > ' + infile + '.slugsin')
        #     print('Computing controller...')
        #     sp = subprocess.Popen(slugs + ' --explicitStrategy --jsonOutput ' + infile + '.slugsin > ' + outfile,
        #                           shell=True, stdout=subprocess.PIPE)
        #     # sp = subprocess.Popen(slugs + ' --extractExplicitPermissiveStrategy ' + infile + '.slugsin > ' + outfile,
        #     #                       shell=True, stdout=subprocess.PIPE)
        #     sp.wait()

                # if args.synFlag:
        print('yay')

        # Added , [] after iset in write_to_slugs_part_dist() input This corresponds to visset_target, not used if target_has_vision is false (default is false)
        # look into what it takes to turn on liveness specifications
        # try allowed_states vs allowed_states[n]
        # seems is realizable when belief_safety=0




        # write_structured_slugs.write_to_slugs_part_dist(infile, gwg, initial[n], moveobstacles[0], iset,[],
        #                                                            targets[n], vel[n], visdist[n], allowed_states[n],
        #                                                            [],
        #                                                            pg[n], belief_safety=0, belief_liveness=0,
        #                                                            target_reachability=False)
        

        # write_structured_slugs_action.write_to_slugs_part_dist(infile, gwg, initial[n], moveobstacles[0], iset,[],
        #                                                            targets[n], vel[n], visdist[n], allowed_states[n],
        #                                                            [],
        #                                                            pg[n], belief_safety=0, belief_liveness=0,
        #                                                            target_reachability=False)

        # write_structured_slugs_no_obs.write_to_slugs_part_dist(infile, gwg, initial[n], moveobstacles[0], iset,[],
        #                                                            targets[n], vel[n], visdist[n], allowed_states[n],
        #                                                            [],
        #                                                            pg[n], belief_safety=0, belief_liveness=0,
        #                                                            target_reachability=False, PUDO_targets = PUDO_t)

        # write_structured_slugs_no_obs_edit4.write_to_slugs_part_dist(infile, gwg, initial[n], moveobstacles[0], iset,[],
        #                                                            targets[n], vel[n], visdist[n], allowed_states[n],
        #                                                            [],
        #                                                            pg[n], belief_safety=0, belief_liveness=0,
        #                                                            target_reachability=False, PUDO_targets = PUDO_t)

        # write_structured_slugs_past_action.write_to_slugs_part_dist(infile, gwg, initial[n], moveobstacles[0], iset,[],
        #                                                            targets[n], vel[n], visdist[n], allowed_states[n],
        #                                                            [],
        #                                                            pg[n], belief_safety=0, belief_liveness=0,
        #                                                            target_reachability=False, PUDO_targets = PUDO_t)

                                                            
        # write_structured_slugs_past_action_foot_stance.write_to_slugs_part_dist(infile, gwg, initial[n], moveobstacles[0], iset,[],
        #                                                            targets[n], vel[n], visdist[n], allowed_states[n],
        #                                                            [],
        #                                                            pg[n], belief_safety=0, belief_liveness=0,
        #                                                            target_reachability=False, PUDO_targets = PUDO_t)

        write_structured_slugs_past_action_foot_stance_MP_specs_step_height_23_38_no_step_over.write_to_slugs_part_dist(infile, gwg, initial[n], moveobstacles[0], iset,[],
                                                                   targets[n], vel[n], visdist[n], allowed_states[n],
                                                                   [],
                                                                   pg[n], belief_safety=0, belief_liveness=0,
                                                                   target_reachability=False, PUDO_targets = PUDO_t)

        # write_structured_slugs_past_action_no_foot_stance.write_to_slugs_part_dist(infile, gwg, initial[n], moveobstacles[0], iset,[],
        #                                                            targets[n], vel[n], visdist[n], allowed_states[n],
        #                                                            [],
        #                                                            pg[n], belief_safety=0, belief_liveness=0,
        #                                                            target_reachability=False, PUDO_targets = PUDO_t)

                                                                   

        # write_structured_slugs_past_action_no_foot_stance_can_turn_first_step_can_make_large_step_exiting_turn.write_to_slugs_part_dist(infile, gwg, initial[n], moveobstacles[0], iset,[],
        #                                                            targets[n], vel[n], visdist[n], allowed_states[n],
        #                                                            [],
        #                                                            pg[n], belief_safety=0, belief_liveness=0,
        #                                                            target_reachability=False, PUDO_targets = PUDO_t)


        # write_structured_slugs_action_belief.write_to_slugs_part_dist(infile, gwg, initial[n], moveobstacles[0], iset,[],
        #                                                            targets[n], vel[n], visdist[n], allowed_states[n],
        #                                                            [],
        #                                                            pg[n], belief_safety=0, belief_liveness=0,
        #                                                            target_reachability=False)

        # write_structured_slugs_PUDO.write_to_slugs_part_dist(infile, gwg, initial[n], moveobstacles[0], iset,[],
        #                                                            targets[n], vel[n], visdist[n], allowed_states[n],
        #                                                            [],
        #                                                            pg[n], belief_safety=0, belief_liveness=0,
        #                                                            target_reachability=False)




            # write_structured_slugs.write_to_slugs_part_dist(infile, gwg, initial[n], moveobstacles[0], iset, [], targets[n], vel[n],
            #                          visdist[n], allowed_states[n],[], pg[n], belief_safety=1, belief_liveness=0,
            #                          target_reachability=False,
            #                          target_has_vision=False, target_vision_dist=1.1, filename_target_vis=None,
            #                          compute_vis_flag=False)
        
        noww = time.time()
        print('Writing specifications took ', noww - then, ' seconds')

        print ('Converting input file...')
        os.system('python compiler.py ' + infile + '.structuredslugs > ' + infile + '.slugsin')
        print('Computing controller...')
        sp = subprocess.Popen(slugs + ' --explicitStrategy --jsonOutput ' + infile + '.slugsin > ' + outfile,
                                  shell=True, stdout=subprocess.PIPE)
            # sp = subprocess.Popen(slugs + ' --extractExplicitPermissiveStrategy ' + infile + '.slugsin > ' + outfile,
            #                       shell=True, stdout=subprocess.PIPE)
        sp.wait()

    # automaton = write_structured_slugs.parseJson(outfile)
    # print(automaton)

    now = time.time()
    print('Synthesis took ', now - then, ' seconds')
    print('Actual synthesis took ', now - noww, ' seconds')

    #### Save for gazebo
    if save_to_Gazebo:
        isetlist = dict()
        for s1 in iset.keys():
            isetlist[s1] = copy.deepcopy(list(iset[s1]))
        # j = json.dumps(isetlist, indent=1)
        # f = open(folder_locn + 'GazeboFiles/' + 'iset_' + version +'.json', "wb")
        # print >> f, j
        # f.close()
        with open(folder_locn + 'GazeboFiles/' + 'iset_' + version +'.json', "wb") as fp:
            json.dump(isetlist, fp)


        allowedstates = dict()
        for n in range(gwg.nagents):
            keyname = 'uav'+str(n+1)
            allowedstates[keyname] = copy.deepcopy(list(allowed_states[n]))

        with open(folder_locn + 'GazeboFiles/' + 'allowedstates_' + version +'.json', "wb") as fp:
            json.dump(allowedstates, fp)
        gwg.save(folder_locn+'GazeboFiles/gridfig'+version)

    #### Run Simulation
    # Simulator.userControlled_imperfect_sensor(filename, gwg, pg, moveobstacles, allowed_states, invisibilityset,
    #                                           sensor_uncertain_dict, sensor_uncertainty,saveImage=folder_locn+'GazeboFiles/livenessExperiment_1agent/trajectory_figs/t-')


    ########## Jonas ##########
    # partitionGrid = dict()
    # partitionGrid[(0,0)] = set(range(h*w)) #Number of belief states - more states -> more refined but slower synthesis
    # pg = [dict.fromkeys((0,0),pg[0])]
    ########## Jonas ##########

    Simulator.userControlled_partition(filename[0], gwg, pg[0], moveobstacles, invisibilityset)

        # userControlled_partition(filename,gwg,partitionGrid,moveobstacles,invisibilityset)
        