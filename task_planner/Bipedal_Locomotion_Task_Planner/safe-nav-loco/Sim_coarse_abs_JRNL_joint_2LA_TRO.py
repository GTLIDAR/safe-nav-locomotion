from gridworld_multi_obs_TRO import *
# from gridworld_multi_obs import *
# import simulateController_colors_belief_slugs as Simulator
# import simulateController_colors_belief_slugs_2LA as Simulator
import simulateController_TRO_Online as Simulator
import copy
import compute_all_vis
import cv2
from gridworld_fine_auto_spec_2l import *

# mapname = 'BeliefEvasion_jrnlt'
# mapname = 'BeliefEvasion_jrnl_CRT3'
# rownum = 7
# colnum = 12

mapname = 'TRO'
rownum = 5
colnum = 6

# mapname = 'KLAUS'
# rownum = 6
# colnum = 8

mapname = 'CCB_n'
rownum = 8
colnum = 8

filename = 'figures/'+mapname+'.png'
image = cv2.imread(filename, cv2.IMREAD_GRAYSCALE)
image = cv2.resize(image,dsize=(colnum,rownum),interpolation=cv2.INTER_AREA)
h, w = image.shape[:2]

folder_locn = 'Examples/'

example_name = 'TRO_ENV_New_GW'
# example_name = 'TRO_ENV_New_GW_KLAUS'

example_name = 'coarse_grid_multi_obs_NO_belief_collision_new_env'







trial_name = folder_locn + example_name

outfile = trial_name + '.json'
infile = copy.deepcopy(trial_name)
gwfile = folder_locn + '/figs/gridworldfig_' + example_name + '.png'

slugs = '../../slugs-master/src/slugs' # Path to slugs

nagents = 1
# targets = [[],[],[],[],[]]
targets = [[]]
# initial = [62]
# moveobstacles = [28]

initial = [56]
# moveobstacles = [61]
moveobstacles = [61,62]

initial = [21]
moveobstacles = [19,20]
# moveobstacles_c = [61]
PUDO_t_c = [30,25]
# initial = [37]
# moveobstacles = [33,34]


initial = [52]
moveobstacles = [42, 49]

filename = [filename,(colnum,rownum),cv2.INTER_AREA]

gwg = Gridworld(filename,nagents=nagents, targets=targets, initial=initial, moveobstacles=moveobstacles)
gwg.colorstates = [set(), set()]
gwg.render()
# gwg.draw_state_labels()
gwg.save(gwfile)
partition = dict()
allowed_states = [[None]] * nagents
pg = [[None]]*nagents
allowed_states[0] = list(set(gwg.states) - set(gwg.obstacles))

pg[0] = {0:set(allowed_states[0])}

# pg[0] = {0:(set(allowed_states[0])-set([37,38,39,49,50,53,54,61,62,63,64,65,66])),1:set([37,38,49,50]),2:set([61,62]),3:set([39]),4:set([63,64]),5:set([53,54,65,66])}
#pg[0] = {0:(set(allowed_states[0])-set([17,18,19,25,26,28,29,30,33,34,35,36,37,38])),1:set([17,18,25,26]),2:set([33,34]),3:set([19]),4:set([35]),5:set([28,29,30,36,37,38])}

visdist = [4,20,3500,3500]
target_vis_dist = 2
vel = [1,2,2,2]
invisibilityset = []
obj = compute_all_vis.img2obj(image)
iset = compute_all_vis.compute_visibility_for_all(obj, h, w, radius=visdist[0])
invisibilityset.append(iset)





# fine level stuff:
mapname_fine = 'fine_abstraction'
nagents = 1
targets = [[]]
scale_f = (int(40*2.8),40)
rownum_f = 26
colnum_f = 26
filename_f = 'figures/'+mapname_fine+'.png'
image_f = cv2.imread(filename_f, cv2.IMREAD_GRAYSCALE)
image_f = cv2.resize(image_f,dsize=(colnum_f,rownum_f),interpolation=cv2.INTER_AREA)
h_f, w_f = image_f.shape[:2]
example_name_f = 'Belief_Evasion_fine_abstraction_straight_nondeterministic'
# example_name_f = 'Belief_Evasion_fine_abstraction_straight_nondeterministic_every_step'
# example_name_f = 'Belief_Evasion_fine_abstraction_nondeterministic_last_step_of_turn_no_short'
# example_name_f = 'Belief_Evasion_fine_abstraction_nondeterministic_sagital_once'
example_name_f = 'Belief_Evasion_fine_staight_nondeterministic_5step'
jsonfile_name_f = folder_locn + "Integration/" + example_name_f + ".json"
trial_name_f = folder_locn + example_name_f
outfile_f = trial_name_f + '.json'
infile_f = copy.deepcopy(trial_name_f)
gwfile_f = folder_locn + '/figs/gridworldfig_' + example_name_f + '.png'
initial_f = [351]
moveobstacles_f = [0]
filename_f = [filename_f,(colnum_f,rownum_f),cv2.INTER_AREA]
gwg_f = Gridworld_fine(filename_f,nagents=nagents, targets=targets, initial_f=initial_f, moveobstacles_f=moveobstacles_f)
gwg_f.colorstates = [set(), set()]
gwg_f.render()
gwg_f.save(gwfile_f)
partition = dict()
allowed_states_f = [[None]] * nagents
pg_f = [[None]]*nagents
allowed_states_f[0] = list(set(gwg.states) - set(gwg.obstacles))
pg_f[0] = {0:allowed_states_f[0]}

invisibilityset_f = []
obj_f = compute_all_vis.img2obj(image_f)
if len(gwg_f.obstacles)>0:
            iset_f = compute_all_vis.compute_visibility_for_all(obj_f, h_f, w_f, radius=visdist[n])
else:
    iset_f = {}
    for state in gwg_f.states:
        iset_f[state] = set()
invisibilityset_f.append(iset_f)

filename_f = []
outfile_f = trial_name_f+'agent'+str(0) +'.json'
filename_f.append(outfile_f)



filename = []
#outfile = trial_name+'agent'+str(0) +'.json'
outfile = trial_name+'.slugsin'
filename.append(outfile)

jsonfile_name = example_name + ".json"
# Simulator.userControlled_partition(filename[0], gwg, pg[0], moveobstacles, invisibilityset, jsonfile_name)
Simulator.userControlled_partition(slugs,filename[0], gwg, pg[0], moveobstacles, invisibilityset, jsonfile_name,filename_f[0], gwg_f, pg_f[0], moveobstacles_f, invisibilityset_f,jsonfile_name_f,allowed_states)