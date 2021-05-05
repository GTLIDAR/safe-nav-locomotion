from gridworld_JRNL_color import *
from gridworld_fine_2l import *
# import simulateController_2labs as Simulator
# import simulateController_2labs_stairN as Simulator
import simulateController_2labs_stairN_color_multiObs as Simulator
import copy
import compute_all_vis
import cv2

mapname_fine = 'fine_abstraction'
scale_f = (int(40*2.8),40)

rownum_f = 26
colnum_f = 26

mapname_coarse = 'BeliefEvasion_jrnl_CRT3'
rownum_c = 7
colnum_c = 12


filename_f = 'figures/'+mapname_fine+'.png'
image_f = cv2.imread(filename_f, cv2.IMREAD_GRAYSCALE)
image_f = cv2.resize(image_f,dsize=(colnum_f,rownum_f),interpolation=cv2.INTER_AREA)
h_f, w_f = image_f.shape[:2]

filename_c = 'figures/'+mapname_coarse+'.png'
image_c = cv2.imread(filename_c, cv2.IMREAD_GRAYSCALE)
image_c = cv2.resize(image_c,dsize=(colnum_c,rownum_c),interpolation=cv2.INTER_AREA)
h_c, w_c = image_c.shape[:2]

folder_locn = 'Examples/'

# example_name_c = 'Belief_Evasion_coarse_cdc_map'
# example_name_c = 'Belief_Evasion_coarse_JRNL_crt3'
example_name_c = 'Belief_Evasion_coarse_JRNL_crt3_2stair_test2'
example_name_c = 'Belief_Evasion_coarse_JRNL_crt3_2stair'
example_name_c = 'Belief_Evasion_coarse_multi_obs_jrnl_stairsNTT'

initial_c = [56]
moveobstacles_c = [61,62]

jsonfile_name_c = folder_locn + "Integration/" + example_name_c + ".json"
trial_name_c = folder_locn + example_name_c

outfile_c = trial_name_c + '.json'
infile_c = copy.deepcopy(trial_name_c)
gwfile_c = folder_locn + '/figs/gridworldfig_' + example_name_c + '.png'


# example_name_f = 'Belief_Evasion_fine_abstraction'
example_name_f = 'Belief_Evasion_fine_abstraction_JRNL_Boundary6_26x26_stair_mod_test'
example_name_f = 'Belief_Evasion_fine_abstraction_JRNL_Boundary6_b2b_stair'
jsonfile_name_f = folder_locn + "Integration/" + example_name_f + ".json"
trial_name_f = folder_locn + example_name_f

outfile_f = trial_name_f + '.json'
infile_f = copy.deepcopy(trial_name_f)
gwfile_f = folder_locn + '/figs/gridworldfig_' + example_name_f + '.png'

nagents = 1
targets = [[]]
initial_f = [351]
moveobstacles_f = [0]


filename_c = [filename_c,(colnum_c,rownum_c),cv2.INTER_AREA]
gwg_c = Gridworld(filename_c,nagents=nagents, targets=targets, initial=initial_c, moveobstacles=moveobstacles_c)
gwg_c.colorstates = [set(), set()]
gwg_c.render()
gwg_c.save(gwfile_c)
partition = dict()
allowed_states_c = [[None]] * nagents
pg_c = [[None]]*nagents
allowed_states_c[0] = list(set(gwg_c.states) - set(gwg_c.obstacles))


# pg_c[0] = {0:allowed_states_c[0]}
# pg_c[0] = {0:(set(allowed_states_c[0])-set([55,56,57,44,58,70,71])),1:set([55]),2:set([56,57,44,58,70,71])}
pg_c[0] = {0:(set(allowed_states_c[0])-set([37,38,39,49,50,53,54,61,62,63,64,65,66])),1:set([37,38,49,50]),2:set([61,62]),3:set([39]),4:set([63,64]),5:set([53,54,65,66])}


filename_f = [filename_f,(colnum_f,rownum_f),cv2.INTER_AREA]
gwg_f = Gridworld_fine(filename_f,nagents=nagents, targets=targets, initial_f=initial_f, moveobstacles_f=moveobstacles_f)
gwg_f.colorstates = [set(), set()]
gwg_f.render()
gwg_f.save(gwfile_f)
partition = dict()
allowed_states_f = [[None]] * nagents
pg_f = [[None]]*nagents
allowed_states_f[0] = list(set(gwg_c.states) - set(gwg_c.obstacles))


pg_f[0] = {0:allowed_states_f[0]}

visdist = [12,20,3500,3500]
target_vis_dist = 2
vel = [1,2,2,2]
invisibilityset_c = []
obj_c = compute_all_vis.img2obj(image_c)
iset_c = compute_all_vis.compute_visibility_for_all(obj_c, h_c, w_c, radius=visdist[0])

invisibilityset_c.append(iset_c)


invisibilityset_f = []
obj_f = compute_all_vis.img2obj(image_f)
if len(gwg_f.obstacles)>0:
            iset_f = compute_all_vis.compute_visibility_for_all(obj_f, h_f, w_f, radius=visdist[n])
else:
    iset_f = {}
    for state in gwg_f.states:
        iset_f[state] = set()
invisibilityset_f.append(iset_f)




filename_c = []
outfile_c = trial_name_c+'agent'+str(0) +'.json'
filename_c.append(outfile_c)
# jsonfile_name_c = example_name_c + ".json"

filename_f = []
outfile_f = trial_name_f+'agent'+str(0) +'.json'
filename_f.append(outfile_f)
# jsonfile_name_f = example_name_f + ".json"
 

Simulator.userControlled_partition(filename_c[0], gwg_c, pg_c[0], moveobstacles_c, invisibilityset_c, jsonfile_name_c,filename_f[0], gwg_f, pg_f[0], moveobstacles_f, invisibilityset_f, jsonfile_name_f)