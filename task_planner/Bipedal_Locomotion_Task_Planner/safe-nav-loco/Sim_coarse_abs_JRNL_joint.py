from gridworld_JRNL_color import *
import simulateController_colors_belief_slugs as Simulator
import copy
import compute_all_vis
import cv2

mapname = 'BeliefEvasion_jrnlt'
mapname = 'BeliefEvasion_jrnl_CRT3'
rownum = 7
colnum = 12

filename = 'figures/'+mapname+'.png'
image = cv2.imread(filename, cv2.IMREAD_GRAYSCALE)
image = cv2.resize(image,dsize=(colnum,rownum),interpolation=cv2.INTER_AREA)
h, w = image.shape[:2]

folder_locn = 'Examples/'
example_name = 'Belief_Evasion_coarse_JRNLtest'
example_name = 'Belief_Evasion_coarse_multi_obs_joint_test2_single_obs'
example_name = 'Belief_Evasion_coarse_seperate_belief_single_obs'
example_name = 'Belief_Evasion_coarse_seperate_belief_single_obs_2t'
example_name = 'Belief_Evasion_coarse_multi_obs_joint_belief_slugs'
# example_name = 'Belief_Evasion_coarse_multi_obs_joint_belief_slugs_colis_next2'


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

# pg[0] = {0:allowed_states[0]}
# pg[0] = {0: set.union(*[set(range(0,10))])  - set(gwg.obstacles), 1: set.union(*[set(range(10,20))])  - set(gwg.obstacles), 2: set.union(*[set(range(20,30))])  - set(gwg.obstacles),
#     		 3: set.union(*[set(range(30,40))])  - set(gwg.obstacles), 4: set.union(*[set(range(40,50))])  - set(gwg.obstacles), 5: set.union(*[set(range(50,60))])  - set(gwg.obstacles),
#     		 6: set.union(*[set(range(60,70))])  - set(gwg.obstacles), 7: set.union(*[set(range(70,80))])  - set(gwg.obstacles), 8: set.union(*[set(range(80,90))])  - set(gwg.obstacles),
#     		 9: set.union(*[set(range(90,100))])  - set(gwg.obstacles)}

# pg[0] = {0: set.union(*[set(range(0,30))])  - set(gwg.obstacles), 1: set.union(*[set(range(30,60))])  - set(gwg.obstacles), 2: set.union(*[set(range(60,90))])  - set(gwg.obstacles),
#     	3: set.union(*[set(range(90,120))])  - set(gwg.obstacles), 4: set.union(*[set(range(120,150))])  - set(gwg.obstacles), 5: set.union(*[set(range(150,180))])  - set(gwg.obstacles),
#     	6: set.union(*[set(range(180,210))])  - set(gwg.obstacles), 7: set.union(*[set(range(210,225))])  - set(gwg.obstacles)}

# pg[0] = {0: set.union(*[set(range(0,180))])  - set(gwg.obstacles) - set([75,76,77,78,79,80,81,90,91,92,93,94,95,96,105,106,107,108,109,110,111,120,121,122,123,124,125,126,135,136,137,138,139,140,141,150,151,152,153,154,155,156,165,166,167,168,169,170,171]), 
#              1: set.union(*[set([75,76,77,90,91,92,105,106,107,120,121,122,135,136,137,150,151,152,165,166,167])])  - set(gwg.obstacles), 
#              2: set.union(*[set([78,93,108,123,138,153,168])])  - set(gwg.obstacles),
#     		 3: set.union(*[set([79,94,109,124,139,154,169])])  - set(gwg.obstacles), 
#              4: set.union(*[set([80,95,110,125,140,155,170])])  - set(gwg.obstacles), 
#              5: set.union(*[set([81,96,111,126,141,156,171])])  - set(gwg.obstacles)}

# pg[0] = {0: set.union(*[set(range(0,180))])  - set(gwg.obstacles) - set([75,76,77,78,79,80,81,90,91,92,93,94,95,96,105,106,107,108,109,110,111,120,121,122,123,124,125,126,135,136,137,138,139,140,141,150,151,152,153,154,155,156,165,166,167,168,169,170,171]), 
#              1: set.union(*[set([75,76,77,90,91,92,105,106,107,120,121,122,135,136,137,150,151,152,165,166,167,78,93,108,123,138,153,168])])  - set(gwg.obstacles), 
#              2: set.union(*[set([79,94,109,124,139,154,169])])  - set(gwg.obstacles), 
#              3: set.union(*[set([80,95,110,125,140,155,170])])  - set(gwg.obstacles), 
#              4: set.union(*[set([81,96,111,126,141,156,171])])  - set(gwg.obstacles) }

# pg[0] = {0:(set(allowed_states[0])-set([12,23,24,25,34,35,37,38,39,45,46,47,48,49,50,56,57,61])),1:set([12,23,24,34,35]),2:set([45,46,56,57]),3:set([25]),4:set([47]),5:set([38,39,48,49,50,61])}
# pg[0] = {0:(set(allowed_states[0])-set([12,23,24,25,34,35,37,38,39,45,46,47,48,49,50,56,57,61])),1:set([12,23,24,34,35]),2:set([45,46,56,57]),3:set([25]),4:set([47,48]),5:set([38,39,49,50,61])}
pg[0] = {0:(set(allowed_states[0])-set([37,38,39,49,50,53,54,61,62,63,64,65,66])),1:set([37,38,49,50]),2:set([61,62]),3:set([39]),4:set([63,64]),5:set([53,54,65,66])}

visdist = [4,20,3500,3500]
target_vis_dist = 2
vel = [1,2,2,2]
invisibilityset = []
obj = compute_all_vis.img2obj(image)
iset = compute_all_vis.compute_visibility_for_all(obj, h, w, radius=visdist[0])
invisibilityset.append(iset)



filename = []
# outfile = trial_name+'agent'+str(0) +'.json'
outfile = trial_name+'.slugsin'
filename.append(outfile)

jsonfile_name = example_name + ".json"
# Simulator.userControlled_partition(filename[0], gwg, pg[0], moveobstacles, invisibilityset, jsonfile_name)
Simulator.userControlled_partition(slugs,filename[0], gwg, pg[0], moveobstacles, invisibilityset, jsonfile_name,allowed_states)