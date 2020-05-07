from gridworld import *
import simulateController as Simulator
import copy
import compute_all_vis
import cv2

# mapname = 'BeliefTestEvasion'
mapname = 'BelieEvasionTwenty'
filename = 'figures/'+mapname+'.png'
image = cv2.imread(filename, cv2.IMREAD_GRAYSCALE)
image = cv2.resize(image,dsize=(15,15),interpolation=cv2.INTER_AREA)
h, w = image.shape[:2]

folder_locn = 'Examples/'
example_name = 'Jonas_Belief_Evasion_Terminal_act_PUDO_blocks'
# example_name = 'Jonas_Belief_Evasion_PUDO'
trial_name = folder_locn + example_name

outfile = trial_name + '.json'
infile = copy.deepcopy(trial_name)
gwfile = folder_locn + '/figs/gridworldfig_' + example_name + '.png'

nagents = 1
# targets = [[],[],[],[],[]]
targets = [[]]
initial = [54]
moveobstacles = [47]

filename = [filename,(15,15),cv2.INTER_AREA]

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

pg[0] = {0: set.union(*[set(range(0,30))])  - set(gwg.obstacles), 1: set.union(*[set(range(30,60))])  - set(gwg.obstacles), 2: set.union(*[set(range(60,90))])  - set(gwg.obstacles),
    	3: set.union(*[set(range(90,120))])  - set(gwg.obstacles), 4: set.union(*[set(range(120,150))])  - set(gwg.obstacles), 5: set.union(*[set(range(150,180))])  - set(gwg.obstacles),
    	6: set.union(*[set(range(180,210))])  - set(gwg.obstacles), 7: set.union(*[set(range(210,225))])  - set(gwg.obstacles)}

block1 = []
block2 = []
block3 = []
block4 = []
block5 = []
block6 = []
block7 = []
block8 = []
block9 = []

for s in gwg.states:
    (row,col)=gwg.coords(s)
    if row<5:
        if col<5:
            block1.append(s)
        elif col<10:
            block2.append(s)
        else:
            block3.append(s)
    elif row<10:
        if col<5:
            block4.append(s)
        elif col<10:
            block5.append(s)
        else:
            block6.append(s)
    else:
        if col<5:
            block7.append(s)
        elif col<10:
            block8.append(s)
        else:
            block9.append(s)

pg[0] = {0: set.union(*[set(block1)])  - set(gwg.obstacles), 1: set.union(*[set(block2)])  - set(gwg.obstacles), 2: set.union(*[set(block3)])  - set(gwg.obstacles),
            3: set.union(*[set(block4)])  - set(gwg.obstacles), 4: set.union(*[set(block5)])  - set(gwg.obstacles), 5: set.union(*[set(block6)])  - set(gwg.obstacles),
            6: set.union(*[set(block7)])  - set(gwg.obstacles), 7: set.union(*[set(block8)])  - set(gwg.obstacles), 8: set.union(*[set(block9)])  - set(gwg.obstacles)}

visdist = [5,20,3500,3500]
target_vis_dist = 2
vel = [3,2,2,2]
invisibilityset = []
obj = compute_all_vis.img2obj(image)
iset = compute_all_vis.compute_visibility_for_all(obj, h, w, radius=visdist[0])
invisibilityset.append(iset)



filename = []
outfile = trial_name+'agent'+str(0) +'.json'
filename.append(outfile)


Simulator.userControlled_partition(filename[0], gwg, pg[0], moveobstacles, invisibilityset)