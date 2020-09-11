import os
from PIL import Image
import PIL
import cv2

# os.chdir(r'/home/jwarnke3/Documents/Slugs/slugs-master/examples/EvasionOnGrid')
#
# specFile="basicEvasion.png"
# pngfile = Image.open(specFile)
# #pngfile.show()
#
# # pngfile.paste((7),(0,0,10,10))
# img=pngfile.resize((10,10),PIL.Image.ANTIALIAS)
#
# img.paste((7),(0,0,10,10))
#
# pixels=img.load()
# pixels[1,5]=0
# pixels[3,5] = 0
# pixels[6,4] = 0
# pixels[6,5] = 0
# pixels[6,6] = 0
# pixels[6,7] = 0
# pixels[6,8] = 0
# # pixels[1,8] = 2
# # pixels[9,5] = 3
# # pixels[9,9] = 27
# pixels[3,5] = 0
# pixels[2,5] = 0
#
# img.save("BeliefEvasion.png")


# os.chdir(r'/home/jwarnke3/Documents/Slugs/Surveillance-Synthesis-Robot/exploration/figures')
#
# filename = 'chicago4_45_2454_5673_map.png'
# # image = cv2.imread(specFile, cv2.IMREAD_GRAYSCALE)
# # print(image)
#
# pngfile2 = Image.open(filename)
# img2=pngfile2.resize((10,10),PIL.Image.ANTIALIAS)
# img2.paste((255),(0,0,10,10))
# # img2.show()
# pixels=img2.load()
#
# # print(pixels[2,2])
# pixels[3,5] = 0
# pixels[6,4] = 0
# pixels[6,5] = 0
# pixels[6,6] = 0
# pixels[6,7] = 0
# pixels[3,5] = 0
# pixels[2,5] = 0
#
# img2.show()
#
# img2.save("BeliefTestEvasion.png")

# os.chdir(r'/home/jwarnke3/Documents/Slugs/Surveillance-Synthesis-Robot/exploration/figures')
#
# filename = 'chicago4_45_2454_5673_map.png'
# # image = cv2.imread(specFile, cv2.IMREAD_GRAYSCALE)
# # print(image)
#
# pngfile2 = Image.open(filename)
# img2=pngfile2.resize((15,15),PIL.Image.ANTIALIAS)
# img2.paste((255),(0,0,15,15))
# # img2.show()
# pixels=img2.load()
#
# # print(pixels[2,2])
# pixels[3,6] = 0
# pixels[3,7] = 0
# pixels[3,8] = 0
# pixels[4,6] = 0
# pixels[4,7] = 0
# pixels[4,8] = 0
# pixels[2,6] = 0
# pixels[2,7] = 0
# pixels[2,8] = 0
# pixels[10,8] = 0
# pixels[10,9] = 0
# pixels[10,10] = 0
# pixels[9,8] = 0
# pixels[9,9] = 0
# pixels[9,10] = 0
# pixels[8,8] = 0
# pixels[8,9] = 0
# pixels[8,10] = 0
#
#
# img2.show()
#
# img2.save("BelieEvasionTwenty.png")

# os.chdir(r'/home/jwarnke3/Documents/Slugs/Surveillance-Synthesis-Robot_realizability/exploration/figures')
#
# filename = 'chicago4_45_2454_5673_map.png'
# # image = cv2.imread(specFile, cv2.IMREAD_GRAYSCALE)
# # print(image)
#
# pngfile2 = Image.open(filename)
# img2=pngfile2.resize((64,30),PIL.Image.ANTIALIAS)
# img2.paste((255),(0,0,64,30))
# # img2.show()
# pixels=img2.load()
#
# pixels[10,20] = 0
# pixels[11,20] = 0
# pixels[10,21] = 0
# pixels[11,21] = 0
#
# pixels[16,6] = 0
# pixels[16,7] = 0
# pixels[17,6] = 0
# pixels[17,7] = 0
#
# pixels[47,12] = 0
# pixels[47,13] = 0
# pixels[48,12] = 0
# pixels[48,13] = 0
#
# pixels[60,20] = 0
# pixels[60,21] = 0
# pixels[61,20] = 0
# pixels[61,21] = 0
#
# pixels[40,6] = 0
# pixels[41,6] = 0
# pixels[40,7] = 0
# pixels[41,7] = 0
# pixels[40,8] = 0
# pixels[41,8] = 0
# pixels[40,9] = 0
# pixels[41,9] = 0
# pixels[40,10] = 0
# pixels[41,10] = 0
# pixels[40,11] = 0
# pixels[41,11] = 0
# pixels[40,12] = 0
# pixels[41,12] = 0
# pixels[40,13] = 0
# pixels[41,13] = 0
# pixels[40,14] = 0
# pixels[41,14] = 0
# pixels[40,15] = 0
# pixels[41,15] = 0
# pixels[40,16] = 0
# pixels[41,16] = 0
# pixels[40,17] = 0
# pixels[41,17] = 0
# pixels[40,18] = 0
# pixels[41,18] = 0
# pixels[40,19] = 0
# pixels[41,19] = 0
#
#
#
# img2.show()
#
# img2.save("BelieEvasion_64_30.png")


# os.chdir(r'/home/jwarnke3/Documents/Slugs/Surveillance-Synthesis-Robot_realizability/exploration/figures')
#
# filename = 'chicago4_45_2454_5673_map.png'
# # image = cv2.imread(specFile, cv2.IMREAD_GRAYSCALE)
# # print(image)
#
# pngfile2 = Image.open(filename)
# img2=pngfile2.resize((15,15),PIL.Image.ANTIALIAS)
# img2.paste((255),(0,0,15,15))
# # img2.show()
# pixels=img2.load()
#
# pixels[4,5] = 0
# pixels[9,7] = 0
# pixels[3,11] = 0
# #
# # pixels[16,6] = 0
# # pixels[16,7] = 0
# # pixels[17,6] = 0
# # pixels[17,7] = 0
# #
# # pixels[47,12] = 0
# # pixels[47,13] = 0
# # pixels[48,12] = 0
# # pixels[48,13] = 0
# #
# # pixels[60,20] = 0
# # pixels[60,21] = 0
# # pixels[61,20] = 0
# # pixels[61,21] = 0
# #
# # pixels[41,18] = 0
# # pixels[40,19] = 0
# # pixels[41,19] = 0
#
#
#
# img2.show()
#
# img2.save("BelieEvasion_fifteen.png")

# os.chdir(r'/home/jwarnke3/Documents/Slugs/Surveillance-Synthesis-Robot_realizability/exploration/figures')
#
# filename = 'chicago4_45_2454_5673_map.png'
# # image = cv2.imread(specFile, cv2.IMREAD_GRAYSCALE)
# # print(image)
#
# pngfile2 = Image.open(filename)
# img2=pngfile2.resize((20,15),PIL.Image.ANTIALIAS)
# img2.paste((255),(0,0,20,15))
# # img2.show()
# pixels=img2.load()
#
#
# pixels[7,4] = 0
# pixels[8,4] = 0
# pixels[7,5] = 0
# pixels[8,5] = 0
#
# pixels[11,11] = 0
# pixels[12,11] = 0
# pixels[13,11] = 0
#
#
#
#
#
# #
# # pixels[16,6] = 0
# # pixels[16,7] = 0
# # pixels[17,6] = 0
# # pixels[17,7] = 0
# #
# # pixels[47,12] = 0
# # pixels[47,13] = 0
# # pixels[48,12] = 0
# # pixels[48,13] = 0
# #
# # pixels[60,20] = 0
# # pixels[60,21] = 0
# # pixels[61,20] = 0
# # pixels[61,21] = 0
# #
# # pixels[41,18] = 0
# # pixels[40,19] = 0
# # pixels[41,19] = 0
#
#
#
# img2.show()
#
# img2.save("BelieEvasion_15_20_sparse_obs.png")



#
# os.chdir(r'/home/jwarnke3/Documents/github/Beliefspace_Planning/Bipedal_Locomotion_Task_Planner/Navigation/figures')
#
# filename = 'chicago4_45_2454_5673_map.png'
# # image = cv2.imread(specFile, cv2.IMREAD_GRAYSCALE)
# # print(image)
#
# pngfile2 = Image.open(filename)
# img2=pngfile2.resize((38,23),PIL.Image.ANTIALIAS)
# img2.paste((255),(0,0,38,23))
# # img2.show()
# pixels=img2.load()
#
#
# pixels[2,5] = 0
# pixels[3,5] = 0
# pixels[4,5] = 0
# pixels[5,5] = 0
# pixels[2,6] = 0
# pixels[3,6] = 0
# pixels[4,6] = 0
# pixels[5,6] = 0
# pixels[2,7] = 0
# pixels[3,7] = 0
# pixels[4,7] = 0
# pixels[5,7] = 0
#
# pixels[16,6] = 0
# pixels[17,6] = 0
# pixels[16,7] = 0
# pixels[17,7] = 0
# pixels[16,8] = 0
# pixels[17,8] = 0
# pixels[16,9] = 0
# pixels[17,9] = 0
# pixels[16,10] = 0
# pixels[17,10] = 0
# pixels[16,11] = 0
# pixels[17,11] = 0
# pixels[16,12] = 0
# pixels[17,12] = 0
# pixels[16,13] = 0
# pixels[17,13] = 0
# pixels[16,14] = 0
# pixels[17,14] = 0
#
#
# pixels[30,17] = 0
# pixels[31,17] = 0
# pixels[30,18] = 0
# pixels[31,18] = 0
#
# pixels[25,6] = 0
# pixels[26,6] = 0
# pixels[27,6] = 0
# pixels[28,6] = 0
# pixels[29,6] = 0
# pixels[30,6] = 0
#
# pixels[25,13] = 0
# pixels[26,13] = 0
# pixels[27,13] = 0
# pixels[28,13] = 0
# pixels[29,13] = 0
# pixels[30,13] = 0
# #
# #
# #
# #
# # #
# # # pixels[16,6] = 0
# # # pixels[16,7] = 0
# # # pixels[17,6] = 0
# # # pixels[17,7] = 0
# # #
# # # pixels[47,12] = 0
# # # pixels[47,13] = 0
# # # pixels[48,12] = 0
# # # pixels[48,13] = 0
# # #
# # # pixels[60,20] = 0
# # # pixels[60,21] = 0
# # # pixels[61,20] = 0
# # # pixels[61,21] = 0
# # #
# # # pixels[41,18] = 0
# # # pixels[40,19] = 0
# # # pixels[41,19] = 0
# #
# #
# #
# img2.show()
# #
# img2.save("BelieEvasion_38_23_Extra_obs.png")



# os.chdir(r'/home/jwarnke3/Documents/Slugs/Surveillance-Synthesis-Robot_realizability_Foot_stance/exploration/figures')
#
# filename = 'chicago4_45_2454_5673_map.png'
# # image = cv2.imread(specFile, cv2.IMREAD_GRAYSCALE)
# # print(image)
#
# pngfile2 = Image.open(filename)
# img2=pngfile2.resize((15,12),PIL.Image.ANTIALIAS)
# img2.paste((255),(0,0,15,12))
# # img2.show()
# pixels=img2.load()
#
#
# pixels[4,4] = 0
# pixels[5,4] = 0
# pixels[6,4] = 0
#
#
#
#
#
#
# img2.show()
#
# img2.save("BelieEvasion_15_12_height_2.png")

# os.chdir(r'../Bipedal_Locomotion_Task_Planner/Navigation/figures')
#
# filename = 'chicago4_45_2454_5673_map.png'
# # image = cv2.imread(specFile, cv2.IMREAD_GRAYSCALE)
# # print(image)
#
# pngfile2 = Image.open(filename)
# img2=pngfile2.resize((15,12),PIL.Image.ANTIALIAS)
# img2.paste((255),(0,0,15,12))
# # img2.show()
# pixels=img2.load()
#
# pixels[4,4] = 0
# pixels[5,4] = 0
# pixels[6,4] = 0
#
#
#
#
#
#
# img2.show()
#
# img2.save("BelieEvasion_15_12_new_3_stat_obs.png")

# os.chdir(r'/home/sa-zhao/Documents/Jonas/Beliefspace_Planning/Bipedal_Locomotion_Task_Planner/2_level_abs/figures')
#
# filename = 'chicago4_45_2454_5673_map.png'
# # image = cv2.imread(specFile, cv2.IMREAD_GRAYSCALE)
# # print(image)
#
# pngfile2 = Image.open(filename)
# img2=pngfile2.resize((15,15),PIL.Image.ANTIALIAS)
# img2.paste((255),(0,0,15,15))
# # img2.show()
# pixels=img2.load()
#
# # pixels[4,5] = 0
# # pixels[9,7] = 0
# # pixels[3,11] = 0
# #
# # pixels[16,6] = 0
# # pixels[16,7] = 0
# # pixels[17,6] = 0
# # pixels[17,7] = 0
# #
# # pixels[47,12] = 0
# # pixels[47,13] = 0
# # pixels[48,12] = 0
# # pixels[48,13] = 0
# #
# # pixels[60,20] = 0
# # pixels[60,21] = 0
# # pixels[61,20] = 0
# # pixels[61,21] = 0
# #
# # pixels[41,18] = 0
# # pixels[40,19] = 0
# # pixels[41,19] = 0
#
#
#
# img2.show()
#
# img2.save("fine_abstraction.png")


# os.chdir(r'/home/sa-zhao/Documents/Jonas/Beliefspace_Planning/Bipedal_Locomotion_Task_Planner/2_level_abs/figures')
#
# filename = 'chicago4_45_2454_5673_map.png'
# # image = cv2.imread(specFile, cv2.IMREAD_GRAYSCALE)
# # print(image)
#
# pngfile2 = Image.open(filename)
# img2=pngfile2.resize((13,9),PIL.Image.ANTIALIAS)
# img2.paste((255),(0,0,13,9))
# # img2.show()
# pixels=img2.load()
#
#
# pixels[0,1] = 0
# pixels[1,1] = 0
# pixels[0,2] = 0
# pixels[1,2] = 0
#
# pixels[5,2] = 0
# pixels[5,3] = 0
# pixels[5,4] = 0
# pixels[5,5] = 0
#
# pixels[8,2] = 0
# pixels[9,2] = 0
# pixels[10,2] = 0
#
# pixels[8,5] = 0
# pixels[9,5] = 0
# pixels[10,5] = 0
#
# pixels[10,7] = 0
# pixels[11,7] = 0
#
# #
# img2.show()
# #
# img2.save("BelieEvasion_coarse.png")

# os.chdir(r'/home/sa-zhao/Documents/Jonas/Beliefspace_Planning/Bipedal_Locomotion_Task_Planner/2_level_abs/figures')
#
# filename = 'chicago4_45_2454_5673_map.png'
# # image = cv2.imread(specFile, cv2.IMREAD_GRAYSCALE)
# # print(image)
#
# pngfile2 = Image.open(filename)
# img2=pngfile2.resize((14,10),PIL.Image.ANTIALIAS)
# img2.paste((255),(0,0,14,10))
# # img2.show()
# pixels=img2.load()
#
#
# pixels[0,2] = 0
# pixels[1,2] = 0
# pixels[0,3] = 0
# pixels[1,3] = 0
#
# pixels[5,3] = 0
# pixels[5,4] = 0
# pixels[5,5] = 0
# pixels[5,6] = 0
#
# pixels[8,3] = 0
# pixels[9,3] = 0
# pixels[10,3] = 0
#
# pixels[8,6] = 0
# pixels[9,6] = 0
# pixels[10,6] = 0
#
# pixels[10,8] = 0
# pixels[11,8] = 0
#
# #
# img2.show()
# #
# img2.save("BeliefEvasion_coarse_14_10.png")

# os.chdir(r'/home/sa-zhao/Documents/Jonas/Beliefspace_Planning/Bipedal_Locomotion_Task_Planner/2_level_abs/figures')
#
# filename = 'chicago4_45_2454_5673_map.png'
# # image = cv2.imread(specFile, cv2.IMREAD_GRAYSCALE)
# # print(image)
#
# pngfile2 = Image.open(filename)
# img2=pngfile2.resize((8,5),PIL.Image.ANTIALIAS)
# img2.paste((255),(0,0,8,5))
# # img2.show()
# pixels=img2.load()
#
#
# pixels[6,0] = 0
# pixels[7,0] = 0
# pixels[6,1] = 0
# pixels[7,1] = 0
# pixels[6,2] = 0
# pixels[7,2] = 0
#
#
# #
# img2.show()
# #
# img2.save("BeliefEvasion_CRT.png")

# os.chdir(r'/home/sa-zhao/Documents/Jonas/Beliefspace_Planning/Bipedal_Locomotion_Task_Planner/2_level_abs/figures')
#
# filename = 'chicago4_45_2454_5673_map.png'
# # image = cv2.imread(specFile, cv2.IMREAD_GRAYSCALE)
# # print(image)
#
# pngfile2 = Image.open(filename)
# img2=pngfile2.resize((10,7),PIL.Image.ANTIALIAS)
# img2.paste((255),(0,0,10,7))
# # img2.show()
# pixels=img2.load()
#
#
# pixels[7,1] = 0
# pixels[8,1] = 0
# pixels[7,2] = 0
# pixels[8,2] = 0
# pixels[7,3] = 0
# pixels[8,3] = 0
#
# pixels[0,0] = 0
# pixels[0,1] = 0
# pixels[0,2] = 0
# pixels[0,3] = 0
# pixels[0,4] = 0
# pixels[0,5] = 0
# pixels[0,6] = 0
# pixels[1,0] = 0
# pixels[2,0] = 0
# pixels[3,0] = 0
# pixels[4,0] = 0
# pixels[5,0] = 0
# pixels[6,0] = 0
# pixels[7,0] = 0
# pixels[8,0] = 0
# pixels[9,0] = 0
# pixels[9,1] = 0
# pixels[9,2] = 0
# pixels[9,3] = 0
# pixels[9,4] = 0
# pixels[9,5] = 0
# pixels[9,6] = 0
# pixels[8,6] = 0
# pixels[7,6] = 0
# pixels[6,6] = 0
# pixels[5,6] = 0
# pixels[4,6] = 0
# pixels[3,6] = 0
# pixels[2,6] = 0
# pixels[1,6] = 0
#
#
#
#
# #
# img2.show()
# #
# img2.save("BeliefEvasion_CRT_boarder.png")


os.chdir(r'/home/sa-zhao/Documents/Jonas/Beliefspace_Planning/Bipedal_Locomotion_Task_Planner/2_level_finer_abs/figures')

filename = 'chicago4_45_2454_5673_map.png'
# image = cv2.imread(specFile, cv2.IMREAD_GRAYSCALE)
# print(image)

pngfile2 = Image.open(filename)
img2=pngfile2.resize((13,7),PIL.Image.ANTIALIAS)
img2.paste((255),(0,0,13,7))
# img2.show()
pixels=img2.load()


pixels[2,1] = 0
pixels[4,3] = 0
pixels[4,5] = 0

pixels[7,1] = 0
pixels[8,1] = 0
pixels[7,2] = 0
pixels[8,2] = 0

pixels[7,4] = 0
pixels[8,4] = 0
pixels[7,5] = 0
pixels[8,5] = 0


pixels[0,0] = 0
pixels[0,1] = 0
pixels[0,2] = 0
pixels[0,3] = 0
pixels[0,4] = 0
pixels[0,5] = 0
pixels[0,6] = 0

pixels[12,0] = 0
pixels[12,1] = 0
pixels[12,2] = 0
pixels[12,3] = 0
pixels[12,4] = 0
pixels[12,5] = 0
pixels[12,6] = 0

pixels[1,0] = 0
pixels[2,0] = 0
pixels[3,0] = 0
pixels[4,0] = 0
pixels[5,0] = 0
pixels[6,0] = 0
pixels[7,0] = 0
pixels[8,0] = 0
pixels[9,0] = 0
pixels[10,0] = 0
pixels[11,0] = 0
pixels[12,0] = 0

pixels[1,6] = 0
pixels[2,6] = 0
pixels[3,6] = 0
pixels[4,6] = 0
pixels[5,6] = 0
pixels[6,6] = 0
pixels[7,6] = 0
pixels[8,6] = 0
pixels[9,6] = 0
pixels[10,6] = 0
pixels[11,6] = 0
pixels[12,6] = 0








#
img2.show()
#
img2.save("BeliefEvasion_CDC.png")