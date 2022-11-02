import os
from PIL import Image
import PIL
import cv2


#add path to figures folder below
# os.chdir(r'/path')

#
# filename = 'fine_abstraction.png'
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




#add path to figures folder below
# os.chdir(r'/path')

filename = '/home/sa-zhao/code/safe-nav-locomotion/task_planner/Bipedal_Locomotion_Task_Planner/safe-nav-loco/figures/fine_abstraction.png'
# image = cv2.imread(specFile, cv2.IMREAD_GRAYSCALE)
# print(image)

pngfile2 = Image.open(filename)
img2=pngfile2.resize((8,8),PIL.Image.ANTIALIAS)
img2.paste((255),(0,0,8,6))
# img2.show()
pixels=img2.load()

'''
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

'''

pixels[0,0] = 0
pixels[1,0] = 0
pixels[2,0] = 0
pixels[3,0] = 0
pixels[4,0] = 0
pixels[5,0] = 0
pixels[6,0] = 0
pixels[7,0] = 0

pixels[0,7] = 0
pixels[1,7] = 0
pixels[2,7] = 0
pixels[3,7] = 0
pixels[4,7] = 0
pixels[5,7] = 0
pixels[6,7] = 0
pixels[7,7] = 0

pixels[7,0] = 0
pixels[7,1] = 0
pixels[7,2] = 0
pixels[7,3] = 0
pixels[7,4] = 0
pixels[7,5] = 0
pixels[7,6] = 0
pixels[7,7] = 0


pixels[0,0] = 0
pixels[0,1] = 0
pixels[0,2] = 0
pixels[0,3] = 0
pixels[0,4] = 0
pixels[0,5] = 0
pixels[0,6] = 0
pixels[0,7] = 0

# pixels[1,1] = 0
# pixels[2,1] = 0

# pixels[3,3] = 0

#
img2.show()
#
img2.save("CCB.png")
