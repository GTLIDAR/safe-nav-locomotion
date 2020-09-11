import sys, code, time
import matplotlib.pyplot as plt

def keyboard(banner=None):
    ''' Function that mimics the matlab keyboard command '''
    # use exception trick to pick up the euclidean_dist frame
    try:
        raise None
    except:
        frame = sys.exc_info()[2].tb_frame.f_back
    print ("# Use quit() to exit :) Happy debugging!")
    # evaluate commands in euclidean_dist namespace
    namespace = frame.f_globals.copy()
    namespace.update(frame.f_locals)
    try:
        code.interact(banner=banner, local=namespace)
    except SystemExit:
        return

def tic():
    #Homemade version of matlab tic and toc functions
    global startTime_for_tictoc
    startTime_for_tictoc = time.time()

def toc():
    if 'startTime_for_tictoc' in globals():
        print ("Elapsed time is " + str(time.time() - startTime_for_tictoc) + " seconds.")
    else:
        print ("Toc: start time not set")

def imagesc(im):
    plt.imshow(im)
    plt.show()
