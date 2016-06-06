import os
import cPickle as pickle
import numpy as np

def pickle_saver(to_be_saved,full_directory):
    with open(full_directory,'wb') as output:
        pickle.dump(to_be_saved,output,-1)

def pickle_loader(full_directory):
    with open(full_directory,'rb') as input:
        return pickle.load(input)

def make_dir(directory):
    if not os.path.exists(directory):
        os.makedirs(directory)
    else:
        print "directory exists, stop now unles you want files overwriten"

def pixel_to_point(idx,navmap):
    xo = navmap.info.origin.position.x
    yo = navmap.info.origin.position.y
    res = navmap.info.resolution
    w = navmap.info.width
    h = navmap.info.height
    x = (idx%w)*res+xo
    y = ((idx)/w)*res+yo
    return np.array([x,y])