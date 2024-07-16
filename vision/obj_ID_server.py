########## INIT ####################################################################################

##### Imports #####

import time
import numpy as np
import copy
from PIL import Image
import open3d as o3d
import matplotlib.pyplot as plt
from magpie.perception import pcd
from magpie import realsense_wrapper as real
from open3d.web_visualizer import draw
import signal

##### Camera Start #####

rsc = real.RealSense()
rsc.initConnection()



########## PERCEPTION SERVER, XML-RPC ##############################################################

class PerceptionServer(  )