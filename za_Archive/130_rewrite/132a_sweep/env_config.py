########## INIT ####################################################################################
import os
import numpy as np
import pybullet_data



########## SETTINGS ################################################################################
np.set_printoptions(
    edgeitems =  16, # Number of items before ...
    linewidth = 200, 
    formatter = dict( float = lambda x: "%.5g" % x ) 
)

ROBOT_URDF_PATH = os.path.expanduser( "~/MAGPIE/090_pdls_responsive/ur_e_description/urdf/ur5e.urdf" )
TABLE_URDF_PATH = os.path.join( pybullet_data.getDataPath(), "table/table.urdf" )
_VERBOSE        = True



########## PYBULLET ################################################################################

_USE_GRAPHICS = False
_BLOCK_ALPHA  = 1.0



########## OBJECTS #################################################################################

_NULL_NAME      = "NOTHING"
_ONLY_RED       = False
_ONLY_PRIMARY   = False
_ONLY_SECONDARY = False
if _ONLY_RED:
    _BLOCK_NAMES  = ['redBlock', _NULL_NAME]
elif _ONLY_PRIMARY:
    _BLOCK_NAMES  = ['redBlock', 'ylwBlock', 'bluBlock', _NULL_NAME,]
elif _ONLY_SECONDARY:
    _BLOCK_NAMES  = ['grnBlock', 'ornBlock', 'vioBlock', _NULL_NAME,]
else:
    _BLOCK_NAMES  = ['redBlock', 'ylwBlock', 'bluBlock', 'grnBlock', 'ornBlock', 'vioBlock', _NULL_NAME,]


_POSE_DIM     = 7
_ACTUAL_NAMES = _BLOCK_NAMES[:-1]
_N_CLASSES    = len( _BLOCK_NAMES )
_BLOCK_SCALE  = 0.038
_MIN_X_OFFSET = 0.400
_ROBOT_SPEED  = _BLOCK_SCALE / 20.0


########## BELIEFS #################################################################################

_CONFUSE_PROB = 0.01 # 0.05 # 0.001 # 0.001 # 0.025 # 0.05
_NULL_THRESH  = 0.75
_EXIST_THRESH = 0.05



########## MEASUREMENTS ############################################################################

_ACCEPT_POSN_ERR =  0.55*_BLOCK_SCALE # 0.50 # 0.65 # 0.75 # 0.85 # 1.00
_Z_SAFE          = 10.00*_BLOCK_SCALE
_MIN_SEP         =  0.40*_BLOCK_SCALE # 0.40 # 0.60 # 0.70 # 0.75
_MIN_X_OFFSET    =  0.400



########## PLANNER #################################################################################

_POST_N_SPINS  = 150
_N_XTRA_SPOTS  = 4
_CHANGE_THRESH = 0.40
_BT_LOOK_DIV   = 1.0