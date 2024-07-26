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



########## GRAPHICS ################################################################################

_USE_GRAPHICS = False
_BLOCK_ALPHA  = 1.0
_CLR_TABLE  = {
    'red': [1.0, 0.0, 0.0,],
    'ylw': [1.0, 1.0, 0.0,],
    'blu': [0.0, 0.0, 1.0,],
    'grn': [0.0, 1.0, 0.0,],
    'orn': [1.0, 0.5, 0.0,],
    'vio': [0.5, 0.0, 1.0,]
}



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

# _BLOCK_SCALE  = 0.025 # Medium Wooden Blocks
_BLOCK_SCALE  = 0.040 # 3D Printed Blocks

_MIN_X_OFFSET = -0.380 - 0.050
_MAX_X_OFFSET = -0.060 + 0.050
_MIN_Y_OFFSET = -0.614 - 0.050
_MAX_Y_OFFSET = -0.290 + 0.050
_MAX_Z_BOUND  = _BLOCK_SCALE*4.0
_X_WRK_SPAN = _MAX_X_OFFSET - _MIN_X_OFFSET
_Y_WRK_SPAN = _MAX_Y_OFFSET - _MIN_Y_OFFSET
_ROBOT_SPEED  = _BLOCK_SCALE / 20.0


########## BELIEFS #################################################################################

_CONFUSE_PROB = 0.025 # 0.05 # 0.001 # 0.001 # 0.025 # 0.05
_NULL_THRESH  = 0.75
_EXIST_THRESH = 0.05


########## OBJECT PERMANENCE #######################################################################

_SCORE_FILTER_EXP  =   0.75 # During a belief update, accept the new score at this rate
_SCORE_DECAY_TAU_S =  10.0 # Score time constant, for freshness 
_OBJ_TIMEOUT_S     = 120.0 # Readings older than this are not considered



########## MEASUREMENTS ############################################################################

_ACCEPT_POSN_ERR =  0.55*_BLOCK_SCALE # 0.50 # 0.65 # 0.75 # 0.85 # 1.00
# _Z_SAFE          = 10.00*_BLOCK_SCALE
_Z_SAFE          = 0.400
_MIN_SEP         =  0.85*_BLOCK_SCALE # 0.40 # 0.60 # 0.70 # 0.75



########## PLANNER #################################################################################

_POST_N_SPINS  = 150
_N_XTRA_SPOTS  = 4
_CHANGE_THRESH = 0.40
_BT_LOOK_DIV   = 0.5