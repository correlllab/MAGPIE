########## INIT ####################################################################################

from time import sleep

from magpie.ur5 import UR5_Interface
from magpie.realsense_wrapper import RealSense

rbt = UR5_Interface()
cam = RealSense()




########## START ###################################################################################

rbt.start()
cam.initConnection()


########## TEST ####################################################################################

##### Robot Test ##########################################################

pose = rbt.get_tcp_pose()
print( pose )
pose[2,3] += 0.050 # Move up 5cm

try:
    rbt.moveL( pose ) # All moves are non-blocking be default
    sleep( 1.5 ) # Wait for the robot to move before attempting shutdown
except Exception as e:
    print( f"Bad Thing Happened: {e}" )

##### Camera Test #########################################################

pcd = cam.getPCD()
print( f"Captured color point cloud with {len(pcd)} points!" )


########## SHUTDOWN ################################################################################

rbt.stop()
cam.disconnect()