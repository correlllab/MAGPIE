from magpie.gripper import Gripper # must import the gripper class
G = Gripper() # create a gripper object
import numpy as np  # import numpy because we are using it below

# Since this is a new grasp, we reset all parameters to their default and start fresh
G.reset_parameters()

# Setting a gentle compliance since the object is medium compliance and to avoid damaging the ripe clementine
# Also allows adaptive gripping
G.set_compliance(margin=1, flexibility=5, finger='both')

# Select an initial gentle force to avoid damaging the clementine
G.set_force(force=0.15, finger='both')

# We estimate the initial goal aperture to gently enclose around the medium compliant, low weight object like a clementine
goal_aperture = 80  # An educated guess considering the size of a clementine
G.set_goal_aperture(aperture=goal_aperture, finger='both', record_load=False)

# Now we check the current aperture after setting the goal aperture
curr_aperture = G.get_aperture(finger='both')

# If current aperture didn't reach the goal due to object compliance, we readjust to the new goal
if curr_aperture != goal_aperture:
    G.set_goal_aperture(aperture=curr_aperture, finger='both')

# After achieving a gentle grip, we increase the force slightly to ensure the object is securely held
# without applying excessive force that could damage the clementine
G.set_force(force=1.0, finger='both')

# Now predict the gripper's state
# The gripper should now be gently and securely holding the clementine without causing damage.
# The force has been increased to 1.0 Newtons to confirm the grasp is secure.