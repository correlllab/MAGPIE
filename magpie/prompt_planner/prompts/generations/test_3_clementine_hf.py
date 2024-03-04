from magpie.gripper import Gripper  # import the gripper class

# Create a gripper object
G = Gripper() 

# Since this is a new grasp, reset parameters to default
G.reset_parameters()

# The goal aperture for this grasp is 51 mm
goal_aperture = 51  # The specified aperture for the grasp

# Setting compliance to handle medium compliance object with low weight
# Compliance settings help in adjusting grip force without damaging the object
G.set_compliance(margin=5, flexibility=4, finger='both')

# Setting initial force to handle the clementine gently at the start of the grasp
G.set_force(1.0, 'both')

# Move the fingers to the specified goal aperture to initially grasp the clementine
G.set_goal_aperture(goal_aperture, finger='both')

# Adjusting goal aperture to confirm the grasp by closing an addition 2mm.
# This step helps in ensuring a secure grip on the clementine. 
current_aperture = G.get_aperture(finger='both')

# If the actual aperture differs from the goal due to object deformation or slippage, adjust accordingly
if current_aperture > goal_aperture: 
    G.set_goal_aperture(current_aperture - 2, finger='both')  # Tighten the grip slightly by 2mm to confirm grasp
    
    # Increase the force slightly after reaching the adjusted goal aperture to ensure the object is held securely without slipping.
    G.set_force(1.5, 'both')