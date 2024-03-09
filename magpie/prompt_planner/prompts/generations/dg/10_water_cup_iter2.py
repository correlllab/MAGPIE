from magpie.gripper import Gripper  # must import the gripper class
import numpy as np  # import numpy because we will use numpy arrays

G = Gripper()  # create a gripper object

# Before starting a new grasp, it's a good idea to reset any previous parameters or configurations
G.reset_parameters()

# Initial setup for a stable and precise grasp for a paper cup with a 60 mm aperture, using minimal force to avoid deformation
goal_aperture = 70.0  # The goal aperture is set to 60 mm to match the size of the paper cup
additional_closure = 5.0  # If the gripper slips, it will close an additional 2 mm
initial_force = 3.0  # The initial force is set to 0.8 Newtons to avoid deforming the paper cup
force_increment = 1.0  # If the gripper slips, the force will increase by 0.4 Newtons

# Setting the compliance for a more stable grasp without causing damage to the cup
G.set_compliance(margin=1, flexibility=3, finger='both')

# Setting the initial force
G.set_force(initial_force, finger='both')

# Attempting to grasp the paper cup by moving the fingers to the specified goal aperture
load_data = G.set_goal_aperture(goal_aperture, finger='both', record_load=True)

# While loop to continuously check for slip and adjust the aperture and force accordingly
slipped = True
applied_force = initial_force
while slipped:
    # Check if the gripper has slipped by checking if the contact force is not met
    slipped = G.check_slip(load_data, initial_force, finger='both')
    
    if slipped:
        # If the gripper has slipped, reduce the goal aperture and increase the force for a better grip
        goal_aperture -= additional_closure
        applied_force += force_increment
        
        # Apply the new force setting
        G.set_force(applied_force, finger='both')
        
        # Move the fingers to the new, adjusted goal aperture and record the load data again
        load_data = G.set_goal_aperture(goal_aperture, finger='both', record_load=True)
        
        print(f"Adjusting for slip: new aperture set to {goal_aperture} mm, force set to {applied_force} N.")