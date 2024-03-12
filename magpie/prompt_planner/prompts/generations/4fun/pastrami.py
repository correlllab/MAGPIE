desc = """
[start of description]

This is a new grasp.
This grasp should be for delicately picking up a thin slice of pastrami, ensuring not to damage the meat.
This is a complete grasp.
This grasp does not contain multiple grasps.
This grasp is for an object with medium compliance.
This grasp is for an object with low weight.
The object has an approximate mass of 20 grams
The object has an approximate spring constant of 500 Newtons per meter.
The gripper and object have an approximate friction coefficient of 0.5
This grasp should set the goal aperture to 20 mm.
If the gripper slips, this grasp should close an additional 2 mm.
Based on object mass and friction coefficient, grasp should initially set the force to 0.392 Newtons.
If the gripper slips, this grasp should increase the output force by 0.1 Newtons. [end of description]
"""
# actual mass: 28g
from magpie.gripper import Gripper # Import the gripper class
G = Gripper() # Create a gripper object
import numpy as np  # Import numpy for calculations

# Reset parameters to default since this is a new, delicate grasp to avoid crushing the slice of pastrami.
G.reset_parameters()

# Setting up initial variables based on the grasp description
goal_aperture = 20  # Goal aperture in mm, for delicately picking up a thin slice of pastrami
complete_grasp = True  # This is a complete grasp
initial_force = 0.392  # Initial force calculated based on object weight and friction coefficient
additional_closure = 2  # Additional closure in mm if the gripper slips
additional_force = 0.1  # Additional output force in Newtons if the gripper slips

# Move quickly (without recording load) to a safe aperture that is slightly wider than the goal aperture, 
# to account for positional inaccuracies and ensure a gentle approach to the object.
G.set_goal_aperture(goal_aperture + 5, finger='both', record_load=False)

# Set compliance to allow for a bit of positional error in case of object deformation or unexpected obstacles,
# with moderate flexibility to adapt to the medium compliance of the object
G.set_compliance(1, 4, finger='both')

# Set initial gripping force, calculated to be sufficient to hold the object without dropping it due to its low weight
G.set_force(initial_force, 'both')

# Predicted: The gripper will approach the object gently and start closing, attempting to reach the desired aperture,
# with an initial force intended to establish a secure yet delicate grasp without damaging the slice of pastrami.
load_data = G.set_goal_aperture(goal_aperture, finger='both')

# Checking if the current aperture is different from the goal, potentially indicating slippage or incomplete grip
curr_aperture = G.get_aperture(finger='both')
applied_force = initial_force  # Starting with the initial force
slip_threshold = initial_force  # Initial force also serves as slip detection threshold

# Checking for slippage
slippage, avg_force, max_force = G.check_slip(load_data, slip_threshold, 'both')

while slippage:
    # If the gripper has slipped, reduce the goal aperture further and increase the gripping force
    goal_aperture = curr_aperture - additional_closure
    applied_force += additional_force  # Increase gripping force incrementally to avoid damaging the object
    
    # Reapply the updated force and attempt to grasp again
    G.set_force(applied_force, 'both')
    load_data = G.set_goal_aperture(goal_aperture, finger='both')
    
    # Check again if the gripper successfully established a firm grip without slippage
    curr_aperture = G.get_aperture(finger='both')
    slippage, avg_force, max_force = G.check_slip(load_data, slip_threshold, 'both')

# If the grasp is complete, optionally perform a final adjustment to ensure the grasp is secure
if complete_grasp:
    curr_aperture = G.get_aperture(finger='both')
    # Adjust the aperture one last time to ensure the grasp is as intended, without further recording
    G.set_goal_aperture(curr_aperture - additional_closure, finger='both', record_load=False)