"""
This is a new grasp.
This grasp should be a secure, enveloping grip around a plastic package of dried rice noodles.
This is a complete grasp.
This grasp does not contain multiple grasps.
This grasp is for an object with low compliance.
This grasp is for an object with low weight.
The object has an approximate mass of 500 grams
The object has an approximate spring constant of 300 Newtons per meter.
The gripper and object have an approximate friction coefficient of 0.4
This grasp should set the goal aperture to 90.0 mm.
If the gripper slips, this grasp should close an additional 5.0 mm.
Based on object mass and friction coefficient, grasp should initially set the force to 12.3 Newtons.
If the gripper slips, this grasp should increase the output force by 0.15 Newtons. [end of description]
"""

from magpie.gripper import Gripper # Import the Gripper class
import numpy as np  # Import numpy for numeric operations

G = Gripper() #Create a Gripper object

# Since this is a new grasp and for a delicate object, reset gripper parameters to default
G.reset_parameters()

# Define initial parameters for the grasp based on the task description
goal_aperture = 90.0  # Goal aperture for the grasp
complete_grasp = True  # This is a complete grasp
initial_force = 12.3  # Initial force calculated from object mass and friction coefficient
additional_closure = 5.0   # Additional closure if the gripper slips
additional_force = 0.15  # Additional force to add if the gripper experiences slip

# Move to a safe aperture that is slightly wider than the goal to position the gripper around the object without recording load
G.set_goal_aperture(goal_aperture + 10, finger='both', record_load=False)

# Setting compliance to allow for some flexibility given the low weight and compliance of the object
G.set_compliance(1, 3, finger='both')

# Setting the initial force to ensure a secure grip without exceeding the object's compliance limit
G.set_force(initial_force, 'both')

# Moving to the goal aperture considering additional closure as a safety measure
load_data = G.set_goal_aperture(goal_aperture, finger='both', record_load=True)

# Check the current aperture after moving
curr_aperture = G.get_aperture(finger='both')

# Setting initial values for slip detection and force application
applied_force = initial_force
slip_threshold = initial_force
slippage, avg_force, max_force = G.check_slip(load_data, slip_threshold, 'both')

# If there is slippage, increase force incrementally and close the gripper a bit more
while slippage:
    # Incrementally close the gripper if there is slippage
    goal_aperture = curr_aperture - additional_closure
    
    # Only increase force if there has been contact
    applied_force += additional_force
    
    # Apply the increased force
    G.set_force(applied_force, 'both')
    
    # Move the gripper fingers to the new goal aperture
    load_data = G.set_goal_aperture(goal_aperture, finger='both', record_load=True)
    
    # Check for slippage again after adjustment
    curr_aperture = G.get_aperture(finger='both')
    slippage, avg_force, max_force = G.check_slip(load_data, slip_threshold, 'both')

    # Prepare for another loop iteration if necessary
    goal_aperture = curr_aperture

# If grasp is complete, slightly tighten the grip for a secure hold
if complete_grasp:
    final_goal_aperture = curr_aperture - 2  # Tighten grip slightly for added security
    G.set_goal_aperture(final_goal_aperture, finger='both', record_load=False)
    print(f"Final aperture: {curr_aperture} mm, Controller Goal Aperture: {final_goal_aperture} mm, Applied Force: {applied_force} N.")
else:
    # If for any reason the grasp is not complete, open the gripper and release the object
    G.open_gripper()