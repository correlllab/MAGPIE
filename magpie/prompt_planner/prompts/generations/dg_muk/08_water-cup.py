"""
[start of description]

    This is not a new grasp.
    This grasp should be for picking up a paper cup filled with water.
    This is a complete grasp.
    This grasp does not contain multiple grasps.
    This grasp is for an object with low compliance.
    This grasp is for an object with low weight.
    The object has an approximate mass of 250.0 grams
    The object has an approximate spring constant of 200.0 Newtons per meter.
    The gripper and object have an approximate friction coefficient of 0.6
    This grasp should set the goal aperture to 75.0 mm.
    If the gripper slips, this grasp should close an additional 2.0 mm.
    Based on object mass and friction coefficient, grasp should initially set the force to 4.08 Newtons.
    If the gripper slips, this grasp should increase the output force by 0.4 Newtons. [end of description]"""

from magpie.gripper import Gripper # Must import the gripper class
import numpy as np  # Importing numpy for calculations

G = Gripper() # Create a gripper object

# This is not a new grasp, therefore, we don't need to reset gripper parameters
# This grasp is for picking up a paper cup filled with water, which has low compliance and is light-weight
# The goal is to achieve a secure, complete grasp without crushing the object

# Set the goal aperture based on object size - 75mm is wide enough to enclose the paper cup without touch
goal_aperture = 75.0 # Given goal aperture
initial_force = 4.08 # Given initial force calculated based on object mass and friction coefficient
additional_closure = 2.0 # Additional closure if the gripper slips
additional_force = 0.4 # Additional force increment if the gripper slips

# Move quickly (without recording load) to a safe aperture that is slightly wider than the goal aperture
G.set_goal_aperture(goal_aperture + 5, finger='both', record_load=False) # Add 5mm to ensure we start without contact

# Set compliance to moderate values, considering the object has low compliance but we don't want to crush it
G.set_compliance(1, 3, finger='both')

# Initially set the force based on object weight and friction coefficient
G.set_force(initial_force, 'both')

# Attempt to close to the goal aperture, recording the load for potential slippage analysis
load_data = G.set_goal_aperture(goal_aperture, finger='both', record_load=True)

# Check current aperture after attempting to set goal aperture
curr_aperture = G.get_aperture(finger='both')
applied_force = initial_force
slip_threshold = initial_force

# Check if the gripper slipped when initially trying to grasp the object
slippage, avg_force, max_force = G.check_slip(load_data, slip_threshold, 'both')

while slippage:
    # Increase the force slightly if the gripper slipped, based on the given increment
    applied_force += additional_force
    
    # Close the gripper an additional 2.0 mm to ensure a firmer grasp
    goal_aperture = curr_aperture - additional_closure
    
    # Apply the updated force
    G.set_force(applied_force, 'both')
    
    # Try to close the gripper again with the updated parameters
    load_data = G.set_goal_aperture(goal_aperture, finger='both', record_load=True)
    
    # Check the aperture again
    curr_aperture = G.get_aperture(finger='both')
    
    # Re-check for slippage with the updated force and closure
    slippage, avg_force, max_force = G.check_slip(load_data, slip_threshold, 'both')

# If the grasp should be complete, just ensure the current position is maintained
# No additional closing is necessary since we're already adjusting for slippage in the loop
print(f"Final attempt: Aperture set to {curr_aperture} mm with applied force of {applied_force} N.")