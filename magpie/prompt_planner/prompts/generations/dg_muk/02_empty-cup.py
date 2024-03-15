"""
[start of description]

    This is not a new grasp.
    This grasp should be for picking up an empty paper cup with an aperture setting of 75 mm.
    This is a complete grasp.
    This grasp does not contain multiple grasps.
    This grasp is for an object with low compliance.
    This grasp is for an object with low weight.
    The object has an approximate mass of 10 grams.
    The object has an approximate spring constant of 50.0 Newtons per meter.
    The gripper and object have an approximate friction coefficient of 0.4.
    This grasp should set the goal aperture to 75.0 mm.
    If the gripper slips, this grasp should close an additional 5.0 mm.
    Based on object mass and friction coefficient, grasp should initially set the force to 0.25 Newtons.
    If the gripper slips, this grasp should increase the output force by 0.25 Newtons. [end of description]
"""

from magpie.gripper import Gripper
import numpy as np

# Create a gripper object
G = Gripper()

# This is not a new grasp, so we do not reset parameters to default.

# Specifications for the paper cup grasp
goal_aperture = 75.0  # in mm
complete_grasp = True  # complete grasp sequence
initial_force = 0.25  # Newtons, calculated from object mass and friction coefficient
additional_closure = 5.0  # mm, if the gripper detects slip this is how much it will close
object_mass = 0.01  # kg, mass of the paper cup
spring_constant = 50.0  # Newtons per meter
mu = 0.4  # friction coefficient

# Move quickly to a safe aperture that is wider than the goal aperture
G.set_goal_aperture(goal_aperture + 5, record_load=False)

# Set compliance and force for this type of grasp
G.set_compliance(1, 3)  # Set a compliance margin and slope.
G.set_force(initial_force)  # Set an initial force based on the object's weight and friction coefficient.

# Attempt to grasp the object
load_data = G.set_goal_aperture(goal_aperture, record_load=True)

# Initial conditions and check for slippage
curr_aperture = G.get_aperture()
applied_force = initial_force
slippage, avg_force, max_force = G.check_slip(load_data, initial_force)

while slippage:
    goal_aperture = curr_aperture - additional_closure  # Adjust goal aperture in case of slippage
    applied_force += 0.025  # Increase the force by the specified amount if there is slippage
    
    # Set the new force and attempt to grasp the object again
    G.set_force(applied_force)
    print(f"Adjusting: New goal aperture: {goal_aperture} mm, New applied force: {applied_force} N.")
    
    load_data = G.set_goal_aperture(goal_aperture, record_load=True)
    
    # Check if the gripper still slips after adjustments
    curr_aperture = G.get_aperture()
    slippage, avg_force, max_force = G.check_slip(load_data, applied_force)
    
if complete_grasp:
    curr_aperture = G.get_aperture()
    print(f"Final aperture: {curr_aperture} mm, Final applied force: {applied_force} N.")
else:
    # If for any reason the grasp is not to be completed
    G.open_gripper()