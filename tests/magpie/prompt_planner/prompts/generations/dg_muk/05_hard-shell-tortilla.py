"""
[start of description]

    This is a new grasp.
    This grasp should be gentle to avoid crushing the fragile shell.
    This is a complete grasp.
    This grasp does not contain multiple grasps.
    This grasp is for an object with low compliance.
    This grasp is for an object with low weight.
    The object has an approximate mass of 50 grams
    The object has an approximate spring constant of 1000 Newtons per meter.
    The gripper and object have an approximate friction coefficient of 0.5
    This grasp should set the goal aperture to 50.0 mm.
    If the gripper slips, this grasp should close an additional 2.0 mm.
    Based on object mass and friction coefficient, grasp should initially set the force to 0.98 Newtons.
    If the gripper slips, this grasp should increase the output force by 1.0 Newtons. [end of description]"""

from magpie.gripper import Gripper
import numpy as np

# Initialize the Gripper object
G = Gripper()

# Since this is a new, delicate grasp, we will start by resetting to default parameters
# Assuming reset_parameters is a function to reset to default parameters for safety, not provided in original class functions.
# G.reset_parameters()

# Details from the description
new_task = True
goal_aperture = 65.0  # Set the goal aperture to 65.0 mm
complete_grasp = True
initial_force = 0.98  # Initial force shared in description based on computation (50g object, friction coefficient of 0.5)
additional_closure = 2.0  # Close additional 2.0 mm if slippage is detected
spring_constant = 1000  # Spring constant of the object
additional_force = (additional_closure * spring_constant) * 0.0001  # Increase output force by 0.2 N if slippage

# Move quickly to a safe aperture that is slightly wider than the goal aperture
G.set_goal_aperture(goal_aperture + 3, record_load=False)

# Set compliance to handle low compliance objects
G.set_compliance(1, 3)

# Set the initial force for the grasp
G.set_force(initial_force)

# Attempt to close to the desired goal aperture
load_data = G.set_goal_aperture(goal_aperture, record_load=True)

# Check for initial slippage and record the average and max force encountered
slippage, avg_force, max_force = G.check_slip(load_data, initial_force)

# Contingency for slippage: adjust grip strength and aperture based on feedback
applied_force = initial_force
k_avg = []  # Initialize list to store spring constants over slip detection
while slippage:
    goal_aperture -= additional_closure
    applied_force += additional_force
    # Log adjustments
    print(f"Adjusting: Goal Aperture to {goal_aperture} mm, Applied Force to {applied_force} N.")

    # Re-attempt to grasp with adjusted parameters
    G.set_force(applied_force)
    load_data = G.set_goal_aperture(goal_aperture, record_load=True)
    slippage, avg_force, max_force = G.check_slip(load_data, applied_force)
    
    # Calculate the distance and update k_avg after reshaping the fingers
    curr_aperture = G.get_aperture()
    distance = goal_aperture - curr_aperture
    k_avg.append(np.mean(avg_force) * distance * 1000.0)  # Assuming distance is in meters for spring constant calculation

# Finalize grasp based on complete_grasp requirement
if complete_grasp:
    curr_aperture = G.get_aperture()
    print(f"Final adjustments: Goal Aperture to {curr_aperture - 2} mm to ensure a secure grip.")
    G.set_goal_aperture(curr_aperture - 2, record_load=False)
else:
    # Assuming open_gripper is a method to fully open the gripper, not provided in original class functions.
    # G.open_gripper()
    print("Opening gripper - incomplete grasp scenario.")