"""
[start of description]

    This is not a new grasp.
    This grasp should be for picking up a soft plastic squeeze bottle full of water with a narrow aperture.
    This is a complete grasp.
    This grasp does not contain multiple grasps.
    This grasp is for an object with low compliance.
    This grasp is for an object with medium weight.
    The object has an approximate mass of 500 grams.
    The object has an approximate spring constant of 150 Newtons per meter.
    The gripper and object have an approximate friction coefficient of 0.4.
    This grasp should set the goal aperture to 48.0 mm.
    If the gripper slips, this grasp should close an additional 2.0 mm.
    Based on object mass and friction coefficient, grasp should initially set the force to 8.2 Newtons.
    If the gripper slips, this grasp should increase the output force by 0.03 Newtons. [end of description]
    """

from magpie.gripper import Gripper  # Must import the gripper class
import numpy as np  # Import numpy for calculations

G = Gripper()  # Create a gripper object

# Since this is not a new grasp, the gripper's parameters do not need to be reset to defaults.

# The goal is to achieve a complete and secure grasp around a soft plastic squeeze bottle full of water.
# The chosen aperture and force settings consider the bottle's characteristics: medium weight, low compliance, and specified measurements.

goal_aperture = 48.0  # Set goal aperture to 48.0 mm based on the object's narrow opening.
initial_force = 12.2  # Initial force setting of 8.2 Newtons based on object mass and friction coefficient.
additional_closure = 2.0  # Additional closure of 5.0 mm if the gripper slips.
additional_force = 0.03  # Increase in force by 0.075 Newtons if the gripper slips.

# Move quickly to a safe aperture that is slightly wider than the goal aperture without recording load, to accommodate the bottle.
G.set_goal_aperture(goal_aperture + 3, finger='both', record_load=False)

# Set compliance to allow some flexibility during the grasp, considering the object's low compliance.
G.set_compliance(1, 3, finger='both')
# Initially set the force based on the calculated value.
G.set_force(initial_force, finger='both')
# Move the fingers to the goal aperture and start recording load data for slip detection.
load_data = G.set_goal_aperture(goal_aperture, finger='both', record_load=True)

# Checking the actual aperture after attempting to reach the goal aperture.
curr_aperture = G.get_aperture(finger='both')
# Initialize applied_force and slip_threshold to initial_force for slip detection.
applied_force = initial_force
slip_threshold = initial_force
# Check for slipping. If it occurs, it indicates that the gripper does not have a solid grasp on the bottle.
slippage, avg_force, max_force = G.check_slip(load_data, slip_threshold, 'both')

# Iteratively adjust the grasp based on slip detection results, increasing force and decreasing aperture as necessary.
while slippage:
    goal_aperture = curr_aperture - additional_closure  # Adjust goal aperture for stronger grip.
    # Increase applied force only if necessary, based on force reading to ensure we're not applying too much force.
    if np.mean(avg_force) > 0.1:  # This acts as a low-pass filter on force adjustments.
        applied_force += additional_force
    G.set_force(applied_force, 'both')  # Update the force based on slip detection and additional force calculation.
    print(f"Adjusting grip: Goal Aperture: {goal_aperture} mm, Applied Force: {applied_force} N.")
    load_data = G.set_goal_aperture(goal_aperture, finger='both', record_load=True)

    # Re-evaluate the grip after adjustment by checking the actual aperture and detecting slippage.
    curr_aperture = G.get_aperture(finger='both')
    slippage, avg_force, max_force = G.check_slip(load_data, slip_threshold, 'both')

# Complete the grip by ensuring a firm grasp. This steps slightly reduces the aperture to ensure the object is held securely.
G.set_goal_aperture(curr_aperture - 2, finger='both', record_load=False)
print(f"Final grasp achieved: Aperture: {curr_aperture - 2} mm, Applied Force: {applied_force} N.")