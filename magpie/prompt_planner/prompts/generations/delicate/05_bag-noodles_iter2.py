from magpie.gripper import Gripper  # Import the necessary Gripper class.
import numpy as np  # NumPy is imported for potential array manipulations, even though this specific example does not utilize it.

# Create a Gripper object instance.
G = Gripper()

# First, reset grasping parameters to default states for new grasp operations.
G.reset_parameters()

# The goal aperture is set to the maximum of 105 mm, as specified for the initial grip on the bag.
goal_aperture = 105.0

# The initial force is set to 2 Newtons (N) as per the requirements for a secure yet delicate grip on the bag.
initial_force = 4.0
G.set_force(initial_force, 'both')  # Apply the force setting to both fingers identically for balanced gripping.

# The initial grasp attempt with the specified goal aperture and force.
load_data = G.set_goal_aperture(goal_aperture, finger='both', record_load=True)

# If the gripper slips (i.e., unable to maintain the set force), the aperture will be decreased by 2 mm, and the force increased by 0.5 N.
additional_closure = 4.0  # mm to close further if slipping is detected.
additional_force = 1.0 # Newtons to add to the grip force if slipping occurs.

# Check for slipping. If it occurs, adjust the goal aperture and force accordingly.
while G.check_slip(load_data, initial_force, 'both'):
    goal_aperture -= additional_closure  # Reduce the goal aperture by the specified additional closure amount.
    initial_force += additional_force  # Increase the force according to the specified additional force.

    # Apply the new settings for both aperture and force.
    G.set_force(initial_force, 'both')
    load_data = G.set_goal_aperture(goal_aperture, finger='both', record_load=True)

    # Informative print statement for debug or monitoring (optional).
    print(f"Adjustments made due to slip: New goal aperture = {goal_aperture} mm, New force = {initial_force} N")