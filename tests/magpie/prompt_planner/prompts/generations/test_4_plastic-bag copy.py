from magpie.gripper import Gripper # Import the Gripper class to use its capabilities
import numpy as np  # Import numpy for potential array manipulations

G = Gripper() # Create an instance of the Gripper

# This is a new task, so we reset all parameters to default and open the gripper.
G.reset_parameters()

# Set the target for how much the gripper should open to hold the plastic bag without causing damage.
# The initial goal aperture is set based on the provided description for the plastic bag.
# This initial aperture ensures a careful approach towards the fragile contents.
goal_aperture = 79 # The desired initial gap between the gripper fingers to safely grab the bag.

# Strategy: Setting a gentle compliance and low force initially to avoid damaging the contents. As we approach the object, 
# and once the goal aperture is reached, we slightly increase the closure and the force to ensure a firm yet safe grip.
G.set_compliance(10, 3, finger='both') # Sets a compliance to allow for some positional error without excessive force.
G.set_force(2.0, 'both') # Sets an initial gentle force to avoid damaging the fragile contents within the bag.

# Move the gripper fingers to the specified goal aperture. This action starts the gripping process.
G.set_goal_aperture(goal_aperture, finger='both')

# Checking the current aperture to adjust if there's a discrepancy between the goal and the actual position.
curr_aperture = G.get_aperture(finger='both')
if curr_aperture != goal_aperture:
    G.set_goal_aperture(goal_aperture, finger='both') # Re-adjust aperture if needed based on actual position.

# Prediction: The gripper has been carefully positioned around the bag, applying gentle force to prevent damage.

# Strategy: To confirm the grasp, the gripper closes an additional 3mm and increases the force slightly.
# This ensures that the grip is secure without applying excessive force that could harm the fragile components.
additional_closure = 3
confirmed_aperture = curr_aperture - additional_closure # Calculate the new goal aperture after tightening the grip.
G.set_goal_aperture(confirmed_aperture, finger='both') # Tighten the grip.
G.set_force(3.0, 'both') # Increase the force slightly to ensure the grasp is confirmed without risking damage.

# Prediction: The gripper has now firmly secured the plastic bag. The increased force and reduced aperture confirm the grip while still being cautious of the contents' fragility.