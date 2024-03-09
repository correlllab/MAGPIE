from magpie.gripper import Gripper # must import the gripper class
import numpy as np  # import numpy because we will use it to analyze the load data

G = Gripper() # create a gripper object

# This grasp starts with setting up an appropriate aperture and force for a delicate paper cup. 
# It's important that the grip is gentle to avoid deformation and that adjustments are made in the event of slipping.

G.set_compliance(1, 3, finger='both') # Set a small margin for error to ensure precision with reasonable flexibility

goal_aperture = 70.0  # Set the goal aperture based on the size needed to grip the paper cup without squeezing too tightly.
stop_force = 0.5  # Start with a gentle force to avoid crushing the object. This is the initial force setting.
additional_closure = 1.0  # If the grip is not secure and slips, we'll close the gripper a bit more.
additional_force = 0.2  # In case of slipping, increase the force slightly to ensure a secure grip without applying too much pressure.

# Set the initial force. Given the object is a paper cup with very low weight, a low force is needed to avoid damage.
G.set_force(stop_force, 'both')

# Move the fingers to the specified goal aperture, recording the load to adjust in case of slip.
load_data = G.set_goal_aperture(goal_aperture, finger='both', record_load=True)

# Checking for slips and adjusting the force and aperture accordingly.
# We assume that the initial settings may not achieve a perfect grip due to inaccuracies in object dimensions or positioning.
applied_force = stop_force  # Initialize applied_force with the initial stop_force value.

while G.check_slip(load_data, stop_force, 'both'):  # Keep checking if the initial force setting results in slipping.
    goal_aperture -= additional_closure  # Reduce goal aperture slightly to tighten the grip.
    applied_force += additional_force  # Increase the force gently to improve grip without causing damage.
    G.set_force(applied_force, 'both')  # Update the applied force on the gripper.
    # Regrip with the new, slightly reduced aperture and increased force, recording load data again for further analysis if necessary.
    load_data = G.set_goal_aperture(goal_aperture, finger='both', record_load=True) 
    print(f"Adjusting grip: Goal Aperture = {goal_aperture}mm, Applied Force = {applied_force}N.")

# At this point, we predict the gripper has successfully adjusted to prevent slip and securely grip the paper cup.