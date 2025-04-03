from magpie.gripper import Gripper # must import the gripper class
import numpy as np  # Importing numpy for potential mathematical operations

G = Gripper() # Create a gripper object

# The goal is to grasp a ripe avocado gently, which requires setting a specific aperture and force,
# adapting these settings if slippage is detected without causing damage to the object.

goal_aperture = 70.0  # Set the goal aperture to 60mm to gently grasp the avocado

# We assume that the avocado has medium compliance and is of low weight, thus setting a moderate starting force
# and a compliance that allows for some flexibility in the grip
initial_force = 1.0  # Initial force set to 1.0N to avoid damaging the avocado
additional_closure = 3.0  # The gripper will close an additional 3.0mm if it slips
additional_force = 0.5  # The force increases by 0.5N if the gripper slips

# Setting a moderate compliance margin and flexibility to accommodate the avocado's medium compliance
G.set_compliance(margin=1, flexibility=4, finger='both')

# Setting the initial force to be gentle on the avocado
G.set_force(initial_force, 'both')

# Moving the gripper fingers to the specified goal aperture
load_data = G.set_goal_aperture(goal_aperture, finger='both', record_load=True)

# Initializing applied_force with initial_force
applied_force = initial_force

# Check if the gripper has slipped, indicating that the grip was not firm enough
while G.check_slip(load_data, applied_force, 'both'):
    print("Slip detected. Adjusting gripper settings.")
    goal_aperture -= additional_closure  # Decreasing the aperture to tighten the grip
    applied_force += additional_force  # Increasing the force for a firmer grip
    G.set_force(applied_force, 'both')  # Updating the force settings
    # Attempting the grasp again with the adjusted settings
    load_data = G.set_goal_aperture(goal_aperture, finger='both', record_load=True)

# The conditions above - decreasing the aperture on slip and increasing the force - ensure that
# adjustments are made to secure the avocado more firmly without causing damage.