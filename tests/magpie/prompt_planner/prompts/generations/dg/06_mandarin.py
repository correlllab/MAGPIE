from magpie.gripper import Gripper # Import the Gripper class
import numpy as np  # Import numpy for potential future data handling

# Create a gripper object
G = Gripper()

# Resetting the gripper's parameters for the new task
G.reset_parameters()

# Setting the initial goal aperture for grasping the mandarin orange
goal_aperture = 50.0  # Goal aperture is set to 50.0mm to gently embrace the mandarin without applying too much pressure

# Setting a more forgiving compliance to accommodate the high compliance and low weight of the mandarin orange
G.set_compliance(margin=1, flexibility=5, finger='both')  # Lower margin and higher flexibility to adjust to the orange's compliance without causing damage

# Setting the initial force to gently grasp the mandarin orange without causing damage
initial_force = 1.25  # Initial force is set to 1.2N to ensure a gentle grasp
G.set_force(initial_force, 'both')

# Attempting the initial grasp
load_data = G.set_goal_aperture(goal_aperture, finger='both', record_load=True)

# Checking if the applied force was sufficient for a secure grasp or if there was a slip
has_slipped = G.check_slip(load_data, initial_force, 'both')

# Initialize the applied force to the initial stop force
applied_force = initial_force

# Applying corrections upon detecting a slip during the initial grasp attempt
while has_slipped:
    # Decrease the goal aperture by 3.0mm for a tighter grasp if a slip is detected
    goal_aperture -= 1.0
    
    # Increase the applied force by 0.6N to enhance the grasping strength without damaging the mandarin
    applied_force += 0.1
    
    # Update the force setting on the gripper to apply the increased force
    G.set_force(applied_force, 'both')
    
    # Attempting the grasp again with the adjusted settings
    load_data = G.set_goal_aperture(goal_aperture, finger='both', record_load=True)
    
    # Check for slip again with the adjusted force and aperture
    has_slipped = G.check_slip(load_data, initial_force, 'both')
    
    print(f"Adjusting goal aperture to {goal_aperture}mm and force to {applied_force}N to secure the grasp.")