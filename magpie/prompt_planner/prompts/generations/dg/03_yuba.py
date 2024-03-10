from magpie.gripper import Gripper
import numpy as np

G = Gripper()

# Reset parameters for a new task.
G.reset_parameters() 

# Initial settings for the gentle grasp
goal_aperture = 50  # Set the goal aperture to 50 mm
initial_force = 0.5  # Initially set the force to 0.3 Newtons

# Set the initial force and aperture.
G.set_force(initial_force, 'both')
load_data = G.set_goal_aperture(goal_aperture, finger='both', record_load=True)  # Record load to monitor for slip

# Slip handling
additional_closure = 2  # Close an additional 1 mm if the gripper slips
force_increment = 0.3  # Increase the force by 0.1 Newtons if the gripper slips
curr_force = initial_force  # Track the current force

while G.check_slip(load_data, curr_force, 'both'):
    goal_aperture -= additional_closure  # Decrease goal aperture to tighten grip
    curr_force += force_increment  # Increase force to ensure a firmer grip
    G.set_force(curr_force, 'both')  # Apply the updated force
    
    # Attempting the grasp again with the stricter settings
    load_data = G.set_goal_aperture(goal_aperture, finger='both', record_load=True)  # Keep recording load
    print(f"Adjusted grasp: goal_aperture = {goal_aperture}mm, force = {curr_force}N. Trying again.")

print("Grasp successful without slipping. Moving forward.")