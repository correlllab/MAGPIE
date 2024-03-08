from magpie.gripper import Gripper  # Import the Gripper class

# Create a Gripper object
G = Gripper()

# Reset parameters for a new task
G.reset_parameters()

# Set the goal aperture to 45.0 mm as this is a precise grasp for a low weight and low compliance object
goal_aperture = 45.0  # in mm

# Initially, set the force to 1.5 Newtons to ensure the taco isn't broken upon grasping
initial_force = 0.5  # in Newtons

# Increase force by 0.5 Newtons if the gripper slips
additional_force = 0.1  # in Newtons

# Close an additional 2.0 mm if the gripper slips while trying to grasp the taco
additional_closure = 1.0  # in mm

# Set the initial force which the gripper applies
G.set_force(initial_force, finger='both')

# Move the gripper fingers to the specified goal aperture; also record load data to check for slip later
load_data = G.set_goal_aperture(goal_aperture, finger='both', record_load=True)

# Assuming compliance is already set to desired values. If not, you can set it using G.set_compliance() here.

# Check if the gripper has slipped. If it has, increase force by 0.5 Newtons and close an additional 2.0 mm
if G.check_slip(load_data, initial_force, finger='both'):
    # Adjust aperture and force based on slip detection
    goal_aperture -= additional_closure  # Adjust aperture by closing an additional 2.0 mm
    applied_force = initial_force + additional_force  # Increase force by 0.5 Newtons
    
    # Update the force settings of the gripper
    G.set_force(applied_force, 'both')
    
    # Try grasping again with new settings
    load_data = G.set_goal_aperture(goal_aperture, finger='both', record_load=True)
    print(f"Slip detected. Adjusting goal aperture to {goal_aperture}mm and force to {applied_force}N.")
else:
    print("Grasp successful with no slip detected.")