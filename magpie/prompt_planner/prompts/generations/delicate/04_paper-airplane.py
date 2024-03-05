from magpie.gripper import Gripper  # Import the Gripper class
G = Gripper()  # Create a Gripper object

# Parameters based on the description
initial_goal_aperture = 20  # Initial goal aperture is 20 mm
additional_closure = 1  # If the gripper slips, close an additional 1 mm
initial_force = 0.15  # Initial force is set to 0.15 Newtons
force_increase = 0.05  # If the gripper slips, increase the force by 0.05 Newtons

# Set initial parameters for the grasp
G.set_force(initial_force, 'both')  # Set the initial force to be gentle as the object is delicate
load_data = G.set_goal_aperture(initial_goal_aperture, finger='both', record_load=True)  # Set the initial goal aperture

# Check for slip and adjust accordingly
while G.check_slip(load_data, initial_force, 'both'):
    # If the gripper slips, decrease aperture by an additional 1 mm and increase force by 0.05 Newtons
    initial_goal_aperture -= additional_closure  # Adjust the goal aperture
    initial_force += force_increase  # Adjust the force
    G.set_force(initial_force, 'both')  # Set the new force
    load_data = G.set_goal_aperture(initial_goal_aperture, finger='both', record_load=True)  # Set the new goal aperture and record load data
    print(f"Slip detected. Adjusting goal aperture to {initial_goal_aperture} mm and force to {initial_force} N.")
    
# At this point, the grasp should have been adjusted to avoid slips
print("Grasp adjustment complete.")