# This grasp is starting fresh, so resetting any previous configurations is a good practice although not specified
G.reset_parameters() 

# Set the goal aperture to 48mm for grabbing the top edge of a medium compliance, low weight, empty cardboard box
goal_aperture = 48 

# Setting compliance with a guessed margin and flexibility for medium compliance object to adapt to the grip gently
# Compliance margin is set to a small value as we require precision for the grasp
# Flexibility is set to a mid-range as the object has medium compliance. 
G.set_compliance(margin=2, flexibility=4, finger='both') 

# Initially setting the force to 1.5N to securely grip the low weight object without damaging it.
# The force is set to a low value considering the object's low weight and to prevent crushing it.
initial_force = 1.5 
G.set_force(initial_force, finger='both') 

# Moving the fingers to the set goal aperture, ready to grasp the object.
# Record the load to monitor if the gripper slips. 
load_data = G.set_goal_aperture(goal_aperture, finger='both', record_load=True)

# Checking the current aperture after setting the goal aperture. Adjust if necessary.
curr_aperture = G.get_aperture(finger='both') 
G.set_goal_aperture(curr_aperture, finger='both')

# Checking for slip. If there is slip, tighten grip by closing an additional 2mm and increase the force by 1N.
additional_closure = 2 
additional_force = 1 
while G.check_slip(load_data, initial_force, 'both', debug=True):
    # Tighten the grip by reducing the aperture and increasing the force incrementally
    goal_aperture -= additional_closure
    initial_force += additional_force
    
    # Apply the new tighter grip settings
    G.set_force(initial_force, 'both')
    load_data = G.set_goal_aperture(goal_aperture, finger='both', record_load=True, debug=True)