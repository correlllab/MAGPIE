# paper plane, sleeps are manual
# Reset parameters for a new grasp task, ensuring all previous settings are cleared.
# G.reset_parameters()

# [Grasping a delicate object like a paper airplane requires precision without exerting excessive force.]
# Setting compliance parameters to allow for a gentle grip, reducing risk of damage to the high compliant, low weight object.
G.set_compliance(margin=1, flexibility=5, finger='both')
# Setting a conservative initial force to avoid crushing the paper airplane, ensuring a gentle grip.
G.set_force(0.15, 'both')

# Getting the current goal aperture to adjust for the size of the object.
# goal_aperture = G.get_goal_aperture(finger='both')
goal_aperture = 18

# Adjusting the goal aperture slightly smaller than the initial to ensure contact with the object, considering its compliance and low weight.
G.set_goal_aperture(goal_aperture - 5, finger='both', record_load=False)

# # [To confirm the grasp without causing damage, a slight increase in force is needed after initial contact.]
# # Increasing the force slightly by 0.5N to ensure a secure grip on the paper airplane without applying too much pressure.
G.set_force(0.65, 'both')

# # Checking the current aperture to ensure the fingers have moved as intended.
curr_aperture = G.get_aperture(finger='both')
print(f'Current aperture: {curr_aperture} mm')

# # If the current aperture is not as expected, adjust the goal aperture slightly to improve the grip.
# # Here we assume there might be a need to adjust based on the object's compliance; adjust based on your guess or requirement.
G.set_goal_aperture(curr_aperture - 2, finger='both', record_load=False)
