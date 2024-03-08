'''
Control a robot gripper with torque control and contact information. 
This is a griper with two independently actuated fingers, each on a 4-bar linkage.
The gripper's parameters can be adjusted corresponding to the type of object that it is trying to grasp.
As well as the kind of grasp it is attempting to perform.
The gripper has a measurable max force of 16N and min force of 0.15N, a maximum aperture of 105mm and a minimum aperture of 1mm.
Some grasps may be incomplete, intended for observing force information about a given object.
Describe the grasp strategy using the following form:

[start of description]
* This {CHOICE: [is, is not]} a new grasp.
* This grasp should be [GRASP_DESCRIPTION: <str>].
* This is a {CHOICE: [complete, incomplete]} grasp.
* This grasp {CHOICE: [does, does not]} contain multiple grasps.
* This grasp is for an object with {CHOICE: [high, medium, low]} compliance.
* This grasp is for an object with {CHOICE: [high, medium, low]} weight.
* The object has an approximate mass of [PNUM: 0.0] grams
* The object has an approximate spring constant of [PNUM: 0.0] Newtons per meter.
* The gripper and object have an approximate friction coefficient of [PNUM: 0.0]
* This grasp should set the goal aperture to [PNUM: 0.0] mm.
* If the gripper slips, this grasp should close an additional [PNUM: 0.0] mm..
* This grasp should initially set the force to [PNUM: 0.0] Newtons.
* If the gripper slips, this grasp should increase the force by [PNUM: 0.0] Newtons.
* [optional] The left finger should move [NUM: 0.0] millimeters inward (positive)/outward (negative).
* [optional] The right finger should move [NUM: 0.0] millimeters inward (positive)/outward (negative).
* [optional] The left finger have velocity [NUM: 0.0] millimeters/sec inward (positive)/outward (negative).
* [optional] The right finger have velocity [NUM: 0.0] millimeters/sec inward (positive)/outward (negative).
* [optional] The gripper should approach at [NUM: 0.0] millimeters away on the Z-axis.
[end of description]

Rules:
1. If you see phrases like [NUM: default_value], replace the entire phrase with a numerical value. If you see [PNUM: default_value], replace it with a positive, non-zero numerical value.
2. If you see phrases like {CHOICE: [choice1, choice2, ...]}, it means you should replace the entire phrase with one of the choices listed. Be sure to replace all of them. If you are not sure about the value, just use your best judgement.
3. If you see phrases like [GRASP_DESCRIPTION: default_value], replace the entire phrase with a brief, high level description of the grasp and the object to be grasp, including physical characteristics or important features.
4. By default the minimum grasp force can be estimated by dividing the object weight (mass * gravitational constant) by the friction coefficient.
5. Using knowledge of the object and the grasp description, set the initial grasp force either to this default value or an appropriate value. Explain your reasoning if you deviate from this default value.
6. Using knowledge of the object and how compliant it is, estimate the spring constant of the object. This can range broadly from 20 N/m for a very soft object to 2000 N/m for a very stiff object. 
9. Using knowledge of the object and the grasp description, if the grasp slips, first estimate an appropriate increase to the aperture closure, and then the gripper output force.
8. By default, the increase in gripper output force is the product of the estimated aperture closure and the spring constant of the object. Explain your reasoning if you deviate from this default value.
10. I will tell you a behavior/skill/task that I want the gripper to perform in the grasp and you will provide the full description of the grasp plan, even if you may only need to change a few lines. Always start the description with [start of description] and end it with [end of description].
11. We can assume that the gripper has a good low-level controller that maintains position and force as long as it's in a reasonable pose.
12. The goal aperture of the gripper will be supplied externally, do not calculate it.
13. Do not add additional descriptions not shown above. Only use the bullet points given in the template.
14. If a bullet point is marked [optional], do NOT add it unless it's absolutely needed.
15. Use as few bullet points as possible. Be concise.
'''