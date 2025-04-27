## this WORKS
R_cw = np.array([ # camera to wrist, rotate pi/2 about Z
    [0, -1, 0],
    [1,  0, 0],
    [0,  0, 1]
])
# R_wkspc_table = sm.SO3.Rx(np.pi).A @ pre_rotation
R_wkspc_table = sm.SO3.Rx(np.pi).A
# R_wkspc_chair = sm.SO3.Rx(np.pi).A @ sm.SO3.Rx(-np.pi/2).A @ sm.SO3.Rz(np.pi/4).A @ pre_rotation
R_wkspc_chair = sm.SO3.Rx(np.pi).A @ sm.SO3.Rx(-np.pi/2).A @ sm.SO3.Rz(np.pi/4).A
R_wkspc_ego = sm.SO3.Rz(0).A # no transformation
wkspc_config = {"chair": R_wkspc_chair, "table": R_wkspc_table, "ego": R_wkspc_ego}
R_wkspc = wkspc_config[wkspc_cam_config]
T_wr = np.array(robot.getPose())
T_init = T_wr.copy()
R_wr = T_wr[:3, :3]
R_corrected, R_c = optimize_and_correct_frame(R_wr.copy())
R_cr = R_cw @ R_wr.T @ pre_rotation
R_cr = R_cw @ R_wr.T
vlm_R_wrist = R_c @ pre_rotation
rotate_wrist_in_wkspc = False
if rotate_wrist_in_wkspc:
    # wrist_pre_rotation = sm.SO3.Rz(0).A
    wrist_pre_rotation = sm.SO3.Rz(np.pi/2).A
    # c_R_wkspc = R_wkspc @ R_cr.T @ wrist_pre_rotation # I don't know why this works
    # c_R_wkspc = R_wkspc @ R_cr.T # I don't know why this works
    c_R_wkspc = R_wkspc @ R_corrected @ pre_rotation # this works
else:
    wrist_pre_rotation = sm.SO3.Rz(-np.pi/2).A
    c_R_wkspc = R_wkspc @ pre_rotation @ R_cr.T # I don't know why this works
    # c_R_wkspc = R_wkspc @ R_cr.T # I don't know why this works
    wrist_rot = sm.SO3.Rz(np.pi/2).A
    c_R_wkspc = R_wkspc @ R_cr.T @ wrist_rot # I don't know why this works
# else:
#     wrist_pre_rotation = sm.SO3.Rz(0).A
#     c_R_wkspc = R_wkspc @ R_cr.T @ sm.SO3.Rz(np.pi/2).A # I don't know why this works
# R_cr = R_cw @ R_wr.T

ttl = np.array([1, 0, 5])
tta = np.array([0, 1, 0])

vlm_R_wrist = R_c @ pre_rotation
wttl = R_c @ pre_rotation @ ttl
print(wttl)
print(vlm_R_wrist @ ttl)


# pre commit, this also works!
R_cw = np.array([ # camera to wrist, rotate pi/2 about Z
    [0, -1, 0],
    [1,  0, 0],
    [0,  0, 1]
])
# R_cw = np.array([ # camera to wrist, rotate pi/2 about Z
#     [0, 1, 0],
#     [-1,  0, 0],
#     [0,  0, 1]
# ])
wrist_pre_rotation = sm.SO3.Rz(-np.pi/2).A
R_wkspc_table = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]]) @ pre_rotation
R_wkspc_chair = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]]) @ sm.SO3.Rx(-np.pi/2).A @ sm.SO3.Rz(np.pi/4).A @ pre_rotation
R_wkspc = R_wkspc_table if wkspc_cam_config == "table" else R_wkspc_chair
T_wr = np.array(robot.getPose())
T_init = T_wr.copy()
R_wr = T_wr[:3, :3]
R_cr = R_cw @ R_wr.T @ pre_rotation
# c_R_wkspc = R_wkspc @ R_cr.T @ sm.SO3.Rz(np.pi/2).A # I don't know why this works
c_R_wkspc = R_wkspc @ R_cr.T @ sm.SO3.Rz(-np.pi/2).A # I don't know why this works
# R_cr = R_cw @ R_wr.T