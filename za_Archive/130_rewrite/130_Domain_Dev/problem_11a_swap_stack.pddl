(define
    (problem theTwoTowers)
    (:domain pick-place-and-stack)
    (:objects ; ALL THE THINGS
        red ; Blocks
        ylw
        grn
        orn

        table ; Base

        poseSwap ; Poses
        targetRed
        targetGrn
        pose06
        pose08
    )
    (:init
        (Waypoint poseSwap) ; Pose Names
        (Waypoint targetRed)
        (Waypoint targetGrn)
        (Waypoint pose06)
        (Waypoint pose08)
        
        (Base table) ; Base Name

        (Graspable red) ; Block Names
        (Graspable ylw)
        (Graspable grn)
        (Graspable orn)

        (GraspObj red targetRed) ; Beginning Object Configs
        (GraspObj orn pose06)
        (GraspObj grn targetGrn) ; Beginning Object Configs
        (GraspObj ylw pose08)
        
        (Supported red table) ; Block Supported state
        (Supported orn red)
        (Supported grn table)
        (Supported ylw grn)
        

        (Free poseSwap)
        

        (HandEmpty)
        (AtPose poseSwap)
        
        (PoseAbove poseSwap table)
        (PoseAbove targetRed table)
        (PoseAbove targetGrn table)
        (PoseAbove pose06 red)
        (PoseAbove pose08 grn)
        
        (Blocked red)
        (Blocked grn)
    )
    (:goal
        (and
            (Supported orn grn)
            (Supported ylw red)
            (HandEmpty)
        )
    )
)