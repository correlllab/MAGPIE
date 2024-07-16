(define
    (problem theTwoTowers)
    (:domain pick-place-and-stack)
    (:objects ; ALL THE THINGS
        red 
        grn

        table

        poseSwap ; Poses
        targetRed
        targetGrn
        pose06
    )
    (:init
        (Waypoint poseSwap) ; Pose Names
        (Waypoint targetRed)
        (Waypoint targetGrn)
        (Waypoint pose06)
        
        (Base table) ; Base Name

        (Graspable red) ; Block Names
        (Graspable grn)

        (GraspObj red targetGrn) ; Beginning Object Configs
        (GraspObj grn pose06)
        
        (Supported red table) ; Block Supported state
        (Supported grn red)

        (Free poseSwap)
        (Free targetRed)

        (HandEmpty)
        (AtPose poseSwap)
        
        
        (PoseAbove poseSwap table)
        (PoseAbove targetRed table)
        (PoseAbove targetGrn table)
        (PoseAbove pose06 red)
        
        (Blocked red)
    )
    (:goal
        (and
            (GraspObj red targetRed) 
            (Supported grn red)
            (HandEmpty)
        )
    )
)