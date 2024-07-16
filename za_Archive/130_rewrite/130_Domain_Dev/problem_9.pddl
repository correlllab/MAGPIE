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
        (GraspObj grn targetRed)
        
        (Supported red table) ; Block Supported state
        (Supported grn table)

        (Free poseSwap)
        (Free pose06)

        (HandEmpty)
        (AtPose poseSwap)
        
        (PoseAbove pose06 red)
        (PoseAbove poseSwap table)
        (PoseAbove targetRed table)
        (PoseAbove targetGrn table)
    )
    (:goal
        (and
            (GraspObj red targetRed) 
            (Supported grn red)
            (HandEmpty)
        )
    )
)