(define
    (problem theTwoTowers)
    (:domain pick-place-and-stack)
    (:objects ; ALL THE THINGS
        red ; Blocks
        ylw
        blu
        grn
        orn
        vio

        table ; Initial support

        pose00 ; Poses
        pose01
        pose02
        pose03
        pose04
        pose05
        pose06
        pose07
        pose08
        pose09
        poseSafe
    )
    (:init
        (Waypoint pose00) ; Pose Names
        (Waypoint pose01)
        (Waypoint pose02)
        (Waypoint pose03)
        (Waypoint pose04)
        (Waypoint pose05)
        (Waypoint pose06)
        (Waypoint pose07)
        (Waypoint pose08)
        (Waypoint pose09)
        (Waypoint poseSafe)

        (Graspable red) ; Block Names
        (Graspable ylw)
        (Graspable blu)
        (Graspable grn)
        (Graspable orn)
        (Graspable vio)

        (Supported red table) ; Block Supported state
        (Supported ylw table)
        (Supported blu table)
        (Supported grn table)
        (Supported orn table)
        (Supported vio table)

        (GraspObj red pose00) ; Beginning Object Configs
        (GraspObj ylw pose01)
        (GraspObj blu pose02)
        (GraspObj grn pose03)
        (GraspObj orn pose04)
        (GraspObj vio pose05)

        (PoseAbove pose06 red)
        (PoseAbove pose07 ylw)

        (PoseAbove pose08 grn)
        (PoseAbove pose09 orn)

        (Free pose06)
        (Free pose07)
        (Free pose08)
        (Free pose09)

        (HandEmpty) ; Robot State
        (AtPose poseSafe)

    )
    (:goal
        (and
            (Supported ylw red) ; Tower A
            (Supported blu ylw)

            (Supported orn grn) ; Tower B
            (Supported vio orn)
        )
    )
)