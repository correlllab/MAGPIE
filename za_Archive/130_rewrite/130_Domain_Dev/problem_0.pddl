; https://planning.wiki/ref/pddl/problem
(define
    (problem theTwoTowers)
    (:domain pick-place-and-stack)
    (:objects ; ALL THE THINGS
        red
        ylw
        blu
        grn
        orn
        vio
        table
    )
    (:init
        (Graspable red) ; Block Names
        (Graspable ylw)
        (Graspable blu)
        (Graspable grn)
        (Graspable orn)
        (Graspable vio)

        (Supported red table) ; Block Names
        (Supported ylw table)
        (Supported blu table)
        (Supported grn table)
        (Supported orn table)
        (Supported vio table)
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