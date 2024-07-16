(define (domain pick-and-place)
  (:requirements :strips )

  ;;;;;;;;;; PREDICATES ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

  (:predicates

    ;;; Objects ;;;
    (GraspObj ?label ?obj) ; The concept of a named object at a pose
    (SafeMotion ?obj1 ?obj2 ?traj) ; Safe path from pose 1 to pose 2
    (SafeCarry ?label ?obj1 ?obj2 ?traj) ; Safe path for object from pose 1 to pose 2
    (StackPlace ?objUp ?objDn1 ?objDn2) ; A pose that forms an arch

    ;;; Domains ;;;
    (Graspable ?label); Name of a real object we can grasp
    (Waypoint ?obj) ; Model of any object we can go to in the world, real or not
  
    ;;; Conditions ;;;
    (AtObj ?obj) ; The effector is at a grasp for this pose
    (Holding ?label) ; The label of the held object
    (HandEmpty) ; Is the robot hand empty?
    (Supported ?labelUp ?labelDn) ; Is the "up" object on top of the "down" object?

    ;;; Checks ;;;
    (FreePlacement ?label ?obj) ; Is there an open spot for placement?
  )

  ;;;;;;;;;; FUNCTIONS ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

  ; (:functions
  ;   (MoveCost ?traj)
  ; )

  ;;;;;;;;;; ACTIONS ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

  (:action move_free
    :parameters (?obj1 ?obj2 ?traj)
    :precondition (and 
                    ;; Knowns ;;
                    (AtObj ?obj1)
                    (HandEmpty)
                    ;; Requirements ;;
                    (SafeMotion ?obj1 ?obj2 ?traj)
                  )
    :effect (and (AtObj ?obj2)
                 (not (AtObj ?obj1))
                 (increase (total-cost) 2)
            ) 
  )

  (:action pick
    :parameters (?label ?obj)
    :precondition (and 
                    ;; Knowns ;;
                    (GraspObj ?label ?obj)
                    (AtObj ?obj)
                    (HandEmpty)
                    ; (Occupied ?obj)
                    (FreePlacement ?label ?obj) ; This is silly but the solver REQUIRES it!
                  )
    :effect (and (Holding ?label) 
                 (not (HandEmpty))
                 (increase (total-cost) 0.5)
            )
    )

  (:action move_holding
    :parameters (?label ?obj1 ?obj2 ?traj)
    :precondition (and 
                    ;; Knowns ;;
                    (Holding ?label)
                    (AtObj ?obj1)
                    ;; Requirements ;;
                    (GraspObj ?label ?obj1)
                    (SafeCarry ?label ?obj1 ?obj2 ?traj)
                  )
    :effect (and (GraspObj ?label ?obj2)
                 (not (GraspObj ?label ?obj1))
                 (AtObj ?obj2)
                 (not (AtObj ?obj1))
                 (increase (total-cost) 1)
            )
  )

  (:action place
    :parameters (?label ?obj)
    :precondition (and 
                    ;; Knowns ;;
                    (AtObj ?obj)
                    (Holding ?label)
                    ;; Requirements ;;
                    (FreePlacement ?label ?obj)
                  )
    :effect (and (HandEmpty) 
                 (not (Holding ?label)) 
                 (not (FreePlacement ?label ?obj))
                ;  (not (Waypoint ?obj))
                 (increase (total-cost) 0.5)
            )
  )
  

  (:action stack
    :parameters (?labelUp ?labelDn1 ?labelDn2 ?objUp ?objDn1 ?objDn2)
    :precondition (and 
                    ;; Knowns ;;
                    (AtObj ?objUp)
                    (Holding ?labelUp)
                    ;; Requirements ;;
                    (FreePlacement ?labelUp ?objUp)
                    (GraspObj ?labelDn1 ?objDn1)
                    (GraspObj ?labelDn2 ?objDn2)
                    (StackPlace ?objUp ?objDn1 ?objDn2)
                  )
    :effect (and (HandEmpty) 
                 (not (Holding ?labelUp))
                 (Supported ?labelUp ?labelDn1)
                 (Supported ?labelUp ?labelDn2)
                 (not (FreePlacement ?labelUp ?objUp)) ; Planner does dumb things if this isn't here
                ;  (not (Waypoint ?objUp))
                 (increase (total-cost) 0)
            )
  )
)  