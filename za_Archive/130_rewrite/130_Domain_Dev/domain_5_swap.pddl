; https://planning.wiki/ref/pddl/domain

; GOAL: Stack 2x towers of 3x Blocks, No Motion

(define (domain pick-place-and-stack)
  (:requirements :strips :negative-preconditions)

  ;;;;;;;;;; PREDICATES ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

  (:predicates

    ;;; Domains ;;;
    (Graspable ?label); Name of a real object we can grasp
    (Waypoint ?pose) ; Model of any object we can go to in the world, real or not

    ;;; Objects ;;;
    (GraspObj ?label ?pose) ; The concept of a named object at a pose
  
    ;;; Object State ;;;
    (Free ?pose) ; This pose is free of objects, therefore we can place something here without collision

  )

  ;;;;;;;;;; ACTIONS ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

  
  (:action move_object
      :parameters (?poseBgn ?poseEnd ?label)
      :precondition (and 
        ;; Domain ;;
        (Graspable ?label)
        (Waypoint ?poseBgn)
        (Waypoint ?poseEnd)
        ;; Object State ;;
        (Free ?poseEnd) ; FREE TOGGLE
        (GraspObj ?label ?poseBgn)
      )
      :effect (and 
        ;; Object State ;;
        (GraspObj ?label ?poseEnd)
        (not (GraspObj ?label ?poseBgn))
        (Free ?poseBgn) ; FREE TOGGLE
        (not (Free ?poseEnd)) ; FREE TOGGLE
      )
  )
  
)