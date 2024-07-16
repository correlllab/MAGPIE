; https://planning.wiki/ref/pddl/domain

; GOAL: Stack 3 Blocks, No Robot, No Motion

(define (domain pick-place-and-stack)
  (:requirements :strips :negative-preconditions)

  ;;;;;;;;;; PREDICATES ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

  (:predicates

    ;;; Domains ;;;
    (Graspable ?label); Name of a real object we can grasp
    (Waypoint ?pose) ; Model of any object we can go to in the world, real or not

    ;;; Objects ;;;
    (GraspObj ?label ?pose) ; The concept of a named object at a pose
    (PoseAbove ?pose ?label) ; The concept of a pose being supported by an object
  
    ;;; Conditions ;;;
    (Supported ?labelUp ?labelDn) ; Is the "up" object on top of the "down" object?
    (Blocked ?label) ; This object cannot be lifted
    (Free ?pose) ; This pose is free of objects, therefore we can place something here without collision

  )

  ;;;;;;;;;; ACTIONS ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

    (:action stack
        :parameters (?labelUp ?poseUp ?labelDn ?prevPose ?prevSupport)
        :precondition (and 
                        ;; Domain ;;
                        (Graspable ?labelUp)
                        (Waypoint ?poseUp)
                        ;; Current State ;;
                        (GraspObj ?labelUp ?prevPose) ; Movement SUBSTITUTE
                        (Free ?poseUp)
                        ;; Requirements ;;
                        (not (Blocked ?labelUp))
                        (Supported ?labelUp ?prevSupport)
                        (PoseAbove ?poseUp ?labelDn)
                        )
        :effect (and (Supported ?labelUp ?labelDn)
                     (not (Supported ?labelUp ?prevSupport))
                     (Blocked ?labelDn)
                     (not (GraspObj ?labelUp ?prevPose)) ; Movement SUBSTITUTE
                     (GraspObj ?labelUp ?poseUp) ; Movement SUBSTITUTE
                     (not (Free ?poseUp))
                )
    )
)