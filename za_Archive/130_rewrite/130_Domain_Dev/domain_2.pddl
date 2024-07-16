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
    (PoseAbove ?pose ?label) ; The concept of a pose being supported by an object
  
    ;;; Conditions ;;;
    (Supported ?labelUp ?labelDn) ; Is the "up" object on top of the "down" object?
    (Blocked ?label) ; This object cannot be lifted
    (Free ?pose) ; This pose is free of objects, therefore we can place something here without collision

    ;;; Robot State ;;;
    (HandEmpty)
    (Holding ?label)

  )

  ;;;;;;;;;; ACTIONS ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

    (:action pick
      :parameters (?label ?pose)
      :precondition (and
        ;; Current State ;;
        (GraspObj ?label ?pose)
        ;; Robot State ;;
        (HandEmpty)
      )
      :effect (and
        ;; Robot State ;;
        (Holding ?label)
        (not (HandEmpty))
      )
    )

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
                        ;; Robot State ;;
                        (Holding ?labelUp)
                        )
        :effect (and (Supported ?labelUp ?labelDn)
                     (not (Supported ?labelUp ?prevSupport))
                     (Blocked ?labelDn)
                     (not (GraspObj ?labelUp ?prevPose)) ; Movement SUBSTITUTE
                     (GraspObj ?labelUp ?poseUp) ; Movement SUBSTITUTE
                     (not (Free ?poseUp))
                     ;; Robot State ;;
                    (HandEmpty)
                    (not (Holding ?labelUp))
                )
    )
)