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
  
    ;;; Object State ;;;
    (Supported ?labelUp ?labelDn) ; Is the "up" object on top of the "down" object?
    (Blocked ?label) ; This object cannot be lifted
    (Free ?pose) ; This pose is free of objects, therefore we can place something here without collision

    ;;; Robot State ;;;
    (HandEmpty)
    (Holding ?label)
    (AtPose ?pose)
  )

  ;;;;;;;;;; ACTIONS ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

  (:action move_free
      :parameters (?poseBgn ?poseEnd)
      :precondition (and 
        ;; Domain ;;
        (Waypoint ?poseBgn)
        (Waypoint ?poseEnd)
        ;; Robot State ;;
        (HandEmpty)
        (AtPose ?poseBgn)
      )
      :effect (and 
        ;; Robot State ;;
        (AtPose ?poseEnd)
        (not (AtPose ?poseBgn))
      )
  )
  
  (:action pick
    :parameters (?label ?pose ?prevSupport)
    :precondition (and
      ;; Domain ;;
      (Graspable ?label)
      (Waypoint ?pose)
      ;; Object State ;;
      (not (Blocked ?label))
      (GraspObj ?label ?pose)
      (Supported ?label ?prevSupport)
      ;; Robot State ;;
      (HandEmpty)
    )
    :effect (and
      ;; Object State ;;
      (Free ?pose) ; FREE TOGGLE
      (not (Supported ?label ?prevSupport))
      (not (Blocked ?prevSupport))
      ;; Robot State ;;
      (Holding ?label)
      (not (HandEmpty))
    )
  )

  (:action move_holding
      :parameters (?poseBgn ?poseEnd ?label)
      :precondition (and 
        ;; Domain ;;
        (Graspable ?label)
        (Waypoint ?poseBgn)
        (Waypoint ?poseEnd)
        ;; Robot State ;;
        (Holding ?label)
        (AtPose ?poseBgn)
        ;; Object State ;;
        ; (Free ?poseEnd) ; FREE TOGGLE
        (GraspObj ?label ?poseBgn)
      )
      :effect (and 
        ;; Robot State ;;
        (AtPose ?poseEnd)
        (not (AtPose ?poseBgn))
        ;; Object State ;;
        (GraspObj ?label ?poseEnd)
        (not (GraspObj ?label ?poseBgn))
        ; (Free ?poseBgn) ; FREE TOGGLE
        ; (not (Free ?poseEnd)) ; FREE TOGGLE
      )
  )

  (:action stack
      :parameters (?labelUp ?poseUp ?labelDn)
      :precondition (and 
                      ;; Domain ;;
                      (Graspable ?labelUp)
                      (Waypoint ?poseUp)
                      ;; Object State ;;
                      (GraspObj ?labelUp ?poseUp) 
                      ; (Free ?poseUp)
                      ;; Requirements ;;
                      (PoseAbove ?poseUp ?labelDn)
                      ;; Robot State ;;
                      (Holding ?labelUp)
                      )
      :effect (and 
                ;; Object State ;;
                (Supported ?labelUp ?labelDn)
                (Blocked ?labelDn)
                (not (Free ?poseUp)) ; FREE TOGGLE
                ;; Robot State ;;
                (HandEmpty)
                (not (Holding ?labelUp))
              )
  )

  (:action place
      :parameters (?label ?pose)
      :precondition (and 
                      ;; Domain ;;
                      (Graspable ?label)
                      (Waypoint ?pose)
                      ;; Object State ;;
                      (GraspObj ?label ?pose) 
                      (Free ?pose) ; FREE TOGGLE
                      ;; Robot State ;;
                      (Holding ?label)
                      )
      :effect (and 
                ;; Object State ;;
                (not (Free ?pose)) ; FREE TOGGLE
                ;; Robot State ;;
                (HandEmpty)
                (not (Holding ?label))
              )
  )
)