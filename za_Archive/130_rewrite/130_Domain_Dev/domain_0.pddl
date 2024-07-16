; https://planning.wiki/ref/pddl/domain
(define (domain pick-place-and-stack)
  (:requirements :strips :negative-preconditions)

  ;;;;;;;;;; PREDICATES ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

  (:predicates

    ;;; Domains ;;;
    (Graspable ?label); Name of a real object we can grasp
  
    ;;; Conditions ;;;
    (Supported ?labelUp ?labelDn) ; Is the "up" object on top of the "down" object?
    (Blocked ?label)

  )

  ;;;;;;;;;; ACTIONS ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

    (:action stack
        :parameters (?labelUp ?labelDn ?prevSupport)
        :precondition (and 
                        (Graspable ?labelUp)
                        (not (Blocked ?labelUp))
                        (Supported ?labelUp ?prevSupport)
                        )
        :effect (and (Supported ?labelUp ?labelDn)
                     (not (Supported ?labelUp ?prevSupport))
                     (Blocked ?labelDn)
                )
    )
)