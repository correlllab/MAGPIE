(define (stream magpie-tamp)

;;;;;;;;;; FUNCTIONS ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; (:function (MoveCost ?traj)
;   (Path ?traj)
;   ; :inputs (?traj)
;   ; :domain (and (Path ?traj))
;   ; :outputs (?dist)
; )

;;;;;;;;;; SYMBOL STREAMS ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

  ; Object Pose Stream ;;
  (:stream sample-object
    :inputs (?label)
    :domain (Graspable ?label)
    :outputs (?obj)
    :certified (and (GraspObj ?label ?obj) (Waypoint ?obj) ) ;(Occupied ?obj))
  )

  ;; Safe Motion Planner ;;
  (:stream find-safe-motion
    :inputs (?obj1 ?obj2)
    :domain (and (Waypoint ?obj1) (Waypoint ?obj2)); (Path ?traj))
    :fluents (AtObj) 
    :outputs (?traj)
    :certified (and (SafeMotion ?obj1 ?obj2 ?traj)) ;(Path ?traj))
  )

  ;; Safe Carry Planner ;;
  (:stream find-safe-carry
    :inputs (?label ?obj1 ?obj2)
    :domain (and (Graspable ?label) (Waypoint ?obj1) (Waypoint ?obj2)); (Path ?traj))
    :fluents (AtObj) 
    :outputs (?traj)
    :certified (and (SafeCarry ?label ?obj1 ?obj2 ?traj)) ;(Path ?traj))
  )

  ;; Stack Location Search ;;
  (:stream find-stack-place
    :inputs (?objDn1 ?objDn2)
    :domain (and (Waypoint ?objDn1) (Waypoint ?objDn2))
    ; :fluents (FreePlacement) 
    :outputs (?objUp)
    :certified (and (StackPlace ?objUp ?objDn1 ?objDn2) (Waypoint ?objUp))
  )


;;;;;;;;;; TESTS ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

  ; Free Placement Test ;;
  (:stream test-free-placment
    :inputs (?label ?obj)
    :domain (and (Graspable ?label) (Waypoint ?obj))
    :certified (FreePlacement ?label ?obj)
  )
)