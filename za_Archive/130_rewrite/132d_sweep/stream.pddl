(define (stream magpie-tamp)

;;;;;;;;;; SYMBOL STREAMS ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

  ;; Stacking Pose Stream ;;
  ; Find free waypoints above existing blocks
  (:stream sample-above
    :inputs (?label)
    :domain (Graspable ?label)
    :outputs (?pose)
    :certified (and (PoseAbove ?pose ?label) (Waypoint ?pose) ) 
  )

  ; ;; Placement Pose Stream ;;
  ; ; Attempts to allocate swap space on the table (the only `Base`)
  ; (:stream sample-free-placment
  ;   :inputs (?label ?baseLabel) 
  ;   :domain (and (Graspable ?label) (Base ?baseLabel))
  ;   :outputs (?pose)
  ;   :certified (and (Waypoint ?pose) (Free ?pose) (PoseAbove ?pose ?baseLabel)) 
  ; )

;;;;;;;;;; TESTS ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

  ;; Free Placement Test ;;
  ; Return true if the pose is open for a block placement
  (:stream test-free-placment
    :inputs (?pose)
    :domain (Waypoint ?pose)
    :certified (Free ?pose) 
  )
)