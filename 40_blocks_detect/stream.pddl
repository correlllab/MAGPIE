(define (stream ryb-blocks)

  (:stream sample-pose
    :inputs (?o ?r)
    :domain (Stackable ?o ?r)
    :outputs (?p)
    :certified (and (Supported ?o ?p ?r)
                    (Pose ?o ?p)) ;(Observable ?p))
  )

)