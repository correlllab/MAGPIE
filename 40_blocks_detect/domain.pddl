(define (domain blocks-tamp)
  (:requirements :strips :equality)
  (:constants base left right head)
  (:predicates
    (Arm ?a)
    (Graspable ?o)
    (Stackable ?o ?r)
    (Scannable ?r)
    (Registerable ?o)


    (Pose ?o ?p)
    (Grasp ?o ?g)
    (Kin ?a ?o ?p ?g ?q ?t)
    (Supported ?o ?p ?r)
    (Vis ?o ?p ?bq ?hq ?ht)

    (VisRange ?o ?p ?bq)
    (RegRange ?o ?p ?bq)

    (AtPose ?o ?p)
    (AtGrasp ?a ?o ?g)
    (HandEmpty ?a)

    (On ?o ?r)
    (Holding ?a ?o)

    (Uncertain ?o)
    (Scanned ?o)
    (Localized ?o)
    (Registered ?o)
  )
  (:functions
    (MoveCost)
    (PickCost)
    (PlaceCost)
    (ScanCost)
    (LocalizeCost ?r ?o)
    (RegisterCost)
  )

  (:action pick
    :parameters (?a ?o ?p ?g ?q ?t)
    :precondition (and (Kin ?a ?o ?p ?g ?q ?t)
                       (Registered ?o) (AtPose ?o ?p) (HandEmpty ?a) (AtBConf ?q))
    :effect (and (AtGrasp ?a ?o ?g) (CanMove)
                 (not (AtPose ?o ?p)) (not (HandEmpty ?a))
                 (increase (total-cost) (PickCost)))
  )

  (:action place
    :parameters (?a ?o ?p ?g ?q ?t)
    :precondition (and (Kin ?a ?o ?p ?g ?q ?t)
                       (AtGrasp ?a ?o ?g) (AtBConf ?q)) ; (Localized ?o)
    :effect (and (AtPose ?o ?p) (HandEmpty ?a) (CanMove)
                 (not (AtGrasp ?a ?o ?g))
                 (increase (total-cost) (PlaceCost)))
  )

  (:action scan
    :parameters (?o ?p ?bq ?hq ?ht)
    :precondition (and (Vis ?o ?p ?bq ?hq ?ht) (VisRange ?o ?p ?bq) (Scannable ?o)
                       (AtPose ?o ?p) (AtBConf ?bq) (Localized ?o))
    :effect (and (Scanned ?o) (CanMove)
                 (increase (total-cost) (ScanCost)))
  )

  (:action localize
    :parameters (?r ?p1 ?o ?p2)
    :precondition (and (Stackable ?o ?r) (Pose ?r ?p1) (Pose ?o ?p2) ; (FiniteScanCost ?r ?o)
                   (AtPose ?o ?p2) (Scanned ?r) (Uncertain ?o))
    :effect (and (Localized ?o) (Supported ?o ?p2 ?r)
                 (not (Uncertain ?o))
                 (increase (total-cost) (LocalizeCost ?r ?o)))
  )

  (:action register
    :parameters (?o ?p ?bq ?hq ?ht)
    :precondition (and (Vis ?o ?p ?bq ?hq ?ht) (RegRange ?o ?p ?bq) (Registerable ?o)
                       (AtPose ?o ?p) (AtBConf ?bq) (Localized ?o))
    :effect (and (Registered ?o) (CanMove)
                 (increase (total-cost) (RegisterCost)))
  )

  (:derived (On ?o ?r)
    (exists (?p) (and (Supported ?o ?p ?r)
                      (AtPose ?o ?p)))
  )

  (:derived (Holding ?a ?o)
    (exists (?g) (and (Arm ?a) (Grasp ?o ?g)
                      (AtGrasp ?a ?o ?g)))
  )

)