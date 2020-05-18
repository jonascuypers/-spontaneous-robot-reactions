(define (problem task)
(:domain robot-response-dog)
(:objects
    pepper - robot
    personbert - person
    brucedog - dog
    kitchen livingroom - location
    dogtreat - holdingobject
    highv-lowa highv-higha lowv-lowa lowv-higha - emotionquadrant
    general kitchenhelping - topic
)
(:init
    (robot-at pepper livingroom)

    (dog-at brucedog livingroom)

    (human-at personbert kitchen)

    (object-at dogtreat kitchen)

    (not (dog-barking brucedog))

    (not (human-talking personbert))

    (human-silent personbert)

    (dog-likes brucedog dogtreat)

    (current-emotion personbert lowv-higha)
    (not (current-emotion personbert highv-higha))
    (not (current-emotion personbert lowv-lowa))
    (not (current-emotion personbert highv-lowa))


    (accepted-to-speak general highv-lowa)
    (accepted-to-speak general lowv-lowa)
    (accepted-to-speak kitchenhelping highv-lowa)
    (accepted-to-speak kitchenhelping highv-higha)
    (accepted-to-speak kitchenhelping lowv-lowa)
    (accepted-to-speak kitchenhelping lowv-higha)

    (not-accepted-to-speak general highv-higha)
    (not-accepted-to-speak general lowv-higha)

)
(:goal (and
))
)
