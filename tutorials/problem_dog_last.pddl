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
    (not (robot-at pepper kitchen))

    (dog-at brucedog livingroom)

    (human-at personbert kitchen)

    (holdingobject-at dogtreat kitchen)

    (not (dog-barking brucedog))

    (dog-silent brucedog)

    (not (human-talking personbert))

    (human-silent personbert)


    (dog-likes brucedog dogtreat)

    (not (current-emotion personbert lowv-lowa))
    (not (current-emotion personbert highv-lowa))
    (not (current-emotion personbert highv-higha))
    (current-emotion personbert lowv-higha)

    (asked-all-good personbert general)

    (accepted-to-speak general highv-lowa)
    (accepted-to-speak general lowv-lowa)
    (accepted-to-speak kitchenhelping highv-lowa)
    (accepted-to-speak kitchenhelping highv-higha)

    (not-accepted-to-speak general highv-higha)
    (not-accepted-to-speak general lowv-higha)
    (not-accepted-to-speak kitchenhelping lowv-lowa)
    (not-accepted-to-speak kitchenhelping lowv-higha)



)
(:goal (and
))
)
