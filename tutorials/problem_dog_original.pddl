(define (problem task)
(:domain robot-response-dog)
(:objects
    pepper - robot
    personbert - person
    brucedog - dog
    kitchen livingroom - location
    dogtreat - holdingobject
    highv-lowa - emotionquadrant
    highv-higha - emotionquadrant
    lowv-lowa - emotionquadrant
    lowv-higha - emotionquadrant
    general - topic
    kitchenhelping - topic

    
)
(:init
    (robot-at pepper livingroom)

    (dog-at brucedog livingroom)

    (human-at personbert kitchen)

    (holdingobject-at dogtreat kitchen)

    (dog-barking brucedog)

    (not (dog-silent brucedog))

    (not (human-talking personbert))

    (human-silent personbert)

    (not (robot-holds pepper dogtreat))

    (dog-likes brucedog dogtreat)

    (current-emotion personbert highv-higha)

    (accepted-to-speak general highv-lowa)
    (accepted-to-speak general lowv-lowa)

    (accepted-to-speak kitchenhelping highv-lowa)
    (accepted-to-speak kitchenhelping highv-higha)

)
(:goal (and
))
)
