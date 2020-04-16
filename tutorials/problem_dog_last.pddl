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

    (holdingobject-at dogtreat kitchen)

    (not (dog-barking brucedog))

    (dog-silent brucedog)

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
    (music-played pepper)
))
)
