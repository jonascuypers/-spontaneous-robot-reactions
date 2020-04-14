(define (problem task)
(:domain robot-response-dog)
(:objects
    pepper - robot
    personbert - person
    brucedog - dog
    kitchen livingroom - location
    dogtreat - holdingobject
)
(:init
    (robot-at pepper livingroom)

    (dog-at brucedog livingroom)


    (holdingobject-at dogtreat kitchen)

    (dog-barking brucedog)

    (not (dog-silent brucedog))

    (human-talking personbert)

    (not (human-silent personbert))

    (not (robot-holds pepper dogtreat))

    (dog-likes brucedog dogtreat)

)
(:goal (and
    (dog-silent brucedog)
))
)
