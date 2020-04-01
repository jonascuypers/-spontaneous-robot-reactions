(define (problem task)
(:domain robot-response-dog)
(:objects
    pepper - robot
    personbert - person
    brucedog - dog
    speech - speech
    kitchen livingroom - location
)
(:init
    (robot-at pepper livingroom)
    (not (robot-at pepper kitchen))

    (dog-at brucedog livingroom)


    (dog-barking brucedog)

    (not (dog-silent brucedog))

    (human-talking speech)


    (speech-from speech personbert)

)
(:goal (and
    (dog-silent brucedog)
))
)
