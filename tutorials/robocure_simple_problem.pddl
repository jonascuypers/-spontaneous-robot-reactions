(define (problem task)
(:domain homelab-education-full-domain)
(:objects
    berta - person
    kenny - robot
    dog1 - dog
    speech1 - speech
)
(:init




    (robot-near-person kenny berta)






    (dog-barking dog1)


    (human-talking speech1)

)
(:goal (and
    (dog-silent dog1)
))
)
