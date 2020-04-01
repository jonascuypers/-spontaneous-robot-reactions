(define (problem task)
(:domain robocure-hospital-education-full-domain)
(:objects
    patient - person
    corridor patient-room - location
    pepper - robot
)
(:init
    (can-move corridor patient-room)
    (can-move patient-room corridor)


    (robot-at pepper corridor)

    (person-at patient patient-room)



)
(:goal (and
    (education-is-done patient)
))
)
