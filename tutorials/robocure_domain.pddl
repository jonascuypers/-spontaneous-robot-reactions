(define (domain robocure-hospital-education-full-domain)
    (:requirements :typing)
    
    ;Type definition
    (:types 
        person      ;
        location    ;
        robot       ;
    )
    
    ;Predicate definition
    (:predicates
		(can-move ?from-waypoint - location ?to-waypoint - location)
		(education-is-done ?person - person)
		(robot-at ?robot - robot ?from-waypoint - location)
		(person-at ?person - person ?from-waypoint - location)
		(robot-near-person ?robot - robot  ?person - person)
		(robot-in-same-room-as ?robot - robot ?person - person)
	)
	
	;Action definition
	
    ; Moves the robot from one waypoint to the other
	(:action move
        :parameters 
            (?robot - robot
             ?from-waypoint - location 
             ?to-waypoint - location)

        :precondition 
            (and 
                (robot-at ?robot ?from-waypoint)
                (can-move ?from-waypoint ?to-waypoint))

        :effect 
            (and 
                (robot-at ?robot ?to-waypoint)
                (not (robot-at ?robot ?from-waypoint))
            
                ; now in the same room as people in the room you just entered
                (forall (?person - person) 
                    (when (person-at ?person ?to-waypoint)
                        (robot-in-same-room-as ?robot ?person)
                    )
                )
                
                ; now no longer in same room as people in room you just left
                (forall (?person - person)
                    (when (person-at ?person ?from-waypoint)
                        (and
                            (not (robot-in-same-room-as ?robot ?person))
                            (not (robot-near-person ?robot ?person))
                        )
                    )
                )
            )
    )
    
    ;When in the same room as a person, move to that person.
    (:action move-to-person-in-room
        :parameters 
            (?robot - robot
             ?person - person)
        :precondition 
            (and 
                (robot-in-same-room-as ?robot ?person))
        :effect (and
            (robot-near-person ?robot ?person)
        )
    )
    
	
	(:action do-education
        :parameters 
            (?robot - robot
			 ?person - person)

        :precondition 
            (and 
				(robot-near-person ?robot ?person)
			)
        :effect 
            (and 
				(education-is-done ?person)
            )
    )
	
    
)
