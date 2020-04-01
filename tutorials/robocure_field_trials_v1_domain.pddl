(define (domain homelab-education-full-domain)
    (:requirements :typing)
    
    ;Type definition
    (:types 
        Person      ;
        Location    ;
        Robot       ;
        Floor       ;
        Door        ;
        Dog         ;
        Speech      ;
    )
    
    ;Predicate definition
    (:predicates
        (can-move ?from-waypoint - Location ?to-waypoint - Location)
        (education-is-done ?person - Person)
        (robot-at-location ?robot - Robot ?location - Location)
        (person-at-location ?person - Person ?location - Location)
        (robot-near-person ?robot - Robot  ?person - Person)

        (elevator-at-floor ?floor - Floor)
        (is-on-floor ?location - Location ?floor - Floor)
        (can-take-elevator-at-location ?location - Location)
        (is-adjacent-to-door ?location - Location ?door - Door)
        (is-door-open ?door - Door)
        (dog-barking ?dog - Dog)
        (dog-silent ?dog - Dog)
        (human-talking ?speech - Speech)
    )
    
    ;Action definition
    
    ; goThroughDoor
    ;
    ; Moves from one side of an (open) door to the other
    ;
    ;
    (:action goThroughDoor
        :parameters
            (?robot         - Robot
             ?door          - Door
             ?from-location - Location
             ?to-location   - Location
             )
        :precondition
            (and
                (robot-at-location ?robot ?from-location)
                (is-adjacent-to-door ?from-location ?door)
                (is-adjacent-to-door ?to-location ?door)
                (is-door-open ?door)
            )   
         :effect
            (and 
                (robot-at-location ?robot ?to-location)
                (not (robot-at-location ?robot ?from-location ) )
            )
    )
    
    ; Move
    ;
    ; Moves the robot from one waypoint to the other.
    ;
    ; @Param robot: The robot to move.
    ; @Param from-waypoint: The current location of the robot.
    ; @Param to-waypoint: The destination location of the robot.
    (:action move
        :parameters 
            (?robot         - Robot
             ?from-waypoint - Location 
             ?to-waypoint   - Location)

        :precondition 
            (and 
                (robot-at-location ?robot ?from-waypoint)
                (can-move ?from-waypoint ?to-waypoint))

        :effect 
            (and 
                (robot-at-location ?robot ?to-waypoint)
                (not (robot-at-location ?robot ?from-waypoint ) )
            )
    )
    
    ; moveToPersonInRoom
    ; 
    ; If the robot is in the same room as the person, this action moves the
    ; robot to within the engagement zone of the person
    ;
    ; @Param robot: The robot to move.
    ; @Param person: The person in the room to move to.
    (:action moveToPersonInRoom
        :parameters 
            (?robot     - Robot
             ?person    - Person
             ?location  - Location)
        :precondition 
            (and 
                (robot-at-location ?robot ?location)
                (person-at-location ?person ?location)
            )
        :effect (and
            (robot-near-person ?robot ?person)
        )
    )
    
    ; doEducation
    ;
    ; Executes a generic education task.
    ;
    ; @Param robot: The robot that needs to perform the education task.
    ; @Param person: The person that needs to be educated.
    (:action doEducation
        :parameters 
            (?robot     - Robot
             ?person    - Person)

        :precondition 
            (and 
                (robot-near-person ?robot ?person)
            )
        :effect 
            (and 
                (education-is-done ?person)
            )
    )
    
    ; openDoorAction
    ;
    ; Opens a specific (IoT enabled) door.
    ;
    ; @Param door: Door to open
    ; @Param door-one-side: First of two locations the door connects.
    ; @Param door-other-side: Second of the two locations the door connects.
    (:action openDoorAction
        :parameters (   ?door               - Door 
                        ?door-one-side      - Location 
                        ?robot              - Robot)
        :precondition (and 
                (is-adjacent-to-door ?door-one-side ?door)
                (robot-at-location ?robot ?door-one-side)
                )
        :effect (and
                (is-door-open ?door)
                )
        
    )

    ; giveTreat
    ;
    ; Turns robot hand to give dog a treat.
    ;
    ; @Param door: Door to open
    ; @Param door-one-side: First of two locations the door connects.
    ; @Param door-other-side: Second of the two locations the door connects.
    (:action trut
        :parameters (   ?dog               - Dog 
                        ?speech            - Speech)
        :precondition (and 
                (dog-barking ?dog)
                (human-talking ?speech)
                )
        :effect (and
                (dog-silent ?dog)
                )
        
    )
    
    ; sayDogSilent
    ;
    ; Say to the dog to be quiet.
    ;
    ; @Param door: Door to open
    ; @Param door-one-side: First of two locations the door connects.
    ; @Param door-other-side: Second of the two locations the door connects.
    (:action sayDogSilentAction
        :parameters (   ?dog               - Dog 
                        ?speech            - Speech)
        :precondition (and 
                (dog-barking ?dog)
                (not (human-talking ?speech) )
                )
        :effect (and
                (dog-silent ?dog)
                )
        
    )
)



