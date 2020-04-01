(define (domain robot-response-dog)
    (:requirements :typing :adl)
    
    ;Type definition
    (:types 
	Robot       ;
        Person      ;
        Dog         ;
        Speech      ;
	Blender     ;
	Location    ;
    )
    
    ;Predicate definition
    (:predicates
	(robot-at ?robot - Robot ?location - Location)
	(dog-at ?dog - Dog ?location - Location)  
	(human-at ?human - Person ?location - Location)        
	(dog-barking ?dog - Dog)
        (dog-silent ?dog - Dog)
        (human-talking ?speech - Speech)
	(human-silent ?speech - Speech)
	(speech-from ?speech - Speech ?person - Person)
    )
    
    ;Action definition

    ; giveTreat
    ;
    ; Turns robot hand to give dog a treat.
    ;
    ; @Param dog: The dog which might be barking
    ; @Param speech: The possible speech
    (:action giveDogTreatAction
        :parameters (   ?dog               - Dog
			?doglocation 	   - Location
			?robot		   - Robot	
                        ?speech            - Speech)
        :precondition (and 
                (dog-barking ?dog)
		(dog-at ?dog ?doglocation)
		(robot-at ?robot ?doglocation)                
		(human-talking ?speech)
                )
        :effect (and
		(not (dog-barking ?dog) )
                (dog-silent ?dog)
                )
        
    )

    (:action makeSilentAction
        :parameters (   ?dog               - Dog 
                        ?speech            - Speech)
        :precondition (and 
                (dog-silent ?dog)
		(human-talking ?speech)
                )
        :effect (and
		(not (human-talking ?speech))
		(human-silent ?speech)
                )
        
    )

    ; sayDogSilent
    ;
    ; Say to the dog to be quiet.
    ;
    ; @Param dog: The dog which might be barking
    ; @Param speech: The possible speech
    (:action sayDogSilentAction
        :parameters (   ?dog               - Dog 
                        ?speech            - Speech)
        :precondition (and 
                (dog-barking ?dog)
                (human-silent ?speech)
                )
        :effect (and
		(not (dog-barking ?dog) )
                (dog-silent ?dog)
                )
        
    )

    ; moveRobotAction
    ;
    ; Say to the dog to be quiet.
    ;
    ; @Param dog: The dog which might be barking
    ; @Param speech: The possible speech
    (:action moveRobotAction
        :parameters (   ?robot             - Robot
			?fromlocation     - Location 
                        ?tolocation       - Location)
        :precondition (and 
                (robot-at ?robot ?fromlocation)
                )
        :effect (and
		(not (robot-at ?robot ?fromlocation) )
		(robot-at ?robot ?tolocation)
                )
        
    )
)


