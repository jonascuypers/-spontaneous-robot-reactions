(define (domain robot-response-dog)
    (:requirements :typing :adl)
    
    ;Type definition
    (:types 
	Robot           ;
        Person          ; 
        Dog             ;
	Blender         ;
	Location        ;
	HoldingObject   ;
	EmotionQuadrant ;
	Topic           ;
    )
    
    ;Predicate definition
    (:predicates
	(robot-at ?robot - Robot ?location - Location)
	(dog-at ?dog - Dog ?location - Location)  
	(human-at ?human - Person ?location - Location)  

	(holdingobject-at ?object - HoldingObject ?location - Location)        

	(dog-barking ?dog - Dog)
        (dog-silent ?dog - Dog)

        (human-talking ?human - Person)
	(human-silent ?human - Person)

	(robot-holds ?robot - Robot ?object - HoldingObject)

	(dog-likes ?dog - Dog ?object - HoldingObject)

	(current-emotion ?person - Person ?emotion - EmotionQuadrant)
	(asked-all-good   ?person - Person ?topic - Topic)
	
	(accepted-to-speak ?topic - Topic ?emotion - EmotionQuadrant)
	(not-accepted-to-speak ?topic - Topic ?emotion - EmotionQuadrant)
	
	(music-played ?robot - Robot)

	(person-cooking ?person - Person)
	
	(loud-volume ?location - Location)
    )
    
    ;Action definition

    ; sayDogSilent
    ;
    ; Say to the dog to be quiet.
    ;
    ; @Param dog: The dog which might be barking
    ; @Param human: The possibly speaking human
    (:action sayDogSilentAction
        :parameters (   ?dog               - Dog 
                        ?human             - Person
                        ?robot             - Robot
                        ?doglocation       - Location)
        :precondition (and 
                (dog-barking ?dog)
                (human-silent ?human)
		(dog-at ?dog ?doglocation)
		(robot-at ?robot ?doglocation)
                )
        :effect (and
		(not (dog-barking ?dog) )
                (dog-silent ?dog)
                )
        
    )
    ; giveTreat
    ;
    ; Turns robot hand to give dog a treat.
    ;
    ; @Param dog: The dog which might be barking
    ; @Param human: The possibly speaking human
    (:action giveDogTreatAction
        :parameters (   ?dog               - Dog
			?doglocation 	   - Location
			?robot		   - Robot	
                        ?human             - Person
			?treat		   - HoldingObject)
        :precondition (and 
                (dog-barking ?dog)
		(dog-at ?dog ?doglocation)
		(robot-at ?robot ?doglocation)                
		(human-talking ?human)
		(robot-holds ?robot ?treat)
		(dog-likes ?dog ?treat)
                )
        :effect (and
		(not (dog-barking ?dog) )
                (dog-silent ?dog)
		(not(robot-holds ?robot ?treat))
                )
        
    )
	
    ; RobotTakeTreat
    ;
    ; Pick up the treat
    (:action RobotTakeTreatAction
        :parameters (   ?robot          - Robot 
                        ?treatlocation  - Location 
			?treat		- HoldingObject)
        :precondition (and 
                (robot-at ?robot ?treatlocation)
                (holdingobject-at ?treat ?treatlocation)
                )
        :effect (and
		(robot-holds ?robot ?treat)
                )
        
    )


    ; moveRobotAction
    ;
    ; Move the robot from a location to an other location
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

    ; AskAllGoodVerboseAction
    ;
    ; Ask the human if everything is ok

    (:action AskAllGoodVerboseAction
        :parameters (   ?robot             - Robot
			?person            - Person 
                        ?location          - Location
			?emotion	   - EmotionQuadrant
			?topic		   - Topic
	)
        :precondition (and 
                (human-at ?person ?location)
		(robot-at ?robot ?location)
		(current-emotion ?person ?emotion)
		(accepted-to-speak ?topic ?emotion)
                )
        :effect (and
		(asked-all-good ?person ?topic)
                )
    )

    ; AskAllGoodPassiveAction
    ;
    ; Ask passively the human if everything is ok

    (:action AskAllGoodPassiveAction
        :parameters (   ?robot             - Robot
			?person            - Person 
                        ?location          - Location
			?emotion	   - EmotionQuadrant
			?topic		   - Topic
	)
        :precondition (and 
                (human-at ?person ?location)
		(robot-at ?robot ?location)
		(current-emotion ?person ?emotion)
		(not-accepted-to-speak ?topic ?emotion)
                )
        :effect (and
		(asked-all-good ?person ?topic)
                )
    )


    ; StartPlayingMusicAction
    ;
    ; Ask the human if everything is ok

    (:action StartPlayingMusicAction
        :parameters (   ?robot             - Robot
			?person            - Person 
                        ?location          - Location
	)
        :precondition (and 
                (human-at ?person ?location)
		(robot-at ?robot ?location)
                )
        :effect (and
		(music-played ?robot)
                )
    )

    
)


