MODULE TARGET_FOLLOWING
!////////////////
!GLOBAL VARIABLES
!////////////////

!Target goal
PERS robtarget cartesianTarget;
PERS jointtarget jointsTarget;

!Robot configuration	
PERS tooldata currentTool;    
PERS wobjdata currentWobj;
PERS speeddata currentSpeed;
PERS zonedata currentZone;
PERS zonedata nonBlockingZone;

!Working Mode
PERS bool move; !Whether the robot should move or not
PERS bool moveMode;  !Type of motion: TRUE -> Cartesian; FALSE -> Joints;
PERS bool commMode;  !Type of communication: TRUE -> blocking FALSE -> Non-blocking;

PROC main()
    !Local Variables
	VAR robtarget aux;
	VAR jointtarget aux2;
	
	VAR pose goToPose;
	VAR pose targetPose;
	VAR pose differentialPose;
	VAR pose incrementalPose;
	
	VAR robjoint goToJoints;
	VAR robjoint targetJoints;
	VAR robjoint differentialJoints;
	VAR robjoint incrementalJoints;
	
	VAR pos translation;
	VAR orient rotation;
	VAR num transMagnitude;
	VAR num rotMagnitude;
	
	VAR num maxIncTrans := 2.0; 		!Translation (we move at most 2 mm);
	VAR num maxIncRot := 0.5;           !Rotation (we move at most 0.5 degrees);
	VAR num maxIncJoints := 0.5;        !Joints motions (we move at most 0.5 degrees per joint);
	VAR num incTrans;
	VAR num incRot;
	VAR num incJoint1;
	VAR num incJoint2;
	VAR num incJoint3;
	VAR num incJoint4;
	VAR num incJoint5;
	VAR num incJoint6;
	
	VAR pos auxVec;
	VAR num auxNum;
	VAR num linearTime;
	VAR num angularTime;
	VAR num jointTime1;
	VAR num jointTime2;
	VAR num jointTime3;
	VAR num jointTime4;
	VAR num jointTime5;
	VAR num jointTime6;
	
	VAR num maxJointTime;
	
	ConfL \Off;
	SingArea \Wrist;

	move := FALSE;
	WHILE TRUE DO
		!Wait until the robot has to move
		WaitUntil move \PollRate:=0.01;
		IF commMode = TRUE THEN !Blocking
			IF moveMode = TRUE THEN !Cartesian
				MoveL cartesianTarget, currentSpeed, currentZone, currentTool \WObj:=currentWobj;
				move := FALSE;
			ELSE !Joint
			    MoveAbsJ jointsTarget, currentSpeed, currentZone, currentTool \Wobj:=currentWobj;
				move := FALSE;
			ENDIF
		ELSE !Non-blocking
			IF moveMode = TRUE THEN !Cartesian
				!Load the current location of the robot in goToPose (previous commanded location in the loop)
				aux := CRobT(\Tool:=currentTool \WObj:=currentWObj);	
				goToPose := [aux.trans,aux.rot];
		
				!Move the robot to target
				WHILE move = TRUE AND moveMode = TRUE DO
					targetPose := [cartesianTarget.trans,cartesianTarget.rot];
					differentialPose := PoseMult(PoseInv(goToPose),targetPose);
    		
					!Translation and Rotation Magnitudes
					transMagnitude := VectMagn(differentialPose.trans); 	
					auxVec := [differentialPose.rot.q2,differentialPose.rot.q3,differentialPose.rot.q4];
					auxNum := differentialPose.rot.q1;
					rotMagnitude := Abs(2*ATan2(VectMagn(auxVec), auxNum));
		
					!Estimated times (in incremetal steps)
					linearTime := transMagnitude/maxIncTrans;
					angulartime := rotMagnitude/maxIncRot;
				
					!Correspondent increments for translation and rotation synchonization
					IF(linearTime >= angularTime) THEN
						incTrans := maxIncTrans;
						IF(linearTime>0.5) THEN
							incRot := incTrans*rotMagnitude/transMagnitude;
						ELSE
							incRot := maxIncRot;
						ENDIF
					ELSE
						incRot := maxIncRot;
						IF(angularTime>0.5) THEN
							incTrans := incRot*transMagnitude/rotMagnitude;
						ELSE
							incTrans := maxIncTrans;
						ENDIF
					ENDIF
		
					!Incremental translation
					IF (transMagnitude>incTrans) THEN
						translation := incTrans/transMagnitude * differentialPose.trans;
					ELSE
						translation := differentialPose.trans;
					ENDIF
					incrementalPose.trans := translation;
			
					!Incremental Rotation		
					IF (rotMagnitude>incRot) THEN
						rotation.q1 := 1.0 + incRot/rotMagnitude * (differentialPose.rot.q1 - 1.0);
						rotation.q2 := incRot/rotMagnitude * differentialPose.rot.q2;
						rotation.q3 := incRot/rotMagnitude * differentialPose.rot.q3;
						rotation.q4 := incRot/rotMagnitude * differentialPose.rot.q4;			
					ELSE
						rotation := differentialPose.rot;
					ENDIF
					incrementalPose.rot := NOrient(rotation);
			
					!New step towards target
					goToPose := PoseMult(goToPose,incrementalPose);
					aux:=[goToPose.trans,goToPose.rot,[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
					MoveL aux, currentSpeed, nonBlockingZone, currentTool \WObj:=currentWobj;
				ENDWHILE
			ELSE 
			    !Joint Motion
				!Load the current location of the robot in goToJoints (previous commanded location in the loop)
				aux2 := CJointT();	
				goToJoints := aux2.robax;
			
				!Move the robot to jointsTarget
				WHILE move = TRUE AND moveMode = FALSE DO
					targetJoints := jointsTarget.robax;
					differentialJoints := [targetJoints.rax_1 - goToJoints.rax_1,
					                       targetJoints.rax_2 - goToJoints.rax_2,
										   targetJoints.rax_3 - goToJoints.rax_3,
										   targetJoints.rax_4 - goToJoints.rax_4,
										   targetJoints.rax_5 - goToJoints.rax_5,
										   targetJoints.rax_6 - goToJoints.rax_6];
					
					!Estimated times (in incremetal steps)
					jointTime1 := Abs(differentialJoints.rax_1)/ maxIncJoints;
					maxJointTime := jointTime1;
					jointTime2 := Abs(differentialJoints.rax_2)/ maxIncJoints;
					if(jointTime2 > maxJointTime)
						maxJointTime := jointTime2;
					jointTime3 := Abs(differentialJoints.rax_3)/ maxIncJoints;
					if(jointTime3 > maxJointTime)
						maxJointTime := jointTime3;
					jointTime4 := Abs(differentialJoints.rax_4)/ maxIncJoints;
					if(jointTime4 > maxJointTime)
						maxJointTime := jointTime4;
					jointTime5 := Abs(differentialJoints.rax_5)/ maxIncJoints;
					if(jointTime5 > maxJointTime)
						maxJointTime := jointTime5;
					jointTime6 := Abs(differentialJoints.rax_6)/ maxIncJoints;
					if(jointTime6 > maxJointTime)
						maxJointTime := jointTime6;
					
				
					!Correspondent increments for joint synchronization
					IF (maxJointTime>0.5) THEN
						incJoint1 := differentialJoints.rax_1 / maxJointTime;
						incJoint2 := differentialJoints.rax_2 / maxJointTime;
						incJoint3 := differentialJoints.rax_3 / maxJointTime;
						incJoint4 := differentialJoints.rax_4 / maxJointTime;
						incJoint5 := differentialJoints.rax_5 / maxJointTime;
						incJoint6 := differentialJoints.rax_6 / maxJointTime;				
						goToJoints := [goToJoints.rax_1 + incJoint1,
									   goToJoints.rax_2 + incJoint2,
									   goToJoints.rax_3 + incJoint3,
									   goToJoints.rax_4 + incJoint4,
									   goToJoints.rax_5 + incJoint5,
									   goToJoints.rax_6 + incJoint6];		
					ELSE
						goToJoints := targetJoints;
					ENDIF
					
					!New step towards target
					aux2 := [goToJoints,[ 0, 9E9, 9E9, 9E9, 9E9, 9E9]];
					MoveAbsJ aux2, currentSpeed, nonBlockingZone, currentTool \Wobj:=currentWobj;
				ENDWHILE
			ENDIF
		ENDIF
	ENDWHILE
ERROR
	move:=FALSE;
	TPWrite "TARGET_FOLLOWING: Error raised. Reinitiating the program.";
	RETRY;
ENDPROC

ENDMODULE