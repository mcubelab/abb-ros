MODULE SERVER

!////////////////
!GLOBAL VARIABLES
!////////////////

!//To modify the default values go to method Initialize
PERS tooldata currentTool := [TRUE,[[0,0,105],[1,0,0,0]],[0.001,[0,0,0.001],[1,0,0,0],0,0,0]];    
PERS wobjdata currentWobj := [FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[808.5,-612.9,0.6],[0.7084,0.00039,-0.00039,0.7058]]];   
PERS speeddata currentSpeed;
PERS zonedata currentZone;

!// Clock Synchronization
PERS bool startLog:=TRUE;
PERS bool startRob:=TRUE;

!// Mutex between logger and changing the tool and work objects
PERS bool frameMutex:=FALSE;

!//PC communication
VAR socketdev clientSocket;
VAR socketdev serverSocket;
VAR num instructionCode;
VAR string idCode;
VAR num params{10};
VAR num specialParams{5};
VAR num nParams;
PERS string ipController:= "192.168.125.1";
!"192.168.1.99";   Real Controller
!"192.168.180.128" VmWare
!"127.0.0.1";      Local Host
PERS num serverPort:= 5000;
PERS num loggerPort:= 5001;

!//Logger sampling rate
PERS num loggerWaitTime:= 0.01;
!PERS num loggerWaitTime:= 0.1; This is adequate for virtual server

!//Motion of the robot
VAR robtarget cartesianTarget;
VAR jointtarget jointsTarget;
VAR bool moveComplete; !True when program pointer leaves a Move instruction.

!//Correct Instruction Execution and possible return values
VAR num ok;
CONST num SERVER_BAD_MSG :=  0;
CONST num SERVER_OK := 1;
CONST num SERVER_COLLISION := 2;
CONST num SERVER_BAD_IK := 3;
CONST num SERVER_BAD_FK := 4;
	
!//Error Handler
VAR errnum ERR_MOTIONSUP := -1;

!//Interrupt to trap the digital output that signals the need to restart the motion of the robot.
VAR intnum iMotionReset;

!// Inverse and forward kinematic results
VAR jointtarget ik_result_j;
VAR robtarget fk_result_c;

!// Upper and lower joint bounds, in degrees
VAR num upper_joint_limits{6};
VAR num lower_joint_limits{6};

!//Other
TASK PERS tooldata calibkinect:=[TRUE,[[-32.9885,0.584218,225.962],[1,0,0,0]],[1,[0,0,0],[1,0,0,0],0,0,0]];
TASK PERS tooldata newvac:=[TRUE,[[-33.5757,-0.325804,225.926],[1,0,0,0]],[1,[0,0,0],[1,0,0,0],0,0,0]];
TASK PERS tooldata stocopt:=[TRUE,[[-78.3527,-3.0397,157.523],[1,0,0,0]],[1,[0,0,0],[1,0,0,0],0,0,0]];
TASK PERS wobjdata wobjstocopt:=[FALSE,TRUE,"",[[500.8,-14,145],[0,0,1,0]],[[0,0,0],[1,0,0,0]]];
TASK PERS tooldata toolshieldcan:=[TRUE,[[-156.443,4.27817,210.91],[1,0,0,0]],[1,[-145,-9.34,218.485],[1,0,0,0],0,0,0]];
TASK PERS tooldata sidd:=[TRUE,[[-144.459,-7.69289,215.816],[1,0,0,0]],[-1,[0,0,0],[1,0,0,0],0,0,0]];
TASK PERS wobjdata wobjshieldcan:=[FALSE,TRUE,"",[[675.4,214.8,189.1],[0,0,1,0]],[[0,0,0],[1,0,0,0]]];
TASK PERS tooldata nishanttest:=[TRUE,[[-81.401,2.516,159.598],[1,0,0,0]],[1,[0,0,0],[1,0,0,0],0,0,0]];
TASK PERS tooldata calibTool:=[TRUE,[[-0.228667,0.269561,182.166],[1,0,0,0]],[0.2,[0,0,0],[1,0,0,0],0,0,0]];
CONST robtarget calib1:=[[599.35,-573.63,1.21],[0.000618842,-0.00183075,0.951211,-0.308535],[-1,-1,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
CONST robtarget calib2:=[[461.90,-573.12,1.11],[0.000622551,-0.0018337,0.951211,-0.308534],[-1,-1,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
CONST robtarget calib3:=[[331.04,-572.22,1.01],[0.0006239,-0.00182736,0.951213,-0.30853],[-1,-2,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
CONST robtarget calib4:=[[213.90,-571.36,0.65],[0.000623773,-0.00182709,0.951213,-0.30853],[-1,-2,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
CONST robtarget calib5:=[[74.79,-571.08,0.44],[0.000624911,-0.00182578,0.951212,-0.308531],[-1,-2,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
CONST robtarget calib6:=[[-71.53,-570.16,0.39],[0.000627229,-0.00182302,0.951212,-0.308531],[-2,-3,1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
CONST robtarget calib7:=[[633.01,41.56,335.98],[0.449198,-0.63276,-0.615056,-0.139795],[-1,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
CONST robtarget calib8:=[[496.48,278.30,375.38],[0.357565,0.832103,0.419119,0.0639613],[0,-1,2,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
CONST robtarget calib9:=[[433.50,177.84,365.22],[0.112886,0.870163,0.299071,0.375007],[0,-2,2,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
CONST robtarget curpos:=[[612.07,11.89,243.18],[0.633169,-0.720998,-0.0564776,0.275807],[-1,0,-3,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
TASK PERS wobjdata wobj1:=[FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[808.5,-612.86,0.59],[0.7084,0.0003882,-0.0003882,0.7058]]];
TASK PERS wobjdata wobjCelllPhone:=[FALSE,TRUE,"",[[491.9,-69.4,152.3],[0,0,1,0]],[[0,0,0],[1,0,0,0]]];
TASK PERS tooldata tool2:=[TRUE,[[0.868263,-1.61762,162.034],[1,0,0,0]],[1,[0,0,0],[1,0,0,0],0,0,0]];
TASK PERS wobjdata locatorWobj:=[FALSE,TRUE,"",[[608,11.9,99.4],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];

!////////////////
!LOCAL METHODS
!////////////////

!Method to parse the message received from a PC through the socket
! Loads values on:
! - instructionCode.
! - idCode: 3 digit identifier of the command. 
! - nParams: Number of received parameters.
! - params{nParams}: Vector of received params.
PROC ParseMsg(string msg)
	!Local variables
	VAR bool auxOk;
	VAR num ind:=1;
	VAR num newInd;
	VAR num length;
	VAR num indParam:=1;
	VAR string subString;
	VAR bool end := FALSE;
	
	length := StrMatch(msg,1,"#");
	IF length > StrLen(msg) THEN
	        !Corrupt message
	        nParams := -1;
	ELSE
		!Find Instruction code
		newInd := StrMatch(msg,ind," ") + 1;
		subString := StrPart(msg,ind,newInd - ind - 1);
		auxOk:= StrToVal(subString, instructionCode);
		IF auxOk = FALSE THEN
		   	!Corrupt instruction code
		        nParams := -1;
		ELSE
			ind := newInd;

			!Find Id Code
			newInd := StrMatch(msg,ind," ") + 1;
			idCode := StrPart(msg,ind,newInd - ind - 1);
			ind := newInd;
	
			!Set of parameters (maximum of 8)
			WHILE end = FALSE DO
			      newInd := StrMatch(msg,ind," ") + 1;
			      IF newInd > length THEN
			            end := TRUE;
			      ELSE
			            subString := StrPart(msg,ind,newInd - ind - 1);
				    auxOk := StrToVal(subString, params{indParam});
	  			    indParam := indParam + 1;
				    ind := newInd;
		              ENDIF
	   
			ENDWHILE
			nParams:= indParam - 1;
		ENDIF
	ENDIF
ENDPROC

!Handshake between server and client:
! - Creates socket.
! - Waits for incoming TCP connection.
PROC ServerCreateAndConnect(string ip, num port)
	VAR string clientIP;
	
	SocketCreate serverSocket;
	SocketBind serverSocket, ip, port;
	SocketListen serverSocket;
	TPWrite "SERVER: Server waiting for incomming connections ...";
	WHILE SocketGetStatus(clientSocket) <> SOCKET_CONNECTED DO
		SocketAccept serverSocket,clientSocket \ClientAddress:=clientIP \Time:=WAIT_MAX;
		IF SocketGetStatus(clientSocket) <> SOCKET_CONNECTED THEN
			TPWrite "SERVER: Problem serving an incomming connection.";
			TPWrite "SERVER: Try reconnecting.";
		ENDIF
		 !//Wait 0.5 seconds for the next reconnection
		 WaitTime 0.5;
	ENDWHILE
	TPWrite "SERVER: Connected to IP " + clientIP;
	
ENDPROC

!//Parameter initialization
!// Loads default values for
!// - Tool.
!// - WorkObject.
!// - Zone.
!// - Speed.
!// Gets joint bounds so they can be used later
PROC Initialize()
	VAR string path;

	currentTool := [TRUE,[[0,0,0],[1,0,0,0]],[0.001,[0,0,0.001],[1,0,0,0],0,0,0]];    
	currentWobj := [FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];
	currentSpeed := [100, 50, 5000, 1000];
	currentZone := [FALSE, 0.3, 0.3,0.3,0.03,0.3,0.03]; !z0
	!SetDO vacuum,0;
	!SetDO solenoid_passthrough,0;
	!SetDO solenoid_lock,0;
	!SetDO solenoid_unlock,0;
	
	!// Get all of the joint bounds for later use
	FOR i FROM 1 TO 6 DO
		path := "MOC/ARM/rob1_" + NumToStr(i,0);
		ReadCfgData path, "upper_joint_bound", upper_joint_limits{i};
		ReadCfgData path, "lower_joint_bound", lower_joint_limits{i};
		
		!// The joint limits are in radians, so convert these to degrees
		upper_joint_limits{i} := upper_joint_limits{i} * 180.0 / pi;
		lower_joint_limits{i} := lower_joint_limits{i} * 180.0 / pi;
	ENDFOR

ENDPROC

!/////////////////
!//Main procedure
!/////////////////
PROC main()
	!//Local variables
	VAR string receivedString;
	VAR string sendString;
	VAR string addString;
	VAR bool connected;  ! //Client connected
	VAR bool reconnected;! //Reconnection During the iteration
	VAR robtarget cartesianPose;
	VAR jointtarget jointsPose;
	VAR clock timer;
	VAR num quatMag;
	
	!//Book error number for error handler
	BookErrNo ERR_MOTIONSUP;
	
	!//Configuration of interrupt that traps digital output that signals the need to restart the motion of the robot.
	!//SetDO USER_RESET_MOTION, 0;
	CONNECT iMotionReset WITH resetMotion;
	!ISignalDO USER_RESET_MOTION,1,iMotionReset;
	
	!// We are not currently changing the frame
	frameMutex:= FALSE;
	
	!//Motion configuration
	!MotionSup \On \TuneValue:=40;
	ConfL \On;
	ConfJ \On;
	SingArea \Wrist;
	!SingArea \Off;
	moveComplete:= TRUE;
	
	!//Timer synchronization with Logger
	!startRob:=TRUE;
	!WaitUntil startLog \PollRate:=0.01;
	!ClkStart timer;

	!//Initialization of WorkObject, Tool, Speed and Zone, 
	!// and get the joint bounds from the current configuration
	Initialize;
	
 	!//Socket connection
	connected:=FALSE;
	ServerCreateAndConnect ipController,serverPort;	
	connected:=TRUE;
	
	!//Infinite Loop
	WHILE TRUE DO
	    !//Initialization of program flow variables
		ok:=SERVER_OK;              !//Correctness of executed instruction.
		reconnected:=FALSE;         !//Has communication dropped after receiving a command?
		addString := "";    		!//String to add to the reply.

		!//Wait for a command
		SocketReceive clientSocket \Str:=receivedString \Time:=WAIT_MAX;
		ParseMsg receivedString;
		
		!//debug info by weiwei
		!TPWrite receivedString;
		!TPWrite ValToStr(params{1});
		!TPWrite ValToStr(params{2});
		!TPWrite ValToStr(params{3});
		!TPWrite ValToStr(params{4});
		!TPWrite ValToStr(params{5});
		!TPWrite ValToStr(params{6});
			
		!//Execution of the command
		TEST instructionCode
		CASE 0: !Ping
			!Message Check
			IF nParams = 0 THEN
				ok := SERVER_OK;
			ELSE
				ok := SERVER_BAD_MSG;
			ENDIF
		CASE 1: !Set Cartesian Coordinates
			IF nParams = 7 THEN
				cartesianTarget :=[[params{1},params{2},params{3}],
						[params{4},params{5},params{6},params{7}],
						[0,0,0,0],
						[9E9,9E9,9E9,9E9,9E9,9E9]];
				ok := SERVER_OK;
				moveComplete := FALSE;
				MoveL cartesianTarget, currentSpeed, currentZone, currentTool \WObj:=currentWobj ;
				TPWrite "MoveL";
				moveComplete := TRUE;
			ELSEIF nParams = 8 THEN
				!If there's an extra parameter, do a cartesian move as a joint move instead
				cartesianTarget :=[[params{1},params{2},params{3}],
						[params{4},params{5},params{6},params{7}],
						[0,0,0,0],
						[9E9,9E9,9E9,9E9,9E9,9E9]];
				ok := SERVER_OK;
				moveComplete := FALSE;
				MoveJ cartesianTarget, currentSpeed, currentZone, currentTool \WObj:=currentWobj ;
				TPWrite "MoveJ";
				moveComplete := TRUE;
			ELSE
				ok := SERVER_BAD_MSG;
			ENDIF	
		CASE 2: !Set Joint Coordinates
			TPWrite ValToStr(nParams);
			IF nParams = 6 THEN
				TPWrite "xx";
				jointsTarget:=[[params{1},params{2},params{3},params{4},params{5},params{6}],
							[ 0, 9E9, 9E9, 9E9, 9E9, 9E9]];
				ok := SERVER_OK;
				moveComplete := FALSE;
				MoveAbsJ jointsTarget, currentSpeed, currentZone, currentTool \Wobj:=currentWobj;
				moveComplete := TRUE;
			ELSE
				TPWrite "yy";
				ok :=SERVER_BAD_MSG;
			ENDIF
		CASE 3: !Get Cartesian Coordinates (with current tool and workobject)
			IF nParams = 0 THEN
				cartesianPose := CRobT(\Tool:=currentTool \WObj:=currentWObj);
				addString := NumToStr(cartesianPose.trans.x,2) + " ";
				addString := addString + NumToStr(cartesianPose.trans.y,2) + " ";
				addString := addString + NumToStr(cartesianPose.trans.z,2) + " ";
				addString := addString + NumToStr(cartesianPose.rot.q1,4) + " ";
				addString := addString + NumToStr(cartesianPose.rot.q2,4) + " ";
				addString := addString + NumToStr(cartesianPose.rot.q3,4) + " ";
				addString := addString + NumToStr(cartesianPose.rot.q4,4); !End of string	
				ok := SERVER_OK;
			ELSE
				ok :=SERVER_BAD_MSG;
			ENDIF
		CASE 4: !Get Joint Coordinates
			IF nParams = 0 THEN
				jointsPose := CJointT();
				addString := NumToStr(jointsPose.robax.rax_1,2) + " ";
				addString := addString + NumToStr(jointsPose.robax.rax_2,2) + " ";
				addString := addString + NumToStr(jointsPose.robax.rax_3,2) + " ";
				addString := addString + NumToStr(jointsPose.robax.rax_4,2) + " ";
				addString := addString + NumToStr(jointsPose.robax.rax_5,2) + " ";
				addString := addString + NumToStr(jointsPose.robax.rax_6,2); !End of string
				ok := SERVER_OK;
			ELSE
				ok:=SERVER_BAD_MSG;
			ENDIF
		CASE 6: !Specify Tool
			IF nParams = 7 THEN
				WHILE (frameMutex) DO
					!// If the frame is being used by logger, wait here
				ENDWHILE
				frameMutex:= TRUE;
				currentTool.tframe.trans.x:=params{1};
				currentTool.tframe.trans.y:=params{2};
				currentTool.tframe.trans.z:=params{3};
				currentTool.tframe.rot.q1:=params{4};
				currentTool.tframe.rot.q2:=params{5};
				currentTool.tframe.rot.q3:=params{6};
				currentTool.tframe.rot.q4:=params{7};
				frameMutex:= FALSE;
				ok := SERVER_OK;
			ELSE
				ok:=SERVER_BAD_MSG;
			ENDIF
		CASE 7: !Specify Work Object
			IF nParams = 7 THEN
				WHILE (frameMutex) DO
					!// If the frame is being used by logger, wait here
				ENDWHILE
				frameMutex:= TRUE;
				currentWobj.oframe.trans.x:=params{1};
				currentWobj.oframe.trans.y:=params{2};
				currentWobj.oframe.trans.z:=params{3};
				currentWobj.oframe.rot.q1:=params{4};
				currentWobj.oframe.rot.q2:=params{5};
				currentWobj.oframe.rot.q3:=params{6};
				currentWobj.oframe.rot.q4:=params{7};
				frameMutex:= FALSE;
				ok := SERVER_OK;
			ELSE
				ok:=SERVER_BAD_MSG;
			ENDIF
		CASE 8: !Specify Speed of the Robot
			IF nParams = 2 THEN
				currentSpeed.v_tcp:=params{1};
				currentSpeed.v_ori:=params{2};
				ok := SERVER_OK;
			ELSE
				ok:=SERVER_BAD_MSG;
			ENDIF
		CASE 9: !Specify ZoneData
			IF nParams = 4 THEN
				IF params{1}=1 THEN
					currentZone.finep := TRUE;
					currentZone.pzone_tcp := 0.0;
					currentZone.pzone_ori := 0.0;
					currentZone.zone_ori := 0.0;
				ELSE
					currentZone.finep := FALSE;
					currentZone.pzone_tcp := params{2};
					currentZone.pzone_ori := params{3};
					currentZone.zone_ori := params{4};
				ENDIF
				ok := SERVER_OK;
			ELSE
				ok:=SERVER_BAD_MSG;
			ENDIF
		CASE 10: !Special Command
			IF nParams = 6 THEN
			    specialParams{1} := params{2};
				specialParams{2} := params{3};
				specialParams{3} := params{4};
				specialParams{4} := params{5};
				specialParams{5} := params{6};
				TEST params{1}				
					CASE 1:
						!Call special command number 1: Bounceless Catch
						Catching;
						ok := SERVER_OK;
					CASE 2:
						!Call special command number 2: NOT SET
						!Special Command 2 is not set. Add some cool regrasp here later.
						ok := SERVER_BAD_MSG;
					CASE 3:
						!Call special command number 3: Enveloping Grasp to Fingertip Grasp
						Env2Fing;
						ok := SERVER_OK;
					CASE 4:
						!Call special command number 4: Vibrate
						Vibrate;
						ok := SERVER_OK;
					CASE 5:
						!Call special command number 5: Fingertip to Enveloping with hand facing downward
						Fing2EnvHD;
						ok := SERVER_OK;	
					DEFAULT:
					    ok:=SERVER_BAD_MSG;
				ENDTEST				
			ELSE
				ok:=SERVER_BAD_MSG;
			ENDIF
		CASE 11: !Toggle vacuum on/off
			IF nParams = 1 THEN
				ok := SERVER_OK;
				IF params{1}=0 THEN
					!SetDO solenoid_passthrough,0;
				ELSEIF params{1}=1 THEN
					!SetDO solenoid_passthrough,1;
				ELSE
					ok:=SERVER_BAD_MSG;
				ENDIF
			ELSE
				ok:=SERVER_BAD_MSG;
			ENDIF
			
		CASE 12: !Inverse Kinematics Solver
			IF nParams = 7 THEN
				!// First, let's make sure the quaternion is normalized
				IF Abs(1.0 - Sqrt(params{4} * params{4} + params{5} * params{5} + params{6} * params{6} + params{7} * params{7})) > 0.001 THEN
					!// If not, then we cannot find the inverse kinematics for this pose
					ok := SERVER_BAD_IK;
				ELSE
					!// Otherwise, let's normalize our quaternion 
					cartesianTarget :=[[params{1},params{2},params{3}],
							NOrient([params{4},params{5},params{6},params{7}]),
							[0,0,0,0],
							[9E9,9E9,9E9,9E9,9E9,9E9]];
					ok := SERVER_OK;
					
					!// Now calculate the joint angles, keeping in mind that if we specified an 
					!// impossible configuration, this will generate an error (See error handler below)
					ik_result_j := CalcJointT(cartesianTarget, currentTool, \WObj:=currentWObj);
					
					!// Store our result in a string to return to the user
					addString := NumToStr(ik_result_j.robax.rax_1,2) + " ";
					addString := addString + NumToStr(ik_result_j.robax.rax_2,2) + " ";
					addString := addString + NumToStr(ik_result_j.robax.rax_3,2) + " ";
					addString := addString + NumToStr(ik_result_j.robax.rax_4,2) + " ";
					addString := addString + NumToStr(ik_result_j.robax.rax_5,2) + " ";
					addString := addString + NumToStr(ik_result_j.robax.rax_6,2); !End of string
				
				ENDIF
			ELSE
				ok:=SERVER_BAD_MSG;
			ENDIF
			
		CASE 13: ! Forward Kinematics Solver
			IF nParams = 6 THEN
				ok := SERVER_OK;
				
				!// First, let's make sure the specified joint angles are within range
				FOR i FROM 1 TO 6 DO
					IF params{i} > upper_joint_limits{i} OR params{i} < lower_joint_limits{i} THEN
						!// If not, then we'll tell the user that their forward kinematics are invalid
						ok := SERVER_BAD_FK;
					ENDIF
				ENDFOR
					
				!// If our joints are within limits, then let's carry on
				IF ok = SERVER_OK THEN
					!// Create a joint target, and then calculate the corresponding cartesian pose
					jointsTarget:=[[params{1},params{2},params{3},params{4},params{5},params{6}],
								[ 0, 9E9, 9E9, 9E9, 9E9, 9E9]];
					fk_result_c := CalcRobT(jointsTarget, currentTool, \WObj:=currentWObj);
					
					!// Now add this pose to our return string
					addString := NumToStr(fk_result_c.trans.x,2) + " ";
					addString := addString + NumToStr(fk_result_c.trans.y,2) + " ";
					addString := addString + NumToStr(fk_result_c.trans.z,2) + " ";
					addString := addString + NumToStr(fk_result_c.rot.q1,4) + " ";
					addString := addString + NumToStr(fk_result_c.rot.q2,4) + " ";
					addString := addString + NumToStr(fk_result_c.rot.q3,4) + " ";
					addString := addString + NumToStr(fk_result_c.rot.q4,4); !End of string
				ENDIF
			ELSE
				ok := SERVER_BAD_MSG;
			ENDIF
			
		CASE 26: !Lock/Unlock tool changer
			IF nParams = 2 THEN
				ok := SERVER_OK;
				IF params{2}=0 THEN
					!SetDO solenoid_lock,0;
					!SetDO solenoid_unlock,1;
				ELSEIF params{2}=1 THEN
					!SetDO solenoid_lock,1;
					!SetDO solenoid_unlock,0;
				ELSE
					ok:=SERVER_BAD_MSG;
				ENDIF
			ELSE
				ok:=SERVER_BAD_MSG;
			ENDIF
			
		CASE 99: !Close Connection
			IF nParams = 0 THEN
				TPWrite "SERVER: Client has closed connection.";
				connected := FALSE;
				!Closing the server
				SocketClose clientSocket;
				SocketClose serverSocket;

				!Reinitiate the server
				ServerCreateAndConnect ipController,serverPort;
				connected := TRUE;
				reconnected := TRUE;
				ok := SERVER_OK;
			ELSE
				ok := SERVER_BAD_MSG;
			ENDIF
		DEFAULT:
			TPWrite "SERVER: Illegal instruction code";
			ok := SERVER_BAD_MSG;
		ENDTEST
		
		!Finally we compose the acknowledge string to send back to the client
		IF connected = TRUE THEN
			IF reconnected = FALSE THEN
				sendString := NumToStr(instructionCode,0);
				sendString := sendString + " " + idCode;
				sendString := sendString + " " + NumToStr(ok,0);
				sendString := sendString + " " + NumToStr(ClkRead(timer),2);
				sendString := sendString + " " + addString + ByteToStr(10\Char);
				SocketSend clientSocket \Str:=sendString;
			ENDIF
		ENDIF
	ENDWHILE
ERROR (LONG_JMP_ALL_ERR)
	TPWrite "SERVER: ------";
    TPWrite "SERVER: Error Handler:" + NumtoStr(ERRNO,0);
    TEST ERRNO
		CASE ERR_MOTIONSUP:
			TPWrite "SERVER: Moton suppervision error.";
			!//Stop the robot motion
			StopMove;

			!//We clear the current path, to remove any residual motions in the path queue.
			ClearPath;
	
			!//Set the target pose of the object to the current location of the robot
			!//When we retry the execution of the program, it will do a MoveL instruction to that target.
			cartesianTarget := CRobT(\Tool:=currentTool \WObj:=currentWObj);
			jointsTarget := CJointT();
	
			!//Enable the motion of the robot 
			StartMove;

			TPWrite "SERVER: Recovered.";
			TPWrite "SERVER: ------";
    		!//Retry execution of the program.
			RETRY;
		CASE ERR_SOCK_CLOSED:
			TPWrite "SERVER: Client has closed connection.";
			TPWrite "SERVER: Closing socket and restarting.";
			TPWrite "SERVER: ------";
    		connected:=FALSE;
			!//Closing the server
			SocketClose clientSocket;
			SocketClose serverSocket;
			!//Reinitiate the server
			ServerCreateAndConnect ipController,serverPort;
			reconnected:= TRUE;
			connected:= TRUE;
			RETRY; 
		
		CASE ERR_ROBLIMIT:
			!// We get here if we have tried to a cartesian position and realized it is unattainable.
			!// Simply set our 'ok' message to be a bad IK, and set the resulting joint values to 0
			ok := SERVER_BAD_IK;
			ik_result_j := [[0,0,0,0,0,0],[ 0, 9E9, 9E9, 9E9, 9E9, 9E9]];
			
			!// We'll skip the instruction that caused the error, which was the thing 
			!//trying to calculate the inverse kinematics
			TRYNEXT;
			
		DEFAULT:
			TPWrite "SERVER: Unknown error.";
			TPWrite "SERVER: Closing socket and restarting.";
			TPWrite "SERVER: ------";
    		connected:=FALSE;
			!//Closing the server
			SocketClose clientSocket;
			SocketClose serverSocket;
			!//Reinitiate the server
			ServerCreateAndConnect ipController,serverPort;
			reconnected:= TRUE;
			connected:= TRUE;
			RETRY;
	ENDTEST
ENDPROC

TRAP resetMotion
    !//Routine triggered when the digital output USER_RESET_MOTION is set to 1.
	!//It signals the need to restart the motion of the robot.
	
	!//We set this two digital outputs back to 0, in preparation for the next time there is a collision
	!SetDO USER_START_OUTPUT, 0;
    !SetDO USER_RESET_MOTION, 0;
	
	!//Note that the motion encoutered a collision 
	ok:= SERVER_COLLISION;
	
	IF moveComplete = TRUE THEN
		!//If the move instruction is complete, there is no need to raise an error. 
		!//Just continue with normal program execution.
		TPWrite "SERVER: ------";
		TPWrite "SERVER: Motion suppervision error after";
		TPWrite "SERVER: move instruction completed.";
	
		!Stop the robot motion
		StopMove;

		!We clear the current path, to remove any residual motions in the path queue.
		ClearPath;
		
		!Restart robot motion execution.
		StartMove;
		TPWrite "SERVER: Recovered.";
		TPWrite "SERVER: ------";
   	ELSE
		!//We signal the restart of the robot motion by raising the ERR_MOTIONSUP error.
		!//It will be handled by the error handler in the main procedure.
		RAISE ERR_MOTIONSUP;
	ENDIF			
ERROR
	RAISE;
ENDTRAP
ENDMODULE
