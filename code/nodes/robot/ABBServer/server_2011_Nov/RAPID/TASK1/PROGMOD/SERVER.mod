MODULE SERVER

!////////////////
!GLOBAL VARIABLES
!////////////////

!//To modify the default values go to method Initialize
PERS tooldata currentTool := [TRUE,[[0,0,105],[1,0,0,0]],[0.001,[0,0,0.001],[1,0,0,0],0,0,0]];    
PERS wobjdata currentWobj := [FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[500,0,250],[0,0,1,0]]];   
PERS speeddata currentSpeed;
PERS zonedata currentZone;

!//Clock Synchronization
PERS bool startLog:=TRUE;
PERS bool startRob:=TRUE;

!//PC communication
VAR socketdev clientSocket;
VAR socketdev serverSocket;
VAR num instructionCode;
VAR string idCode;
VAR num params{10};
VAR num nParams;
PERS string ipController:= "192.168.1.99";
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
	
!//Error Handler
VAR errnum ERR_MOTIONSUP := -1;

!//Interrupt to trap the digital output that signals the need to restart the motion of the robot.
VAR intnum iMotionReset;

!//Other
TASK PERS tooldata calibkinect:=[TRUE,[[-32.9885,0.584218,225.962],[1,0,0,0]],[1,[0,0,0],[1,0,0,0],0,0,0]];

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
PROC Initialize()
	currentTool := [TRUE,[[0,0,0],[1,0,0,0]],[0.001,[0,0,0.001],[1,0,0,0],0,0,0]];    
	currentWobj := [FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];
	currentSpeed := [100, 50, 5000, 1000];
	currentZone := [FALSE, 0.3, 0.3,0.3,0.03,0.3,0.03]; !z0
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
	
	!//Book error number for error handler
	BookErrNo ERR_MOTIONSUP;
	
	!//Configuration of interrupt that traps digital output that signals the need to restart the motion of the robot.
	!//SetDO USER_RESET_MOTION, 0;
	CONNECT iMotionReset WITH resetMotion;
	ISignalDO USER_RESET_MOTION,1,iMotionReset;
	
	!//Motion configuration
	MotionSup \On \TuneValue:=70;
	ConfL \Off;
	SingArea \Wrist;
	moveComplete:= TRUE;
	
	!//Timer synchronization with Logger
	startRob:=TRUE;
	WaitUntil startLog \PollRate:=0.01;
	ClkStart timer;

	!//Initialization of WorkObject, Tool, Speed and Zone
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
				moveComplete := TRUE;
			ELSE
				ok := SERVER_BAD_MSG;
			ENDIF	
		CASE 2: !Set Joint Coordinates
			IF nParams = 6 THEN
				jointsTarget:=[[params{1},params{2},params{3},params{4},params{5},params{6}],
							[ 0, 9E9, 9E9, 9E9, 9E9, 9E9]];
				ok := SERVER_OK;
				moveComplete := FALSE;
				MoveAbsJ jointsTarget, currentSpeed, currentZone, currentTool \Wobj:=currentWobj;
				moveComplete := TRUE;
			ELSE
				ok :=SERVER_BAD_MSG;
			ENDIF
		CASE 3: !Get Cartesian Coordinates (with current tool and workobject)
			IF nParams = 0 THEN
				cartesianPose := CRobT(\Tool:=currentTool \WObj:=currentWObj);
				addString := NumToStr(cartesianPose.trans.x,2) + " ";
				addString := addString + NumToStr(cartesianPose.trans.y,2) + " ";
				addString := addString + NumToStr(cartesianPose.trans.z,2) + " ";
				addString := addString + NumToStr(cartesianPose.rot.q1,3) + " ";
				addString := addString + NumToStr(cartesianPose.rot.q2,3) + " ";
				addString := addString + NumToStr(cartesianPose.rot.q3,3) + " ";
				addString := addString + NumToStr(cartesianPose.rot.q4,3); !End of string	
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
				currentTool.tframe.trans.x:=params{1};
				currentTool.tframe.trans.y:=params{2};
				currentTool.tframe.trans.z:=params{3};
				currentTool.tframe.rot.q1:=params{4};
				currentTool.tframe.rot.q2:=params{5};
				currentTool.tframe.rot.q3:=params{6};
				currentTool.tframe.rot.q4:=params{7};
				ok := SERVER_OK;
			ELSE
				ok:=SERVER_BAD_MSG;
			ENDIF
		CASE 7: !Specify Work Object
			IF nParams = 7 THEN
				currentWobj.oframe.trans.x:=params{1};
				currentWobj.oframe.trans.y:=params{2};
				currentWobj.oframe.trans.z:=params{3};
				currentWobj.oframe.rot.q1:=params{4};
				currentWobj.oframe.rot.q2:=params{5};
				currentWobj.oframe.rot.q3:=params{6};
				currentWobj.oframe.rot.q4:=params{7};
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
		CASE 11: !Toggle vacuum on/off
			IF nParams = 1 THEN
				ok := SERVER_OK;
				IF params{1}=0 THEN
					SetDO vacuum,0;
				ELSEIF params{1}=1 THEN
					SetDO vacuum,1;
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
				sendString := sendString + " " + addString;
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
	SetDO USER_START_OUTPUT, 0;
    SetDO USER_RESET_MOTION, 0;
	
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