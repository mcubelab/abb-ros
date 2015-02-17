MODULE SERVER
!////////////////
!GLOBAL VARIABLES
!////////////////

!Robot configuration	
PERS tooldata currentTool;    
PERS wobjdata currentWobj;
PERS speeddata currentSpeed;
PERS zonedata currentZone;
PERS zonedata nonBlockingZone;

!Intra-task Communication
PERS robtarget cartesianTarget;
PERS jointtarget jointsTarget;
PERS bool move;  !Whether the robot should move or not.
PERS bool commMode;  !Communication mode: TRUE -> Blocking; FALSE -> Non-blocking.
PERS bool moveMode;  !Motion mode: TRUE -> Cartesian; FALSE -> Joints.

!PC communication
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
!PERS num loggerWaitTime:= 0.1;
PERS num loggerWaitTime:= 0.01;

PERS num pollRate:=0.01;
!PERS num pollRate:=0.01;

!////////////////
!LOCAL METHODS
!////////////////

!Method to parse the message received from a PC through the socket
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
		nParams := -1;
	ELSE
		!Instruction code
		newInd := StrMatch(msg,ind," ") + 1;
		subString := StrPart(msg,ind,newInd - ind - 1);
		auxOk:= StrToVal(subString, instructionCode);
		ind := newInd;

		!Id Code
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
ENDPROC

PROC ServerCreateAndConnect(string ip, num port)
	VAR string clientIP;
	
	SocketCreate serverSocket;
	SocketBind serverSocket, ip, port;
	SocketListen serverSocket;
	TPWrite "SERVER: Server waiting for incomming connections ...";
	WHILE SocketGetStatus(clientSocket) <> SOCKET_CONNECTED DO
		SocketAccept serverSocket,clientSocket \ClientAddress:=clientIP \Time:=WAIT_MAX;
		IF SocketGetStatus(clientSocket) <> SOCKET_CONNECTED THEN
			TPWrite "SERVER: Problem serving an incomming connection. Try reconnecting.";
		ENDIF
		 !Wait 0.5 seconds for the next reconnection
		 WaitTime 0.5;
	ENDWHILE
	TPWrite "SERVER: Connected to Client at IP " + clientIP;
ENDPROC

PROC Initialize()
	currentTool := [TRUE,[[0,0,0],[1,0,0,0]],[0.001,[0,0,0.001],[1,0,0,0],0,0,0]];    
	currentWobj := [FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];
	currentSpeed := [500, 150, 5000, 1000];
	currentZone := [FALSE, 0.3, 0.3,0.3,0.03,0.3,0.03]; !z0
	nonBlockingZone := [FALSE, 0.3, 0.3,0.3,0.03,0.3,0.03]; !z0
	move := FALSE;
	commMode := TRUE;
	moveMode := TRUE;

ENDPROC

PROC main()
	!Local variables
	VAR string receivedString;
	VAR string sendString;
	VAR string addString;
	VAR bool ok;         ! Message was correct
	VAR bool connected;  ! client connected
	VAR robtarget position;
 	VAR jointtarget joints;
	
	!move:=FALSE;
	!Initialization
	Initialize;
	connected:=FALSE;
	ServerCreateAndConnect ipController,serverPort;	
	
	!Get current position of the robot
	cartesianTarget := CRobT(\Tool:=currentTool \WObj:=currentWObj);
	jointsTarget := CJointT();
	WHILE TRUE DO
		SocketReceive clientSocket \Str:=receivedString \Time:=WAIT_MAX;
		connected:=TRUE;
		ParseMsg receivedString;
		
		addString := "";
		!Execution of the command
		TEST instructionCode
		CASE 0: !Ping
		    !Message Check
			IF nParams = 0 THEN
				ok :=TRUE;
			ELSE
				ok :=FALSE;
			ENDIF
		CASE 1: !Set Cartesian Coordinates
			IF nParams = 7 THEN
				cartesianTarget :=[[params{1},params{2},params{3}],
						  [params{4},params{5},params{6},params{7}],
					  	  [0,0,0,0],
					  	  [9E9,9E9,9E9,9E9,9E9,9E9]];
				moveMode := TRUE; !Cartesian Mode
				move := TRUE;
				IF commMode = TRUE THEN
					WaitUntil move = FALSE;
				ENDIF
				ok :=TRUE;
			ELSE
				ok :=FALSE;
			ENDIF	
		CASE 2: !Set Joint Coordinates
			IF nParams = 6 THEN
				jointsTarget:=[[params{1},params{2},params{3},params{4},params{5},params{6}],
				               [ 0, 9E9, 9E9, 9E9, 9E9, 9E9]];
				moveMode := FALSE;!Joint Mode
				move := TRUE;
				IF commMode = TRUE THEN
					WaitUntil move = FALSE;
				ENDIF
				ok :=TRUE;
			ELSE
				ok :=FALSE;
			ENDIF
		CASE 3: !Get Cartesian Coordinates (with current tool and workobject)
			IF nParams = 0 THEN
				position := CRobT(\Tool:=currentTool \WObj:=currentWObj);
				addString := NumToStr(position.trans.x,2) + " ";
				addString := addString + NumToStr(position.trans.y,2) + " ";
				addString := addString + NumToStr(position.trans.z,2) + " ";
				addString := addString + NumToStr(position.rot.q1,3) + " ";
				addString := addString + NumToStr(position.rot.q2,3) + " ";
				addString := addString + NumToStr(position.rot.q3,3) + " ";
				addString := addString + NumToStr(position.rot.q4,3); !End of string	
				ok:=TRUE;
			ELSE
				ok :=FALSE;
			ENDIF
		CASE 4: !Get Joint Coordinates
			IF nParams = 0 THEN
				joints := CJointT();
				addString := NumToStr(joints.robax.rax_1,2) + " ";
				addString := addString + NumToStr(joints.robax.rax_2,2) + " ";
				addString := addString + NumToStr(joints.robax.rax_3,2) + " ";
				addString := addString + NumToStr(joints.robax.rax_4,2) + " ";
				addString := addString + NumToStr(joints.robax.rax_5,2) + " ";
				addString := addString + NumToStr(joints.robax.rax_6,2); !End of string
				ok:=TRUE;
			ELSE
				ok:=FALSE;
			ENDIF
		CASE 5: !Stop Motion (Only works on non-blocking mode)
			IF nParams = 0 THEN
				move := FALSE;
				WaitTime 0.5;
				cartesianTarget := CRobT(\Tool:=currentTool \WObj:=currentWObj);
				jointsTarget := CJointT();
				ok:=TRUE;
			ELSE
				ok:=FALSE;
			ENDIF
		CASE 6: !Specify Tool
			IF nParams = 7 THEN
				!If the robot is moving in non-blocking mode, we stop it first.
				move := FALSE;
				currentTool.tframe.trans.x:=params{1};
				currentTool.tframe.trans.y:=params{2};
				currentTool.tframe.trans.z:=params{3};
				currentTool.tframe.rot.q1:=params{4};
				currentTool.tframe.rot.q2:=params{5};
				currentTool.tframe.rot.q3:=params{6};
				currentTool.tframe.rot.q4:=params{7};
				cartesianTarget := CRobT(\Tool:=currentTool \WObj:=currentWObj);
				jointsTarget := CJointT();
				ok:=TRUE;
			ELSE
				ok:=FALSE;
			ENDIF
		CASE 7: !Specify Work Object
			IF nParams = 7 THEN
				!If the robot is moving in non-blocking mode, we stop it first.
				move := FALSE;
				currentWobj.oframe.trans.x:=params{1};
				currentWobj.oframe.trans.y:=params{2};
				currentWobj.oframe.trans.z:=params{3};
				currentWobj.oframe.rot.q1:=params{4};
				currentWobj.oframe.rot.q2:=params{5};
				currentWobj.oframe.rot.q3:=params{6};
				currentWobj.oframe.rot.q4:=params{7};
				cartesianTarget := CRobT(\Tool:=currentTool \WObj:=currentWObj);
				jointsTarget := CJointT();
				ok:=TRUE;
			ELSE
				ok:=FALSE;
			ENDIF
		CASE 8: !Specify Speed of the Robot
			IF nParams = 2 THEN
				move := FALSE;
				currentSpeed.v_tcp:=params{1};
				currentSpeed.v_ori:=params{2};
				ok:=TRUE;
			ELSE
				ok:=FALSE;
			ENDIF
		CASE 9: !Specify ZoneData
			IF nParams = 1 THEN
				move := FALSE;
				ok:=TRUE;
				IF params{1}=0 THEN
					currentZone:=fine;
				ELSEIF params{1}=1 THEN
					currentZone:=z0;
				ELSEIF params{1}=2 THEN
					currentZone:=z1;
				ELSEIF params{1}=3 THEN
					currentZone:=z5;
				ELSEIF params{1}=4 THEN
					currentZone:=z10;
				ELSE
					ok:=FALSE;
				ENDIF
			ELSE
				ok:=FALSE;
			ENDIF
		CASE 10: !Specify communication mode 
			IF nParams = 1 THEN
				move := FALSE;
				ok:=TRUE;
				IF params{1}=0 THEN
					commMode := FALSE;
				ELSEIF params{1}=1 THEN
					commMode := TRUE;
				ELSE
					ok:=FALSE;
				ENDIF
			ELSE
				ok := FALSE;
			ENDIF
		CASE 11: !Toggle vacuum on/off
			IF nParams = 1 THEN
				ok:=TRUE;
				IF params{1}=0 THEN
					SetDO vacuum,0;
				ELSEIF params{1}=1 THEN
					SetDO vacuum,1;
				ELSE
					ok:=FALSE;
				ENDIF
			ELSE
				ok:=FALSE;
			ENDIF
		CASE 99: !Close Connection
			IF nParams = 0 THEN
				move := FALSE;
				connected := FALSE;
				!Closing the server
				SocketClose clientSocket;
				SocketClose serverSocket;
				!Reinitiate the server
				ServerCreateAndConnect ipController,serverPort;
				cartesianTarget := CRobT(\Tool:=currentTool \WObj:=currentWObj);
				jointsTarget := CJointT();
				ok := TRUE;
			ELSE
				ok := FALSE;
			ENDIF
		DEFAULT:
			TPWrite "SERVER: Illegal instruction code";
			ok:=FALSE;
		ENDTEST
		
		!Finally we compose the acknowledge string to send back to the client
		IF connected = TRUE THEN
			sendString:= NumToStr(instructionCode,0) + " " + idCode;
			IF ok=TRUE THEN
				sendString := sendString + " 1 ";
			ELSE
				sendString := sendString + " 0 ";
			ENDIF
			sendString := sendString + addString;
			SocketSend clientSocket \Str:=sendString;
		ENDIF
	ENDWHILE
ERROR
	IF ERRNO=ERR_SOCK_CLOSED THEN
		TPWrite "SERVER: Client has closed connection.";
	ELSE
		TPWrite "SERVER: Connection lost: Unknown problem.";
	ENDIF
	connected:=FALSE;
	!move:=FALSE;
	Initialize;
	
	!Closing the server
	SocketClose clientSocket;
	SocketClose serverSocket;
	!Reinitiate the server
	ServerCreateAndConnect ipController,serverPort;
	cartesianTarget := CRobT(\Tool:=currentTool \WObj:=currentWObj);
	jointsTarget := CJointT();
	RETRY;
ENDPROC
ENDMODULE