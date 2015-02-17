MODULE SERVER

!////////////////
!GLOBAL VARIABLES
!////////////////
!Robot configuration	
!//To modify the default values go to method Initialize
PERS tooldata currentTool := [TRUE,[[0,0,155.5],[1,0,0,0]],[0.001,[0,0,0.001],[1,0,0,0],0,0,0]];    
PERS wobjdata currentWobj := [FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];   
PERS speeddata currentSpeed;
PERS zonedata currentZone;

!Clock Synchronization
PERS bool startLog:=TRUE;
PERS bool startRob:=TRUE;

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

PERS num loggerWaitTime:= 0.01;
!PERS num loggerWaitTime:= 0.1;

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
			TPWrite "SERVER: Problem serving an incomming connection. Try reconnecting.";
		ENDIF
		 !Wait 0.5 seconds for the next reconnection
		 WaitTime 0.5;
	ENDWHILE
	TPWrite "SERVER: Connected to Client at IP " + clientIP;
	
ENDPROC

!Parameter initialization
! Loads default values for
! - Tool.
! - WorkObject.
! - Zone.
! - Speed.
PROC Initialize()
	currentTool := [TRUE,[[0,0,0],[1,0,0,0]],[0.001,[0,0,0.001],[1,0,0,0],0,0,0]];    
	currentWobj := [FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];
	currentSpeed := [100, 50, 5000, 1000];
	currentZone := [FALSE, 0.3, 0.3,0.3,0.03,0.3,0.03]; !z0
ENDPROC

!Main procedure
PROC main()
	!Local variables
	VAR string receivedString;
	VAR string sendString;
	VAR string addString;
	VAR bool ok;         ! Message was correct
	VAR bool connected;  ! client connected
	VAR bool reconnected;! Reconnection During the iteration
	VAR robtarget cartesianTarget;
	VAR robtarget cartesianPose;
	VAR jointtarget jointsTarget;
 	VAR jointtarget jointsPose;
	
	VAR clock timer;
	
	ConfL \Off;
	SingArea \Wrist;
	
	startRob:=TRUE;
	WaitUntil startLog \PollRate:=0.01;
	ClkStart timer;

	!Initialization
	Initialize;

 	!Socket connection
	connected:=FALSE;
	ServerCreateAndConnect ipController,serverPort;	
	connected:=TRUE;

	!Get current position of the robot
	!cartesianTarget := CRobT(\Tool:=currentTool \WObj:=currentWObj);
	!jointsTarget := CJointT();
	WHILE TRUE DO
		SocketReceive clientSocket \Str:=receivedString \Time:=WAIT_MAX;
		reconnected:=FALSE;
		!connected:=TRUE;
		ParseMsg receivedString;
		
		!String to add to the reply
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
				MoveL cartesianTarget, currentSpeed, currentZone, currentTool \WObj:=currentWobj ;
				ok :=TRUE;
			ELSE
				ok :=FALSE;
			ENDIF	
		CASE 2: !Set Joint Coordinates
			IF nParams = 6 THEN
				jointsTarget:=[[params{1},params{2},params{3},params{4},params{5},params{6}],
				               [ 0, 9E9, 9E9, 9E9, 9E9, 9E9]];
		                MoveAbsJ jointsTarget, currentSpeed, currentZone, currentTool \Wobj:=currentWobj;
				ok :=TRUE;
			ELSE
				ok :=FALSE;
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
				ok:=TRUE;
			ELSE
				ok :=FALSE;
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
				ok:=TRUE;
			ELSE
				ok:=FALSE;
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
				ok:=TRUE;
			ELSE
				ok:=FALSE;
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
				ok:=TRUE;
			ELSE
				ok:=FALSE;
			ENDIF
		CASE 8: !Specify Speed of the Robot
			IF nParams = 2 THEN
				currentSpeed.v_tcp:=params{1};
				currentSpeed.v_ori:=params{2};
				ok:=TRUE;
			ELSE
				ok:=FALSE;
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
				ok:=TRUE;
			ELSE
				ok:=FALSE;
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

				connected := FALSE;
				!Closing the server
				SocketClose clientSocket;
				SocketClose serverSocket;

				!Reinitiate the server
				ServerCreateAndConnect ipController,serverPort;
				connected := TRUE;
				reconnected := TRUE;
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
			IF reconnected = FALSE THEN
				sendString:= NumToStr(instructionCode,0) + " " + idCode;
				IF ok=TRUE THEN
					sendString := sendString + " 1 " + NumToStr(ClkRead(timer),2);
				ELSE
					sendString := sendString + " 0 " + NumToStr(ClkRead(timer),2);
				ENDIF
				sendString := sendString + " " + addString;
				SocketSend clientSocket \Str:=sendString;
			ENDIF
		ENDIF
	ENDWHILE
ERROR
	IF ERRNO=ERR_SOCK_CLOSED THEN
		TPWrite "SERVER: Client has closed connection.";
	ELSE
		TPWrite "SERVER: Connection lost: Unknown problem.";
	ENDIF

	!Closing the server
	SocketClose clientSocket;
	SocketClose serverSocket;

	!Reinitialize
	connected:=FALSE;
	!We keep the current values for WorkObject, Tool, Zone ans Speed
	!Initialize;	
	ServerCreateAndConnect ipController,serverPort;
	connected:=TRUE;
	reconnected:=TRUE;
	RETRY;
ENDPROC
ENDMODULE