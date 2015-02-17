MODULE SERVER1
!Basic Structure for the Server running inside the controller 
Local PERS tooldata currentTool := [TRUE,[[0,0,105],[1,0,0,0]],[0.001,[0,0,0.001],[1,0,0,0],0,0,0]];    
Local PERS wobjdata currentWobj := [FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[522.4,-26.8,546.3],[0,0,-1,0]]];

Local VAR speeddata currentSpeed := [500, 500, 5000, 1000];
Local VAR speeddata vibrationSpeed := [2000, 2000, 5000, 1000];
Local VAR zonedata currentZone:= fine;
!CONST string ipController:= "127.0.0.1";
Local CONST string ipController:= "192.168.1.99";

Local VAR socketdev clientSocket;
Local VAR socketdev serverSocket;
Local PERS bool logForce:=FALSE;
Local VAR bool connected;
Local VAR num instructionCode;
Local VAR string idCode;
Local VAR num params{15};
TASK PERS tooldata tool12:=[TRUE,[[0,0,0],[1,0,0,0]],[-1,[0,0,0],[1,0,0,0],0,0,0]];
TASK PERS tooldata toooldummy:=[TRUE,[[0,0,0],[1,0,0,0]],[-1,[0,0,0],[1,0,0,0],0,0,0]];
TASK PERS tooldata exptool:=[TRUE,[[-90,0,266.2],[1,0,0,0]],[1,[0,0,0],[1,0,0,0],0,0,0]];
TASK PERS tooldata covertool:=[TRUE,[[-93,0,256],[1,0,0,0]],[1,[0,0,0],[1,0,0,0],0,0,0]];
LOCAL PERS tooldata s11tool1:=[TRUE,[[0,0,0],[1,0,0,0]],[1,[0,0,0],[1,0,0,0],0,0,0]];

PROC ParseMsg(string msg)
	!Local variables
	VAR bool auxOk;
	VAR num ind:=1;
	VAR num newInd;
	VAR num length;
	VAR num indParam:=1;
	VAR string subString;
	
	length := StrLen(msg);
	!Instruction code
	newInd := StrMatch(msg,ind," ") + 1;
	subString := StrPart(msg,ind,newInd - ind - 1);
	auxOk:= StrToVal(subString, instructionCode);
	ind := newInd;
	
	!Id Code
	newInd := StrMatch(msg,ind," ") + 1;
	idCode := StrPart(msg,ind,newInd - ind - 1);
	ind := newInd;
	
	WHILE ind<length DO
		newInd := StrMatch(msg,ind," ") + 1;
		subString := StrPart(msg,ind,newInd - ind - 1);
		auxOk := StrToVal(subString, params{indParam});
		indParam := indParam + 1;
		ind := newInd;
	ENDWHILE
	!TPWrite msg;
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

PROC main()
!PROC server2()

	!Local variables
	VAR string receivedString;
	VAR string sendString;
	VAR string jointString;
	VAR robtarget posToGo;
	VAR jointtarget jointsToGo;
	VAR jointtarget joints;
	VAR robtarget actualPos;
	VAR bool ok;
	VAR clock timer;

	connected:=FALSE;
	logforce:=FALSE;
	currentTool := [TRUE,[[0,0,0],[1.0,0,0,0]],[0.001,[0,0,0.001],[1,0,0,0],0,0,0]];    
    currentWobj := [FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];
    SingArea\Wrist;
    ConfL\Off;
	ServerCreateAndConnect ipController,5000;	
	WHILE TRUE DO
		SocketReceive clientSocket \Str:=receivedString \Time:=WAIT_MAX;
		connected:=TRUE;
		!Here is where we are supposed to parse the message and execute the appropiate movements
		!TPWrite receivedString;
		ParseMsg receivedString;
		!Execution of the command
		TEST instructionCode
		CASE 0:
			!Ping
			ok:=TRUE;
		CASE 1:
			!Absolute Movement in Cartesian Coordinates
			posToGo:=[[params{1},params{2},params{3}],[params{4},params{5},params{6},params{7}],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
			MoveL posToGo, currentSpeed, currentZone, currentTool\WObj:=currentWobj;
			ok:=TRUE;
		CASE 3:
			jointsToGo:= [ [ params{1}, params{2}, params{3}, params{4}, params{5}, params{6}], [ 0, 9E9, 9E9, 9E9, 9E9, 9E9] ];
			MoveAbsJ jointsToGo, currentSpeed, currentZone, currentTool \Wobj:=currentWobj;
			ok:=TRUE;
		CASE 5:
			!Specify the Tool
			currentTool.tframe.trans.x:=params{1};
			currentTool.tframe.trans.y:=params{2};
			currentTool.tframe.trans.z:=params{3};
			currentTool.tframe.rot.q1:=params{4};
			currentTool.tframe.rot.q2:=params{5};
			currentTool.tframe.rot.q3:=params{6};
			currentTool.tframe.rot.q4:=params{7};
			ok:=TRUE;
		CASE 6:
			!TPWrite NumToStr(params{1},3) + " " + NumToStr(params{2},3) + " " + NumToStr(params{3},3) + " " + NumToStr(params{4},3) + " " + NumToStr(params{5},3) + " " + NumToStr(params{6},3) + " " + NumToStr(params{7},3);
			!Specify the Work Object
			currentWobj.oframe.trans.x:=params{1};
			currentWobj.oframe.trans.y:=params{2};
			currentWobj.oframe.trans.z:=params{3};
			currentWobj.oframe.rot.q1:=params{4};
			currentWobj.oframe.rot.q2:=params{5};
			currentWobj.oframe.rot.q3:=params{6};
			currentWobj.oframe.rot.q4:=params{7};
			ok:=TRUE;
		CASE 7:
			!Specify the Speed of the Robot
			currentSpeed.v_tcp:=params{1};
			currentSpeed.v_ori:=params{2};
			currentSpeed.v_leax:=params{3};
			currentSpeed.v_reax:=params{4};
			ok:=TRUE;
		CASE 8:
			!Specify the ZoneData for the Robot
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
			
		CASE 9:
			ok:=TRUE;
			joints := CJointT();
			jointString := NumToStr(joints.robax.rax_1,3) + " ";
			jointString := jointString + NumToStr(joints.robax.rax_2,3) + " ";
			jointString := jointString + NumToStr(joints.robax.rax_3,3) + " ";
			jointString := jointString + NumToStr(joints.robax.rax_4,3) + " ";
			jointString := jointString + NumToStr(joints.robax.rax_5,3) + " ";
			jointString := jointString + NumToStr(joints.robax.rax_6,3);
				
		CASE 11:
			logForce:=TRUE;
			ok:=TRUE;
		CASE 12:
			logForce:=FALSE;
			ok:=TRUE;
		CASE 13:
			actualPos := CRobT(\Tool:=tool0 \WObj:=wobj0);
			ClkReset timer;
			ClkStart timer;
			WHILE ClkRead(timer) < params{1} DO
				MoveL Offs(actualPos,0,-1,0), vmax, z0, tool0 \WObj:=wobj0;
				MoveL Offs(actualPos,-0.7071,-1.7071,0), vmax, z0, tool0 \WObj:=wobj0;
				MoveL Offs(actualPos,0,-2.4142,0), vmax, z0, tool0 \WObj:=wobj0;
				MoveL Offs(actualPos,0,-1.4142,0), vmax, z0, tool0 \WObj:=wobj0;				
				MoveL Offs(actualPos,0.7071,-0.7071,0), vmax, z0, tool0 \WObj:=wobj0;				
				MoveL actualPos, vmax, z0, tool0 \WObj:=wobj0;
			ENDWHILE
			ClkStop timer;
			MoveL actualPos, vmax, fine, tool0 \WObj:=wobj0;
			ok:=TRUE;
		DEFAULT:
			TPWrite "SERVER: Illegal instruction code";
			ok:=FALSE;
		ENDTEST
			
		!Finally we compose the acknowledge string to send back to the client
		sendString:= idCode;
		IF ok=TRUE THEN
			sendString := sendString + " 1 ";
		ELSE
			sendString := sendString + " 0 ";
		ENDIF
		
		IF instructionCode=9 THEN
			! If we wanted the joint position, add that to our string
			sendString := sendString + jointString;
		ELSE
			! Otherwise, always add the cartesian position of the robot
			actualPos := CRobT(\Tool:=currentTool \WObj:=currentWObj);
			sendString := sendString + NumToStr(actualPos.trans.x,2) + " ";
			sendString := sendString + NumToStr(actualPos.trans.y,2) + " ";
			sendString := sendString + NumToStr(actualPos.trans.z,2) + " ";
			sendString := sendString + NumToStr(actualPos.rot.q1,3) + " ";
			sendString := sendString + NumToStr(actualPos.rot.q2,3) + " ";
			sendString := sendString + NumToStr(actualPos.rot.q3,3) + " ";
			sendString := sendString + NumToStr(actualPos.rot.q4,3);
		ENDIF
		
		IF connected=TRUE THEN
			SocketSend clientSocket \Str:=sendString;
		ENDIF
	ENDWHILE
ERROR
	IF ERRNO=ERR_SOCK_CLOSED THEN
		TPWrite "SERVER: Client has closed connection.";
	ELSE
		TPWrite "SERVER: Server disconnected: Unknown problem.";
	ENDIF
	connected:=FALSE;
	!Closing the server
	SocketClose clientSocket;
	SocketClose serverSocket;
	!Reinitiate the server
	ServerCreateAndConnect ipController,5000;
	RETRY;
ENDPROC
ENDMODULE