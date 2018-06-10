MODULE SERVER

    !////////////////
    !GLOBAL VARIABLES
    !////////////////

    !//To modify the default values go to method Initialize
    PERS tooldata currentTool2:=[TRUE,[[0,0,0],[1,0,0,0]],[0.001,[0,0,0.001],[1,0,0,0],0,0,0]];
    PERS wobjdata currentWobj2:=[FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];
    PERS speeddata currentSpeed2;
    PERS zonedata currentZone2;

    !// Clock Synchronization
    PERS bool startLog2:=TRUE;
    PERS bool startRob2:=TRUE;

    !// Mutex between logger and changing the tool and work objects
    PERS bool frameMutex2:=FALSE;
    PERS num motSupValue2 := 90; 

    !//PC communication
    VAR socketdev clientSocket;
    VAR socketdev serverSocket;
    VAR num instructionCode; 
    VAR string idCode;
    VAR num params{10};
    VAR num specialParams{5};
    VAR num nParams;
    !PERS string ipController:= "192.168.37.3"; !"mCube APC 1600id"
    PERS string ipController2:="192.168.125.1";  !"mCube"
    PERS num serverPort2:=5002;
    PERS num loggerPort2:=5003;

    !//Logger sampling rate
    PERS num loggerWaitTime2:=0.01;
    !PERS num loggerWaitTime:= 0.1; This is adequate for virtual server

    !//Motion of the robot
    VAR robtarget cartesianTarget;
    VAR jointtarget jointsTarget;
    VAR bool moveComplete;  !True when program pointer leaves a Move instruction.

    !//Buffered move variables
    CONST num MAX_BUFFER:=512;
    VAR num BUFFER_POS:=0;
    VAR num BUFFER_JOINT_POS:=0;
    VAR robtarget bufferTargets{MAX_BUFFER};
    VAR speeddata bufferSpeeds{MAX_BUFFER};
    VAR jointtarget bufferJointPos{MAX_BUFFER};
    VAR speeddata bufferJointSpeeds{MAX_BUFFER};

    !//External axis position variables
    VAR extjoint externalAxis;

    !//Circular move buffer
    VAR robtarget circPoint;


    !//Correct Instruction Execution and possible return values
    VAR num ok;
    VAR num collision;
    CONST num SERVER_BAD_MSG:=0;
    CONST num SERVER_OK:=1;
    CONST num SERVER_COLLISION:=2;
    CONST num SERVER_BAD_IK:=3;
    CONST num SERVER_BAD_FK:=4;

    !//Error Handler
    VAR errnum ERR_MOTIONSUP:=-1;

    !//Interrupt to trap the digital output that signals the need to restart the motion of the robot.
    VAR intnum iMotionReset;

    !// Inverse and forward kinematic results
    VAR jointtarget ik_result_j;
    VAR robtarget fk_result_c;

    !// Upper and lower joint bounds, in degrees
    VAR num upper_joint_limits{7};
    VAR num lower_joint_limits{7};
    PERS wobjdata wobj1:=[FALSE,TRUE,"",[[600,0,0],[1,0,0,0]],[[0,0,0],[0.7071,0,-0.7071,0]]];

    !// Hand related
    VAR bool isHandCalibrated;
    VAR num handSpeed;
    VAR num handPosition;
    VAR errnum myerrnum; 
    
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
        VAR bool end:=FALSE;

        length:=StrMatch(msg,1,"#");
        IF length>StrLen(msg) THEN
            !Corrupt message
            nParams:=-1;
        ELSE
            !Find Instruction code
            newInd:=StrMatch(msg,ind," ")+1;
            subString:=StrPart(msg,ind,newInd-ind-1);
            auxOk:=StrToVal(subString,instructionCode);
            IF auxOk=FALSE THEN
                !Corrupt instruction code
                nParams:=-1;
            ELSE
                ind:=newInd;

                !Find Id Code
                newInd:=StrMatch(msg,ind," ")+1;
                idCode:=StrPart(msg,ind,newInd-ind-1);
                ind:=newInd;

                !Set of parameters (maximum of 8)
                WHILE end=FALSE DO
                    newInd:=StrMatch(msg,ind," ")+1;
                    IF newInd>length THEN
                        end:=TRUE;
                    ELSE
                        subString:=StrPart(msg,ind,newInd-ind-1);
                        auxOk:=StrToVal(subString,params{indParam});
                        indParam:=indParam+1;
                        ind:=newInd;
                    ENDIF
                ENDWHILE
                nParams:=indParam-1;
            ENDIF
        ENDIF
    ENDPROC

    !Handshake between server and client:
    ! - Creates socket.
    ! - Waits for incoming TCP connection.
    PROC ServerCreateAndConnect(string ip,num port)
        VAR string clientIP;

        SocketCreate serverSocket;
        SocketBind serverSocket,ip,port;
        SocketListen serverSocket;
        TPWrite "SERVER: Server waiting for incomming connections ...";
        WHILE SocketGetStatus(clientSocket)<>SOCKET_CONNECTED DO
            SocketAccept serverSocket,clientSocket\ClientAddress:=clientIP\Time:=WAIT_MAX;
            IF SocketGetStatus(clientSocket)<>SOCKET_CONNECTED THEN
                TPWrite "SERVER: Problem serving an incomming connection.";
                TPWrite "SERVER: Try reconnecting.";
            ENDIF
            !//Wait 0.5 seconds for the next reconnection
            WaitTime 0.5;
        ENDWHILE
        TPWrite "SERVER: Connected to IP "+clientIP;

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

        currentTool2:=[TRUE,[[0,0,0],[1,0,0,0]],[0.001,[0,0,0.001],[1,0,0,0],0,0,0]];
        currentWobj2:=[FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];
        currentSpeed2:=[100,50,5000,1000];
        currentZone2:=[FALSE,0.3,0.3,0.3,0.03,0.3,0.03];        !z0

        !// Get all of the joint bounds for later use
        FOR i FROM 1 TO 7 DO
            path:="MOC/ARM/rob_L_"+NumToStr(i,0);
            ReadCfgData path,"upper_joint_bound",upper_joint_limits{i};
            ReadCfgData path,"lower_joint_bound",lower_joint_limits{i};

            !// The joint limits are in radians, so convert these to degrees
            upper_joint_limits{i}:=upper_joint_limits{i}*180.0/pi;
            lower_joint_limits{i}:=lower_joint_limits{i}*180.0/pi;
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
        VAR bool connected;        ! //Client connected
        VAR bool reconnected;      ! //Reconnection During the iteration
        VAR robtarget cartesianPose;
        VAR jointtarget jointsPose;
        VAR clock timer;
        VAR num quatMag;
        VAR num ind;
        VAR bool move;
        
        !//Book error number for error handler
        BookErrNo ERR_MOTIONSUP;

        !//Configure the interrupt "iMotionReset"
        !//to traps a raise on the digital output "USER_RESET_MOTION"
        !//Meant to signal the need to restart the motion of the robot.
        !//SetDO USER_RESET_MOTION, 0;

        !// We are not currently changing the frame
        frameMutex2:=FALSE;

        !//Motion configuration
        MotionSup\On\TuneValue:=100;
        SingArea\Off;
        !Use \Wrist if we want the robot to charge its course to avoid singularities.
        ConfL\Off;
        !Use \On if we want the robot to enforce the configuration specified in MoveL
        ConfJ\Off;
        moveComplete:=TRUE;
        collision:=0;

        !//Timer synchronization with Logger
        startRob2:=TRUE;
        WaitUntil startLog2\PollRate:=0.01;
        ClkStart timer;

        !//Initialization of WorkObject, Tool, Speed, Zone and Inertia. 
        !//Get the joint limits from Robot Configuration.
        Initialize;
        Hand_Initialize \Calibrate;

        !//Socket connection
        connected:=FALSE;
        ServerCreateAndConnect ipController2,serverPort2;
        connected:=TRUE;

        !//Infinite loop to serve commands
        WHILE TRUE DO
            !//Initialization of program flow variables
            ok:=SERVER_OK;
            !//Correctness of executed instruction.
            reconnected:=FALSE;
            !//Has communication dropped after receiving a command?
            addString:="";
            !//String to add to the reply.

            !//Receive a command
            SocketReceive clientSocket\Str:=receivedString\Time:=WAIT_MAX;
            ParseMsg receivedString;
            collision:=0;
            
            TPWrite NumToStr(instructionCode, 0);
            !//Execution of the command
            TEST instructionCode
            CASE 0:
                !Ping
                IF nParams=0 THEN
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
            CASE 1:
                !Set Cartesian Coordinates
                IF nParams=7 THEN
                    !Linear moves. Specify (x, y, z), rotation as quaternion. 
                    !For the arm angle, use the current pose
                    cartesianPose:=CRobT(\Tool:=currentTool2\WObj:=currentWobj2);
                    cartesianTarget:=[[params{1},params{2},params{3}],
                                  [params{4},params{5},params{6},params{7}],
                                  [0,0,0,0],
                                  [cartesianPose.extax.eax_a,9E9,9E9,9E9,9E9,9E9]];
                    ok:=SERVER_OK;
                    moveComplete:=FALSE;
                    MoveL cartesianTarget,currentSpeed2,currentZone2,currentTool2\WObj:=currentWobj2;
                    moveComplete:=TRUE;
                ELSEIF nParams=8 THEN
                    !Linear moves. Specify (x, y, z), rotation as quaternion and 8th parameter is "arm angle"
                    cartesianTarget:=[[params{1},params{2},params{3}],
                                  [params{4},params{5},params{6},params{7}],
                                  [0,0,0,0],
                                  [params{8},9E9,9E9,9E9,9E9,9E9]];
                    ok:=SERVER_OK;
                    moveComplete:=FALSE;
                    MoveL cartesianTarget,currentSpeed2,currentZone2,currentTool2\WObj:=currentWobj2;
                    moveComplete:=TRUE;
                ELSEIF nParams=9 THEN
                    !Empty parameter so cartesian move interpolating in joints
                    !Specify (x, y, z), rotation as quaternion. 
                    cartesianPose:=CRobT(\Tool:=currentTool2\WObj:=currentWobj2);
                    cartesianTarget:=[[params{1},params{2},params{3}],
                                   [params{4},params{5},params{6},params{7}],
                                   [0,0,0,0],
                                   [cartesianPose.extax.eax_a,9E9,9E9,9E9,9E9,9E9]];
                    ok:=SERVER_OK;
                    moveComplete:=FALSE;
                    MoveJ cartesianTarget,currentSpeed2,currentZone2,currentTool2\WObj:=currentWobj2;
                    moveComplete:=TRUE;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
            CASE 2:
                !Set Joint Coordinates
                IF nParams=7 THEN
                    move:=true;
                    FOR i FROM 1 TO 7 DO
                        IF params{i}>upper_joint_limits{i}OR params{i}<lower_joint_limits{i} THEN
                            !// If not, then we'll tell the user that their forward kinematics are invalid
                            ok:=SERVER_BAD_FK;
                            move:=false;
                        ENDIF
                    ENDFOR
                    IF move=TRUE THEN
                        jointsTarget:=[[params{1},params{2},params{3},params{4},params{5},params{6}],
                                   [params{7},9E9,9E9,9E9,9E9,9E9]];
                        ok:=SERVER_OK;
                        moveComplete:=FALSE;
                        MoveAbsJ jointsTarget,currentSpeed2,currentZone2,currentTool2
                    \Wobj:=currentWobj2;
                        moveComplete:=TRUE;
                    ENDIF
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

            CASE 3:
                !Get Cartesian Coordinates (with current tool and workobject)
                IF nParams=0 THEN
                    cartesianPose:=CRobT(\Tool:=currentTool2\WObj:=currentWobj2);
                    addString:=NumToStr(cartesianPose.trans.x,2)+" ";
                    addString:=addString+NumToStr(cartesianPose.trans.y,2)+" ";
                    addString:=addString+NumToStr(cartesianPose.trans.z,2)+" ";
                    addString:=addString+NumToStr(cartesianPose.rot.q1,4)+" ";
                    addString:=addString+NumToStr(cartesianPose.rot.q2,4)+" ";
                    addString:=addString+NumToStr(cartesianPose.rot.q3,4)+" ";
                    addString:=addString+NumToStr(cartesianPose.rot.q4,4);
                    !End of string
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
            CASE 4:
                !Get Joint Coordinates
                IF nParams=0 THEN
                    jointsPose:=CJointT();
                    addString:=NumToStr(jointsPose.robax.rax_1,2)+" ";
                    addString:=addString+NumToStr(jointsPose.robax.rax_2,2)+" ";
                    addString:=addString+NumToStr(jointsPose.robax.rax_3,2)+" ";
                    addString:=addString+NumToStr(jointsPose.robax.rax_4,2)+" ";
                    addString:=addString+NumToStr(jointsPose.robax.rax_5,2)+" ";
                    addString:=addString+NumToStr(jointsPose.robax.rax_6,2);
                    !End of string
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
            CASE 5:
                !Set motion supervision threshold
                IF nParams=1 AND params{1}>=0 AND params{1}<=300 THEN
                    MotionSup\On\TuneValue:=params{1};
                    motSupValue2:=params{1};
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
            CASE 6:
                !Specify Tool
                IF nParams=7 THEN
                    WHILE (frameMutex2) DO
                        !// If the frame is being used by logger, wait here
                    ENDWHILE
                    frameMutex2:=TRUE;
                    currentTool2.tframe.trans.x:=params{1};
                    currentTool2.tframe.trans.y:=params{2};
                    currentTool2.tframe.trans.z:=params{3};
                    currentTool2.tframe.rot.q1:=params{4};
                    currentTool2.tframe.rot.q2:=params{5};
                    currentTool2.tframe.rot.q3:=params{6};
                    currentTool2.tframe.rot.q4:=params{7};
                    frameMutex2:=FALSE;
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
            CASE 7:
                !Specify Work Object
                IF nParams=7 THEN
                    WHILE (frameMutex2) DO
                        !// If the frame is being used by logger, wait here
                    ENDWHILE
                    frameMutex2:=TRUE;
                    currentWobj2.oframe.trans.x:=params{1};
                    currentWobj2.oframe.trans.y:=params{2};
                    currentWobj2.oframe.trans.z:=params{3};
                    currentWobj2.oframe.rot.q1:=params{4};
                    currentWobj2.oframe.rot.q2:=params{5};
                    currentWobj2.oframe.rot.q3:=params{6};
                    currentWobj2.oframe.rot.q4:=params{7};
                    frameMutex2:=FALSE;
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
            CASE 14:
                !Specify Inertia
                IF nParams=7 THEN
                    currentTool2.tload.mass:=params{1};
                    currentTool2.tload.cog.x:=params{2};
                    currentTool2.tload.cog.y:=params{3};
                    currentTool2.tload.cog.z:=params{4};
                    currentTool2.tload.ix:=params{5};
                    currentTool2.tload.iy:=params{6};
                    currentTool2.tload.iz:=params{7};
                    currentTool2.tload.aom.q1:=1.0;
                    currentTool2.tload.aom.q2:=0.0;
                    currentTool2.tload.aom.q3:=0.0;
                    currentTool2.tload.aom.q4:=0.0;
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
            CASE 7:
                !Specify Work Object
                IF nParams=7 THEN
                    WHILE (frameMutex2) DO
                        !// If the frame is being used by logger, wait here
                    ENDWHILE
                    frameMutex2:=TRUE;
                    currentWobj2.oframe.trans.x:=params{1};
                    currentWobj2.oframe.trans.y:=params{2};
                    currentWobj2.oframe.trans.z:=params{3};
                    currentWobj2.oframe.rot.q1:=params{4};
                    currentWobj2.oframe.rot.q2:=params{5};
                    currentWobj2.oframe.rot.q3:=params{6};
                    currentWobj2.oframe.rot.q4:=params{7};
                    frameMutex2:=FALSE;
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
            CASE 8:
                !Specify Speed of the Robot
                IF nParams=2 THEN
                    currentSpeed2.v_tcp:=params{1};
                    currentSpeed2.v_ori:=params{2};
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
            CASE 9:
                !Specify ZoneData
                IF nParams=4 THEN
                    IF params{1}=1 THEN
                        currentZone2.finep:=TRUE;
                        currentZone2.pzone_tcp:=0.0;
                        currentZone2.pzone_ori:=0.0;
                        currentZone2.zone_ori:=0.0;
                    ELSE
                        currentZone2.finep:=FALSE;
                        currentZone2.pzone_tcp:=params{2};
                        currentZone2.pzone_ori:=params{3};
                        currentZone2.zone_ori:=params{4};
                    ENDIF
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
            CASE 10:
                !Get Robot Angle
                IF nParams=0 THEN
                    cartesianPose:=CRobT(\Tool:=currentTool2\WObj:=currentWobj2);
                    addString:=NumToStr(cartesianPose.extax.eax_a,2)+" ";
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                
            CASE 12:
                !Inverse Kinematics Solver
                IF nParams=8 THEN
                    !// First, let's make sure the quaternion is normalized
                    IF Abs(1.0-Sqrt(params{4}*params{4}+params{5}*params{5}+params{6}*params{6}+params{7}*params{7}))>0.001 THEN
                        !// If not, then we cannot find the inverse kinematics for this pose
                        ok:=SERVER_BAD_IK;
                    ELSE
                        !// Otherwise, let's normalize our quaternion 
                        cartesianTarget:=[[params{1},params{2},params{3}],
                            [params{4},params{5},params{6},params{7}],
                            [-1, -1, 0, 11], 
                            [params{8},9E9,9E9,9E9,9E9,9E9]];
                        !// third row used to be [0, 0, 0, 0]
                        ok:=SERVER_OK;

                        !// Now calculate the joint angles, keeping in mind that if we specified an 
                        !// impossible configuration, this will generate an error (See error handler below)
                        ik_result_j:=CalcJointT(cartesianTarget,currentTool2,\WObj:=currentWobj2, \ErrorNumber:=myerrnum);

                        !// Store our result in a string to return to the user
                        addString:=NumToStr(ik_result_j.robax.rax_1,2)+" ";
                        addString:=addString+NumToStr(ik_result_j.robax.rax_2,2)+" ";
                        addString:=addString+NumToStr(ik_result_j.robax.rax_3,2)+" ";
                        addString:=addString+NumToStr(ik_result_j.robax.rax_4,2)+" ";
                        addString:=addString+NumToStr(ik_result_j.robax.rax_5,2)+" ";
                        addString:=addString+NumToStr(ik_result_j.robax.rax_6,2)+" ";
                        addString:=addString+NumToStr(ik_result_j.extax.eax_a,2)+" ";
                        addString:=addString+NumToStr(myerrnum,2);
                        !End of string
                    ENDIF
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

            CASE 13:
                ! Forward Kinematics Solver
                IF nParams=7 THEN
                    ok:=SERVER_OK;

                    !// First, let's make sure the specified joint angles are within range
                    FOR i FROM 1 TO 7 DO
                        IF params{i}>upper_joint_limits{i}OR params{i}<lower_joint_limits{i} THEN
                            !// If not, then we'll tell the user that their forward kinematics are invalid
                            ok:=SERVER_BAD_FK;
                        ENDIF
                    ENDFOR

                    !// If our joints are within limits, then let's carry on
                    IF ok=SERVER_OK THEN
                        !// Create a joint target, and then calculate the corresponding cartesian pose
                        jointsTarget:=[[params{1},params{2},params{3},params{4},params{5},params{6}],
                                [params{7},9E9,9E9,9E9,9E9,9E9]];
                        fk_result_c:=CalcRobT(jointsTarget,currentTool2,\WObj:=currentWobj2);

                        !// Now add this pose to our return string
                        addString:=NumToStr(fk_result_c.trans.x,2)+" ";
                        addString:=addString+NumToStr(fk_result_c.trans.y,2)+" ";
                        addString:=addString+NumToStr(fk_result_c.trans.z,2)+" ";
                        addString:=addString+NumToStr(fk_result_c.rot.q1,4)+" ";
                        addString:=addString+NumToStr(fk_result_c.rot.q2,4)+" ";
                        addString:=addString+NumToStr(fk_result_c.rot.q3,4)+" ";
                        addString:=addString+NumToStr(fk_result_c.rot.q4,4);
                        !End of string
                    ENDIF
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
            CASE 15:
                !Specify Max acceleration
                IF nParams=2 THEN
                    PathAccLim TRUE\AccMax:=params{1},TRUE\DecelMax:=params{2};
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
            CASE 16:
                !Hand Jog In
                IF nParams=0 THEN
                    Hand_JogInward;
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
            CASE 17:
                !Hand Jog Out
                IF nParams=0 THEN
                    Hand_JogOutward;
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
            CASE 18:
                !Hand Move To
                IF nParams=1 THEN
                    Hand_MoveTo params{1};
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
            CASE 19:
                !Hand Calibrate
                IF nParams=0 THEN
                    Hand_DoCalibrate;
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
            CASE 20:
                !Hand Stop
                IF nParams=0 THEN
                    Hand_Stop;
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
            CASE 21:
                !Set max Speed
                IF nParams=1 THEN
                    Hand_SetMaxSpeed params{1};
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
            CASE 22:
                !Set gripping force
                IF nParams=1 THEN
                    Hand_SetHoldForce params{1};
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
            CASE 23:
                !Check if gripper is calibrated
                IF nParams=0 THEN
                    isHandCalibrated := Hand_IsCalibrated();
                    IF isHandCalibrated THEN
                        addString:=NumToStr(1,2);
                    ELSE
                        addString:=NumToStr(0,2);
                    ENDIF
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
            CASE 24:
                !Get current gripper position
                IF nParams=0 THEN
                    handPosition := Hand_GetActualPos();
                    addString:=NumToStr(handPosition,2);
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
            CASE 25:
                !Hand Grip Inward
                IF nParams=1 THEN
                    Hand_GripInward \holdForce:=params{1};
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
            CASE 26:
                !Hand Grip Outward
                IF nParams=1 THEN
                    Hand_GripOutward \holdForce:=params{1};
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                
            CASE 30:
                !Add Cartesian Coordinates to buffer
                IF nParams=7 THEN
                    cartesianTarget:=[[params{1},params{2},params{3}],
                                    [params{4},params{5},params{6},params{7}],
                                    [0,0,0,0],
                                    externalAxis];
                    IF BUFFER_POS<MAX_BUFFER THEN
                        BUFFER_POS:=BUFFER_POS+1;
                        bufferTargets{BUFFER_POS}:=cartesianTarget;
                        bufferSpeeds{BUFFER_POS}:=currentSpeed2;
                    ENDIF
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

            CASE 31:
                !Clear Cartesian Buffer
                IF nParams=0 THEN
                    BUFFER_POS:=0;
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

            CASE 32:
                !Get Buffer Size
                IF nParams=0 THEN
                    addString:=NumToStr(BUFFER_POS,2);
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

            CASE 33:
                !Execute moves in cartesianBuffer as linear moves
                IF nParams=0 THEN
                    FOR i FROM 1 TO (BUFFER_POS) DO
                        IF collision=0 THEN
                            moveComplete:=FALSE;
                            MoveL bufferTargets{i},bufferSpeeds{i},currentZone2,currentTool2\WObj:=currentWobj2;
                            moveComplete:=TRUE;
                        ENDIF
                    ENDFOR
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

            CASE 35:
                !Specify circPoint for circular move, and then wait on toPoint
                IF nParams=7 THEN
                    circPoint:=[[params{1},params{2},params{3}],
                            [params{4},params{5},params{6},params{7}],
                            [0,0,0,0],
                            externalAxis];
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

            CASE 36:
                !specify toPoint, and use circPoint specified previously
                IF nParams=7 THEN
                    cartesianTarget:=[[params{1},params{2},params{3}],
                                    [params{4},params{5},params{6},params{7}],
                                    [0,0,0,0],
                                    externalAxis];
                    MoveC circPoint,cartesianTarget,currentSpeed2,currentZone2,currentTool2\WObj:=currentWobj2;
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

            CASE 37:
                !Add Joint Positions to buffer
                IF nParams=7 THEN
                    jointsTarget:=[[params{1},params{2},params{3},params{4},params{5},params{6}],
                            [params{7},9E9,9E9,9E9,9E9,9E9]];
                    IF BUFFER_JOINT_POS<MAX_BUFFER THEN
                        BUFFER_JOINT_POS:=BUFFER_JOINT_POS+1;
                        bufferJointPos{BUFFER_JOINT_POS}:=jointsTarget;
                        bufferJointSpeeds{BUFFER_JOINT_POS}:=currentSpeed2;
                    ENDIF
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

            CASE 38:
                !Clear Joint Position Buffer
                IF nParams=0 THEN
                    BUFFER_JOINT_POS:=0;
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

            CASE 39:
                !Get Joint Position Buffer Size
                IF nParams=0 THEN
                    addString:=NumToStr(BUFFER_JOINT_POS,2);
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

            CASE 40:
                !Execute moves in bufferJointPos
                IF nParams=0 THEN
                    moveComplete:=FALSE;
                    !Trapezoidal velocity
                    bufferJointSpeeds{1}.v_tcp:=bufferJointSpeeds{1}.v_tcp*0.50;
                    bufferJointSpeeds{1}.v_ori:=bufferJointSpeeds{1}.v_ori*0.50;
                    bufferJointSpeeds{2}.v_tcp:=bufferJointSpeeds{2}.v_tcp*0.75;
                    bufferJointSpeeds{2}.v_ori:=bufferJointSpeeds{2}.v_ori*0.75;
                    bufferJointSpeeds{BUFFER_JOINT_POS-1}.v_tcp:=bufferJointSpeeds{BUFFER_JOINT_POS-1}.v_tcp*0.75;
                    bufferJointSpeeds{BUFFER_JOINT_POS-1}.v_ori:=bufferJointSpeeds{BUFFER_JOINT_POS-1}.v_ori*0.75;
                    bufferJointSpeeds{BUFFER_JOINT_POS}.v_tcp:=bufferJointSpeeds{BUFFER_JOINT_POS}.v_tcp*0.50;
                    bufferJointSpeeds{BUFFER_JOINT_POS}.v_ori:=bufferJointSpeeds{BUFFER_JOINT_POS}.v_ori*0.50;
                    !Trapezoidal velocity

                    FOR i FROM 1 TO (BUFFER_JOINT_POS) DO
                        IF collision=0 THEN
                            IF i=BUFFER_JOINT_POS THEN
                                moveComplete:=FALSE;
                                MoveAbsJ bufferJointPos{i},bufferJointSpeeds{i},currentZone2,currentTool2,\Wobj:=currentWobj2;
                                moveComplete:=TRUE;
                            ELSE
                                moveComplete:=FALSE;
                                MoveAbsJ bufferJointPos{i},bufferJointSpeeds{i},z1,currentTool2,\Wobj:=currentWobj2;
                                moveComplete:=TRUE;
                            ENDIF
                        ENDIF
                    ENDFOR
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !End the added section for buffer.
            CASE 41:
                !Hand Turn on Blow 1
                IF nParams=0 THEN
                    Hand_TurnOnBlow1;
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
            CASE 42:
                !Hand Turn off Blow 1
                IF nParams=0 THEN
                    Hand_TurnOffBlow1;
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
            CASE 43:
                !Hand Turn on Vacuum 1
                IF nParams=0 THEN
                    Hand_TurnOnVacuum1;
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
            CASE 44:
                !Hand Turn off Vacuum 1
                IF nParams=0 THEN
                    Hand_TurnOffVacuum1;
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
            CASE 45:
                !Get vacuum pressure
                IF nParams=0 THEN
                    handPosition := Hand_GetVacuumPressure1();
                    addString:=NumToStr(handPosition,2);
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

            CASE 99:
                !Close Connection
                IF nParams=0 THEN
                    TPWrite "SERVER: Client has closed connection.";
                    connected:=FALSE;
                    !Closing the server
                    SocketClose clientSocket;
                    SocketClose serverSocket;

                    !Reinitiate the server
                    ServerCreateAndConnect ipController2,serverPort2;
                    connected:=TRUE;
                    reconnected:=TRUE;
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
            DEFAULT:
                TPWrite "SERVER: Illegal instruction code";
                ok:=SERVER_BAD_MSG;
            ENDTEST

            !Finally we compose the acknowledge string to send back to the client
            abort:
            IF connected=TRUE THEN
                IF reconnected=FALSE THEN
                    IF collision=1 THEN
                        ok:=SERVER_COLLISION;
                        TPWrite "Sending message back with collision";
                    ENDIF
                    sendString:=NumToStr(instructionCode,0);
                    sendString:=sendString+" "+idCode;
                    sendString:=sendString+" "+NumToStr(ok,0);
                    sendString:=sendString+" "+NumToStr(ClkRead(timer),2);
                    sendString:=sendString+" "+addString+ByteToStr(10\Char);
                    SocketSend clientSocket\Str:=sendString;
                ENDIF
            ENDIF
        ENDWHILE
    ERROR (LONG_JMP_ALL_ERR)
        TPWrite "SERVER: ------";
        TPWrite "SERVER: Error Handler:"+NumtoStr(ERRNO,0);
        TEST ERRNO
        CASE ERR_MOTIONSUP:
            !TPWrite "SERVER: Moton suppervision error.";
            !//Stop the robot motion
            !StopMove;

            !//Clear the current path from any residual motions in the path queue.
            !ClearPath;

            !//Just in case, set the target pose of the object to current location
            !//When we retry the execution of the program, it will do a MoveL instruction to that target.
            cartesianTarget:=CRobT(\Tool:=currentTool2\WObj:=currentWobj2);
            jointsTarget:=CJointT();

            !//Enable the motion of the robot 
            StartMove;

            !//Retry execution of the program.
            !RETRY;
            TRYNEXT;
            !May be we should do here TRYNEXT?

        CASE ERR_SOCK_CLOSED:
            TPWrite "SERVER: Client has closed connection.";
            TPWrite "SERVER: Closing socket and restarting.";
            TPWrite "SERVER: ------";
            connected:=FALSE;

            !//Closing the server
            SocketClose clientSocket;
            SocketClose serverSocket;

            !//Reinitiate the server
            ServerCreateAndConnect ipController2,serverPort2;
            reconnected:=TRUE;
            connected:=TRUE;
            RETRY;

        CASE ERR_ROBLIMIT:
            !// Out of reach cartesian target.
            ok:=SERVER_BAD_IK;
            ik_result_j:=[[0,0,0,0,0,0],[0,9E9,9E9,9E9,9E9,9E9]];

            !// Skip the instruction computing the IK that caused the error
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
            ServerCreateAndConnect ipController2,serverPort2;
            reconnected:=TRUE;
            connected:=TRUE;
            RETRY;
        ENDTEST
    ENDPROC

    TRAP resetMotion
        !//Routine triggered when the digital output USER_RESET_MOTION is set to 1.
        !//The trigger is raised by the INIT task when a motion suppervision interrrupt is triggered.
        !//It signals the need to restart the motion of the robot.

        !//Note that the motion encoutered a collision
        !//This will show up in the message reply. 
        ok:=SERVER_COLLISION;
        collision:=1;

        IF moveComplete=TRUE THEN
            !Stop the robot motion
            StopMove;

            !//If the move instruction is complete, we do not need to raise ERR_MOTIONSUP 
            !//to restart the motion of the robot. 
            TPWrite "SERVER: ------";
            TPWrite "SERVER: Motion suppervision error happened after";
            TPWrite "SERVER: move instruction was completed.";

            !We clear the current path, to remove any residual motions in the path queue.
            ClearPath;

            !Restart robot motion execution.
            StartMove;

            TPWrite "SERVER: Recovered.";
            TPWrite "SERVER: ------";
            !//Prepare system for the next time there is a collision
            !SetDO USER_START_OUTPUT,0;
            !SetDO USER_RESET_MOTION,0;

        ELSE
            !Stop the robot motion
            StopMove;

            TPWrite "SERVER: ------";
            TPWrite "SERVER: Motion suppervision error happened before";
            TPWrite "SERVER: move instruction was completed.";

            !We clear the current path, to remove any residual motions in the path queue.
            ClearPath;

            TPWrite "SERVER: Recovered.";
            TPWrite "SERVER: ------";

            !//We signal the restart of the robot motion by raising the ERR_MOTIONSUP error.
            !//It will be handled by the error handler in the main procedure.
            !//Prepare system for the next time there is a collision
            !SetDO USER_START_OUTPUT,0;
            !SetDO USER_RESET_MOTION,0;
            RAISE ERR_MOTIONSUP;
        ENDIF


    ERROR
        RAISE ;
    ENDTRAP
ENDMODULE
