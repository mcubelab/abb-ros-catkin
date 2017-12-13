MODULE INIT
!Task to check when the motion task is stopped after a motion suppervision error.
!In that case the task raises a DO_RESET_ERROR and restarts the tasks with DO_START
!This is a SEMISTATIC Task, so it will start automatically.

!Main procedure
!It calls to the routine that cheks for motion suppervision error
!at at 2 Hz

!//Boolean for task restart synchronization between the different tasks
PERS bool reStartTask;

!//Boolean for resetting INIT
PERS bool resetInit;

PROC main()
    MotionSupReset;
    WaitTime 0.5;    
    !Note that this will only loop when the task is configured to be semistatic.
ENDPROC


!Checking routine 
PROC MotionSupReset()
    IF OpMode()=OP_AUTO AND DOutput(SO_MOTIONSUP_ON)=1 AND DOutput(SO_TROB1_EXEC)=0 AND resetInit=FALSE THEN
        !If:
	! 1- The robot is in automode, and
        ! 2- Motion suppervision was just triggered, and 
        ! 3- Task ROB_1 is stopped
        ! 4- We were not here just before 
        !Then: Restart execution.
        TPWrite "INIT: Motion suppervision captured.";
        WaitTime 2.5;    
        Set DO_RESET_ERROR; !Raise reset error
        WaitTime 2.5;    
        Set DO_START; !Restart tasks
        reStartTask:=TRUE;
        resetInit:=TRUE;
        
    ELSEIF resetInit=TRUE THEN
    !//    If we just raised reset and restart
    !//    Then: Reset error signals and prepare for next error
        TPWrite "INIT: Reseting/preparing for next error.";
        resetInit:=FALSE;
        Reset DO_RESET_ERROR;
        Reset DO_START;
    !//ELSEIF (DOutput(DO_RESET_ERROR)=1 OR DOutput(DO_START)=1) AND DOutput(SO_TROB1_EXEC)=1 THEN
    !//    If:
    !//      1- A reset error was raised or we issued a restart task, and
    !//      3- Task ROB_1 is back executing
    !//    Then: Reset error sigansl
    ENDIF
ENDPROC
ENDMODULE
