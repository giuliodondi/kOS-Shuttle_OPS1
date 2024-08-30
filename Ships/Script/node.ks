@LAZYGLOBAL OFF.
CLEARSCREEN.

RUNPATH("0:/Libraries/maths_library").
RUNPATH("0:/Libraries/misc_library").
RUNPATH("0:/Libraries/vehicle_library").

GLOBAL g0 IS 9.80665.


FUNCTION rotate_upvec {
	PARAMETER aimvec.
	PARAMETEr upvec.
	
	LOCAL rollgain IS 0.8.
	
	LOCAL deltaroll IS rollgain*(SHIP:CONTROL:PILOTROLL - SHIP:CONTROL:PILOTROLLTRIM). 
	
	
	IF ABS(deltaroll)>0.1 {
		SET upvec TO rodrigues(upvec,aimvec,-deltaroll).
	}
	
	RETURN upvec.


}


FUNCTION execute_manoeuvre {

	//check engines 
	IF (get_running_engines():LENGTH = 0) {
		PRINT "No active engines,  aborting." .
		RETURN.
	}

	CLEARSCREEN.
	SET TERMINAL:WIDTH TO 55.
	SET TERMINAL:HEIGHT TO 20.	

	SAS OFF.
	ON SAS {
		SAS OFF.
		PRESERVE.
	}
	
	STEERINGMANAGER:RESETPIDS().
	STEERINGMANAGER:RESETTODEFAULT().
	SET STEERINGMANAGER:MAXSTOPPINGTIME TO 1.
	

	LOCAL nxtnode IS nextnode.

	LOCAL nodevec IS nxtnode:deltav:NORMALIZED. 
	LOCAL upvec IS SHIP:FACING:TOPVECTOR.

	LOCAL P_steer IS SHIP:FACING.
	
	lock steering to P_steer.

	LOCAL rollangle IS 0.

	LOCAL ignitionflag IS FALSE.
	LOCAL quitflag IS FALSE.
	
	
	LOCAL shutdownT IS 0.
	
	LOCAL nodeDV IS nxtnode:deltav:MAG.
		
	LOCAL burnT IS  burnDT(nodeDV).
	
	PRINTPLACE("Node Delta V : " + round(nodeDV, 2) + "m/s",30,0,1).
	PRINTPLACE("Approx. Node Burn T  : " + sectotime(burnT),30,0,2).

	LOCAL abortflag IS FALSE.

	UNTIL FALSE {
	
		IF HASNODE {
			set nodevec tO nxtnode:deltav:NORMALIZED. 
		}
		
		IF (VANG(P_steer:VECTOR,SHIP:FACING:FOREVECTOR) < 10 ) {
			SET upvec TO rotate_upvec(nodevec,upvec).
		} ELSE {
			SET upvec TO SHIP:FACING:TOPVECTOR.
		}
		
		SET P_steer TO aimAndRoll(nodevec, upvec , rollangle, FALSE). 
		
		IF ignitionflag AND (quitflag OR abortflag) {BREAK.}
		
		IF (NOT ignitionflag) {
			LOCAL node_eta IS nxtnode:ETA.
			warp_controller(node_eta, FALSE, 25).
			PRINTPLACE("Node ETA : " + sectotime(node_eta),30,0,4).
			
			if (node_eta < 0.1) {
				SET ignitionflag TO TRUE.
				SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 1.
				SET shutdownT TO TIME:SECONDS - node_eta + burnT.
			}
			
		} ELSE {
			PRINTPLACE("Shutdown : " + sectotime(shutdownT - TIME:SECONDS),30,0,4).
			
			if (TIME:SECONDS>=shutdownT) {
				set quitflag to TRUE.
				BREAK.
			}
			
			if (SHIP:CONTROL:PILOTMAINTHROTTLE < 0.01) {
				set abortflag to TRUE.
				BREAK.
			}
		}
		
		wait 0.02.
	
	}
	
	IF (abortflag) {
		PRINTPLACE("Manoeuvre aborted by the pilot",30,0,4).
	} ELSE {
		//wait for node to be gone 
		PRINTPLACE("Checking for completion...",30,0,4).
		UNTIL FALSE {
			local nodelist is ALLNODES.
			IF NOT (nodelist:contains(nxtnode)) {BREAK.}
			wait 0.02.
		}
	
		PRINTPLACE("Manoeuvre complete",30,0,6).
	}
	
	SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.
	
	PRINTPLACE("Killing rotation...",30,0,8).
	
	LOCK STEERING TO "kill".
	
	WAIT 2.
	
	UNLOCK STEERING.
	
	clearscreen.

}

execute_manoeuvre().