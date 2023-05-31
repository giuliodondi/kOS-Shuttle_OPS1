RUNPATH("0:/Libraries/maths_library").
RUNPATH("0:/Libraries/misc_library").

GLOBAL g0 IS 9.80665.

FUNCTION thrustrot {

	PARAMETER ref_fore.
	PARAMETER ref_up.

	LOCAL thrvec IS v(0,0,0).
	local offs is v(0,0,0).
	LOCAL thr is 0.
	
	list ENGINES in all_eng.
	FOR e IN all_eng {
		IF e:ISTYPE("engine") {
			IF e:IGNITION {
				SET thr TO thr + (e:MAXTHRUST).
				//set x to x + 1.
				local vel is -e:POSITION:NORMALIZED*e:MAXTHRUST.
				set offs to offs + vel.
			}
		}
	}	
	set thrvec to (offs/thr):NORMALIZED .
	local ship_fore IS SHIP:FACING:VECTOR:NORMALIZED.
	
	local out is ship_fore - thrvec.
	
	RETURN out.
}




//	Returns a kOS direction for given aim vector, reference up vector and roll angle.
//corrects for thrust offset
FUNCTION aimAndRoll {
	DECLARE PARAMETER aimVec.	//	Expects a vector
	DECLARE PARAMETER upVec.	//	Expects a vector
	DECLARE PARAMETER rollAng.	//	Expects a scalar
	
	
	//clearvecdraws().
	//arrow(upVec,"upvec",v(0,0,0),10,0.05).
	
	SET upVec TO VXCL(aimVec,upVec):NORMALIZED.
	SET upVec TO rodrigues(upVec, aimVec, rollAng).
	
	//print "      " at (2,45).
	//print rollAng at (2,45).
	
	
	//arrow(aimVec,"forevec",v(0,0,0),10,0.05).
	//arrow(upVec,"topvec",v(0,0,0),10,0.05).
	
	LOCAL current_up IS VXCL(aimVec,SHIP:FACING:TOPVECTOR):NORMALIZED.
	//arrow(current_up,"current_up",v(0,0,0),10,0.05).

	SET aimVec TO aimVec + thrustrot(aimVec,upVec).
	

	RETURN LOOKDIRUP(aimVec, upVec).
}


//measures current total engine thrust an isp, as well as theoretical max engine thrust at this altitude

FUNCTION get_thrust_isp {


	LOCAL thr is 0.
	LOCAL iisspp IS 0.		   
	
	list ENGINES in all_eng.
	FOR e IN all_eng {
		IF e:ISTYPE("engine") {
			IF e:IGNITION {
				SET thr TO thr + (e:AVAILABLETHRUST * 1000).
				SET iisspp TO iisspp + e:vacuumisp*(e:AVAILABLETHRUST * 1000).								
			}
		}
	}	
	
	RETURN LIST(thr,iisspp).
}

FUNCTION burnDT {
	PARAMETER dV.
	
	
	LOCAL out IS get_thrust_isp().
	LOCAL iisp IS out[1].
	LOCAL thr IS out[0].
	
	LOCAL vex IS g0*iisp.
	
	LOCAL mdot IS thr/vex.
	
	RETURN (SHIP:MASS*1000/(mdot))*( 1 - CONSTANT:E^(-dV/vex) ).
}


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


FUNCTION loop {
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
	
	PRINTPLACE("Manoeuvre Delta V : " + round(nodeDV, 2) + "m/s",30,0,1).
	PRINTPLACE("Manoeuvre Burn T  : " + sectotime(burnT),30,0,2).
	
	WHEN nxtnode:ETA <0.5 THEN {
		//calculate time of burn
		
		SET ignitionflag TO TRUE.
		SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 1.
		
		SET shutdownT TO TIME:SECONDS +burnT.
		
		WHEN TIME:SECONDS>=shutdownT THEN {
			SET quitflag TO TRUE.
			SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.
		}
	}
	
	LOCAL abortflag IS FALSE.

	UNTIL FALSE {
	
		IF (ignitionflag AND SHIP:CONTROL:PILOTMAINTHROTTLE < 0.01) {
			SET abortflag TO TRUE.
		}
	
		IF HASNODE {
			set nodevec tO nxtnode:deltav:NORMALIZED. 
			//SET nodeDV TO nxtnode:deltav:MAG.
		}
		
		IF (VANG(P_steer:VECTOR,SHIP:FACING:FOREVECTOR) < 10 ) {
			SET upvec TO rotate_upvec(nodevec,upvec).
		} ELSE {
			SET upvec TO SHIP:FACING:TOPVECTOR.
		}
		
		SET P_steer TO aimAndRoll(nodevec, upvec , rollangle). 
		
		IF ignitionflag AND (quitflag OR abortflag) {BREAK.}
		
		IF NOT ignitionflag {
			LOCAL node_eta IS nextnode:ETA.
			warp_controller(node_eta, FALSE, 25).
			PRINTPLACE("Node ETA : " + sectotime(node_eta),30,0,4).
		} ELSE {
			PRINTPLACE("Shutdown : " + sectotime(shutdownT - TIME:SECONDS),30,0,4).
		}
		
		wait 0.1.
	
	}
	
	IF (abortflag) {
		PRINTPLACE("Manoeuvre aborted by the pilot",30,0,4).
	} ELSE {
		PRINTPLACE("Manoeuvre complete",30,0,4).
	}
	
	PRINTPLACE("Killing rotation...",30,0,6).
	
	LOCK STEERING TO "kill".
	
	WAIT 2.
	
	UNLOCK STEERING.
	
	clearscreen.

}




loop().