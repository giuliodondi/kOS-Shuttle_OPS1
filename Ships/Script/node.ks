@LAZYGLOBAL OFF.
CLEARSCREEN.
SET TERMINAL:WIDTH TO 55.
SET TERMINAL:HEIGHT TO 20.	

RUNPATH("0:/Libraries/maths_library").
RUNPATH("0:/Libraries/misc_library").
RUNPATH("0:/Libraries/vehicle_library").
RUNPATH("0:/Libraries/aerosim_library").

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
	
	IF (not HASNODE) {
		PRINT "No manoeuvre node,  aborting." .
		RETURN.
	}
	
	
	SAS OFF.
	ON SAS {
		SAS OFF.
		PRESERVE.
	}

	STEERINGMANAGER:RESETPIDS().
	STEERINGMANAGER:RESETTODEFAULT().
	SET STEERINGMANAGER:MAXSTOPPINGTIME TO 1.
	
	LOCAL upvec IS SHIP:FACING:TOPVECTOR.
	LOCAL P_steer IS SHIP:FACING.
	lock steering to P_steer.
	LOCAL rollangle IS 0.

	LOCAL ignitionflag IS FALSE.
	LOCAL quitflag IS FALSE.
	LOCAL abortflag IS FALSE.
	
	LOCAL nxtnode IS nextnode.
	
	local cur_orb_vel is v(0,0,0).
	local ref_normv is v(0,0,0).
	
	local timer is timer_factory().
	
	local dv_accum_sensed is 0.
	local dv_accum_calc is 0.
	local dv_sensed_thresh is 0.001.
	local no_dv_max_t is 5.
	local no_dv_thresh is 0.
	
	local node_vec is v(0,0,0).
	LOCAL nodeDV IS 0.
	LOCAL burnT IS 0.
	LOCAL node_eta IS 0.
	LOCAL ignitionT IS 0.
	LOCAL shutdownT IS 0.
	
	local steervec is SHIP:FACING:FOREVECTOR.
	
	local node_tangent is 0.
	local node_normal is 0.
	local node_binormal is 0.
	
	until false {
		timer:update().
		
		local cur_p is -ship:orbit:body:position.
		local cur_v is SHIP:VELOCITY:orbit.
		local cur_n is vcrs(cur_p, cur_v).
		
		local i_tangent is cur_p:normalized.
		local i_normal is cur_v:normalized.
		local i_binormal is vcrs(i_normal, i_tangent).
		
		local dv_sensed is cur_v - cur_orb_vel.
		set cur_orb_vel to cur_v.
		
		local dv_grav is simple_g(cur_p) * timer:last_dt.
		
		local dv_calc is dv_sensed - dv_grav.
		if (abs(dv_calc:mag) < dv_sensed_thresh) {
			set dv_calc to v(0,0,0).
		}
		
		set dv_accum_sensed to dv_accum_sensed + dv_sensed:mag. 
		set dv_accum_calc to dv_accum_calc + dv_calc:mag. 
		
		IF (NOT ignitionflag) {
			
			set dv_accum_calc to 0.
		
			set nxtnode to nextnode.
			
			set node_vec to nxtnode:deltav:normalized.
			set nodeDV to nxtnode:deltav:MAG.
			set burnT to burnDT(nodeDV).
			set node_eta to nxtnode:ETA.
			
			set node_tangent to vdot(node_vec, i_tangent).
			set node_normal to vdot(node_vec, i_normal).
			set node_binormal to vdot(node_vec, i_binormal).
			
			PRINTPLACE("Node Delta V : " + round(nodeDV, 2) + "m/s",30,0,1).
			PRINTPLACE("Approx. Node Burn T  : " + sectotime(burnT),30,0,2).
			PRINTPLACE("Node ETA : " + sectotime(-node_eta),30,0,4).
			
			if (node_eta < 0.1) {
				SET ignitionflag TO TRUE.
				SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 1.
				set ignitionT to timer:last_sampled_t.
				SET shutdownT TO ignitionT - node_eta + burnT.
				set no_dv_thresh to nodeDV * no_dv_max_t / burnT.
			}
			
			warp_controller(node_eta, FALSE, 25).
		
		} else {
			
			local dv_left is nodeDV - dv_accum_calc.
			
			PRINTPLACE("Delta V sensed : " + round(dv_accum_calc, 1),30,0,4).
			PRINTPLACE("Delta V left : " + round(dv_left, 1),30,0,5).
			PRINTPLACE("Approx. Shutdown  : " + sectotime(shutdownT - timer:last_sampled_t),30,0,6).
			
			if (dv_left <= 0) {
				set quitflag to TRUE.
			}
			
			if (SHIP:CONTROL:PILOTMAINTHROTTLE < 0.01) {
				set abortflag to TRUE.
			}
			
			if ((timer:last_sampled_t > ignitionT + no_dv_max_t) and (dv_accum_calc < no_dv_thresh)) {
			
			}
		}
		
		
		set steervec to node_tangent*i_tangent + node_normal*i_normal + node_binormal*i_binormal.
		
		IF (VANG(P_steer:VECTOR,SHIP:FACING:FOREVECTOR) < 10 ) {
			SET upvec TO rotate_upvec(steervec,upvec).
		} ELSE {
			SET upvec TO SHIP:FACING:TOPVECTOR.
		}
		
		SET P_steer TO aimAndRoll(steervec, upvec , rollangle, FALSE). 
		
		if (abortflag or quitflag) {
			break.
		}
	
		wait 0.
	}
	
	IF (abortflag) {
		PRINTPLACE("Manoeuvre aborted",30,0,10).
	} ELSE {
		PRINTPLACE("Manoeuvre complete",30,0,10).
	}
	
	SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.
	
	PRINTPLACE("Killing rotation...",30,0,8).
	
	LOCK STEERING TO "kill".
	
	WAIT 2.
	
	UNLOCK STEERING.
	
	clearscreen.

}

execute_manoeuvre().