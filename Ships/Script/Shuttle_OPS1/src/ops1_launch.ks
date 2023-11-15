@LAZYGLOBAL OFF.
GLOBAL quit_program IS FALSE.

function launch{
	CLEARSCREEN.
	SET CONFIG:IPU TO 600.					//	Required to run the script fast enough.
	
	
	
	//	Load libraries
	RUNPATH("0:/Shuttle_entrysim/landing_sites").
	RUNPATH("0:/Libraries/misc_library").	
	RUNPATH("0:/Libraries/maths_library").	
	RUNPATH("0:/Libraries/navigation_library").	
	RUNPATH("0:/Libraries/vehicle_library").	
	RUNPATH("0:/Libraries/aerosim_library").	
	
	
	RUNPATH("0:/Shuttle_OPS1/src/ops1_interface").
	RUNPATH("0:/Shuttle_OPS1/src/ops1_vehicle_library").
	RUNPATH("0:/Shuttle_OPS1/src/ops1_targeting_library").
	RUNPATH("0:/Shuttle_OPS1/src/ops1_upfg_library").
	RUNPATH("0:/Shuttle_OPS1/src/ops1_abort_library").
	RUNPATH("0:/Shuttle_OPS1/src/ops1_gui_library.ks").
	
	wait until ship:unpacked and ship:loaded.
	
	make_main_ascent_gui().
	make_ascent_traj1_disp().
	
	initialise_shuttle().
	prepare_launch().	
	
	//need to have initalised the vehicle first for the vessel name
	prepare_telemetry().
	
	ascent_gui_set_cutv_indicator(target_orbit["velocity"]).
	
	GLOBAL dataviz_executor IS loop_executor_factory(
												0.3,
												{
													dataViz().
												}
	).
	
	IF (NOT countdown()) {
		WAIT 5.
		RETURN.
	}
	if (NOT open_loop_ascent()) {
		RETURN.
	}
	if (NOT closed_loop_ascent()) {
		RETURN.
	}
}




function countdown{


	LOCK THROTTLE to throttleControl().
	SAS OFF.
	
	//this sets the pilot throttle command to some value so that it's not zero if the program is aborted
	SET SHIP:CONTROL:PILOTMAINTHROTTLE TO vehicle["stages"][vehiclestate["cur_stg"]]["Throttle"].
	
	//precaution
	SET vehicle["ign_t"] TO TIME:SECONDS + vehicle_countdown.
	
	addGUIMessage(" T MINUS 10").
	
	local TT IS TIME:SECONDS + 10 - vehicle["preburn"].
	LOCAL monitor_rsls IS FALSE.
	WHEN  TIME:SECONDS>=TT  THEN {
			addGUIMessage("GO FOR MAIN ENGINES START").
			SET monitor_rsls TO TRUE.
			stage.
			WAIT 0.
		}
		
	UNTIL (	TIME:SECONDS >= vehicle["ign_t"] ) {
		IF (monitor_rsls) {
			LOCAL abort_detect IS SSME_out().
	
			IF abort_detect {
				addGUIMessage("RSLS ABORT.").
				shutdown_all_engines().
				LOCK THROTTLE to 0.
				SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.
				UNLOCK STEERING.
				RETURN FALSE.
			}
		}
	}
	
	SET surfacestate["MET"] TO TIME:SECONDS. 
	SET vehicle["ign_t"] TO TIME:SECONDS. 
	LOCK STEERING TO control["steerdir"].
	addGUIMessage("BOOSTER IGNITION").
	stage.
	wait 0.
	when (SHIP:VERTICALSPEED > 1) THEN {
		addGUIMessage("LIFT-OFF CONFIRMED").
	}
	
	RETURN TRUE.
	
}



declare function open_loop_ascent{
	

	SET STEERINGMANAGER:ROLLCONTROLANGLERANGE TO 180.
	set_steering_high().
	
	getState().

	SET control["steerdir"] TO LOOKDIRUP(-SHIP:ORBIT:BODY:POSITION, SHIP:FACING:TOPVECTOR).
		
	local steer_flag IS false.
	
	SET control["refvec"] TO HEADING(control["launch_az"] + 180, 0):VECTOR.																			   
	
	SET vehiclestate["ops_mode"] TO 1.
	
	getState().
	
	WHEN SHIP:VERTICALSPEED >= vehicle["pitch_v0"] THEN {
		addGUIMessage("ROLL PROGRAM").	
		SET steer_flag TO true.
		
		//reset throttle to maximum
		SET vehicle["stages"][1]["Throttle"] TO  vehicle["maxThrottle"].
		
		WHEN SHIP:VERTICALSPEED >= 100 AND ABS(get_roll_lvlh() - control["roll_angle"]) < 7 THEN {
			addGUIMessage("ROLL PROGRAM COMPLETE").
			set_steering_med().
		}
		
		WHEN vehiclestate["staging_in_progress"] THEN {
			SET steer_flag TO FALSE.
		}
	}
	
	UNTIL FALSE {	
		if (quit_program) {
			RETURN FALSE.
		}
	
		getState().
		monitor_abort().
		srb_staging().
		
		LOCAL tt IS TIME:SECONDS.
		
		IF (tt - vehicle["ign_t"] >= vehicle["handover"]["time"] ) {BREAK.}
		
		local aimVec is HEADING(control["launch_az"],open_loop_pitch(SHIP:VELOCITY:SURFACE:MAG)):VECTOR.
		
		IF steer_flag {
			set control["steerdir"] TO aimAndRoll(aimVec, control["refvec"], control["roll_angle"]).
		}
	}
	RETURN TRUE.
	
}


declare function closed_loop_ascent{
	
	getState().
	
	set_steering_low().
	
	IF HASTARGET = TRUE AND (TARGET:BODY = SHIP:BODY) {
		//hard-coded time shift of 5 minutes
		SET target_orbit TO tgt_j2_timefor(target_orbit,300).
	}													 
	SET control["refvec"] TO -SHIP:ORBIT:BODY:POSITION:NORMALIZED.
	
	LOCAL x IS setupUPFG(target_orbit).
	SET upfgInternal TO x[0].
	SET usc TO x[1].
	SET usc["lastvec"] TO vecYZ(thrust_vec()).
	SET usc["lastthrot"] TO vehicle["stages"][vehiclestate["cur_stg"]]["Throttle"].
	
	SET vehiclestate["ops_mode"] TO 2.
	
	addGUIMessage("RUNNING UPFG ALGORITHM").

	UNTIL FALSE{
		if (quit_program) {
			RETURN FALSE.
		}
	
		IF usc["itercount"]=0 { //detects first pass or convergence lost
			WHEN usc["conv"]=1 THEN {
				addGUIMessage("GUIDANCE CONVERGED IN " + usc["itercount"] + " ITERATIONS").
			}
		}				

		//abort must be set up before getstate so the stage is reconfigured 
		//and then adjusted to the current fuel mass
		monitor_abort().
		//move it here so that we have the most accurate time figure for staging checks
		getState().
		
		
		//detect terminal conditions
		
		//see if we're at the last stage and close to flameout 
		//this also takes care of staging during ssme phase
		IF ssme_staging_flameout() {
			addGUIMessage("LOW LEVEL").
			BREAK.
		}
		
		//check for orbital terminal conditions 
		IF (DEFINED RTLSAbort) {
			
			LOCAL tgtsurfvel IS RTLS_rvline(downrangedist(launchpad,SHIP:GEOPOSITION )*1000).
		
			IF ( RTLSAbort["flyback_flag"] AND upfgInternal["tgo"] < 60 AND ( (usc["conv"]=1 AND upfgInternal["tgo"] < upfgFinalizationTime) OR SHIP:VELOCITY:SURFACE:MAG >= 0.9*tgtsurfvel ) ) {
				addGUIMessage("POWERED PITCH-DOWN").
				BREAK.
			}
		} ELSE {
			IF (usc["conv"]=1 AND (upfgInternal["tgo"] < upfgFinalizationTime AND SHIP:VELOCITY:ORBIT:MAG>= 0.9*target_orbit["velocity"])) OR (SHIP:VELOCITY:ORBIT:MAG>= 0.995*target_orbit["velocity"]) {
				addGUIMessage("TERMINAL GUIDANCE").
				BREAK.
			}
		}
		
		//changed this to read the abort triggered flag so it works for all kinds of aborts														 
		IF NOT (abort_modes["triggered"]) {
			IF HASTARGET = TRUE AND (TARGET:BODY = SHIP:BODY) {
				SET target_orbit TO tgt_j2_timefor(target_orbit,upfgInternal["tgo"]).
			}	
		}
			
			
		SET upfgInternal TO upfg_wrapper(upfgInternal).
		
		IF NOT vehiclestate["staging_in_progress"] { //AND usc["conv"]=1
			SET control["steerdir"] TO aimAndRoll(vecYZ(usc["lastvec"]):NORMALIZED, control["refvec"], control["roll_angle"]).									
		} 
		IF vehicle["stages"][vehiclestate["cur_stg"]]["mode"] <> 2 {
			SET vehicle["stages"][vehiclestate["cur_stg"]]["Throttle"] TO usc["lastthrot"].		
		}
	}
	
	SET vehiclestate["ops_mode"] TO 3.
 
	SET usc["terminal"] TO TRUE.
	
	//min throttle for any case
	fix_minimum_throttle(). 
	
	//put RTLS terminal logic in its own block
	IF (DEFINED RTLSAbort) {

		set_steering_high().
		
		LOCAL steervec IS SHIP:FACING:FOREVECTOR.
		
		UNTIL FALSE{
			getState().
			
			LOCAL pitchdowntgtvec IS SHIP:VELOCITY:SURFACE:NORMALIZED.
			LOCAL upvec IS -SHIP:ORBIT:BODY:POSITION:NORMALIZED.
			LOCAL rotvec IS VCRS(-SHIP:ORBIT:BODY:POSITION:NORMALIZED, SHIP:VELOCITY:SURFACE:NORMALIZED):NORMALIZED.		
			
			IF (VANG(steervec, pitchdowntgtvec) > 10) {
				SET steervec tO rodrigues(SHIP:FACING:FOREVECTOR, rotvec,10).
				SET steervec TO VXCL(rotvec,steervec).
			} ELSE {
				SET steervec TO pitchdowntgtvec.
			}
			LOCK STEERING TO LOOKDIRUP(steervec, upvec).
			
			SET target_orbit["range"] TO downrangedist(launchpad,SHIP:GEOPOSITION )*1000.
			LOCAL tgtsurfvel IS RTLS_rvline(target_orbit["range"]).
			
			IF (SHIP:VELOCITY:SURFACE:MAG >= tgtsurfvel OR SSME_flameout()) {
				BREAK.
			}
			
		}
	} ELSE {
		
		addGUIMessage("WAITING FOR MECO").
	
		UNTIL FALSE {
			getState().
			IF (SHIP:VELOCITY:ORBIT:MAG >= target_orbit["velocity"] OR SSME_flameout()) {
				BREAK.
			}
		}
	
	}
	
	SET vehiclestate["staging_in_progress"] TO TRUE.	//so that vehicle perf calculations are skipped in getState
	
	LOCK STEERING TO "kill".
	LOCK THROTTLE to 0.
	SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.
	shutdown_all_engines().
	stop_oms_dump(TRUE).
	RCS ON.
	
	//ET sep loop
	LOCAL etsep_t IS TIME:SECONDS.
	WHEN ( TIME:SECONDS > etsep_t + 1.5) THEN {
		SET SHIP:CONTROL:TOP TO 1.
		
		WHEN ( TIME:SECONDS > etsep_t + 2.5) THEN {
			STAGE.
			
			WHEN ( TIME:SECONDS > etsep_t + 15) THEN {
				SET vehiclestate["ops_mode"] TO 4.
			}
		}
	}
	addGUIMessage("STAND-BY FOR ET SEP").
	UNTIL FALSE{
		getState().
		
		IF (vehiclestate["ops_mode"] = 4) {
			BREAK.
		}
		WAIT 0.1.
	}
	
	//to disable RCS separation maneouvre
	SET SHIP:CONTROL:NEUTRALIZE TO TRUE.
	
	close_umbilical().
	disable_TVC().
	
	// IF RTLS enter GRTLS loop and exit
	IF (DEFINED RTLSAbort) {
		
		shutdown_all_engines().
		
		dataviz_executor["stop_execution"]().
		
		close_all_GUIs().
		
		GRTLS().
		RETURN.
	}
	
	UNLOCK THROTTLE.
	UNLOCK STEERING.
	SAS ON.
	
	SET vehiclestate["staging_in_progress"] TO TRUE.	//so that vehicle perf calculations are skipped in getState
	
	print_ascent_report().
	
	UNTIL (AG9 or quit_program). {
		getState().
		WAIT 0.2.
	}
		
	RETURN TRUE.
}


launch().


dataviz_executor["stop_execution"]().
close_all_GUIs().