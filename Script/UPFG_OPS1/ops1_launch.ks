function launch{
	CLEARSCREEN.
	SET TERMINAL:WIDTH TO 65.
	SET TERMINAL:HEIGHT TO 59.
	//	Settings for UPFG
	SET CONFIG:IPU TO 600.					//	Required to run the script fast enough.
	GLOBAL upfgFinalizationTime IS 5.		//	When time-to-go gets below that, keep attitude stable and simply count down time to cutoff.
	GLOBAL upfgConvergenceTgo IS 0.2.	//	Maximum difference between consecutive UPFG T-go predictions that allow accepting the solution.
	GLOBAL upfgConvergenceVec IS 15.	//	Maximum angle between guidance vectors calculated by UPFG between stages that allow accepting the solution.

	
	//	Load vessel config file
	
	IF (vesselfilename:ENDSWITH(".ks")=TRUE) {
		SET vesselfilename TO vesselfilename:REMOVE( vesselfilename:FIND(".ks") ,3 ).
	}
	RUNPATH("./VESSELS/" + vesselfilename + ".ks").
	
	//	Load libraries
	RUNPATH("0:/Shuttle_entrysim/landing_sites").
	RUNPATH("0:/Libraries/misc_library").	
	RUNPATH("0:/Libraries/maths_library").	
	RUNPATH("0:/Libraries/navigation_library").	
		
		
	RUNPATH("ops1_interface").
	RUNPATH("ops1_vehicle_library").
	RUNPATH("ops1_targeting_library").
	RUNPATH("ops1_upfg_library").
	RUNPATH("ops1_abort_library").
	
	//conic state extrapolation function / gravity integrator
	RUNPATH("upfg__cser_sg_simple").
	
	
	if logdata=TRUE {	
		GLOBAL loglex IS LEXICON(
										"Time",0,
										"Altitude",0,
										"Dwnrg Dst",0,
										"Stage",0,
										"Mass",0,
										"TWR",0,
										"Throt",0,
										"AZ(cmd)",0,
										"HAOA",0,
										"Pitch",0,
										"VAOA",0,
										"Surfvel",0,
										"Orbvel",0,
										"Incl",0,
										"Ecctr",0
		).
		log_data(loglex,"./LOGS/" + vesselfilename + "_log").
	}
	
	wait until ship:unpacked and ship:loaded.
	
	PRINT " INITIALISING GLOBAL VARIABLES" AT (0,1).
	//generic variables
	
	GLOBAL ops_mode IS 0.
	

	//Nav variables
	GLOBAL launchpad IS SHIP:GEOPOSITION.
	
	GLOBAL surfacestate IS  LEXICON("MET",0,"az",0,"pitch",0,"alt",0,"vs",0,"hs",0,"vdir",0,"hdir",0,"q",0).
	GLOBAL orbitstate IS  LEXICON("radius",0,"velocity",0). 
	GLOBAL control Is LEXICON(
		"launch_az",0,
		"steerdir", LOOKDIRUP(SHIP:FACING:FOREVECTOR, SHIP:FACING:TOPVECTOR),
		"refvec", v(0,0,0)
	).
	
	
	initialise_shuttle().
	prepare_launch().	
	countdown().
	open_loop_ascent().
	closed_loop_ascent().
	
}



declare function countdown{


	LOCK THROTTLE to throttleControl().
	SAS OFF.
	local line is 30.
	print " COUNTDOWN:" AT (0,line).
	
	
	
	local TT IS TIME:SECONDS + 10 - vehicle["preburn"].
	WHEN  TIME:SECONDS>=TT  THEN {
			set line TO line + 2.
			print " IGNITION SEQUENCE START." at (0,line).
			stage.	
		}
	from { LOCAL i IS 10.} until i = 0 step{ SET i TO i-1.} do{
		
		set line TO line + 2.
		print " T MINUS "+ i at (0,line).
		wait 1.
	}	
	
	SET surfacestate["MET"] TO TIME:SECONDS. 
	SET vehicle["ign_t"] TO TIME:SECONDS. 
	LOCK STEERING TO control["steerdir"].
	stage.	
	wait until ship:unpacked and ship:loaded.
	
}



declare function open_loop_ascent{
	
	drawUI().
	addMessage("LIFT-OFF!").
	
	SET STEERINGMANAGER:MAXSTOPPINGTIME TO 0.9.
	
	//this sets the pilot throttle command to some value so that it's not zero if the program is aborted
	SET SHIP:CONTROL:PILOTMAINTHROTTLE TO vehicle["stages"][vehiclestate["cur_stg"]]["Throttle"].
	
	
	get_mass_bias().
	getState().

	SET control["steerdir"] TO LOOKDIRUP(-SHIP:ORBIT:BODY:POSITION, SHIP:FACING:TOPVECTOR).
		
	local steer_flag IS false.
	
	SET control["refvec"] TO HEADING(control["launch_az"] + 180, 0):VECTOR.
	LOCAL scale IS MIN(0.2,0.15*( (target_orbit["radius"]:MAG - BODY:RADIUS)/250000 - 1)).																				   
	
	SET ops_mode TO 1.
	getState().
	
	GLOBAL rollrefspeed IS 36.	
	
	WHEN SHIP:VERTICALSPEED >=rollrefspeed THEN {
		addMessage("ROLL PROGRAM").	
		SET steer_flag TO true.
	}
	
	
	
	UNTIL FALSE {	
		getState().
		monitor_abort().
		
		IF (vehicle["handover"]:HASKEY("time")) {
			IF ( (TIME:SECONDS - vehicle["ign_t"]) >= vehicle["handover"]["time"] ) {BREAK.}
		}
		ELSE {
			IF vehiclestate["cur_stg"]=vehicle["handover"]["stage"] {
				vehicle["handover"]:ADD("time", TIME:SECONDS - vehicle["ign_t"] ).
				BREAK.
			}
		}
		
		local aimVec is HEADING(control["launch_az"],pitch(SHIP:VELOCITY:SURFACE:MAG,rollrefspeed,scale)):VECTOR.

		
		IF steer_flag { set control["steerdir"] TO aimAndRoll(aimVec,control["refvec"],vehicle["roll"]). }
		
		dataViz().
		WAIT 0.
	}
	
}


declare function closed_loop_ascent{
	
	
	SET ops_mode TO 2.
	
	drawUI().
	getState().
	SET STEERINGMANAGER:MAXSTOPPINGTIME TO 0.4.
	SET STEERINGMANAGER:ROLLTS TO 40.
	SET STEERINGMANAGER:YAWTS TO 4.
	SET STEERINGMANAGER:YAWPID:KD TO 0.1.
	SET STEERINGMANAGER:ROLLPID:KD TO 0.4.
	
	IF HASTARGET = TRUE AND (TARGET:BODY = SHIP:BODY) {
		//hard-coded time shift of 5 minutes
		SET target_orbit TO tgt_j2_timefor(target_orbit,300).
	}													 
	SET control["refvec"] TO -SHIP:ORBIT:BODY:POSITION:NORMALIZED.
	IF (DEFINED RTLSAbort) {
		RTLS_burnout_mass().
	}
	LOCAL x IS setupUPFG(target_orbit).
	GLOBAL upfgInternal IS x[0].
	GLOBAL usc IS x[1].
	addMessage("RUNNING UPFG ALGORITHM").
	
	SET usc["lastvec"] TO vecYZ(thrust_vec()).
	SET usc["lastthrot"] TO vehicle["stages"][vehiclestate["cur_stg"]]["Throttle"].

	dataViz().

	UNTIL FALSE{
		IF usc["itercount"]=0 { //detects first pass or convergence lost
			WHEN usc["conv"]=1 THEN {
				addMessage("GUIDANCE CONVERGED IN " + usc["itercount"] + " ITERATIONS").
			}
		}														  
		
		IF (usc["terminal"]=TRUE) { 
			IF maxthrust=0 {BREAK.}
		}  
		ELSE {			
			IF (DEFINED RTLSAbort) {
				IF ( RTLSAbort["flyback_flag"] AND ( (usc["conv"]=1 AND upfgInternal["tgo"] < upfgFinalizationTime ) OR ( (usc["conv"]<>1 OR SHIP:VELOCITY:ORBIT:MAG>= 0.9*target_orbit["velocity"]) AND upfgInternal["tgo"] < 60 ) ) ) {
					BREAK.
				}
			} ELSE {
				IF (usc["conv"]=1 AND (upfgInternal["tgo"] < upfgFinalizationTime AND SHIP:VELOCITY:ORBIT:MAG>= 0.9*target_orbit["velocity"])) OR (SHIP:VELOCITY:ORBIT:MAG>= 0.995*target_orbit["velocity"]) {
					BREAK.
				}
			}
			
			//changed this to read the abort triggered flag so it works for all kinds of aborts														 
			IF NOT (abort_modes["triggered"]) {
				IF HASTARGET = TRUE AND (TARGET:BODY = SHIP:BODY) {
					SET target_orbit TO tgt_j2_timefor(target_orbit,upfgInternal["tgo"]).
				}	
			}
			
		}
		
		//abort must be set up before getstate so the stage is reconfigured 
		//andthen adjusted to the current fuel mass
		monitor_abort().
		getState().
			
		SET upfgInternal TO upfg_wrapper(upfgInternal).
		
		IF NOT vehiclestate["staging_in_progress"] { //AND usc["conv"]=1
			SET control["steerdir"] TO aimAndRoll(vecYZ(usc["lastvec"]):NORMALIZED,control["refvec"],vehicle["roll"]).										
		} 
		IF vehicle["stages"][vehiclestate["cur_stg"]]["mode"] <> 2 {
			SET vehicle["stages"][vehiclestate["cur_stg"]]["Throttle"] TO usc["lastthrot"].		
		}
		dataViz().
		WAIT 0.
	}
	SET ops_mode TO 3.
 
	SET usc["terminal"] TO TRUE.
	
	//put RTLS terminal logic in its own block
	IF (DEFINED RTLSAbort) {
		LOCAL pitchdowntgtvec IS SHIP:VELOCITY:SURFACE:NORMALIZED.
		LOCAL upvec IS -SHIP:ORBIT:BODY:POSITION:NORMALIZED.
		
		LOCAL rotvec IS VCRS(-SHIP:ORBIT:BODY:POSITION:NORMALIZED, SHIP:VELOCITY:SURFACE:NORMALIZED):NORMALIZED.								  
		
		addMessage("POWERED PITCH-DOWN").
		
		SET STEERINGMANAGER:MAXSTOPPINGTIME TO 1.2.
		
		SET vehicle["stages"][vehiclestate["cur_stg"]]["Throttle"] TO 0.65.		
		
		LOCAL steervec IS SHIP:FACING:FOREVECTOR.
		
		UNTIL FALSE{
			getState().
			
			IF (VANG(steervec, pitchdowntgtvec) > 10) {
				SET steervec tO rodrigues(SHIP:FACING:FOREVECTOR, rotvec,10).
				SET steervec TO VXCL(rotvec,steervec).
			} ELSE {
				SET steervec TO pitchdowntgtvec.
			}
			LOCK STEERING TO LOOKDIRUP(steervec, upvec).
			
			
			//to update tgo and vgo figures, not really critical
			SET upfgInternal TO upfg_wrapper(upfgInternal).
			
			LOCAL cur_horV IS VXCL(upvec,SHIP:VELOCITY:ORBIT):MAG.
			
			IF (SHIP:VELOCITY:ORBIT:MAG >= target_orbit["velocity"] OR SSME_flameout()) {
				LOCK STEERING TO "kill".
				LOCK THROTTLE to 0.
				SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.
				LIST ENGINES IN Eng.
				FOR E IN Eng {IF e:ISTYPE("engine") {E:SHUTDOWN.}}
				BREAK.
			}
			
			
			dataViz().
			WAIT 0.
		}
	} ELSE {
		
		addMessage("WAITING FOR MECO").
	
		UNTIL FALSE {
			getState().
			IF (SHIP:VELOCITY:ORBIT:MAG >= target_orbit["velocity"] OR SSME_flameout()) {
				LOCK STEERING TO "kill".
				LOCK THROTTLE to 0.
				SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.
				LIST ENGINES IN Eng.
				FOR E IN Eng {IF e:ISTYPE("engine") {E:SHUTDOWN.}}
				BREAK.
			}
			dataViz().
			WAIT 0.
		}
	
	}
	
	//just in case it hasn't been done previously
	LOCK THROTTLE to 0.
	SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.
	SET control["steerdir"] TO SHIP:FACING.
	LOCK STEERING TO control["steerdir"].
	LIST ENGINES IN Eng.
	FOR E IN Eng {IF e:ISTYPE("engine") {E:SHUTDOWN.}}
	UNLOCK THROTTLE.
	
	SAS ON.
	RCS ON.
	
	SET vehiclestate["staging_in_progress"] TO TRUE.	//so that vehicle perf calculations are skipped in getState
	
	
	//ET sep loop
	LOCAL etsep_t IS TIME:SECONDS.
	WHEN ( TIME:SECONDS > etsep_t + 1.5) THEN {
		SET SHIP:CONTROL:TOP TO 1.
		
		WHEN ( TIME:SECONDS > etsep_t + 2.5) THEN {
			STAGE.
			
			WHEN ( TIME:SECONDS > etsep_t + 15) THEN {
				SET SHIP:CONTROL:NEUTRALIZE TO TRUE.
				UNLOCK THROTTLE.
				UNLOCK STEERING.
				close_umbilical().
				disable_TVC().
				SET ops_mode TO 4.
			}
		}
	}
	addMessage("STAND-BY FOR ET SEP").
	UNTIL FALSE{
		getState().
		
		IF (ops_mode = 4) {
			BREAK.
		}
	
		dataViz().
		WAIT 0.
	}
	
	// IF RTLS enter GRTLS loop and exit
	IF (DEFINED RTLSAbort) {
		LIST ENGINES IN Eng.
		FOR E IN Eng {IF e:ISTYPE("engine") {E:SHUTDOWN.}}
		GRTLS().
		RETURN.
	}
	
	UNLOCK THROTTLE.
	UNLOCK STEERING.
	SET SHIP:CONTROL:NEUTRALIZE TO TRUE.
	
	
	
	drawUI().
	UNTIL AG9 {
		getState().
		dataViz().
		WAIT 0.
	}
		
}


launch().