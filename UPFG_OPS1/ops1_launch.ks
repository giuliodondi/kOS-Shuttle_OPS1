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
	GLOBAL staginginprogress IS FALSE.
	

	//Nav variables
	GLOBAL launchpad IS SHIP:GEOPOSITION.
	GLOBAL P_refVec IS v(0,0,0).
	GLOBAL P_steer IS LOOKDIRUP(SHIP:FACING:FOREVECTOR, SHIP:FACING:TOPVECTOR).	
	GLOBAL attitude IS LIST(90,0).
	GLOBAL surfacestate IS  LEXICON("MET",0,"az",0,"pitch",0,"alt",0,"vs",0,"hs",0,"vdir",0,"hdir",0,"q",0).
	GLOBAL orbitstate IS  LEXICON("radius",0,"velocity",0). 
	

	
	
	
	initialise_vehicle().
	prepare_launch().
	
	//IF (DEFINED TALAbort) {
	//	UNSET TALAbort.
	//}
	//SET orbitstate["radius"] TO rodrigues(vecYZ(-SHIP:ORBIT:BODY:POSITION:NORMALIZED)*(BODY:RADIUS + 118000),-target_orbit["normal"],5).
	//
	////TAL_tgt_vec(orbitstate["radius"]).
	//
	//setup_TAL().
	
	countdown().
	open_loop_ascent().
	closed_loop_ascent().
	
}



declare function countdown{


	
	LOCK STEERING TO P_steer.
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
	stage.	
	wait until ship:unpacked and ship:loaded.
}



declare function open_loop_ascent{
	
	drawUI().
	addMessage("LIFT-OFF!").
	
	//this sets the pilot throttle command to some value so that it's not zero if the program is aborted
	SET SHIP:CONTROL:PILOTMAINTHROTTLE TO vehicle["stages"][j]["Throttle"].
	
	
	get_mass_bias().
	getState().

	
	//until false {print vehicle["stages"].}
		
	IF (vehicle["handover"]:HASKEY("time")) {
		SET vehicle["handover"]["time"] to vehicle["ign_t"] + vehicle["handover"]["time"].
	}
	SET P_steer TO LOOKDIRUP(-SHIP:ORBIT:BODY:POSITION, SHIP:FACING:TOPVECTOR).
		
	local steer_flag IS false.
	
	SET P_refVec TO HEADING(attitude[1] + 180, 0):VECTOR.
	LOCAL scale IS MIN(0.2,0.15*( (target_orbit["radius"]:MAG - BODY:RADIUS)/250000 - 1)).																				   
	
	SET ops_mode TO 1.
	getState().
	
	WHEN ALT:RADAR >=200 THEN {
		addMessage("BEGINNING ROLL PROGRAM").	
		SET steer_flag TO true.
	}
	
	LOCAL targetspeed IS 85 + ((45 - 90)/(1.5-1))*(thrust[0]/(g0*vehicle["stages"][j]["m_initial"]) - 1).	
	local aimVec is HEADING(attitude[1],attitude[0]):VECTOR.
	
	WHEN SHIP:VERTICALSPEED > targetspeed THEN { 
		addMessage("PITCHING DOWNRANGE").
	}
	
	
	
	UNTIL FALSE {	
		getState().
		monitor_abort().
		
		IF (vehicle["handover"]:HASKEY("time")) {
			IF TIME:SECONDS >= (vehicle["handover"]["time"] ) {BREAK.}
		}
		ELSE {
			IF j=vehicle["handover"]["stage"] {
				vehicle["handover"]:ADD("time", TIME:SECONDS ).
				BREAK.
			}
		}

		
		SET attitude[0] TO pitch(SHIP:VELOCITY:SURFACE:MAG,targetspeed,scale).
		
		SET aimVec TO HEADING(attitude[1],attitude[0]):VECTOR.

		
		IF steer_flag { set P_steer TO aimAndRoll(aimVec,P_refVec,vehicle["roll"]). }
		
		dataViz().
		WAIT 0.
	}
	
}


declare function closed_loop_ascent{
	
	
	SET ops_mode TO 2.
	IF HASTARGET = TRUE AND (TARGET:BODY = SHIP:BODY) {
		//hard-coded time shift of 5 minutes
		SET target_orbit TO tgt_j2_timefor(target_orbit,300).
	}													 
	SET P_refVec TO -SHIP:ORBIT:BODY:POSITION:NORMALIZED.
	addMessage("RUNNING UPFG ALGORITHM").
	drawUI().
	getState().
	IF (DEFINED RTLSAbort) {
		RTLS_burnout_mass().
	} ELSE {
		SET target_orbit["normal"] TO targetNormal(target_orbit["inclination"], target_orbit["LAN"]).
	}
	LOCAL x IS setupUPFG(target_orbit).
	GLOBAL upfgInternal IS x[0].
	GLOBAL usc IS x[1].
	
	SET usc["lastvec"] TO vecYZ(thrust_vec()).
	SET usc["lastthrot"] TO vehicle["stages"][j]["Throttle"].

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
				IF ( RTLSAbort["flyback_flag"] AND ( (usc["conv"]=1 AND upfgInternal["tgo"] < upfgFinalizationTime ) OR ( (usc["conv"]<>1 OR SHIP:VELOCITY:ORBIT:MAG>= 0.9*target_orbit["velocity"]) AND upfgInternal["tgo"] < 40 ) ) ) {
					SET ops_mode TO 3.
					SET usc["terminal"] TO TRUE.
					BREAK.
				}
			} ELSE {
				IF (usc["conv"]=1 AND (upfgInternal["tgo"] < upfgFinalizationTime AND SHIP:VELOCITY:ORBIT:MAG>= 0.9*target_orbit["velocity"])) OR (SHIP:VELOCITY:ORBIT:MAG>= 0.995*target_orbit["velocity"]) {
					IF ops_mode=2 {
						SET ops_mode TO 3.
						SET usc["terminal"] TO TRUE.
						addMessage("WAITING FOR MECO").
						WHEN SHIP:VELOCITY:ORBIT:MAG >= target_orbit["velocity"] THEN {
							LOCK STEERING TO "kill".
							LOCK THROTTLE to 0.
							SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.
							LIST ENGINES IN Eng.
							FOR E IN Eng {IF e:ISTYPE("engine") {E:SHUTDOWN.}}
						}
					}
				}
			}
			
			//changed this to read the abort triggered flag so it works for all kinds of aborts														 
			IF NOT (abort_modes["triggered"]) {
				IF HASTARGET = TRUE AND (TARGET:BODY = SHIP:BODY) {
					SET target_orbit TO tgt_j2_timefor(target_orbit,upfgInternal["tgo"]).
				}	
				SET target_orbit["normal"] TO targetNormal(target_orbit["inclination"], target_orbit["LAN"]).
				SET target_orbit["perivec"] TO target_perivec().
			}
			
		}
		
		//abort must be set up before getstate so the stage is reconfigured 
		//andthen adjusted to the current fuel mass
		monitor_abort().
		getState().
		
		//do them here bc the bo mass must be updated the first time only
		//after getstate had upated the stage burn times
		IF (DEFINED RTLSAbort) {
			RTLS_burnout_mass().
		}
			
		SET upfgInternal TO upfg_wrapper(upfgInternal).
		
		IF NOT staginginprogress { //AND usc["conv"]=1
			SET P_steer TO aimAndRoll(vecYZ(usc["lastvec"]):NORMALIZED,P_refVec,vehicle["roll"]).										
		} 
		IF vehicle["stages"][j]["mode"] <> 2 {
			SET vehicle["stages"][j]["Throttle"] TO usc["lastthrot"].		
		}
		dataViz().
		WAIT 0.
	}
	
	
	
	//put RTLS terminal logic in its own block
	IF (DEFINED RTLSAbort) {
		LOCAL pitchdownvec IS SHIP:VELOCITY:SURFACE:NORMALIZED.
		LOCAL upvec IS -SHIP:ORBIT:BODY:POSITION:NORMALIZED.
		
		//LOCAL rightvec IS VCRS(upvec, pitchdownvec ).
		//SET pitchdownvec TO rodrigues(pitchdownvec,rightvec,2).
		//local aimv IS VXCL(rightvec,SHIP:FACING:FOREVECTOR).
		//SET P_steer TO aimAndRoll(pitchdownvec:NORMALIZED,upvec,0).	
		
		LOCK STEERING TO LOOKDIRUP(pitchdownvec, upvec).
		
		SET STEERINGMANAGER:MAXSTOPPINGTIME TO 0.5.
		
		addMessage("POWERED PITCH-DOWN").

		
		UNTIL FALSE{
			getState().
			
			//to update tgo and vgo figures, not really critical
			SET upfgInternal TO upfg_wrapper(upfgInternal).
			
			LOCAL cur_horV IS VXCL(upvec,SHIP:VELOCITY:ORBIT):MAG.
			
			IF (cur_horV >= target_orbit["velocity"]) {
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
	SET P_steer TO SHIP:FACING.
	LOCK STEERING TO P_steer.
	LIST ENGINES IN Eng.
	FOR E IN Eng {IF e:ISTYPE("engine") {E:SHUTDOWN.}}
	UNLOCK THROTTLE.
	
	SAS ON.
	RCS ON.
	
	SET staginginprogress TO TRUE.	//so that vehicle perf calculations are skipped in getState
	
	
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