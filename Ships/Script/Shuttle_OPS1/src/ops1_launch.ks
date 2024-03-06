@LAZYGLOBAL OFF.
GLOBAL quit_program IS FALSE.

function launch{
	CLEARSCREEN.
	SET CONFIG:IPU TO 800.					//	Required to run the script fast enough.
	
	
	
	//	Load libraries
	RUNPATH("0:/Shuttle_OPS3/landing_sites").
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
	
	LOCAL dap IS ascent_dap_factory().
	
	wait until ship:unpacked and ship:loaded.
	
	make_main_ascent_gui().
	make_ascent_traj1_disp().
	
	initialise_shuttle().
	prepare_launch().	
	
	//need to have initalised the vehicle first for the vessel name
	prepare_telemetry().
	
	ascent_gui_set_cutv_indicator(target_orbit["velocity"]).
	
	GLOBAL dataviz_executor IS loop_executor_factory(
												0.15,
												{
													dap:steer_css().
													dap:thr_control_css().
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

	SAS OFF.
	
	warp_window(target_orbit["warp_dt"]).	
	
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
		if (quit_program) {
			RETURN FALSE.
		}
		IF (monitor_rsls) {
			LOCAL abort_detect IS measure_update_engines().
	
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

	addGUIMessage("BOOSTER IGNITION").
	stage.
	
	set dap:steer_refv to HEADING(control["launch_az"] + 180, 0):VECTOR.	
	LOCK STEERING TO dap:steer_dir.
	LOCK THROTTLE to dap:thr_cmd.
	
	wait 0.
	when (SHIP:VERTICALSPEED > 1) THEN {
		addGUIMessage("LIFT-OFF CONFIRMED").
	}
	
	RETURN TRUE.
	
}



declare function open_loop_ascent{

	SET STEERINGMANAGER:ROLLCONTROLANGLERANGE TO 180.
	set_steering_med().
	
	getState().
		
	local steer_flag IS false.
																			   
	
	getState().
	
	WHEN SHIP:VERTICALSPEED >= (vehicle["pitch_v0"] - 3) THEN {
		addGUIMessage("ROLL PROGRAM").	
		SET steer_flag TO true.
		
		set vehiclestate["phase"] TO 1.
		
		//reset throttle to maximum
		SET vehicle["stages"][1]["Throttle"] TO  vehicle["maxThrottle"].
		
		WHEN SHIP:VERTICALSPEED >= 100 AND ABS(get_roll_lvlh() - control["roll_angle"]) < 7 THEN {
			addGUIMessage("ROLL PROGRAM COMPLETE").
			set_steering_low().
		}
		
		WHEN vehiclestate["staging_in_progress"] THEN {
			SET steer_flag TO FALSE.
		}
	}
	
	UNTIL FALSE {	
		if (quit_program) {
			RETURN FALSE.
		}
	
		monitor_abort().
		getState().
		srb_staging().
		
		LOCAL tt IS TIME:SECONDS.
		
		IF (tt - vehicle["ign_t"] >= vehicle["handover"]["time"] ) {BREAK.}
		
		local aimVec is HEADING(control["launch_az"],open_loop_pitch(SHIP:VELOCITY:SURFACE:MAG)):VECTOR.
		
		IF steer_flag {
			SET control["aimvec"] TO aimVec.
			SET control["steerdir"] TO steeringControl().
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
		
	SET target_orbit["normal"] TO upfg_normal(target_orbit["inclination"], target_orbit["LAN"]).
	
	SET control["refvec"] TO -SHIP:ORBIT:BODY:POSITION:NORMALIZED.
	
	SET upfgInternal TO setupUPFG().
	
	SET vehiclestate["phase"] TO 2.
	
	addGUIMessage("RUNNING UPFG ALGORITHM").

	UNTIL FALSE{
		if (quit_program) {
			RETURN FALSE.
		}
	
		IF (upfgInternal["itercount"] = 0) { //detects first pass or convergence lost
			WHEN (upfgInternal["s_conv"]) THEN {
				addGUIMessage("GUIDANCE CONVERGED IN " + upfgInternal["itercount"] + " ITERATIONS").
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
		
			IF ( RTLSAbort["flyback_flag"] AND (upfgInternal["tgo"] < 60) AND ( (upfgInternal["s_conv"] AND upfgInternal["tgo"] < upfgInternal["terminal_time"]) OR SHIP:VELOCITY:SURFACE:MAG >= 0.9*tgtsurfvel ) ) {
				addGUIMessage("POWERED PITCH-DOWN").
				BREAK.
			}
		} ELSE {
			IF (upfgInternal["s_conv"] AND (upfgInternal["tgo"] < upfgInternal["terminal_time"] AND SHIP:VELOCITY:ORBIT:MAG>= 0.9*target_orbit["velocity"])) OR (SHIP:VELOCITY:ORBIT:MAG>= 0.995*target_orbit["velocity"]) {
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
		
		upfg_sense_current_state(upfgInternal).
		
		IF (DEFINED RTLSAbort) {
			RTLS_burnout_mass().
			SET upfgInternal["mbod"] TO vehicle["mbod"].
			
			IF (NOT RTLSAbort["flyback_flag"]) {
			
				LOCAL stg IS get_stage().
			
				//extrapolate state to the end of ppa
				SET upfgInternal["t_cur"] TO upfgInternal["t_cur"] - RTLSAbort["pitcharound"]["dt"].
				SET upfgInternal["r_cur"] TO upfgInternal["r_cur"] + upfgInternal["v_cur"] * RTLSAbort["pitcharound"]["dt"].
				SET upfgInternal["m_cur"] TO upfgInternal["m_cur"] - stg["engines"]["flow"] * RTLSAbort["pitcharound"]["dt"].
				SET upfgInternal["tb_cur"] TO upfgInternal["tb_cur"] - RTLSAbort["pitcharound"]["dt"].
			
			}
		}
			
			
		upfg(
			vehicle["stages"]:SUBLIST(vehiclestate["cur_stg"],vehicle:LENGTH-vehiclestate["cur_stg"]),
			target_orbit,
			upfgInternal
		).

		//for debugging
		//clearvecdraws().
		//arrow_ship(vecyz(upfgInternal["steering"]),"steer").
		//arrow_ship(vecyz(upfgInternal["ix"]),"ix").
		//arrow_ship(vecyz(upfgInternal["iy"]),"iy").
		//arrow_ship(vecyz(upfgInternal["iz"]),"iz").
		//arrow_body(vecyz(upfgInternal["rd"]),"rd").
		
		//fuel dissipation and flyback trigger logic
		IF (DEFINED RTLSAbort) {

			LOCAL PEG_Tc IS (upfgInternal["mbo_T"] - upfgInternal["tgo"]).
		
			IF (NOT (RTLSAbort["flyback_flag"] AND RTLSAbort["pitcharound"]["complete"] )) {
			
				//force unconverged until flyback
				SET upfgInternal["s_conv"] TO FALSE.
				SET upfgInternal["iter_conv"] TO 0.
				//itercount must be reset so we don't end up with a huge iterations count at PPA
				//BUT DON'T RESET IT TO ZERO BC AT EVERY LOOP A WHEN CHECK WILL BE ADDED!
				SET upfgInternal["itercount"] TO 1.
			
				LOCAL RTLS_steering IS V(0,0,0).
				
				IF ( NOT RTLSAbort["pitcharound"]["triggered"] ) {
					//dissipation 
					SET RTLSAbort["C1"] TO RTLS_C1(RTLSAbort["theta_C1"]).
					SET RTLS_steering TO RTLSAbort["C1"]:NORMALIZED.
					
					//range lockout bc lose to the site guidance is unreliable 
					LOCAL range_now IS greatcircledist(RTLS_tgt_site_vector(), orbitstate["radius"]).
					
					IF (PEG_Tc > RTLSAbort["Tc"]) OR (range_now < RTLSAbort["flyback_range_lockout"]) {
						SET RTLSAbort["flyback_conv"] TO RTLSAbort["flyback_iter"].
					} ELSE {
						SET RTLSAbort["flyback_conv"] TO MIN( 1, RTLSAbort["flyback_conv"] + 1).
					}
					
					IF (RTLSAbort["flyback_conv"] = 1) {
						SET RTLSAbort["pitcharound"]["target"] TO upfgInternal["steering"]. 
					}
					
					SET RTLSAbort["pitcharound"]["dt"] TO RTLS_pitchover_t(RTLSAbort["C1"], RTLSAbort["pitcharound"]["target"]).
					
					IF (RTLSAbort["flyback_conv"] = 1) {
						LOCAL pitchover_bias IS 0.5 * RTLSAbort["pitcharound"]["dt"].
						
						IF (PEG_Tc <= (1 + pitchover_bias)) {
							addGUIMessage("POWERED PITCH-AROUND TRIGGERED").
							SET RTLSAbort["pitcharound"]["refvec"] TO - VCRS(orbitstate["radius"], RTLSAbort["C1"]).
							SET RTLSAbort["pitcharound"]["target"] TO VXCL(RTLSAbort["pitcharound"]["refvec"], RTLSAbort["pitcharound"]["target"]).
							SET RTLSAbort["pitcharound"]["triggered"] TO TRUE.
							SET RTLSAbort["pitcharound"]["complete"] TO FALSE.
							//precaution for convergence display
							SET RTLSAbort["flyback_conv"] TO RTLSAbort["flyback_iter"].
						} 
					}
					
				} ELSE {
					//powered pitcharound					
					//get the current thrust vector, project in the plane containing the peg vector (flyback guidance command) and C1,
					//rotate ahead by a few degrees
					
					set_steering_high().
					
					SET vehicle["roll"] TO 0.
					SET control["roll_angle"] TO 0.
					
					SET control["refvec"] TO VXCL(vecYZ(RTLSAbort["pitcharound"]["refvec"]),SHIP:FACING:TOPVECTOR).
					
					LOCAL thrust_facing IS VXCL(RTLSAbort["pitcharound"]["refvec"],vecYZ(thrust_vec()):NORMALIZED).
					
					SET RTLS_steering TO rodrigues(thrust_facing, RTLSAbort["pitcharound"]["refvec"], 20). 
					
					IF (VANG(thrust_facing, RTLSAbort["pitcharound"]["target"]) < 10) {
						SET RTLS_steering TO RTLSAbort["pitcharound"]["target"].
						set_steering_low().
						SET RTLSAbort["pitcharound"]["complete"] TO TRUE.
						SET RTLSAbort["flyback_flag"] TO TRUE.
						SET upfgInternal["s_flyback"] TO TRUE.
						SET upfgInternal["s_throt"] TO TRUE.
						SET control["refvec"] TO -SHIP:ORBIT:BODY:POSITION:NORMALIZED.
						
					}	
				
				}
				
				SET control["aimvec"] TO vecYZ(RTLS_steering).
				SET control["steerdir"] TO steeringControl().
			}
		
			SET RTLSAbort["Tc"] TO PEG_Tc.
		}
		
		
		IF (upfgInternal["s_conv"] AND NOT vehiclestate["staging_in_progress"]) {
			SET control["aimvec"] TO vecYZ(upfgInternal["steering"]):NORMALIZED.
			SET control["steerdir"] TO steeringControl().
			local stg is get_stage().
			IF stg["mode"] <> 2 {
				//round to nearest percentage
				SET stg["Throttle"] TO upfgInternal["throtset"].//FLOOR(upfgInternal["throtset"], 2).		
			}			
		} 
		
	}
	
	SET vehiclestate["phase"] TO 3.
 
	SET upfgInternal["terminal"] TO TRUE.
	
	
	
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
				SET steervec tO rodrigues(SHIP:FACING:FOREVECTOR, rotvec, 10).
				SET steervec TO VXCL(rotvec,steervec).
			} ELSE {
				SET steervec TO pitchdowntgtvec.
			}
			
			SET control["steerdir"] TO LOOKDIRUP(steervec, upvec).
			
			LOCAL rng IS downrangedist(launchpad,SHIP:GEOPOSITION )*1000.
			LOCAL tgtsurfvel IS RTLS_rvline(rng).
			
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
				SET vehiclestate["phase"] TO 4.
			}
		}
	}
	addGUIMessage("STAND-BY FOR ET SEP").
	
	UNTIL FALSE{
		getState().
		
		IF (vehiclestate["phase"] = 4) {
			BREAK.
		}
		WAIT 0.1.
	}
	
	//to disable RCS separation maneouvre
	SET SHIP:CONTROL:NEUTRALIZE TO TRUE.
	
	close_umbilical().
	disable_TVC().
	
	UNLOCK THROTTLE.
	UNLOCK STEERING.
	SAS ON.
	
	// IF RTLS enter GRTLS loop and exit
	IF (DEFINED RTLSAbort) {
		RETURN TRUE.
	}
	
	print_ascent_report().
	
	UNTIL (AG9 or quit_program). {
		getState().
		WAIT 0.2.
	}
		
	RETURN TRUE.
}


launch().

CLEARSCREEN.
dataviz_executor["stop_execution"]().
close_all_GUIs().



IF (DEFINED RTLSAbort) {
	RUN "0:/ops3".
}