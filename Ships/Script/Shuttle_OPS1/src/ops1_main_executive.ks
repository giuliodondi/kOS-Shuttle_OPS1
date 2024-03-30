@LAZYGLOBAL OFF.
CLEARSCREEN.
SET CONFIG:IPU TO 1200.	
GLOBAL quit_program IS FALSE.
global debug_mode is false.


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

function ops1_main_exec {

	close_all_GUIs().
	
	make_main_ascent_gui().
	make_ascent_traj1_disp().
	
	wait until ship:unpacked and ship:loaded.
	
	initialise_shuttle().
	prepare_launch().	
	
	
	//need to have initalised the vehicle first for the vessel name
	prepare_telemetry().
	
	ascent_gui_set_cutv_indicator(target_orbit["velocity"]).
	
	GLOBAL dap IS ascent_dap_factory().
	
	GLOBAL dap_gui_executor IS loop_executor_factory(
												0.15,
												{
													//clearscreen.
													clearvecdraws().
													
													if (is_dap_auto()) {
														dap:steer_auto_thrvec().
														dap:thr_control_auto().
													} else if (is_dap_css()) {
														dap:steer_css().
														dap:thr_control_css().
													}
													
													set get_stage()["Throttle"] to dap:thr_rpl_tgt.
													
													if (debug_mode) {
														
														dap:print_debug(2).
														
														arrow_ship(3 * dap:steer_thrvec,"steer_thrvec").
														arrow_ship(2 * dap:steer_dir:forevector,"forevec").
														arrow_ship(2 * dap:steer_dir:topvector,"topvec").
													
													}
													
													dataViz().
												}
	).
	
	IF (NOT ops1_countdown()) {
		if (NOT quit_program) {
			WAIT 5.
		}
		RETURN.
	}
	
	ops1_first_stage().
	
	if (quit_program) {RETURN.}
	
	setupUPFG().
	
	//so we skip directly to rtls or contingency in case of a first stage abort 
	abort_handler().
	getState().
	
	// check for abort modes, proceed to nominal, rtls or contingency
	
	ops1_second_stage_nominal().
	
	if (quit_program) {RETURN.}
	
	
	//handle sequence for rtls and contingency 
	
	ops1_et_sep().
	
	ops1_termination().
}




function ops1_countdown{

	//needed to update the dap at least once
	wait 0.

	SAS OFF.
	
	//this sets the pilot throttle command to some value so that it's not zero if the program is aborted
	LOCK THROTTLE to dap:thr_cmd.
	
	//precaution
	SET vehicle["ign_t"] TO TIME:SECONDS + vehicle_countdown.
	
	addGUIMessage(" T MINUS 10").
	
	set vehiclestate["major_mode"] TO 101.
	
	set dap:thr_max to vehicle["maxThrottle"].
	set dap:thr_min to vehicle["minThrottle"].
	SET dap:thr_rpl_tgt TO convert_ssme_throt_rpl(1).
	set dap:steer_refv to HEADING(target_orbit["launch_az"] + 180, 0):VECTOR.	
	dap:set_steering_low().
	set dap:steer_roll to vehicle["roll"].
	
	local TT IS TIME:SECONDS + 10 - vehicle["preburn"].
	LOCAL monitor_rsls IS FALSE.
	WHEN  TIME:SECONDS>=TT  THEN {
			addGUIMessage("GO FOR MAIN ENGINES START").
			SET monitor_rsls TO TRUE.
			stage.
			WAIT 0.
		}
		
	UNTIL (	TIME:SECONDS >= vehicle["ign_t"] ) {
		SET SHIP:CONTROL:PILOTMAINTHROTTLE TO dap:thr_cmd.
	
		if (quit_program) {
			RETURN FALSE.
		}
		IF (monitor_rsls) {
			measure_update_engines().
	
			IF (vehicle["ssme_out_detected"]) {
				addGUIMessage("RSLS ABORT.").
				shutdown_all_engines().
				LOCK THROTTLE to 0.
				SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.
				UNLOCK STEERING.
				RETURN FALSE.
			}
		}
	}
	
	
	LOCK STEERING TO dap:steer_dir.
	
	addGUIMessage("BOOSTER IGNITION").
	stage.
	SET surfacestate["time"] TO TIME:SECONDS. 
	SET vehicle["ign_t"] TO TIME:SECONDS. 
	
	until false {
		wait 0.	
		if (SHIP:VERTICALSPEED > 1) {break.}
	}
	
	addGUIMessage("LIFT-OFF CONFIRMED").
	
	set vehiclestate["major_mode"] TO 102.
	
	RETURN TRUE.
	
}



function ops1_first_stage {
		
	local steer_flag IS false.
	local throt_flag IS false.
	local roll_program_flag IS false.
																			   
	getState().
	
	WHEN SHIP:VERTICALSPEED >= (vehicle["roll_v0"]) THEN {
		addGUIMessage("ROLL PROGRAM").	
		SET steer_flag TO true.
		SET throt_flag TO true.
		
		//reset throttle to maximum
		SET dap:thr_rpl_tgt TO vehicle["nominalThrottle"].
	}
	
	UNTIL FALSE {	
		if (quit_program) {
			RETURN.
		}
	
		abort_handler().
		getState().
		
		IF (surfacestate["MET"] >= vehicle["handover"]["time"] ) {BREAK.}
		
		if vehiclestate["staging_in_progress"] {
			SET steer_flag TO FALSE.
		}
		
		local aimVec is HEADING(target_orbit["launch_az"],open_loop_pitch(SHIP:VELOCITY:SURFACE:MAG)):VECTOR.
		
		
		IF steer_flag {		
			dap:set_steering_ramp().
			dap:set_steer_tgt(aimVec).
		}
		
		set dap:thr_max to vehicle["maxThrottle"].
		set dap:thr_min to vehicle["minThrottle"].
		
		// q bucket and throttling logic
		
		if (abort_modes["triggered"]) {
			set dap:thr_rpl_tgt to vehicle["maxThrottle"].
		} else {
			if (throt_flag) {
				if (vehicle["qbucket"]) {
				
					IF (surfacestate["q"] >=  surfacestate["maxq"] ) {
						SET surfacestate["maxq"] TO surfacestate["q"].
						set vehicle["max_q_reached"] to FALSE.
					} else {
						if (NOT vehicle["max_q_reached"]) {
							addGUIMessage("VEHICLE HAS REACHED MAX-Q").
							set vehicle["max_q_reached"] to TRUE.
						}
						
						if (surfacestate["q"] < 0.95*surfacestate["maxq"]) {
							addGUIMessage("GO AT THROTTLE-UP").
							set vehicle["qbucket"] to FALSE.
						}
					}
				
					set dap:thr_rpl_tgt to vehicle["qbucketThrottle"].
				} else {
					if (NOT vehicle["max_q_reached"]) AND (surfacestate["q"] > vehicle["qbucketval"]) {
						set vehicle["qbucket"] to TRUE.
						addGUIMessage("THROTTLING DOWN").
					} else {
						set vehicle["qbucket"] to FALSE.
					}
				
					set dap:thr_rpl_tgt to vehicle["nominalThrottle"].
				}
			
			}
		}
	}
	
}


function ops1_second_stage_nominal {
	
	dap:set_steering_low().
	
	set dap:steer_refv to -SHIP:ORBIT:BODY:POSITION:NORMALIZED.
	
	SET vehiclestate["major_mode"] TO 103.
	
	addGUIMessage("RUNNING UPFG ALGORITHM").
	
	//upfg loop
	UNTIL FALSE{
		if (quit_program) {
			RETURN.
		}
	
		IF (upfgInternal["itercount"] = 0) { //detects first pass or convergence lost
			WHEN (upfgInternal["s_conv"]) THEN {
				addGUIMessage("GUIDANCE CONVERGED IN " + upfgInternal["itercount"] + " ITERATIONS").
			}
		}	

		abort_handler().
		getState().
		
		if (vehicle["low_level"]) {
			addGUIMessage("LOW LEVEL").
			BREAK.
		}
		
		if (upfgInternal["s_meco"]) {
			addGUIMessage("TERMINAL GUIDANCE").
			BREAK.
		}	
		
		upfg_sense_current_state(upfgInternal).
		
		upfg(
			vehicle["stages"]:SUBLIST(vehiclestate["cur_stg"],vehicle:LENGTH-vehiclestate["cur_stg"]),
			target_orbit,
			upfgInternal
		).
		
		IF (upfgInternal["s_conv"] AND NOT vehiclestate["staging_in_progress"]) {
			dap:set_steer_tgt(vecYZ(upfgInternal["steering"])).
			set dap:thr_rpl_tgt to upfgInternal["throtset"].	
		}
	}
	
	//terminal loop 
	
	set dap:thr_rpl_tgt to dap:thr_min.
	
	addGUIMessage("WAITING FOR MECO").
	
	UNTIL FALSE {
		getState().
		IF (SHIP:VELOCITY:ORBIT:MAG >= target_orbit["velocity"] OR SSME_flameout()) {
			BREAK.
		}
	}
	
	SET vehiclestate["staging_in_progress"] TO TRUE.	//so that vehicle perf calculations are skipped in getState
	
	//stop oms dump for intact aborts
	LOCK THROTTLE to 0.
	SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.
	shutdown_ssmes().
	stop_oms_dump(TRUE).
	dap:set_rcs(TRUE).
	
	SET vehiclestate["major_mode"] TO 104.
}


function ops1_et_sep {
	parameter fast_sep is false.
	
	local pre_sequence_t is 0.
	local pre_sep_t is 0.
	local translation_t is 0.
	
	if (NOT fast_sep) {
		addGUIMessage("STAND-BY FOR ET SEP").
		set pre_sequence_t to 2.
		set pre_sep_t to 2.
		set translation_t to 15.
	} else {
		addGUIMessage("FAST ET SEP").
		set pre_sequence_t to 0.1.
		set pre_sep_t to 0.5.
		set translation_t to 5.
	}
	
	
	//ET sep loop
	LOCAL sequence_start_t IS surfacestate["time"].
	LOCAL sequence_exit is false.
	WHEN ( surfacestate["time"] > sequence_start_t + pre_sequence_t) THEN {
		SET SHIP:CONTROL:TOP TO 1.
		SET SHIP:CONTROL:FORE TO 1.
		
		WHEN ( surfacestate["time"] > sequence_start_t + pre_sep_t) THEN {
			et_sep().
			
			WHEN ( surfacestate["time"] > sequence_start_t + translation_t) THEN {
				set sequence_exit to true.
				//to disable RCS separation maneouvre
				SET SHIP:CONTROL:NEUTRALIZE TO TRUE.
			}
		}
	}
	
	
	UNTIL FALSE{
		getState().
		
		IF (sequence_exit) {
			BREAK.
		}
		WAIT 0.1.
	}
	
	close_umbilical().
	disable_TVC().
	
	//this is presumably where we add logic for atttude control in contingency
	
	UNLOCK THROTTLE.
	UNLOCK STEERING.
	SAS ON.
	
	
	RETURN.
}

//figure out what to do based on mode and abort modes 
function ops1_termination {
	
	if (vehiclestate["major_mode"] = 104) {
		print_ascent_report().
	
		WAIT 5.
	}

}


function closed_loop_ascent_bk{
	


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
		measure_update_engines().
		//monitor_abort().	//disabled
		abort_region_determinator().
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
			
				dap:set_steering_low().
			
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
					
					dap:set_steering_high().
					
					set vehicle["roll"] to 0.
					set dap:steer_roll to 0.
					set dap:max_steervec_corr to 8.
					
					set dap:steer_refv to VXCL(vecYZ(RTLSAbort["pitcharound"]["refvec"]),SHIP:FACING:TOPVECTOR).
					LOCAL thrust_facing IS VXCL(RTLSAbort["pitcharound"]["refvec"],vecYZ(thrust_vec()):NORMALIZED).
					SET RTLS_steering TO rodrigues(thrust_facing, RTLSAbort["pitcharound"]["refvec"], 45). 
					IF (VANG(thrust_facing, RTLSAbort["pitcharound"]["target"]) < 20) {
						SET RTLSAbort["pitcharound"]["complete"] TO TRUE.
						SET RTLSAbort["flyback_flag"] TO TRUE.
						SET upfgInternal["s_flyback"] TO TRUE.
						SET upfgInternal["s_throt"] TO TRUE.
						set dap:steer_refv to -SHIP:ORBIT:BODY:POSITION:NORMALIZED.
						
					}	
				
				}
				
				dap:set_steer_tgt(vecYZ(RTLS_steering)).
			}
		
			SET RTLSAbort["Tc"] TO PEG_Tc.
		}
		
		
		IF (upfgInternal["s_conv"] AND NOT vehiclestate["staging_in_progress"]) {
			dap:set_steer_tgt(vecYZ(upfgInternal["steering"])).
		
			set dap:thr_rpl_tgt to upfgInternal["throtset"].	
		} 
		
	}
	
	SET vehiclestate["major_mode"] TO 3.
 
	SET upfgInternal["terminal"] TO TRUE.
	
	set dap:thr_rpl_tgt to dap:thr_min.
	
	//put RTLS terminal logic in its own block
	IF (DEFINED RTLSAbort) {

		dap:set_steering_high().
		
		set dap:steer_refv to -SHIP:ORBIT:BODY:POSITION:NORMALIZED.
		
		UNTIL FALSE{
			getState().
			
			LOCAL steervec IS SHIP:VELOCITY:SURFACE:NORMALIZED.
			LOCAL upvec IS -SHIP:ORBIT:BODY:POSITION:NORMALIZED.
			
			set dap:steer_tgtdir to LOOKDIRUP(steervec, upvec).

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
	dap:set_rcs(TRUE).
	
	//ET sep loop
	LOCAL etsep_t IS TIME:SECONDS.
	WHEN ( TIME:SECONDS > etsep_t + 1.5) THEN {
		SET SHIP:CONTROL:TOP TO 1.
		
		WHEN ( TIME:SECONDS > etsep_t + 2.5) THEN {
			STAGE.
			
			WHEN ( TIME:SECONDS > etsep_t + 15) THEN {
				SET vehiclestate["major_mode"] TO 4.
			}
		}
	}
	addGUIMessage("STAND-BY FOR ET SEP").
	
	UNTIL FALSE{
		getState().
		
		IF (vehiclestate["major_mode"] = 4) {
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


ops1_main_exec().

CLEARSCREEN.
dap_gui_executor["stop_execution"]().
close_all_GUIs().



//IF (DEFINED RTLSAbort) {
//	RUN "0:/ops3".
//}