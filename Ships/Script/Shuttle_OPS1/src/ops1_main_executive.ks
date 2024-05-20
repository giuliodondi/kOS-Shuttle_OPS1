@LAZYGLOBAL OFF.
CLEARSCREEN.
SET CONFIG:IPU TO 1200.	
GLOBAL quit_program IS FALSE.
global debug_mode is false.
global dap_debug is false.


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
													//protection
													if (SAS) {
														SAS OFF.
													}
													
													if (is_dap_auto()) {
														dap:steer_auto_thrvec().
														dap:thr_control_auto().
													} else if (is_dap_css()) {
														dap:steer_css().
														dap:thr_control_css().
													}
													
													set get_stage()["Throttle"] to dap:thr_rpl_tgt.
													
													if (dap_debug) {
														//clearscreen.
														clearvecdraws().
														
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
		CLEARSCREEN.
		dap_gui_executor["stop_execution"]().
		close_all_GUIs().
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
	
	if (abort_modes["rtls_active"]) {
		ops1_second_stage_rtls().
	}
	
	if (quit_program) {RETURN.}
	
	if (abort_modes["cont_2eo_active"]) {		//probably won't handle 3eo in this block
		ops1_second_stage_contingency().
	}
	
	if (quit_program) {RETURN.}
	
	ops1_et_sep().
	
	ops1_termination().
}




function ops1_countdown{

	//needed to update the dap at least once
	wait 0.

	SAS OFF.
	toggle_roll_rcs().
	
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
		
		if (vehicle["ssme_out_detected"]) {
			set dap:thr_rpl_tgt to vehicle["maxThrottle"].
		} else {
			if (throt_flag) {
				if (vehicle["qbucket"]) {
				
					IF (surfacestate["vs"] <= 50 ) or (surfacestate["q"] >=  surfacestate["maxq"] ) {
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

	freeze_abort_gui(false).
	
	dap:set_steering_low().
	
	set dap:steer_refv to -SHIP:ORBIT:BODY:POSITION:NORMALIZED.
	
	SET target_orbit["normal"] TO upfg_normal(target_orbit["inclination"], target_orbit["LAN"]).
	
	SET vehiclestate["major_mode"] TO 103.
	
	//upfg loop
	UNTIL FALSE{
		if (quit_program) {
			RETURN.
		}

		abort_handler().
		getState().
		
		if (abort_modes["cont_2eo_active"]) or (abort_modes["cont_3eo_active"]) or (abort_modes["rtls_active"]) {
			return.
		}
		
		if (vehicle["low_level"]) {
			addGUIMessage("LOW LEVEL").
			BREAK.
		}
		
		if (upfgInternal["s_meco"]) {
			addGUIMessage("TERMINAL GUIDANCE").
			BREAK.
		}	
		
		IF HASTARGET = TRUE AND (TARGET:BODY = SHIP:BODY) {
			tgt_j2_timefor(target_orbit, upfgInternal["tgo"]).
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
		
		if (debug_mode) {
			clearvecdraws().
			arrow_ship(vecyz(upfgInternal["steering"]),"steer").
			arrow_ship(vecyz(upfgInternal["ix"]),"ix").
			arrow_ship(vecyz(upfgInternal["iy"]),"iy").
			arrow_ship(vecyz(upfgInternal["iz"]),"iz").
			
			arrow_body(vecyz(vxcl(upfgInternal["iy"], upfgInternal["r_cur"])),"r_proj").
			arrow_body(vecyz(upfgInternal["rd"]),"rd").
			arrow_body(vecyz(target_orbit["normal"]),"rd").
		}
	}
	
	//terminal loop 
	
	SET upfgInternal["terminal"] TO TRUE.
	
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
	
	SET vehiclestate["major_mode"] TO 104.
}


function ops1_second_stage_rtls {

	SET vehiclestate["major_mode"] TO 601.
	
	dap:set_steering_low().
	
	set dap:steer_refv to -SHIP:ORBIT:BODY:POSITION:NORMALIZED.

	until false {
		if (quit_program) {
			RETURN.
		}

		abort_handler().
		getState().
		
		if (abort_modes["cont_2eo_active"]) or (abort_modes["cont_3eo_active"]) {
			return.
		}
		
		if (vehicle["low_level"]) {
			addGUIMessage("LOW LEVEL").
			BREAK.
		}
		
		if (upfgInternal["s_meco"]) {
			addGUIMessage("TERMINAL GUIDANCE").
			BREAK.
		}
		
		upfg_sense_current_state(upfgInternal).
		
		IF (NOT RTLSAbort["flyback_flag"]) {
		
			LOCAL stg IS get_stage().
		
			//extrapolate state to the end of ppa
			SET upfgInternal["t_cur"] TO upfgInternal["t_cur"] - RTLSAbort["pitcharound"]["dt"].
			SET upfgInternal["r_cur"] TO upfgInternal["r_cur"] + upfgInternal["v_cur"] * RTLSAbort["pitcharound"]["dt"].
			SET upfgInternal["m_cur"] TO upfgInternal["m_cur"] - stg["engines"]["flow"] * RTLSAbort["pitcharound"]["dt"].
			SET upfgInternal["tb_cur"] TO upfgInternal["tb_cur"] - RTLSAbort["pitcharound"]["dt"].
		
		}
		
		upfg(
			vehicle["stages"]:SUBLIST(vehiclestate["cur_stg"],vehicle:LENGTH-vehiclestate["cur_stg"]),
			target_orbit,
			upfgInternal
		).
		
		LOCAL PEG_Tc IS (upfgInternal["mbo_T"] - upfgInternal["tgo"]).
		
		IF (NOT (RTLSAbort["flyback_flag"] AND RTLSAbort["pitcharound"]["complete"] )) {
		
			dap:set_steering_low().
		
			//force unconverged until flyback
			SET upfgInternal["s_conv"] TO FALSE.
			SET upfgInternal["iter_conv"] TO 0.
			SET upfgInternal["itercount"] TO 0.
		
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
					LOCAL pitchover_bias IS 0.5 * RTLSAbort["pitcharound"]["dt"].
					
					IF (PEG_Tc <= (1 + pitchover_bias)) {
						addGUIMessage("POWERED PITCH-AROUND").
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
				
				LOCAL thrust_facing IS VXCL(RTLSAbort["pitcharound"]["refvec"],vecYZ(thrust_vec()):NORMALIZED).
				
				local rtls_ppa_angle is min(45, VANG(thrust_facing, RTLSAbort["pitcharound"]["target"])).
				
				IF (rtls_ppa_angle < 5) {
					SET RTLSAbort["pitcharound"]["complete"] TO TRUE.
					SET RTLSAbort["flyback_flag"] TO TRUE.
					SET upfgInternal["s_flyback"] TO TRUE.
					SET upfgInternal["s_throt"] TO TRUE.
					set dap:steer_refv to -SHIP:ORBIT:BODY:POSITION:NORMALIZED.
					SET RTLS_steering TO RTLSAbort["pitcharound"]["target"].
				} else {
					set dap:steer_refv to VXCL(vecYZ(RTLSAbort["pitcharound"]["refvec"]),SHIP:FACING:TOPVECTOR).
					SET RTLS_steering TO rodrigues(thrust_facing, RTLSAbort["pitcharound"]["refvec"], rtls_ppa_angle). 
				}
			
			}
			
			dap:set_steer_tgt(vecYZ(RTLS_steering)).
			
		} else {
			IF (upfgInternal["s_conv"] AND NOT vehiclestate["staging_in_progress"]) {
				dap:set_steer_tgt(vecYZ(upfgInternal["steering"])).
				set dap:thr_rpl_tgt to upfgInternal["throtset"].	
			}
		}
	
		SET RTLSAbort["Tc"] TO PEG_Tc.
		
	}
	
	//powered pitchdown
	
	SET upfgInternal["terminal"] TO TRUE.
	
	addGUIMessage("POWERED PITCH-DOWN").
	
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
	
	SET vehiclestate["staging_in_progress"] TO TRUE.	//so that vehicle perf calculations are skipped in getState
	
	//stop oms dump for intact aborts
	LOCK THROTTLE to 0.
	SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.
	shutdown_ssmes().
	stop_oms_dump(TRUE).
	
	SET vehiclestate["major_mode"] TO 602.		//?????
}


function ops1_second_stage_contingency {

	addGUIMessage("Contingency not yet implemented, please quit program").

	until false {
		if (quit_program) {
			RETURN.
		}
		
		if  (abort_modes["cont_3eo_active"]) {
			return.
		}
	}
}


function ops1_et_sep {
	parameter fast_sep is false.
	
	dap:set_rcs(TRUE).
	toggle_roll_rcs(true).
	ssme_out_safing().
	
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
	
	//this is presumably where we add logic for atttude control in contingency
	
	UNLOCK THROTTLE.
	UNLOCK STEERING.
	SAS ON.
	
	
	RETURN.
}

//figure out what to do based on mode and abort modes 
function ops1_termination {

	local nominal_flag is (not abort_triggered()).

	if (nominal_flag or abort_modes["ato_active"]) {
		print_ascent_report().
		WAIT 5.
	}
	
	CLEARSCREEN.
	clearvecdraws().
	dap_gui_executor["stop_execution"]().
	close_all_GUIs().
	
	if (abort_modes["tal_active"]) {
		RUN "0:/ops3"("tal", abort_modes["tal_tgt_site"]["site"]).
	} else if (abort_modes["rtls_active"]) {
		RUN "0:/ops3"("grtls", abort_modes["rtls_tgt_site"]["site"]).
	}

}

ops1_main_exec().
