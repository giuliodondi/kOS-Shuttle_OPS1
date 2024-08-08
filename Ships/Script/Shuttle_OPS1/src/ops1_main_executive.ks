@LAZYGLOBAL OFF.
CLEARSCREEN.
SET CONFIG:IPU TO 1200.	
GLOBAL quit_program IS FALSE.
global debug_mode is false.
global dap_debug is true.


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
		clean_up_ops1().
		RETURN.
	}
	
	ops1_first_stage().
	
	if (quit_program) {
		clean_up_ops1().
		RETURN.
	}
	
	setupUPFG().
	
	//so we skip directly to rtls or contingency in case of a first stage abort 
	abort_handler().
	getState().
	
	// check for abort modes, proceed to nominal, rtls or contingency
	if NOT (abort_modes["cont_2eo_active"] or abort_modes["cont_3eo_active"] or abort_modes["rtls_active"]) {
		ops1_second_stage_nominal().
	}
	
	if (quit_program) {
		clean_up_ops1().
		RETURN.
	}
	
	//handle sequence for rtls and contingency 
	
	if (abort_modes["rtls_active"]) {
		ops1_second_stage_rtls().
	}
	
	if (quit_program) {
		clean_up_ops1().
		RETURN.
	}
	
	if (abort_modes["cont_2eo_active"]) {		//probably won't handle 3eo in this block
		ops1_second_stage_contingency().
	}
	
	if (quit_program) {
		clean_up_ops1().
		RETURN.
	}
	
	ops1_et_sep().
	
	ops1_termination().
}




function ops1_countdown{

	//needed to update the dap at least once
	wait 0.

	SAS OFF.
	switch_att_rcs(false, false, false).
	
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
	
	//vertical speed greater than 1 three times in a row
	local liftoff_sample_c is 0.
	until false {
		if (liftoff_sample_c >= 3) {
			break.
		}
		if (SHIP:VERTICALSPEED > 1) {
			set liftoff_sample_c to liftoff_sample_c + 1.
		}
		wait 0.1.
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
	
	WHEN (surfacestate["vs"] >= vehicle["roll_v0"]) THEN {
		addGUIMessage("ROLL PROGRAM").	
		SET steer_flag TO true.
		SET throt_flag TO true.
		
		//reset throttle to maximum
		SET dap:thr_rpl_tgt TO vehicle["nominalThrottle"].
	}
	
	//main ascent loop
	UNTIL FALSE {	
		if (quit_program) {
			RETURN.
		}
		
		if (debug_mode) {
			clearvecdraws().
		}
	
		abort_handler().
		getState().
		
		IF (vehicle["stages"][vehiclestate["cur_stg"]]["Tstage"] <= 4 ) {
			BREAK.
		}
		
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
		//modification - meaure max-Q separately from throttling so it's done for aborts as well 
		if (NOT vehicle["max_q_reached"]) AND (surfacestate["vs"] > 100) {
			IF (surfacestate["q"] >=  surfacestate["maxq"] ) {
				SET surfacestate["maxq"] TO surfacestate["q"].
				set vehicle["max_q_reached"] to FALSE.
			} else {
				addGUIMessage("VEHICLE HAS REACHED MAX-Q").
				set vehicle["max_q_reached"] to TRUE.
			}
		}
		
		if (get_engines_out() > 0) {
			set dap:thr_rpl_tgt to vehicle["maxThrottle"].
			set throt_flag to false.
		} else {
			if (throt_flag) {
				if (vehicle["qbucket"]) {
					if (vehicle["max_q_reached"]) AND (surfacestate["q"] < 0.95*surfacestate["maxq"]) {
						addGUIMessage("GO AT THROTTLE-UP").
						set vehicle["qbucket"] to FALSE.
					}
					set dap:thr_rpl_tgt to vehicle["qbucketThrottle"].
				} else {
					if (NOT vehicle["max_q_reached"]) AND (surfacestate["q"] > vehicle["qbucketval"]) {
						set vehicle["qbucket"] to TRUE.
						addGUIMessage("THROTTLING DOWN").
					}
				
					set dap:thr_rpl_tgt to vehicle["nominalThrottle"].
				}
			
			}
		}
	}
	
	
	//separation loop
	//central block to handle nominal srb sep and 3eo contingency breakout to the et sep block
	
	SET vehiclestate["staging_in_progress"] TO TRUE.
	
	//save the 3eo flag so we don't mess things up if 3eo happens during the srb sep loop
	local _3eo_et_sep is abort_modes["cont_3eo_active"].
	local staging_met is surfacestate["MET"] + 100000.
	
	local srb_tailoff_thr is 0.
	
	if (_3eo_et_sep) {
		addGUIMessage("EMERGENCY ET SEP").
		dap:set_steering_high().
		dap:set_rcs(TRUE).
		dap:set_steer_tgt(surfacestate["surfv"]:NORMALIZED).
		set dap:thrust_corr to FALSE.
		set srb_tailoff_thr to 500.
	} else {
		addGUIMessage("STAND-BY FOR SRB SEP").
		set srb_tailoff_thr to 350.
	}
	
	local break_sep_loop is false.
	local rcs_translation_sep is _3eo_et_sep.
	
	UNTIL FALSE {	
		if (quit_program) {
			RETURN.
		}
		
		if (break_sep_loop) {
			break.
		}
		
		if (debug_mode) {
			clearvecdraws().
		}
		
		abort_handler().
		getState().
		
		local srb_tailoff is (get_srb_thrust()<srb_tailoff_thr).
		
		local stg_elapsed is (surfacestate["MET"] - staging_met).
		
		if (_3eo_et_sep) {
	
			set break_sep_loop to srb_tailoff.
		} else  {
		
			set break_sep_loop to (stg_elapsed >= 5).
		
			//at tailoff, trigger srb sep
			if (NOT vehicle["srb_sep_flag"]) and (srb_tailoff) {
				wait until stage:ready.
				STAGE.
				
				//measure and save conditions at staging 
				LOCAL ve_stg IS surfacestate["surfv"]:MAG.

				SET vehiclestate["srb_sep"]["time"] TO surfacestate["MET"].
				SET vehiclestate["srb_sep"]["ve"] TO ve_stg.
				SET vehiclestate["srb_sep"]["alt"] TO surfacestate["alt"].
				
				increment_stage().
				
				set staging_met to (vehiclestate["staging_time"] - vehicle["ign_t"]).
				
				set vehicle["srb_sep_flag"] to true.
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
	
	//stop oms dump for intact aborts
	LOCK THROTTLE to 0.
	SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.
	shutdown_ssmes().
	stop_oms_dump(TRUE).
	
	SET vehiclestate["major_mode"] TO 104.
}


function ops1_second_stage_rtls {

	SET vehiclestate["major_mode"] TO 601.
	
	freeze_abort_gui(true).
	
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
		
		LOCAL steervec IS surfacestate["surfv"]:NORMALIZED.
		LOCAL upvec IS -SHIP:ORBIT:BODY:POSITION:NORMALIZED.
		
		set dap:steer_tgtdir to LOOKDIRUP(steervec, upvec).

		LOCAL rng IS downrangedist(launchpad,SHIP:GEOPOSITION )*1000.
		LOCAL tgtsurfvel IS RTLS_rvline(rng).
		
		IF (abs(surfacestate["horiz_dwnrg_v"]) >= tgtsurfvel OR SSME_flameout()) {
			BREAK.
		}
		
	}
	
	//stop oms dump for intact aborts
	LOCK THROTTLE to 0.
	SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.
	shutdown_ssmes().
	stop_oms_dump(TRUE).
	
	SET vehiclestate["major_mode"] TO 602.		//?????
}


function ops1_second_stage_contingency {

	addGUIMessage("Contingency not yet implemented, please quit program").
	
	set dap:steer_refv to -SHIP:ORBIT:BODY:POSITION:NORMALIZED.
	
	set vehicle["roll"] to 0.
	set dap:steer_roll to 0.

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

	if (vehicle["et_sep_flag"]) {return.}
	
	set dap:steer_refv to -SHIP:ORBIT:BODY:POSITION:NORMALIZED.
	
	set dap:thrust_corr to FALSE.
	dap:set_rcs(TRUE).
	switch_att_rcs().
	
	shutdown_ssmes().	//for good measure
	SET vehicle["meco_flag"] TO TRUE.
	ssme_out_safing().
	
	SET vehiclestate["staging_in_progress"] TO TRUE.	//so that vehicle perf calculations are skipped in getState

	addGUIMessage("STAND-BY FOR ET SEP").
	
	//3 modes: nominal, immediate, rate-sep 

	local et_sep_mode is et_sep_mode_determinator().
	
	LOCAL sequence_trigger_t IS surfacestate["time"].
	
	local pre_sequence_t is 0.
	local pre_sep_t is 0.
	local translation_t is 0.
	
	if (et_sep_mode = "nominal") {
		set pre_sequence_t to 2.
		set pre_sep_t to 2.
		set translation_t to 15.
	} else if (et_sep_mode = "immediate") {
		set pre_sequence_t to 0.
		set pre_sep_t to 0.3.
		set translation_t to 7.
	} else if (et_sep_mode = "rate") {
		set pre_sequence_t to 0.
		set pre_sep_t to 0.
		set translation_t to 7.
	}
	
	LOCAL sequence_start is false.
	LOCAL sequence_end is false.
	
	//calculate a pitch-up steering direction for contingencies
	local cur_steer is dap:steer_dir.
	local pitch_up_steer is rodrigues(cur_steer:forevector, -cur_steer:starvector, 20).
	
	UNTIL FALSE{
		getState().
		
		IF (sequence_end) {
			BREAK.
		}
		
		//rate sep is different
		if (et_sep_mode = "rate") {
		
		} else {
			//nominal and immediate sep work the same except with different timings
			
			if (NOT sequence_start) and (surfacestate["time"] > sequence_trigger_t + pre_sequence_t) {
				set sequence_start to true.
				set sequence_trigger_t to surfacestate["time"].
				
				SET SHIP:CONTROL:TOP TO 1.
				SET SHIP:CONTROL:FORE TO 1.
				
				//if immediate, pitch up 20°
				//work out whether to translate sideways as well
				if (et_sep_mode = "immediate") {
					
					dap:set_steer_tgt(pitch_up_steer).
				
				}
			}
			
			if (not vehicle["et_sep_flag"]) and (surfacestate["time"] > sequence_trigger_t + pre_sep_t) {
				set vehicle["et_sep_flag"] to true.
				set sequence_trigger_t to surfacestate["time"].
				
				et_sep().
			}
			
			if (vehicle["et_sep_flag"]) and (surfacestate["time"] > sequence_trigger_t + translation_t) {
				set sequence_end to true.
				set sequence_trigger_t to surfacestate["time"].
				
				SET SHIP:CONTROL:NEUTRALIZE TO TRUE.
			}
		}
		
		WAIT 0.1.
	}
	
	close_umbilical().
	
	//if we're in a contingency do more stuff 
	
	//do a re-orientation after et-sep since we might be in a weird attitude
	//after et sep set toggle serc off in the dap
	
	dap:toggle_serc(false).
	
	if (abort_modes["cont_2eo_active"] OR abort_modes["cont_3eo_active"]) {
		dap:set_steering_free().
		dap:set_steer_tgt(pitch_up_steer).
		until false {
			getState().
			
			set vehicle["roll"] to 0.
			set dap:steer_roll to 0.
			
			if (ABS(dap:steer_roll - dap:cur_steer_roll) < 5) and (dap:roll_rate < 1) {
				BREAK.
			}
			
			wait 0.1.
		}
	}

}

//figure out what to do based on mode and abort modes 
function ops1_termination {

	local nominal_flag is (not abort_triggered()).

	if (nominal_flag or abort_modes["ato_active"]) {
		print_ascent_report().
		WAIT 5.
	}
	
	clean_up_ops1().
	
	if (abort_modes["tal_active"]) {
		RUN "0:/ops3"("tal", abort_modes["tal_tgt_site"]["site"]).
	} else if (abort_modes["rtls_active"]) {
		RUN "0:/ops3"("grtls", abort_modes["rtls_tgt_site"]["site"]).
	}

}

function clean_up_ops1 {
	CLEARSCREEN.
	clearvecdraws().
	dap_gui_executor["stop_execution"]().
	close_all_GUIs().

	UNLOCK THROTTLE.
	UNLOCK STEERING.
	SAS ON.	//for good measure
}

ops1_main_exec().
