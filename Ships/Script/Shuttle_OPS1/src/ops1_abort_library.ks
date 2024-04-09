//main abort lexicon
//abort boundaries are defined with velocity in their own functions
GLOBAL abort_modes IS LEXICON( 
					"trigger_t",time:seconds + 10000000000,
					"abort_initialised", false,
					"manual_abort", false,
					"rtls_active", false,
					"tal_active", false,
					"ato_active", false,
					"cont_2eo_active", false,
					"cont_3eo_active", false,
					"ssmes_out", list(),
					"oms_dump",FALSE,
					"rtls_tgt_site", "",
					"tal_tgt_site", "",
					"tal_candidates", "",
					"1eo_tal_sites", "",
					"2eo_tal_sites", "",
					"ecal_candidates", "",
					"intact_modes", lexicon(
												"1eo", lexicon(
															"rtls", true,	// default mode at liftoff
															"tal", false,
															"ato", false,
															"meco", false
													),
												"2eo", lexicon(
															"rtls", false,		//required for 2eo rtls completion
															"droop", false,
															"tal", false,
															"ato", false,
															"meco", false
													)
									),
					"2eo_cont_mode", "XXXXX",
					"3eo_cont_mode", "XXXXX"
							
).

function abort_triggered {
	return abort_modes["rtls_active"] or abort_modes["tal_active"] or abort_modes["ato_active"] or abort_modes["cont_2eo_active"] or abort_modes["cont_3eo_active"].
}


function initialise_abort_sites {

	set abort_modes["rtls_tgt_site"] to get_RTLS_site().
	
	local normv is vecyz(upfg_normal(target_orbit["inclination"], target_orbit["LAN"])).
	local posv is - SHIP:ORBIT:BODY:POSITION.
	local velv is vcrs(posv, normv).
	
	set abort_modes["tal_candidates"] to get_sites_downrange(
								ldgsiteslex,
								posv,
								velv,
								5000,
								20000
	).
	
	
	set abort_modes["ecal_candidates"] to get_sites_downrange(
								ldgsiteslex,
								posv,
								velv,
								1000,
								2000
	).


}


//gather abort redion determinator, abort initialiser, and anything else that may come up
function abort_handler {

	//before getstate we need to update engines and setup aborts so the stage is reconfigured 
	//and then adjusted to the current fuel mass
	measure_update_engines().
	
	intact_abort_region_determinator().
	
	contingency_abort_region_determinator().
	
	IF EXISTS("0:/abort_modes_dump.txt") {
		DELETEPATH("0:/abort_modes_dump.txt").
	}
	
	log abort_modes:dump() to "0:/abort_modes_dump.txt".
	
	abort_initialiser().

}

//determine available intact abort modes for 1,2 eng out cases
function intact_abort_region_determinator {

	//intact modes are frozen if a contingency is active 
	if (abort_modes["cont_2eo_active"]) or (abort_modes["cont_3eo_active"]) or (abort_modes["rtls_active"]) {
		return.
	}

	clearscreen.
	
	if (abort_modes["intact_modes"]["1eo"]["rtls"]) {
		local neg_return is RTLS_boundary().
		
		if (neg_return) {
			addGUIMessage("NEGATIVE RETURN").
			set abort_modes["intact_modes"]["1eo"]["rtls"] to false.
		}
	}
	
	local two_eng_lex is build_engines_lex(2).
	local one_eng_lex is build_engines_lex(1).
	
	local two_eng_perf is veh_perf_estimator(two_eng_lex).
	local one_eng_perf is veh_perf_estimator(one_eng_lex).
	
	LOCAL tal_2e_dv is list(). 
	LOCAL tal_1e_dv is list(). 
	
	if (abort_modes["intact_modes"]["2eo"]["ato"]) or (abort_modes["intact_modes"]["2eo"]["meco"]) {
		// after single engine p2ato the only cases are at least an ato or a 3 engine out abort, no need for tal beyond this
		set abort_modes["intact_modes"]["1eo"]["tal"] to false.
		set abort_modes["intact_modes"]["2eo"]["tal"] to false.
	} else {
		
		local two_eng_best_tal is lexicon("site", "", "deltav", -10000000000).
		local one_eng_best_tal is lexicon("site", "", "deltav", -10000000000).
		
		LOCAL current_normal IS currentNormal().
		
		for s in abort_modes["tal_candidates"] {
		
			local tal_meco_v is tal_site_meco_velocity(s, current_normal).
			
			LOCAL dv2site IS (tal_meco_v - orbitstate["velocity"]).
			
			local two_eng_tal_dv_excess is estimate_excess_deltav(
													orbitstate["radius"],
													orbitstate["velocity"],
													dv2site,
													two_eng_perf
			).
			
			local two_eng_tal_dv is lexicon(
										"site", s, 
										"deltav", two_eng_tal_dv_excess
			).
			
			if (two_eng_tal_dv_excess > 0) {
				tal_2e_dv:add(two_eng_tal_dv).
			}
			
			if (two_eng_tal_dv_excess > two_eng_best_tal["deltav"]) {
				set two_eng_best_tal to two_eng_tal_dv.
			}
			
			
			local one_eng_tal_dv_excess is estimate_excess_deltav(
													orbitstate["radius"],
													orbitstate["velocity"],
													dv2site,
													one_eng_perf
			).
			
			local one_eng_tal_dv is lexicon(
										"site", s, 
										"deltav", one_eng_tal_dv_excess
			).
			
			if (one_eng_tal_dv_excess > 0) {
				tal_1e_dv:add(one_eng_tal_dv).
			}
			
			if (one_eng_tal_dv_excess > one_eng_best_tal["deltav"]) {
				set one_eng_best_tal to one_eng_tal_dv.
			}
		}
		
		if (not abort_modes["intact_modes"]["1eo"]["tal"]) {
			if (NOT abort_modes["intact_modes"]["1eo"]["rtls"]) {
				// if we're not tal by negative return, force tal mode with the least bad site 
				if (tal_2e_dv:length = 0) {
					tal_2e_dv:add(two_eng_best_tal).
				}
				
			}
			
			if (tal_2e_dv:length > 0) {
				addGUIMessage("TWO ENGINE TAL").
				set abort_modes["intact_modes"]["1eo"]["tal"] to true.
			}
		}
		
		if (not abort_modes["intact_modes"]["2eo"]["tal"]) {
			if (tal_1e_dv:length > 0) { 
				addGUIMessage("SINGLE ENGINE TAL").
				set abort_modes["intact_modes"]["2eo"]["tal"] to true.
			}
		}
		
	}
	
	set abort_modes["1eo_tal_sites"] to tal_2e_dv.
	set abort_modes["2eo_tal_sites"] to tal_1e_dv.
	
	//if tal we don't want to calculate ato or meco boundaries
	if (abort_modes["tal_active"]){
		return.
	}
	
	local ato_tgt_orbit is get_ato_tgt_orbit().
	
	LOCAL ato_dv IS (ato_tgt_orbit["velvec"] - orbitstate["velocity"]).
	
	if (NOT abort_modes["intact_modes"]["1eo"]["meco"]) {
		local two_eng_ato_dv_excess is estimate_excess_deltav(
												orbitstate["radius"],
												orbitstate["velocity"],
												ato_dv,
												two_eng_perf
		
		).
		
		if (two_eng_ato_dv_excess > 0) {
			if (not abort_modes["intact_modes"]["1eo"]["ato"]) {
				addGUIMessage("PRESS TO ATO").
				set abort_modes["intact_modes"]["1eo"]["ato"] to true.
			}
		}
	}
	
	if (not abort_modes["intact_modes"]["2eo"]["meco"]) {
	
		local one_eng_ato_dv_excess is estimate_excess_deltav(
												orbitstate["radius"],
												orbitstate["velocity"],
												ato_dv,
												one_eng_perf
		
		).
		
		if (one_eng_ato_dv_excess > 0) {
			if (not abort_modes["intact_modes"]["2eo"]["ato"]) {
				addGUIMessage("SINGLE ENGINE PRESS TO ATO").
				set abort_modes["intact_modes"]["2eo"]["ato"] to true.
			}
		}
	}
	
	//if ato we don't want to calculate meco boundaries
	if (abort_modes["ato_active"]) {
		return.
	}
	
	
	local meco_vel is cutoff_velocity_vector(
										orbitstate["radius"],
										-target_orbit["normal"],
										target_orbit["velocity"],
										target_orbit["fpa"]
	).
	
	LOCAL meco_dv IS (meco_vel - orbitstate["velocity"]).
	
	
	local two_eng_meco_dv_excess is estimate_excess_deltav(
											orbitstate["radius"],
											orbitstate["velocity"],
											meco_dv,
											two_eng_perf
	
	).
	
	if (two_eng_meco_dv_excess > 0) {
		if (not abort_modes["intact_modes"]["1eo"]["meco"]) {
			addGUIMessage("PRESS TO MECO").
			set abort_modes["intact_modes"]["1eo"]["meco"] to true.
			set abort_modes["intact_modes"]["1eo"]["ato"] to false.
		}
	} 
	
	local one_eng_meco_dv_excess is estimate_excess_deltav(
											orbitstate["radius"],
											orbitstate["velocity"],
											meco_dv,
											one_eng_perf
	
	).
	
	if (one_eng_meco_dv_excess > 0) {
		if (not abort_modes["intact_modes"]["2eo"]["meco"]) {
			addGUIMessage("SINGLE ENGINE PRESS TO MECO").
			set abort_modes["intact_modes"]["2eo"]["meco"] to true.
			set abort_modes["intact_modes"]["2eo"]["ato"] to false.
		}
	} 
	
}

function contingency_abort_region_determinator {

	//if 2eo is not active, update the mode 
	if (not abort_modes["cont_2eo_active"]) {
		if (abort_modes["rtls_active"]) {
			//rtls contingency modes 
		} else {
			//droop boundary missing
			if (abort_modes["intact_modes"]["2eo"]["droop"]) or 
				(abort_modes["intact_modes"]["2eo"]["tal"]) or 
				(abort_modes["intact_modes"]["2eo"]["ato"]) or 
				(abort_modes["intact_modes"]["2eo"]["meco"]) {
				set abort_modes["2eo_cont_mode"] to "BLANK".
			} else if ((vehiclestate["major_mode"] < 103) or (contingency_2eo_blue_boundary())) {
				set abort_modes["2eo_cont_mode"] to "BLUE".
			} else {
				set abort_modes["2eo_cont_mode"] to "GREEN".
			}
		}
	}
	
	//if 3eo is not active, update the mode 
	if (not abort_modes["cont_3eo_active"]) {
		if (abort_modes["rtls_active"]) {
			//rtls contingency modes 
		} else {
			if (orbitstate["velocity"]:MAG >= 6700) {
				set abort_modes["3eo_cont_mode"] to "BLANK". 
			} else if (vehiclestate["major_mode"] < 103) {
				set abort_modes["3eo_cont_mode"] to "BLUE". 
			} else {
				set abort_modes["3eo_cont_mode"] to "GREEN". 
			}
		}
	}

}

//determine if it's time to initialise an abort and which mode to activate
function abort_initialiser {

	local engines_out is get_engines_out().
	
	local zero_engout is (engines_out = 0).
	local one_engout is (engines_out = 1).
	local two_engout is (engines_out = 2).
	local three_engout is (engines_out = 3).
	
	//if first stage and not 3eo exit, we initialise the abort after srb sep
	if (vehiclestate["major_mode"] < 103) and (not three_engout) {
		return.
	}

	
	if (vehicle["ssme_out_detected"]) {
		//setup the abort trigger
		
		set vehicle["ssme_out_detected"] to false.
		//prepare for intitialisation 
		set abort_modes["abort_initialised"] to false.
		
		//the abort is triggered right now except in a 1eo case, we give 10 seconds for manual abort trigger
		//but we shouldn't wait if an abort was already triggered
		local abort_trigger_t is -0.2.
		
		if (one_engout) and (NOT (abort_modes["rtls_active"] or abort_modes["tal_active"] or abort_modes["ato_active"])) {
			set abort_trigger_t to 10.
		}
		
		set abort_modes["trigger_t"]	TO surfacestate["MET"] + abort_trigger_t.
		
	} else if (abort_modes["manual_abort"]) {
		set abort_modes["trigger_t"]	TO surfacestate["MET"] -0.2.
	}
	
	//exit if we're not past the abort trigger or if the abort is already initialised
	if (surfacestate["MET"] <= abort_modes["trigger_t"]) or (abort_modes["abort_initialised"]) {
		return.
	}
	
	//decide which mode to trigger 
	//oms dump logic decided on a mode-by-mode basis
	
	//intact aborts
	if (zero_engout) or (one_engout) {
		
		local one_eo_ato_abort is false.
		local one_eo_tal_abort is false.
		local one_eo_rtls_abort is false.
		
		//premature optimisation is the root of all evil
		
		//if there's an active mode we triggered a manual abort and hten we had an engine failure, re-initialise the same mode 
		if (abort_modes["rtls_active"]) {
			set one_eo_rtls_abort to true.
			set one_eo_tal_abort to false.
			set one_eo_ato_abort to false.
		} else if (abort_modes["tal_active"]) {
			set one_eo_rtls_abort to false.
			set one_eo_tal_abort to true.
			set one_eo_ato_abort to false.
		} else if (abort_modes["ato_active"]) {
			set one_eo_rtls_abort to false.
			set one_eo_tal_abort to false.
			set one_eo_ato_abort to true.
		} else if (abort_modes["manual_abort"]) {
			//in case of a manual abort, read and trigger the selected mode 
			if (is_abort_rtls_selected()) {
				set one_eo_rtls_abort to true.
				set one_eo_tal_abort to false.
				set one_eo_ato_abort to false.
			} else if (is_abort_tal_selected()) {
				set one_eo_rtls_abort to false.
				set one_eo_tal_abort to true.
				set one_eo_ato_abort to false.
			} else if (is_abort_ato_selected()) {
				set one_eo_rtls_abort to false.
				set one_eo_tal_abort to false.
				set one_eo_ato_abort to true.
			}
		} else {
			//in case of an auto abort we choose MECO-ATO-TAL-RTLS in this order of preference
			
			if (abort_modes["intact_modes"]["1eo"]["meco"]) {
				set one_eo_rtls_abort to false.
				set one_eo_tal_abort to false.
				set one_eo_ato_abort to false.
			} else if (abort_modes["intact_modes"]["1eo"]["ato"]) {
				set one_eo_rtls_abort to false.
				set one_eo_tal_abort to false.
				set one_eo_ato_abort to true.
			} else if (abort_modes["intact_modes"]["1eo"]["tal"]) {
				set one_eo_rtls_abort to false.
				set one_eo_tal_abort to true.
				set one_eo_ato_abort to false.
			} else if (abort_modes["intact_modes"]["1eo"]["rtls"]) {
				set one_eo_rtls_abort to true.
				set one_eo_tal_abort to false.
				set one_eo_ato_abort to false.
			}
			
		}
		
		set abort_modes["rtls_active"] to one_eo_rtls_abort.
		set abort_modes["tal_active"] to one_eo_tal_abort.
		set abort_modes["ato_active"] to one_eo_ato_abort.
		
		//now initialise the mode 
		if (abort_modes["rtls_active"]) {
			//setup rtls	
			addGUIMessage("ABORT RTLS").
			setup_RTLS().
		} else if (abort_modes["tal_active"]) {
			//setup tal 
			addGUIMessage("ABORT TAL").
			setup_TAL().
		} else if (abort_modes["ato_active"]) {
			//setup ato 
			addGUIMessage("ABORT ATO/AOA").
			setup_ATO().
		}
	} else if (two_engout) {
		//2eo is a contingency unless one of the intact modes is available
		
		local two_eo_ato_abort is false.
		local two_eo_tal_abort is false.
		local two_eo_rtls_abort is false.
		local two_eo_cont_abort is false.
		
		//droop still missing
		if (abort_modes["intact_modes"]["2eo"]["rtls"]) {
			set two_eo_cont_abort to false.
			set two_eo_rtls_abort to true.
			set two_eo_tal_abort to false.
			set two_eo_ato_abort to false.
		} else if (abort_modes["intact_modes"]["2eo"]["meco"]) {
			set two_eo_cont_abort to false.
			set two_eo_rtls_abort to false.
			set two_eo_tal_abort to false.
			set two_eo_ato_abort to false.
		} else if (abort_modes["intact_modes"]["2eo"]["ato"]) {
			set two_eo_cont_abort to false.
			set two_eo_rtls_abort to false.
			set two_eo_tal_abort to false.
			set two_eo_ato_abort to true.
		} else if (abort_modes["intact_modes"]["2eo"]["tal"]) {
			set two_eo_cont_abort to false.
			set two_eo_rtls_abort to false.
			set two_eo_tal_abort to true.
			set two_eo_ato_abort to false.
		} else {
			set two_eo_cont_abort to true.
			set two_eo_rtls_abort to false.
			set two_eo_tal_abort to false.
			set two_eo_ato_abort to false.
		}
		
		set abort_modes["cont_2eo_active"] to two_eo_cont_abort. 
		set abort_modes["rtls_active"] to two_eo_rtls_abort.
		set abort_modes["tal_active"] to two_eo_tal_abort. 
		set abort_modes["ato_active"] to two_eo_ato_abort. 
		
		if (abort_modes["tal_active"]) {
			//setup tal 
			addGUIMessage("ABORT 2EO TAL").
			setup_TAL().
		} else if (abort_modes["ato_active"]) {
			//setup ato 
			addGUIMessage("ABORT 2EO ATO/AOA").
			setup_ATO().
		}
		
		
	} else if (three_engout) {
		//3eo is always a contingency even in the blank region 
		set abort_modes["cont_3eo_active"] to true. 
	}

	set abort_modes["abort_initialised"] to true.

}


//	RTLS functions


//bias first-stage trajectory scale for lofting
FUNCTION first_stage_engout_lofting_bias {
	
	local engines_out is get_engines_out().
	local abort_t is 1000.
	
	if (engines_out > 0) {
		set abort_t to abort_modes["ssmes_out"][0]["time"].
	}
	
	//decreasing multiplying factor with engines out
	local engout_fac is (0.375 * engines_out + 0.625) * engines_out.
	
	RETURN max(engout_fac * 0.33*(1 - abort_t/122), 0).
	
}

//RTLs runway vector in UPFG coordinates
FUNCTION RTLS_tgt_site_vector {
	RETURN vecYZ(pos2vec(abort_modes["rtls_tgt_site"])).
}

//taken from the paper on rtls trajectory shaping
//reference theta + corrections for off-nominal conditions at staging
//all velocities are surface, v_i is velocity at abort or staging
FUNCTION RTLS_dissip_theta_pert {
	PARAMETER v_r.
	PARAMETER v_stg.
	PARAMETER alt_stg.
	PARAMETER thr.
	
	LOCAL vr2 IS v_r^2.
	
	//rs-25 nominal staging
	//LOCAL dv IS v_stg - 1498.12.
	//LOCAL dalt IS alt_stg - 47219.
	
	//rs-25D nominal staging 
	LOCAL dv IS v_stg - 1297.
	LOCAL dalt IS alt_stg - 49222.8.
	LOCAL thr_corr IS 4300000/thr.
	
	LOCAL theta_nom IS  (77.628302208 - 0.0346624*v_r + 7.86842e-6*vr2).
    
    LOCAL dalt_corr IS (2.649396 -1.91421084e-3*v_r + 4.693065e-7*vr2)*dalt.
    
    LOCAL dv_corr IS (349.344 - 0.2934698*v_r + 6.58751e-5*vr2)*dv.
    
    LOCAL dtheta_dv IS (21740.8 -15.1078*v_r + 3.281949e-3*vr2)*0.3048.
    
    LOCAL theta_pert IS  theta_nom - ( dalt_corr + dv_corr )/dtheta_dv.
	
	SET theta_pert TO ARCSIN(limitarg(thr_corr*SIN(theta_pert))).
	
	RETURN CLAMP(theta_pert, 30, 88).

}

//c1 vector in upfg coordinates
//meant to be repeatedly so the vector is always in the plane of current surface velocity
FUNCTION RTLS_C1 {
	PARAMETER theta.

	LOCAL pos IS orbitstate["radius"].
	LOCAL ve IS vecYZ(surfacestate["surfv"]).

	LOCAL normvec IS -VCRS(pos, ve):NORMALIZED.
	
	LOCAL horiz IS VCRS(pos,normvec):NORMALIZED.
	LOCAL C1 IS rodrigues(horiz, normvec, theta).
	
	RETURN C1.
}

FUNCTION RTLS_pitchover_t {
	PARAMETER c1_vec.
	PARAMETER pitcharound_vec.
	
	LOCAL pitchover_rate IS 10.		//degrees per second 
	
	RETURN VANG(pitcharound_vec, c1_vec)/pitchover_rate.
}

//RV-line, return the coefficients, 0 is linear (DISTANCE IN METRES) and 1 is constant
FUNCTION RTLS_rvline_coefs {
	//RETURN LIST(0.00242, 768.72).		//this is the original from the rtls traj-shaping paper 
	//RETURN LIST(0.0031, 559.49).		//this was lifted from the sts-1 abort analysis paper 
	//RETURN LIST(0.004, 234).			//this worked before ops3
	RETURN LIST(0.0035, 370).
}

//calculate the desired RV-line velocity  (DISTANCE IN METRES)
FUNCTION RTLS_rvline {
	PARAMETER rng.
	LOCAL coefs IS RTLS_rvline_coefs().
	RETURN coefs[0] * rng + coefs[1].
}

FUNCTION RTLS_burnout_mass {
	parameter m_final.

	//add 2% of ssme fuel and 20% of oms fuel
	//this NEEDs to be called after we update the final mass with the right value
	SET vehicle["mbod"] TO m_final + 0.02 * vehicle["SSME_prop_0"] + vehicle["OMS_prop_dump_frac"] * vehicle["OMS_prop_0"].
}

//to be called before launch, will fin the closest landing site 
//to the launchpad
FUNCTION get_RTLS_site {
	LOCAL closest_out IS get_closest_site(ldgsiteslex).
	RETURN closest_out[1].
}


FUNCTION RTLS_boundary_vel {
	RETURN (2930 - 7.8 * ABS(target_orbit["inclination"])).
}

FUNCTION RTLS_boundary{
	
	LOCAL dwnrg_speed IS current_horiz_dwnrg_speed(SHIP:GEOPOSITION, SHIP:VELOCITY:ORBIT).
	
	LOCAL boundary_vel IS RTLS_boundary_vel().

	RETURN (dwnrg_speed > RTLS_boundary_vel()).
}

//compare current velocity with negative return boundary to see if we should flyback immediately
FUNCTION RTLS_immediate_flyback {
	
	LOCAL dwnrg_speed IS current_horiz_dwnrg_speed(SHIP:GEOPOSITION, SHIP:VELOCITY:ORBIT).

	RETURN dwnrg_speed > RTLS_boundary_vel() - 80.
}

FUNCTION setup_RTLS {
	
	//do it immediately so it's ready when the gui first wants to update it 
	make_rtls_traj2_disp().
	
	
	start_oms_dump().
	
	// so g-limiting is skipped
	set vehicle["glim"] to 10.
	
	local engines_out is get_engines_out().
	
	local throt_val is vehicle["maxThrottle"].
	
	if (get_engines_out() < 1) {
		set throt_val to 0.69 * vehicle["maxThrottle"].
	}
	
	//redefine vehicle 
	measure_update_engines().
	local engines_lex is build_engines_lex().
	
	LOCAL current_m IS SHIP:MASS*1000.
	local res_left IS get_shuttle_res_left().
	local m_final is current_m - res_left.
	
	setup_shuttle_stages(
						current_m,
						m_final,
						engines_lex,
						throt_val
	).
	reset_stage().

	vehicle:ADD("mbod",0).
	
	RTLS_burnout_mass(m_final).		
	
	LOCAL cur_stg IS get_stage().

	//time to desired burnout mass
	LOCAL dmbo_t IS (cur_stg["m_initial"] - vehicle["mbod"]) * cur_stg["Tstage"]/cur_stg["m_burn"].
	
	//so that downrange distance calculations are correct
	SET launchpad TO abort_modes["RTLS"]["tgt_site"].
	
	LOCAL flyback_immediate IS RTLS_immediate_flyback().

	
	//target orbit with cutoff conditions
	LOCAL cutoff_alt IS 72.
	LOCAL cutoff_fpa IS 8.
	
	LOCAL curR IS orbitstate["radius"].
	
	LOCAL cut_r IS (curR:NORMALIZED)*(cutoff_alt*1000 + SHIP:BODY:RADIUS).
	
	LOCAL tgtsitevec IS RTLS_tgt_site_vector().
	
	//plane defined as containing current and target position
	//LOCAL normvec IS VCRS(cut_r, tgtsitevec):NORMALIZED.
	
	//plane defined from current position and velocity, must point to the launch site though
	LOCAL normvec IS -VCRS(curR, vecYZ(surfacestate["surfv"])):NORMALIZED.
	
	LOCAL rng IS greatcircledist(tgtsitevec, cut_r) * 1000.
	
	//target site position at desired meco
	LOCAL shifted_tgtsitevec IS RTLS_shifted_tgt_site_vector(dmbo_t).
	
	SET target_orbit TO LEXICON(
							"mode", 5,
							"normal", normvec,
							"radius", cut_r,
							"velocity", 2200,
							"fpa", cutoff_fpa,
							"rtls_cutv", 2200,
							"cutoff alt", cutoff_alt,
							"rtheta", rng,
							"rtls_tgt", shifted_tgtsitevec
	).
	
	//abort control lexicon
	
	GLOBAL RTLSAbort IS LEXICON (
								"t_abort", surfacestate["MET"],
								"theta_C1", theta,
								"C1",rtlsC1v,
								"Tc",0,
								"pitcharound",LEXICON(
													"refvec",normvec,
													"dt", RTLS_pitchover_t(rtlsC1v, rtls_pitcharound_tgtv),
													"triggered",flyback_immediate,
													"complete",FALSE,
													"target", rtls_pitcharound_tgtv
													
												),
								"flyback_range_lockout", 100,
								"flyback_iter",-2,
								"flyback_conv",-2,
								"flyback_flag", FALSE
	).
	
	//the theta angle requires the velocity at abort init 
	//if one engine out use the greater of the failure time or the srb staging time
	//if zero engine out use the srb time 
	
	
	LOCAL theta IS RTLS_dissip_theta_pert(abort_modes["abort_v"], vehiclestate["srb_sep"]["v"], vehiclestate["srb_sep"]["alt"], get_stage()["engines"]["thrust"] ).
	
	LOCAL rtlsC1v IS  RTLS_C1(theta).	
	LOCAL rtls_pitcharound_tgtv IS rodrigues(
											rtlsC1v,
											normvec,
											2*VANG(rtlsC1v, curR)
											).
	
	//prepare upfg
	SET upfgInternal TO resetUPFG().
	
	SET upfgInternal["terminal_time"] TO 15.
	SET upfgInternal["tgo_conv"] TO 2.
	SET upfgInternal["throtset"] TO 0.96 * throt_val.
	
	IF (flyback_immediate) {
		addGUIMessage("IMMEDIATE POWERED PITCH-AROUND").
	}
	
	//signal to the rest of the program that rtls is in progress
	lexx.
}



//	TAL functions


//give the position of a TAL site, returns a corrected position within a set crossrange distance
//from the abeam position on the current orbital plane

FUNCTION TAL_site_xrange_shift {
	PARAMETER tal_site_vec.
	PARAMETER current_normal.
	
	LOCAL tal_site_proj IS VXCL(current_normal,tal_site_vec).
	
	//now find the plane containing both the shifted site and its projection onto the orbital plane 
	LOCAL abeam_norm IS VCRS(tal_site_vec:NORMALIZED,tal_site_proj:NORMALIZED):NORMALIZED.
	
	//get the angle between the two
	//clamp this angle so that the crossrange is within a certain value
	LOCAL max_xrange IS 850.	//in km
	LOCAL abeam_angle IS signed_angle(tal_site_vec,tal_site_proj,abeam_norm,0).
	SET abeam_angle TO SIGN(abeam_angle)*CLAMP(ABS(abeam_angle),0,max_xrange*1000*180/(CONSTANT:PI*BODY:RADIUS)).
	
	LOCAL tgtvec IS rodrigues(tal_site_vec,abeam_norm,abeam_angle):NORMALIZED*BODY:RADIUS.
	
	//clearvecdraws().
	//arrow_body(vecYZ(tal_site_vec * 2),"tal_site_vec").
	//arrow_body(vecYZ(tal_site_proj * 2),"tal_site_proj").
	//arrow_body(vecYZ(tgtvec * 2),"tgtvec").

	RETURN tgtvec.
}

function tal_site_meco_velocity {
	PARAMETER sname.
	PARAMETER current_normal.

	LOCAL site IS ldgsiteslex[sname].
			
	local rwypos is 0.
	
	IF (site:ISTYPE("LEXICON")) {
		set rwypos to site["position"].
	} ELSE IF (site:ISTYPE("LIST")) {
		set rwypos to site[0]["position"].
	}
	
	LOCAL sitepos IS vecYZ(pos2vec(rwypos)).
	
	LOCAL site_plane IS VXCL(current_normal,sitepos).
	
	//shift ahead by half an hour
	LOCAL sitepos_shifted IS vecYZ(pos2vec(shift_pos(vecYZ(sitepos),-1800))).
	
	//correct shifted site within cossrange
	LOCAL sitepos_candidate IS TAL_site_xrange_shift(sitepos_shifted,current_normal).
	
	LOCAL site_normal IS - VCRS(orbitstate["radius"], sitepos_candidate):NORMALIZED.
	
	//estimate deltav to curve velocity to point to the target site
	LOCAL tgtMECOvel IS 7250.
	LOCAL cutoffVel IS VCRS(orbitstate["radius"],site_normal):NORMALIZED*tgtMECOvel.
	
	return cutoffVel.
}



//find new normal vector for TAL targeting
//find the plane containing the current pos vector ( in UPFG coords!!) and
//the target vector
FUNCTION TAL_normal {
	parameter cur_pos.
	
	//construct the plane of tal
	LOCAL tgtnorm IS VCRS(cur_pos, abort_modes["tal_tgt_site"]["position"]):NORMALIZED.
	
	RETURN tgtnorm.
}


FUNCTION TAL_tgt_vec {
	parameter tal_site.
	PARAMETER posvec.
	
	//to calculate the target vector, find the landing site abeam position along the orbit
	//keeping in mind the Earth rotation
	//then correct the abeam position so that it's within a certain crossrange from the site
	
	//first, find the abeam position
	//guess the true anomaly of the abeam position on the target orbit
	//convert to time interval and shift the target site
	//project the shifted site on the orbit 
	//measure the signed angle between abeam and target
	//drive this angle to zero by iterating over eta
	//save the final shifted site position
	
	LOCAL current_normal IS currentNormal().
	
	
	LOCAL site IS ldgsiteslex[tal_site].
			
	local rwypos is 0.
	
	IF (site:ISTYPE("LEXICON")) {
		set rwypos to site["position"].
	} ELSE IF (site:ISTYPE("LIST")) {
		set rwypos to site[0]["position"].
	}
	
	LOCAL sitevec IS pos2vec(rwypos).

	//find the ttue anomaly of the current position based on the current orbital params
	LOCAL eta_cur IS (target_orbit["SMA"]*(1-target_orbit["ecc"]^2)/posvec:MAG - 1)/target_orbit["ecc"].
	SET eta_cur TO ARCCOS(limitarg(eta_cur)).
	
	LOCAL d_eta IS 30.
	LOCAL shifted_site IS v(0,0,0).
	LOCAL shifted_site_proj IS v(0,0,0).
	
	UNTIL FALSE {
	
		LOCAL eta_guess  IS fixangle(eta_cur + d_eta).
		
		LOCAL abeam_dt IS eta_to_dt(eta_guess,target_orbit["SMA"],target_orbit["ecc"]).
		SET abeam_dt TO abeam_dt - eta_to_dt(eta_cur,target_orbit["SMA"],target_orbit["ecc"]).
		LOCAL abeam_pos IS rodrigues(posvec,current_normal,d_eta).
		
		SET shifted_site TO vecYZ(pos2vec(shift_pos(sitevec,-abeam_dt))).
		
		SET shifted_site_proj TO VXCL(current_normal,shifted_site).
		
		LOCAL eta_error IS signed_angle(abeam_pos,shifted_site_proj,current_normal,0).
		
		IF (ABS(eta_error) < 0.01) {
			BREAK.
		}
		
		SET d_eta TO d_eta + eta_error.
	}
	
	local tal_xrange_tgtvec is TAL_site_xrange_shift(shifted_site,current_normal).
	
	//clearvecdraws().
	//arrow_body(vecYZ(tal_xrange_tgtvec * 2),"tal_tgt_vec").
	//arrow_body(sitevec * 2,"sitevec").
	//arrow_body(vecYZ(shifted_site * 2),"shifted_site").
	//arrow_body(vecYZ(current_normal*BODY:RADIUS * 2),"current_normal").

	RETURN tal_xrange_tgtvec.
}

//cutoff altitude is set as the apoapsis
//cutoff velocity is calculated so that the ballistic trajectory intersects the surface at the target vector
FUNCTION TAL_cutoff_params {
	PARAMETER tgt_orb.
	parameter cur_r.
	PARAMETER cutoff_r.

	SET tgt_orb["normal"] TO TAL_normal(cur_r).
	SET tgt_orb["Inclination"] TO VANG(-tgt_orb["normal"],v(0,0,1)).
	
	set tgt_orb["radius"] to cutoff_r.
	
	//shifts underground the radius of the ballistic impact point
	LOCAL radius_bias IS 200.	//in km
	
	
	LOCAL AP is cutoff_r:MAG.
	LOCAL tgt_vec_radius IS BODY:RADIUS - radius_bias*1000.
	SET tgt_orb["fpa"] TO 0.
	SET tgt_orb["eta"] TO 180.
	
	LOCAL tgt_eta IS 180 + signed_angle(tgt_orb["radius"], abort_modes["tal_tgt_site"]["position"], tgt_orb["normal"], 0).
	
	SET tgt_orb["ecc"] TO (AP - tgt_vec_radius)/(AP + tgt_vec_radius*COS(tgt_eta)).
	SET tgt_orb["SMA"] TO AP/(1 + tgt_orb["ecc"] ).
	SET tgt_orb["velocity"] TO orbit_alt_vel(AP, tgt_orb["SMA"]).
	SET tgt_orb["apoapsis"] TO (AP - BODY:RADIUS)/1000.
	SET tgt_orb["periapsis"] TO (2*tgt_orb["SMA"] - AP - BODY:RADIUS)/1000.
	
	clearvecdraws().
	arrow_body(vecYZ(cur_r * 2),"cur_r").
	arrow_body(vecYZ(cutoff_r * 2),"cutoff_r").
	arrow_body(vecYZ(abort_modes["tal_tgt_site"]["position"] * 2),"tgt_vec").
	arrow_body(vecYZ(tgt_orb["normal"]*BODY:RADIUS * 2),"normal").
	
	RETURN tgt_orb.
}



FUNCTION setup_TAL{

	local engines_out is get_engines_out().
	
	local selected_tal_site is "".
	local tal_tgt_pos is 0.
	
	//detect if we alreay initialised the site 
	if (abort_modes["tal_tgt_site"] = "") {
	
		if (engines_out <2) {
			//if manual abort use the gui selection 
			//if auto choose at random between the 1eo sites
			
			if (abort_modes["manual_abort"]) {
				set selected_tal_site to get_gui_tal_site().
			} else {
				set selected_tal_site to select_rand(abort_modes["1eo_tal_sites"])["site"].
			}
			
		} else {
			//if 2eo choose the least DV site of the 2eo sites
			local tal_best_sdv is lexicon(
									"site", "", 
									"deltav", -10000000000
			).
			
			for sdv in abort_modes["2eo_tal_sites"] {
				if (tal_best_sdv["deltav"] < sdv["deltav"]) {
					set tal_best_sdv to sdv.
				}
			}
			
			set selected_tal_site to tal_best_sdv["site"].
		}
		
		local tal_tgtvec_ is TAL_tgt_vec(selected_tal_site, orbitstate["radius"]).
		
		set abort_modes["tal_tgt_site"] to lexicon(
												"site", selected_tal_site,
												"position", tal_tgtvec_
		
		).	
	}
	
	
	addGUIMessage("SELECTED TAL SITE IS " + abort_modes["tal_tgt_site"]["site"]).
	
	SET target_orbit["mode"] TO 6.
	SET target_orbit["cutoff alt"] TO CLAMP(0.98 * target_orbit["cutoff alt"], 110, 130).		//force cutoff alt 
	SET target_orbit["apoapsis"] TO target_orbit["cutoff alt"].
	
	SET target_orbit TO TAL_cutoff_params(target_orbit, orbitstate["radius"], target_orbit["radius"]).
	
	SET upfgInternal["s_init"] TO FALSE.
	
	ascent_gui_set_cutv_indicator(target_orbit["velocity"]).
	
	set upfgInternal["throtset"] to get_stage()["Throttle"].
	
	local two_engout is (engines_out > 1).
	
	start_oms_dump(two_engout).

	//trigger the roll to heads-up if it hasn't already, important for reentry 
	WHEN ( surfacestate["MET"] > (abort_modes["trigger_t"] + 40) ) THEN {
		roll_heads_up().
	}

}


// ATO functions


function get_ato_tgt_orbit {
	
	local ato_apoapsis is MIN(200, 0.8*target_orbit["apoapsis"]).
	
	local ato_cutoff_alt is 0.98 * target_orbit["cutoff alt"].
	local ato_cutoff_radius is (ato_cutoff_alt * 1000 + SHIP:BODY:RADIUS).
	
	//230 m/s burn to circularise at apoapsis
	local ato_ap_v is orbit_alt_vsat(ato_cutoff_radius) - 180.
	
	local ato_sma is 2/ato_cutoff_radius - ato_ap_v^2/BODY:MU.
	set ato_sma to 1/ato_sma.
	
	local ato_periapsis is (ato_sma - SHIP:BODY:RADIUS)/500 - ato_apoapsis.
	
	local sma_ is orbit_appe_sma(ato_apoapsis, ato_periapsis).
	local ecc_ is orbit_appe_ecc(ato_apoapsis, ato_periapsis).
	
	local eta_ is orbit_alt_eta(ato_cutoff_radius, sma_, ecc_).
	
	local ato_cutoff_vel is orbit_alt_vel(ato_cutoff_radius, sma_).
	
	local fpa_ is orbit_eta_fpa(eta_, sma_, ecc_).
	
	local normvec is currentNormal().
	local inclination_ is VANG(normvec,v(0,0,1)).
	
	local velvec is cutoff_velocity_vector(
										orbitstate["radius"],
										- normvec,
										ato_cutoff_vel,
										fpa_
	).
	
	return lexicon(
					"apoapsis", ato_apoapsis,
					"periapsis", ato_periapsis,
					"cutoff_alt", ato_cutoff_alt,
					"radius", ato_cutoff_radius,
					"SMA", sma_,
					"ecc", ecc_,
					"eta", eta_,
					"velocity", ato_cutoff_vel,
					"fpa", fpa_,
					"normvec", normvec,
					"Inclination", inclination_,
					"velvec", velvec
	).

}


FUNCTION setup_ATO {
	
	local ato_tgt_orbit is get_ato_tgt_orbit().
	
	//UPFG mode is the nominal one, only change MECO targets
	//lower apoapsis (not too low)
	SET target_orbit["apoapsis"] TO ato_tgt_orbit["apoapsis"].
	SET target_orbit["periapsis"] TO ato_tgt_orbit["periapsis"].
	//lower cutoff altitude
	SET target_orbit["cutoff alt"] TO ato_tgt_orbit["cutoff_alt"].
	SET target_orbit["radius"] TO ato_tgt_orbit["radius"].
	//variable iy steering 
	SET target_orbit["normal"] TO ato_tgt_orbit["normvec"].
	SET target_orbit["Inclination"] TO VANG(target_orbit["normal"],v(0,0,1)).
	
	//force cutoff altitude, free true anomaly
	SET target_orbit["mode"] TO 7.
	
	
	SET target_orbit["SMA"] TO ato_tgt_orbit["SMA"].
	SET target_orbit["ecc"] TO ato_tgt_orbit["ecc"].

	set target_orbit["eta"] to ato_tgt_orbit["eta"].
	
	set target_orbit["velocity"] to ato_tgt_orbit["velocity"].
	
	set target_orbit["fpa"] to ato_tgt_orbit["fpa"].
	
	SET upfgInternal["s_init"] TO FALSE.
	
	ascent_gui_set_cutv_indicator(target_orbit["velocity"]).
	
	local engines_out is get_engines_out().
	
	set upfgInternal["throtset"] to get_stage()["Throttle"].
	
	//no oms dump for ato/aoa
}





//		CONTINGENCY functions

function contingency_2eo_blue_boundary {
	
	local boundary_hdot is 220.7 + 6.583 * ABS(target_orbit["inclination"]) .

	return (SHIP:VERTICALSPEED >= boundary_hdot).
}