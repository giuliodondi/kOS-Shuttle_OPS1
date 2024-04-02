//main abort lexicon
//abort boundaries are defined with velocity in their own functions
GLOBAL abort_modes IS LEXICON( 
					"triggered",FALSE,
					"trigger_t",time:seconds + 10000000000,
					"rtls_active", false,
					"tal_active", false,
					"ato_active", false,
					"cont_active", false,
					"ssmes_out", list(),
					"oms_dump",FALSE,
					"rtls_site", "",
					"tal_candidates", "",
					"ecal_candidates", "",
					"intact_modes", lexicon(
												"1eo", lexicon(
															"rtls", true,	// default mode at liftoff
															"tal", false,
															"ato", false,
															"meco", false
													),
												"2eo", lexicon(
															"droop", false,
															"tal", false,
															"ato", false,
															"meco", false
													)
									)
					"2eo_cont_mode", "",
					"3eo_cont_mode", ""
							
).

function abort_triggered {
	return abort_modes["rtls_active"] or abort_modes["tal_active"] or abort_modes["ato_active"] or abort_modes["cont_active"].
}


function initialise_abort_sites {

	set abort_modes["rtls_tgt"] to get_RTLS_site().
	
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
	
	//abort_initialiser().

}

//determine available intact abort modes for 1,2 eng out cases
function intact_abort_region_determinator {

	//intact modes are frozen if a contingency is active 
	if (abort_modes["cont_active"]) {
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
	

	local 1eo_available_modes is list().
	local 2eo_available_modes is list().
	local 3eo_available_modes is list().
	
	local two_eng_lex is build_engines_lex(2).
	local one_eng_lex is build_engines_lex(1).
	
	local two_eng_perf is veh_perf_estimator(two_eng_lex).
	local one_eng_perf is veh_perf_estimator(one_eng_lex).
	
	local meco_vel is cutoff_velocity_vector(
										orbitstate["radius"],
										-target_orbit["normal"],
										target_orbit["velocity"],
										target_orbit["fpa"]
	).
	
	LOCAL meco_dv IS (meco_vel - orbitstate["velocity"]).
	
	local ato_tgt_orbit is get_ato_tgt_orbit().
	
	LOCAL ato_dv IS (ato_tgt_orbit["velvec"] - orbitstate["velocity"]).
	
	local two_eng_meco_dv_excess is estimate_excess_deltav(
											orbitstate["radius"],
											orbitstate["velocity"],
											meco_dv,
											two_eng_perf
	
	).
	
	local two_eng_ato_dv_excess is estimate_excess_deltav(
											orbitstate["radius"],
											orbitstate["velocity"],
											ato_dv,
											two_eng_perf
	
	).
	
	if (two_eng_meco_dv_excess > 0) {
		if (not abort_modes["intact_modes"]["1eo"]["meco"]) {
			addGUIMessage("PRESS TO MECO").
			set abort_modes["intact_modes"]["1eo"]["meco"] to true.
			set abort_modes["intact_modes"]["1eo"]["ato"] to false.
		}
	} else if (two_eng_ato_dv_excess > 0) {
		if (not abort_modes["intact_modes"]["1eo"]["ato"]) {
			addGUIMessage("PRESS TO ATO").
			set abort_modes["intact_modes"]["1eo"]["ato"] to true.
		}
	}
	
	local one_eng_meco_dv_excess is estimate_excess_deltav(
											orbitstate["radius"],
											orbitstate["velocity"],
											meco_dv,
											one_eng_perf
	
	).
	
	local one_eng_ato_dv_excess is estimate_excess_deltav(
											orbitstate["radius"],
											orbitstate["velocity"],
											ato_dv,
											one_eng_perf
	
	).
	
	if (one_eng_meco_dv_excess > 0) {
		if (not abort_modes["intact_modes"]["2eo"]["meco"]) {
			addGUIMessage("SINGLE ENGINE PRESS TO MECO").
			set abort_modes["intact_modes"]["2eo"]["meco"] to true.
			set abort_modes["intact_modes"]["2eo"]["ato"] to false.
		}
	} else if (one_eng_ato_dv_excess > 0) {
		if (not abort_modes["intact_modes"]["2eo"]["ato"]) {
			addGUIMessage("SINGLE ENGINE PRESS TO ATO").
			set abort_modes["intact_modes"]["2eo"]["ato"] to true.
		}
	}
	
	if (abort_modes["intact_modes"]["2eo"]["ato"]) {
		// after single engine p2ato the only cases are at least an ato or a 3 engine out abort, no need for tal beyond this
		set abort_modes["intact_modes"]["1eo"]["tal"] to false.
		set abort_modes["intact_modes"]["2eo"]["tal"] to false.
	} else {
	
		LOCAL tal_2e_dv is list(). 
		LOCAL tal_1e_dv is list(). 
		
		local 2eng_best_tal is lexicon("site", "", "deltav", -10000000000).
		local 1eng_best_tal is lexicon("site", "", "deltav", -10000000000).
		
		LOCAL current_normal IS currentNormal().
		
		for s in abort_modes["tal_candidates"] {
			LOCAL site IS ldgsiteslex[s].
			
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
			LOCAL dv2site IS (cutoffVel - orbitstate["velocity"]).
			
			local two_eng_tal_dv_excess is estimate_excess_deltav(
													orbitstate["radius"],
													orbitstate["velocity"],
													dv2site,
													two_eng_perf
			).
			
			local 2eng_tal_dv is lexicon(
										"site", s, 
										"deltav", two_eng_tal_dv_excess
			).
			
			if (two_eng_tal_dv_excess > 0) {
				tal_2e_dv:add(2eng_tal_dv).
			}
			
			if (two_eng_tal_dv_excess > 2eng_best_tal["deltav"]) {
				set 2eng_best_tal to 2eng_tal_dv.
			}
			
			
			local one_eng_tal_dv_excess is estimate_excess_deltav(
													orbitstate["radius"],
													orbitstate["velocity"],
													dv2site,
													one_eng_perf
			).
			
			local 1eng_tal_dv is lexicon(
										"site", s, 
										"deltav", one_eng_tal_dv_excess
			).
			
			if (one_eng_tal_dv_excess > 0) {
				tal_1e_dv:add(1eng_tal_dv).
			}
			
			if (one_eng_tal_dv_excess > 1eng_best_tal["deltav"]) {
				set 1eng_best_tal to 1eng_tal_dv.
			}
		}
		
		if (not abort_modes["intact_modes"]["1eo"]["tal"]) {
			if (NOT abort_modes["intact_modes"]["1eo"]["rtls"]) {
				// if we're not tal by negative return, force tal mode with the least bad site 
				if (tal_2e_dv:size = 0) {
					tal_2e_dv:add(2eng_best_tal).
				}
				
			}
			
			if (tal_2e_dv:size > 0) {
				addGUIMessage("TWO ENGINE TAL").
				set abort_modes["intact_modes"]["1eo"]["tal"] to true.
			}
		}
		
		if (not abort_modes["intact_modes"]["2eo"]["tal"]) {
			if (tal_1e_dv:size > 0) { 
				addGUIMessage("SINGLE ENGINE TAL").
				set abort_modes["intact_modes"]["2eo"]["tal"] to true.
			}
		}
	
	}
	
}

function contingency_abort_region_determinator {
	
	if (abort_modes["rtls_active"]) {
		//rtls contingency modes 
	} else {
	
		//droop boundary missing
		if (contingency_2eo_blue_boundary()) {
			set abort_modes["2eo_cont_mode"] to "green".
		} else {
			set abort_modes["2eo_cont_mode"] to "blue".
		}
		
		if (orbitstate["velocity"]:MAG >= 6700) {
			set abort_modes["3eo_cont_mode"] to "blank". 
		} else if (vehiclestate["major_mode"] < 103) {
			set abort_modes["3eo_cont_mode"] to "blue". 
		} else {
			set abort_modes["3eo_cont_mode"] to "green". 
		}
	
	}

}

//determine if it's time to initialise an abort and them ode to activate
function abort_initialiser {

	//may be wrong, I believe there is no case where a contingency abort is downmoded to something else
	if (abort_modes["cont_active"]) {
		return.
	}

	local engines_out is get_engines_out().
	
	local zero_engout is (engines_out = 0).
	local one_engout is (engines_out = 1).
	local two_engout is (engines_out = 2).
	local three_engout is (engines_out = 3).
	
	//deal with first stage 
	if (vehiclestate["major_mode"] < 103) {
	
		if (three_engout) {
			
		}
		
		return.
	}
	
	if (vehicle["ssme_out_detected"]) {
		//setup the abort trigger in the future
		set vehicle["ssme_out_detected"] to false.
		
		//the time in the future is 1 second by default except if there are multiple modes available
		//which should only happen for 1eng out , for 0eng out the abort is triggered manually outside
		//otoh we can only enter this block if we have at least 1 eng out
		local abort_trigger_t is 1.
		
		if (abort_modes["available_modes"][engines_out]:length > 1) {
			set abort_trigger_t to 10.
		}
		
		set abort_modes["trigger_t"]	TO surfacestate["MET"] + abort_trigger_t.
		
	}
	
	//exit if we're not past the abort trigger 
	if (surfacestate["MET"] < abort_modes["trigger_t"]) {
		return.
	} else {
		set abort_modes["triggered"] to true.
	}
	
	//oms dump logic decided on a mode-by-mode basis
	

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

//to be called before launch, will fin the closest landing site 
//to the launchpad
FUNCTION get_RTLS_site {
	LOCAL closest_out IS get_closest_site(ldgsiteslex).
	RETURN closest_out[1].
}


FUNCTION RTLS_boundary_vel {
	RETURN (2880 - 7.8 * ABS(target_orbit["inclination"])).
}

FUNCTION RTLS_boundary{
	
	LOCAL dwnrg_speed IS current_horiz_dwnrg_speed(SHIP:GEOPOSITION, SHIP:VELOCITY:ORBIT).
	
	LOCAL boundary_vel IS RTLS_boundary_vel().

	RETURN (dwnrg_speed > RTLS_boundary_vel()).
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
	//arrow_body(vecYZ(tal_site_vec),"tal_site_vec").
	//arrow_body(vecYZ(tal_site_proj),"tal_site_proj").
	//arrow_body(vecYZ(tgtvec),"tgtvec").

	RETURN tgtvec.
}




// ATO functions


function get_ato_tgt_orbit {
	
	local ato_apoapsis is MIN(160, 0.8*target_orbit["apoapsis"]).
	
	local ato_cutoff_alt is 0.95 * target_orbit["cutoff alt"].
	local ato_cutoff_radius is (ato_cutoff_alt * 1000 + SHIP:BODY:RADIUS).
	
	//300 m/s burn to circularise at apoapsis
	local ato_ap_v is orbit_alt_vsat(ato_cutoff_radius) - 300.
	
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
										normvec,
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




//		CONTINGENCY functions

function contingency_2eo_blue_boundary {
	
	local boundary_hdot is 220.7 + 6.583 * ABS(target_orbit["inclination"]) .

	return (SHIP:VERTICALSPEED >= boundary_hdot).
}