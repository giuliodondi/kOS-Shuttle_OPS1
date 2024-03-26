//main abort lexicon
//abort boundaries are defined with velocity in their own functions
GLOBAL abort_modes IS LEXICON( 
					"triggered",FALSE,
					"t_abort_true",0,
					"t_abort",0,
					"abort_v",0,
					"staging",LEXICON(
							"v",0,
							"alt",0
							),
					"oms_dump",FALSE,
					"rtls_site", "",
					"tal_candidates", "",
					"ecal_candidates", ""
							
).


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


function abort_region_determinator {

	clearscreen.
	
	local three_eng_lex is build_engines_lex(3, 0).
	local two_eng_lex is build_engines_lex(2, 0).
	local one_eng_lex is build_engines_lex(1, 0).
	
	local three_eng_perf is veh_perf_estimator(three_eng_lex).
	local two_eng_perf is veh_perf_estimator(two_eng_lex).
	local one_eng_perf is veh_perf_estimator(one_eng_lex).
	
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
	
	local one_eng_meco_dv_excess is estimate_excess_deltav(
											orbitstate["radius"],
											orbitstate["velocity"],
											meco_dv,
											one_eng_perf
	
	).
	
	print "meco 2e " + two_eng_meco_dv_excess at (0,1).
	print "meco 1e " + one_eng_meco_dv_excess at (0,2).
	
	local ato_tgt_orbit is get_ato_tgt_orbit().
	
	LOCAL ato_dv IS (ato_tgt_orbit["velvec"] - orbitstate["velocity"]).
		
	local two_eng_ato_dv_excess is estimate_excess_deltav(
											orbitstate["radius"],
											orbitstate["velocity"],
											ato_dv,
											two_eng_perf
	
	).
	
	local one_eng_ato_dv_excess is estimate_excess_deltav(
											orbitstate["radius"],
											orbitstate["velocity"],
											ato_dv,
											one_eng_perf
	
	).
	
	print "ato 2e " + two_eng_ato_dv_excess at (0,4).
	print "ato 1e " + one_eng_ato_dv_excess at (0,5).
	
	LOCAL tal_2e_dv is lexicon(). 
	
	
	LOCAL current_normal IS currentNormal().
	
	local line is 7.
	
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
		
		print s + " " + two_eng_tal_dv_excess at (0,line).
		set line to line + 1.
		
		tal_2e_dv:add(s, two_eng_tal_dv_excess).
	}
	

}


//	RTLS functions


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
	
	local ato_apoapsis is MIN(150, 0.8*target_orbit["apoapsis"]).
	local ato_periapsis is 0.4 * target_orbit["periapsis"].
	local ato_cutoff_alt is 0.985*target_orbit["cutoff alt"].
	
	local ato_cutoff_radius is (ato_cutoff_alt * 1000 + SHIP:BODY:RADIUS).
	
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

