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
					"RTLS",LEXICON(
							"active",TRUE,
							"triggered",FALSE,
							"tgt_site", get_RTLS_site()
							),
					"TAL",LEXICON(
							"active",FALSE,
							"triggered",FALSE,
							"tgt_site", LATLNG(0,0)
							),
					"ATO",LEXICON(
							"active",FALSE,
							"triggered",FALSE
							),
					"MECO",LEXICON(
							"active",FALSE,
							"triggered",FALSE
							)
							
).


//abort monitor and trigger 

FUNCTION monitor_abort {

	LOCAL current_t IS TIME:SECONDS - vehicle["ign_t"].
	
	LOCAL abort_detect IS SSME_out().
	
	IF abort_detect {
		addGUIMessage("ENGINE OUT DETECTED.").
		SET abort_modes["triggered"] TO TRUE.
		SET abort_modes["t_abort_true"] TO current_t.
		SET abort_modes["t_abort"] TO MAX( current_t + 1, vehicle["handover"]["time"] + 2 ).
		SET abort_modes["abort_v"] TO SHIP:VELOCITY:SURFACE:MAG.
		SET vehicle["maxThrottle"] TO 1.
		SET vehicle["stages"][vehiclestate["cur_stg"]]["Throttle"] TO 1.
	}


	IF (NOT abort_modes["triggered"]) {
		//set the correct abort mode to active and print information
		
		IF abort_modes["RTLS"]["active"] {
			IF RTLS_boundary() {
				SET abort_modes["RTLS"]["active"] TO FALSE.
				SET abort_modes["TAL"]["active"] TO TRUE.
				addGUIMessage("NEGATIVE RETURN").
			}
		} ELSE IF abort_modes["TAL"]["active"] {
			IF TAL_boundary() {
				SET abort_modes["TAL"]["active"] TO FALSE.
				SET abort_modes["ATO"]["active"] TO TRUE.
				addGUIMessage("PRESS TO ATO.").
			}
		} ELSE IF abort_modes["ATO"]["active"] {
			IF ATO_boundary() {
				SET abort_modes["ATO"]["active"] TO FALSE.
				SET abort_modes["MECO"]["active"] TO TRUE.
				addGUIMessage("PRESS TO MECO.").
			}
		}
	
	} ELSE {
		//check if conditions are right to setup the abort
		IF (abort_modes["RTLS"]["active"] ) {
			IF abort_detect {
				addGUIMessage("ABORT RTLS AT "+sectotime(abort_modes["t_abort"])).
				SET abort_modes["RTLS"]["triggered"] TO TRUE.
			}
		
			//need to check the time becase we wait for second stage for RTLS
			IF ( current_t >= abort_modes["t_abort"] ) {
				SET abort_modes["RTLS"]["active"] TO FALSE.
				setup_RTLS().
			}
		} ELSE IF (abort_modes["TAL"]["active"] ) {
			IF abort_detect {
				addGUIMessage("ABORT TAL AT "+sectotime(abort_modes["t_abort"])).
				SET abort_modes["TAL"]["triggered"] TO TRUE.
			}
			//no need to check the time for TAL
			SET abort_modes["TAL"]["active"] TO FALSE.
			setup_TAL().
		}  ELSE IF (abort_modes["ATO"]["active"] ) {
			IF abort_detect {
				addGUIMessage("ABORT ATO / AOA AT "+sectotime(abort_modes["t_abort"])).
				SET abort_modes["ATO"]["triggered"] TO TRUE.
			}
			//no need to check the time for ATO / AOA
			SET abort_modes["ATO"]["active"] TO FALSE.
			setup_ATO().
		
		}  ELSE IF (abort_modes["MECO"]["active"] ) {
			IF abort_detect {
				addGUIMessage("PRESSING TO MECO").
			}

			SET abort_modes["MECO"]["active"] TO FALSE.
			setup_MECO_ENGOUT().
		
		}
	
	}

}







//		RTLS FUNCTIONS 


//bias first-stage trajectory scale for lofting
FUNCTION RTLS_first_stage_lofting_bias {
	PARAMETER abort_t.
	
	RETURN 0.33*(1 - abort_t/122).
	
}


//to be called before launch, will fin the closest landing site 
//to the launchpad
FUNCTION get_RTLS_site {
	LOCAL closest_out IS get_closest_site(ldgsiteslex).
	RETURN closest_out[1].
}


//RTLs runway vector in UPFG coordinates
FUNCTION RTLS_tgt_site_vector {
	RETURN vecYZ(pos2vec(abort_modes["RTLS"]["tgt_site"])).
}

//takes in a dt parameter, positive values shift the target EAST
FUNCTION RTLS_shifted_tgt_site_vector {
	PARAMETER dt.

	LOCAL pos_earth_fixed IS pos2vec(abort_modes["RTLS"]["tgt_site"]).
	
	LOCAL shifted_pos IS R(0, BODY:angularvel:mag * (-dt)* constant:RadToDeg, 0)*pos_earth_fixed.
	
	RETURN vecYZ(shifted_pos).
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

	LOCAL pos IS vecYZ(-SHIP:ORBIT:BODy:POSITION).
	LOCAL ve IS vecYZ(SHIP:VELOCITY:SURFACE).

	LOCAL normvec IS -VCRS(pos, ve):NORMALIZED.
	
	LOCAL horiz IS VCRS(pos,normvec):NORMALIZED.
	LOCAL C1 IS rodrigues(horiz, normvec, theta).
	
	RETURN C1.
}

FUNCTION RTLS_pitchover_t {
	PARAMETER c1_vec.
	PARAMETER pitcharound_vec.
	
	LOCAL pitchover_rate IS 20.		//degrees per second 
	
	RETURN VANG(pitcharound_vec, c1_vec)/pitchover_rate.
}

//RV-line, return the coefficients, 0 is linear (DISTANCE IN METRES) and 1 is constant
FUNCTION RTLS_rvline_coefs {
	//RETURN LIST(0.00242, 768.72).		//this is the original from the rtls traj-shaping paper 
	//RETURN LIST(0.0031, 559.49).		//this was lifted from the sts-1 abort analysis paper 
	RETURN LIST(0.004, 334).
}

//calculate the desired RV-line velocity  (DISTANCE IN METRES)
FUNCTION RTLS_rvline {
	PARAMETER rng.
	LOCAL coefs IS RTLS_rvline_coefs().
	RETURN coefs[0] * rng + coefs[1].
}

FUNCTION RTLS_burnout_mass {

	//add 1% of ssme fuel and 20% of oms fuel
	//this NEEDs to be called after we update the final mass with the right value
	SET vehicle["mbod"] TO vehicle["stages"][vehicle["stages"]:LENGTH - 1]["m_final"] + 0.01 * vehicle["SSME_prop_0"] + 0.2 * vehicle["OMS_prop_0"].
}

FUNCTION RTLS_boundary_vel {
	RETURN (2930 - 7.8 * ABS(target_orbit["inclination"])).
}

FUNCTION RTLS_boundary{
	
	LOCAL dwnrg_speed IS current_horiz_dwnrg_speed(SHIP:GEOPOSITION, SHIP:VELOCITY:ORBIT).
	
	LOCAL boundary_vel IS RTLS_boundary_vel().

	RETURN dwnrg_speed > RTLS_boundary_vel().
}

//compare current velocity with negative return boundary to see if we should flyback immediately
FUNCTION RTLS_immediate_flyback {
	
	LOCAL dwnrg_speed IS current_horiz_dwnrg_speed(SHIP:GEOPOSITION, SHIP:VELOCITY:ORBIT).

	RETURN dwnrg_speed > RTLS_boundary_vel() - 80.
}

FUNCTION setup_RTLS {

	IF (DEFINED RTLSAbort) {
		RETURN.
	}
	
	//vehicle stuff immediately so we can measure the running oms
	start_oms_dump().
	set_steering_low().
	
	//do it immediately so it's ready when the gui first wants to update it 
	make_rtls_traj2_disp().
	
	//redefine vehicle 
	
	//save the current engine lex (no oms)
	LOCAL cur_stg IS get_stage().
	LOCAL ssme_only_englex IS cur_stg["engines"].
	
	measure_update_engines().
	
	SET vehicle["stages"] TO vehicle["stages"]:SUBLIST(0,3).
	
	SET vehicle["stages"][2]["staging"]["type"] TO "depletion".
	SET vehicle["stages"][2]["mode"] TO 1.
	SET vehicle["stages"][2]["Throttle"] TO 1.
	vehicle["stages"][2]:REMOVE("glim").
	SET vehicle["stages"][2]["engines"] TO build_engines_lex().
	
	LOCAL current_m IS SHIP:MASS*1000.
	local res_left IS get_shuttle_res_left().
	
	update_stage2(current_m, res_left).

	vehicle:ADD("mbod",0).
	
	RTLS_burnout_mass().		

	//time to desired burnout mass
	LOCAL dmbo_t IS (vehicle["stages"][2]["m_initial"] - vehicle["mbod"]) * vehicle["stages"][2]["Tstage"]/vehicle["stages"][2]["m_burn"].
	
	//so that downrange distance calculations are correct
	SET launchpad TO abort_modes["RTLS"]["tgt_site"].
	
	LOCAL t_abort IS TIME:SECONDS.
	
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
	LOCAL normvec IS -VCRS(vecYZ(-SHIP:ORBIT:BODy:POSITION), vecYZ(SHIP:VELOCITY:SURFACE)):NORMALIZED.
	
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
	
	LOCAL theta IS RTLS_dissip_theta_pert(abort_modes["abort_v"], abort_modes["staging"]["v"], abort_modes["staging"]["alt"], get_stage()["engines"]["thrust"] ).
	
	LOCAL rtlsC1v IS  RTLS_C1(theta).	
	LOCAL rtls_pitcharound_tgtv IS rodrigues(
											rtlsC1v,
											normvec,
											2*VANG(rtlsC1v, curR)
											).
	LOCAL lexx IS  LEXICON (
								"t_abort",t_abort,
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
	
	//prepare upfg
	SET upfgInternal TO resetUPFG().
	
	SET upfgInternal["terminal_time"] TO 15.
	SET upfgInternal["tgo_conv"] TO 2.
	SET upfgInternal["throtset"] TO 0.96.
	
	IF (flyback_immediate) {
		addGUIMessage("IMMEDIATE POWERED PITCH-AROUND").
	}
	
	//signal to the rest of the program that rtls is in progress
	GLOBAL RTLSAbort IS lexx.
}


	// GRTLS



FUNCTION set_target_nz {
	
	IF (NZHOLD["cur_nz"] >= 1) {
		SET NZHOLD["tgt_nz"] TO 0.7 + ABS(NZHOLD["cur_hdot"]/655.32).
		SET NZHOLD["tgt_nz"] TO MAX(2.5,MIN( 3.9, NZHOLD["tgt_nz"])).
	}

}


FUNCTION nz_update_pitch {
	PARAMETER cur_pch.
	PARAMETER delta_nz IS 0.


	LOCAL deltapch IS - (0.9 + delta_nz*2)*NZHOLD["dt"].
	SET cur_pch TO cur_pch + deltapch.
	
	 
	RETURN cur_pch.
}

FUNCTION GRTLS {

	CLEARSCREEN.
	RUNPATH("0:/Shuttle_entrysim/src/entry_utility").
	RUNPATH("0:/Shuttle_entrysim/src/approach_utility").
	RUNPATH("0:/Shuttle_entrysim/src/veh_control_utility").
	RUNPATH("0:/Shuttle_entrysim/VESSELS/DECQ_Shuttle_mono/aerosurfaces_control").
	
	STEERINGMANAGER:RESETPIDS().
	STEERINGMANAGER:RESETTODEFAULT().
	
	SET STEERINGMANAGER:MAXSTOPPINGTIME TO 3.


	//SET STEERINGMANAGER:PITCHTS TO 4.0.
	SET STEERINGMANAGER:YAWTS TO 3.
	SET STEERINGMANAGER:ROLLTS TO 3.

	IF (STEERINGMANAGER:PITCHPID:HASSUFFIX("epsilon")) {
		SET STEERINGMANAGER:PITCHPID:EPSILON TO 0.1.
		SET STEERINGMANAGER:YAWPID:EPSILON TO 0.1.
		SET STEERINGMANAGER:ROLLPID:EPSILON TO 0.1.
	}
	
	IF (DEFINED FLAPPID) {UNSET FLAPPID.}


	LOCAL pitch0 IS 40.
	LOCAL pitchf IS 10.
	LOCAL tgt_hdot IS -150.
	LOCAL firstroll IS 28.
	
	LOCAL t_grtls IS TIME:SECONDS.

	LOCAL aimvec IS SHIP:VELOCITY:SURFACE:NORMALIZED.
	LOCAL upvec IS -SHIP:ORBIT:BODY:POSITION:NORMALIZED.
	LOCAL rightvec IS VCRS(upvec, aimvec ).
	
	LOCAL aerosurfaces_control IS aerosurfaces_control_factory().
	
	reset_pids().
	
	aerosurfaces_control["set_aoa_feedback"](0).

	SET aimvec TO rodrigues(aimvec,rightvec,-pitch0).
	
	RCS ON.
	SAS OFF.
	SET vehiclestate["phase"] TO 5.
	LOCK STEERING TO LOOKDIRUP(aimvec, upvec).
	
	
	GLOBAL NZHOLD IS LEXICON(
							"cur_t",TIME:SECONDS,
							"dt",0,
							"tgt_nz",0,
							"cur_nz",0,
							"cur_hdot",SHIP:VERTICALSPEED,
							"cmd_pch",pitch0
	
	).
	
	LOCAL pitchv IS pitch0.
	LOCAL rollv IS 0.
	
	LOCAL P_att IS SHIP:FACING.
	
	WHEN (TIME:SECONDS > t_grtls + 15) THEN {
		LOCK STEERING TO P_att.
	}
	
	//dap controller object
	LOCAL dap IS dap_controller_factory().
	
	
	//run the control loop 
	//faster than the main loop 
	local control_loop is loop_executor_factory(
								0.3,
								{
									SET P_att TO dap:reentry_auto(rollv,pitchv).
								}
							).
	
	
	LOCAL deltanz IS  -  NZHOLD["tgt_nz"].
	
	//WHEN ( SHIP:ALTITUDE < 60000 OR (NZHOLD["cur_nz"] <0 AND NZHOLD["cur_nz"] > -2 AND SHIP:ALTITUDE < 70000) ) THEN {
	WHEN (SHIP:ALTITUDE < 65000) THEN {
		SET vehiclestate["phase"] TO 6.
	}
	
	

	LOCAL nz_decr IS 0.
	
	UNTIL FALSE {
	
		LOCAL prev_nz IS NZHOLD["cur_nz"].
		
		SET NZHOLD TO update_g_force(NZHOLD).
		
		IF (vehiclestate["phase"] >= 6 ) {
			flaptrim_control(TRUE, aerosurfaces_control).
			aerosurfaces_control["deflect"]().
		
			IF (NZHOLD["tgt_nz"] = 0) {
				set_target_nz().
			}
			
			SET deltanz TO NZHOLD["cur_nz"] -  NZHOLD["tgt_nz"].
			
			IF (vehiclestate["phase"]=6) {
				IF (NZHOLD["cur_nz"]>0) {
					//switch if we exceed the nz threshold
					IF (NZHOLD["tgt_nz"]>0 AND NZHOLD["cur_nz"] >= NZHOLD["tgt_nz"]) {
						SET vehiclestate["phase"] TO 7.
					} ELSE {
						//see if nz has peaked, if so register
						IF (prev_nz >= NZHOLD["cur_nz"]) {
							SET nz_decr TO nz_decr + 1.
							//three strikes and we switch modes
							IF nz_decr = 3 {
								SET vehiclestate["phase"] TO 7.
							}
						} ELSE {
							SET nz_decr TO 0.
						}
					}
				}
			} ELSE IF (vehiclestate["phase"] = 7 ) {
				LOCAL delta_nz IS MAX(0, NZHOLD["cur_nz"] - NZHOLD["tgt_nz"]).
				SET pitchv TO MAX(pitchf,MIN(pitch0, nz_update_pitch(pitchv, delta_nz))).
				SET NZHOLD["cmd_pch"] TO pitchv.
			}
			
			
			IF ((NZHOLD["cur_hdot"]> tgt_hdot ) AND (pitchv < pitchf*1.1)) {
				BREAK.
			}
			
		} ELSE {
			IF SHIP:ALTITUDE < 65000 {
				SET vehiclestate["phase"] TO 6.
			}
		}
		

		GRTLS_dataViz().
		WAIT 0.05.
	}
	
	
	//prepare entry guidance
	GLOBAL pitchprof_segments IS LIST(
								LIST(350,5),
								LIST(1800,20)
								).
	GLOBAL bypass_pitchprof_def IS TRUE.
								
	GLOBAL sim_end_conditions IS LEXICON(
						"altitude",12000,
						"surfvel",350,
						"range_bias",0
	).
	GLOBAL start_guid_flag IS TRUE.
	
	GLOBAL grtls_skip_to_TAEM IS TRUE.
	
	SET TERMINAL:WIDTH TO 50.
	SET TERMINAL:HEIGHT TO 30.
	
	SAS ON.
	UNLOCK STEERING.
	RUN "0:/entry".

}




//		TAL FUNCTIONS 




//hard-coded selector of TAL sites based on inclination
FUNCTION get_TAL_site {
	PARAMETER DVrem.

	LOCAL selectedSite IS "NONE".
	
	IF NOT (DEFINED TAL_site) {
		LOCAL current_normal IS currentNormal().
				
		LOCAL candidate_sites IS LIST().
		
		local i is 0.
		LOCAL min_dv_miss IS -1e20.
		FOR s in ldgsiteslex:KEYS {
			LOCAL site IS ldgsiteslex[s].
		
			LOCAL sitepos IS vecYZ(pos2vec(site["position"])).
			
			LOCAL site_plane IS VXCL(current_normal,sitepos).
			
			//first see if the candidate site is downrange
			IF (signed_angle(orbitstate["radius"],site_plane,-current_normal,0) > 0) {
				
				//shift ahead by half an hour
				LOCAL sitepos_shifted IS vecYZ(pos2vec(shift_pos(vecYZ(sitepos),-1800))).
				
				//correct shifted site within cossrange
				LOCAL sitepos_candidate IS TAL_site_xrange_shift(sitepos_shifted,current_normal).
				
				LOCAL site_normal IS - VCRS(orbitstate["radius"], sitepos_candidate):NORMALIZED.
				
				//estimate deltav to curve velocity to point to the target site
				LOCAL tgtMECOvel IS 7650.
				LOCAL cutoffVel IS VCRS(orbitstate["radius"],site_normal):NORMALIZED*tgtMECOvel.
				LOCAL dv2site IS (cutoffVel - orbitstate["velocity"]):MAG.
				
				//current remaining deltaV minus the estimate
				LOCAL dv_excess IS 0.9*DVrem - dv2site.
				
				IF dv_excess>0 {
					//if the excess deltav is positive this is a good candidate
					candidate_sites:ADD(s).
				} ELSE {
					//else, keep track of the "least bad" candidate site 
					//this is the fallback option if no good candidates are present
					IF (dv_excess > min_dv_miss) {
						SET min_dv_miss TO dv_excess.
						SET selectedSite TO s.
					}
				}
			}
		}
		
		IF (candidate_sites:LENGTH > 0) {
			GLOBAL TAL_site IS select_rand(candidate_sites).
		}
	}
	
	addGUIMessage("SELECTED TAL SITE IS " + TAL_site).
	RETURN ldgsiteslex[TAL_site]["position"].

}

//TAL target site vector in UPFG coordinates
FUNCTION TAL_tgt_site_vector {
	RETURN vecYZ(pos2vec(abort_modes["TAL"]["tgt_site"])).
}

//find new normal vector for TAL targeting
//find the plane containing the current pos vector ( in UPFG coords!!) and
//the target vector
FUNCTION TAL_normal {

	LOCAL cur_pos IS -vecYZ(SHIP:ORBIT:BODY:POSITION).
		
	//find the position vector of the target site
	
	//construct the plane of rtls
	LOCAL dr IS TALAbort["tgt_vec"] - cur_pos.
	LOCAL tgtnorm IS VCRS(TALAbort["tgt_vec"],dr):NORMALIZED.
	
	//clearvecdraws().
	//arrow_body(vecYZ(cur_pos),"cur_pos").
	//arrow_body(vecYZ(TALAbort["tgt_vec"]),"tgt_vec").
	//arrow_body(vecYZ(tgtnorm*BODY:RADIUS),"tgtnorm").
	
	RETURN tgtnorm.
}


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



FUNCTION TAL_tgt_vec {
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
	
	LOCAL sitevec IS TAL_tgt_site_vector().
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
		LOCAL abeam_pos IS rodrigues(posvec,-current_normal,d_eta).
		
		SET shifted_site TO vecYZ(pos2vec(shift_pos(vecYZ(sitevec),-abeam_dt))).
		
		SET shifted_site_proj TO VXCL(current_normal,shifted_site).
		
		LOCAL eta_error IS signed_angle(abeam_pos,shifted_site_proj,-current_normal,0).
		
		IF (ABS(eta_error) < 0.01) {
			BREAK.
		}
		
		SET d_eta TO d_eta + eta_error.
	}

	RETURN TAL_site_xrange_shift(shifted_site,current_normal).
}

//cutoff altitude is set as the apoapsis
//cutoff velocity is calculated so that the ballistic trajectory intersects the surface at the target vector
FUNCTION TAL_cutoff_params {
	PARAMETER tgt_orb.
	PARAMETER cutoff_r.

	SET tgt_orb["normal"] TO TAL_normal().
	SET tgt_orb["Inclination"] TO VANG(-tgt_orb["normal"],v(0,0,1)).
	
	set tgt_orb["radius"] to cutoff_r.
	
	//shifts underground the radius of the ballistic impact point
	LOCAL radius_bias IS 200.	//in km
	
	
	LOCAL AP is cutoff_r:MAG.
	LOCAL tgt_vec_radius IS BODY:RADIUS - radius_bias*1000.
	SET tgt_orb["fpa"] TO 0.
	SET tgt_orb["eta"] TO 180.
	
	LOCAL tgt_eta IS 180 + signed_angle(tgt_orb["radius"], TALAbort["tgt_vec"], tgt_orb["normal"], 0).
	
	SET tgt_orb["ecc"] TO (AP - tgt_vec_radius)/(AP + tgt_vec_radius*COS(tgt_eta)).
	SET tgt_orb["SMA"] TO AP/(1 + tgt_orb["ecc"] ).
	SET tgt_orb["velocity"] TO orbit_alt_vel(AP, tgt_orb["SMA"]).
	SET tgt_orb["apoapsis"] TO (AP - BODY:RADIUS)/1000.
	SET tgt_orb["periapsis"] TO (2*tgt_orb["SMA"] - AP - BODY:RADIUS)/1000.
	
	//clearscreen.
	//print target_orbit.
	//print tgt_eta at (0,40).
	//print greatcircledist(vecYZ(cutoff_r),vecYZ(TALAbort["tgt_vec"])).
	//until false{}
	
	RETURN tgt_orb.
}


FUNCTION TAL_boundary {
	RETURN SHIP:VELOCITY:ORBIT:MAG > 4350.
}


FUNCTION setup_TAL {

	IF (DEFINED TALAbort) {
		RETURN.
	}
	
	//vehicle stuff immediately so we can measure the running oms
	start_oms_dump().
	
	//performance calculations first since we need an estimate of the deltaV remaining
	//save the current engine lex (no oms)
	LOCAL cur_stg IS get_stage().
	LOCAL ssme_only_englex IS cur_stg["engines"].
	
	measure_update_engines().
	
	SET vehicle["stages"] TO vehicle["stages"]:SUBLIST(0,3).
	
	SET vehicle["stages"][2]["staging"]["type"] TO "depletion".
	SET vehicle["stages"][2]["mode"] TO 1.
	SET vehicle["stages"][2]["Throttle"] TO 1.
	vehicle["stages"][2]:REMOVE("glim").
	SET vehicle["stages"][2]["engines"] TO build_engines_lex().
	
	LOCAL current_m IS SHIP:MASS*1000.
	local res_left IS get_shuttle_res_left().
	
	update_stage2(current_m, res_left).
	
	
	//estimate deltaV remaining in constant-thrust depletion stage
	LOCAL ve2 IS vehicle["stages"][2]["engines"]["isp"]*g0.
	LOCAL at2 IS vehicle["stages"][2]["engines"]["thrust"]*vehicle["stages"][2]["Throttle"]/vehicle["stages"][2]["m_initial"].
	LOCAL tu2 IS ve2/at2.
	LOCAL tb2 IS vehicle["stages"][2]["Tstage"].
	
	LOCAL DVrem IS ve2*LN(tu2/(tu2-tb2)).
	
	
	SET abort_modes["TAL"]["tgt_site"] TO get_TAL_site(DVrem).
	
	SET target_orbit["mode"] TO 6.
	SET target_orbit["cutoff alt"] TO CLAMP(0.83 * target_orbit["cutoff alt"], 110, 140).		//force cutoff alt 
	SET target_orbit["apoapsis"] TO target_orbit["cutoff alt"].
	
	
	// declare it to signal that TAL has bene initialised
	GLOBAL TALAbort IS LEXICON (
		"t_abort",TIME:SECONDS,
		"tgt_vec", TAL_tgt_vec(orbitstate["radius"])
	).
	
	SET target_orbit TO TAL_cutoff_params(target_orbit, target_orbit["radius"]).
	
	SET upfgInternal["s_init"] TO FALSE.
	
	ascent_gui_set_cutv_indicator(target_orbit["velocity"]).
	
	//trigger the roll to heads-up if it hasn't already, important for reentry 
	WHEN ( TIME:SECONDS > (TALAbort["t_abort"] + 40) ) THEN {
		roll_heads_up().
	}
}

	
	
	
	
//		ATO / AOA FUNCTIONS 

//find new normal vector for ATO targeting
//find the plane containing the current pos vector ( in UPFG coords!!) and vel vector
FUNCTION ATO_normal {

	LOCAL cur_pos IS -vecYZ(SHIP:ORBIT:BODY:POSITION).
	LOCAL cur_vel IS vecYZ(SHIP:VELOCITY:ORBIT).

	LOCAL tgtnorm IS VCRS(cur_pos,cur_vel):NORMALIZED.
	
	//clearvecdraws().
	//arrow_body(vecYZ(cur_pos),"cur_pos").
	//arrow_body(vecYZ(cur_vel),"cur_vel").
	//arrow_body(vecYZ(tgtnorm*BODY:RADIUS),"tgtnorm").
	
	RETURN tgtnorm.
}

//basically the same as normal cutoff params for mode 1 except we force normal vector in-plane
FUNCTION ATO_cutoff_params {
	PARAMETER tgt_orb.
	PARAMETER cutoff_r.
	
	SET tgt_orb["normal"] TO ATO_normal().
	SET tgt_orb["Inclination"] TO VANG(tgt_orb["normal"],v(0,0,1)).
	
	set tgt_orb["radius"] to cutoff_r.
	
	local cut_alt is tgt_orb["radius"]:MAG.
	set tgt_orb["eta"] to orbit_alt_eta(cut_alt, tgt_orb["SMA"], tgt_orb["ecc"]).
	
	set tgt_orb["velocity"] to orbit_alt_vel(cut_alt, tgt_orb["SMA"]).
	
	set tgt_orb["fpa"] to orbit_eta_fpa(tgt_orb["eta"], tgt_orb["SMA"], tgt_orb["ecc"]).

	RETURN tgt_orb.
}

FUNCTION ATO_boundary {
	RETURN SHIP:VELOCITY:ORBIT:MAG > 6100.
}
	

FUNCTION setup_ATO {

	IF (DEFINED ATOAbort) {
		RETURN.
	}
	
	// declare it to signal that ATO has been initialised
	GLOBAL ATOAbort IS LEXICON (
		"t_abort",TIME:SECONDS
	).
	
	//UPFG mode is the nominal one, only change MECO targets
	//lower apoapsis (not too low)
	SET target_orbit["apoapsis"] TO MIN(160, 0.8*target_orbit["apoapsis"]).
	SET target_orbit["periapsis"] TO 0.5 * target_orbit["periapsis"].
	//lower cutoff altitude
	SET target_orbit["cutoff alt"] TO 0.985*target_orbit["cutoff alt"].
	SET target_orbit["radius"] TO target_orbit["radius"]:NORMALIZED*(target_orbit["cutoff alt"] * 1000 + SHIP:BODY:RADIUS).
	//variable iy steering 
	SET target_orbit["normal"] TO ATO_normal().
	SET target_orbit["Inclination"] TO VANG(target_orbit["normal"],v(0,0,1)).
	
	//force cutoff altitude, free true anomaly
	SET target_orbit["mode"] TO 7.
	
	
	SET target_orbit["SMA"] TO orbit_appe_sma(target_orbit["apoapsis"], target_orbit["periapsis"]).
	SET target_orbit["ecc"] TO orbit_appe_ecc(target_orbit["apoapsis"], target_orbit["periapsis"]).
	
	LOCAL rdmag IS target_orbit["cutoff alt"]*1000 + SHIP:BODY:RADIUS.
	
	set target_orbit["eta"] to orbit_alt_eta(rdmag, target_orbit["SMA"], target_orbit["ecc"]).
	
	set target_orbit["velocity"] to orbit_alt_vel(rdmag, target_orbit["SMA"]).
	
	set target_orbit["fpa"] to orbit_eta_fpa(target_orbit["eta"], target_orbit["SMA"], target_orbit["ecc"]).
	
	
	//need to take care of the stages, contrary to RTLS and TAL we might already be in constant-G mode
	IF (vehiclestate["cur_stg"]=2) {
		
		SET vehicle["stages"] TO vehicle["stages"]:SUBLIST(0,3).
	
		SET vehicle["stages"][2]["staging"]["type"] TO "depletion".
		SET vehicle["stages"][2]["mode"] TO 1.
		SET vehicle["stages"][2]["Throttle"] TO 1.
		vehicle["stages"][2]:REMOVE("glim").
		
		LOCAL current_m IS SHIP:MASS*1000.
		local res_left IS get_shuttle_res_left().
		
		
		
		SET vehicle["stages"][2]["engines"] TO build_engines_lex().
		
		update_stage2(current_m, res_left).
		
	} ELSE IF (vehiclestate["cur_stg"]=3) {
	
		SET vehicle["stages"] TO vehicle["stages"]:SUBLIST(0,4).
	
		SET vehicle["stages"][3]["staging"]["type"] TO "depletion".
		SET vehicle["stages"][3]["mode"] TO 1.
		SET vehicle["stages"][3]["Throttle"] TO 1.
		vehicle["stages"][3]:REMOVE("glim").
		vehicle["stages"][3]:REMOVE("throt_mult").
		SET vehicle["stages"][3]["engines"] TO build_engines_lex().
		
		LOCAL current_m IS SHIP:MASS*1000.
		local res_left IS get_shuttle_res_left().
		
		update_stage3(current_m, res_left).
		
	} 
	
	SET upfgInternal["s_init"] TO FALSE.
	
	ascent_gui_set_cutv_indicator(target_orbit["velocity"]).
}



//		POST-ATO ENGINE OUT FUNCTIONS


FUNCTION setup_MECO_ENGOUT {
	IF (DEFINED MECO_ENGOUT) {
		RETURN.
	}
	
	// declare it to signal that ATO has been initialised
	GLOBAL MECO_ENGOUT IS LEXICON (
		"t_abort",TIME:SECONDS
	).

	//no changes in targeting, just convert the stages to constant thrust depletion stages and trigger dump 
	

	IF (vehiclestate["cur_stg"]=3) {
	
		SET vehicle["stages"] TO vehicle["stages"]:SUBLIST(0,4).
	
		SET vehicle["stages"][3]["staging"]["type"] TO "depletion".
		SET vehicle["stages"][3]["mode"] TO 1.
		SET vehicle["stages"][3]["Throttle"] TO 1.
		vehicle["stages"][3]:REMOVE("glim").
		vehicle["stages"][3]:REMOVE("throt_mult").
		SET vehicle["stages"][3]["engines"] TO build_engines_lex().
		
		LOCAL current_m IS SHIP:MASS*1000.
		local res_left IS get_shuttle_res_left().
		
		update_stage3(current_m, res_left).
		
	} ELSE IF (vehiclestate["cur_stg"]=4) {
	
		SET vehicle["stages"][4]["staging"]["type"] TO "depletion".
		SET vehicle["stages"][4]["mode"] TO 1.
		SET vehicle["stages"][4]["Throttle"] TO 1.
		SET vehicle["stages"][4]["engines"] TO build_engines_lex().
		
		LOCAL current_m IS SHIP:MASS*1000.
		local res_left IS get_shuttle_res_left().
		
		update_stage4(current_m, res_left).
		
	} 
	
	ascent_gui_set_cutv_indicator(target_orbit["velocity"]).
}