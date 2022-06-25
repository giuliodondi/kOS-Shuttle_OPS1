//main abort lexicon
//define abort boundaries here
GLOBAL abort_modes IS LEXICON( 
					"triggered",FALSE,
					"t_abort",0,
					"abort_v",0,
					"RTLS",LEXICON(
							"boundary",225,
							"active",TRUE,
							"tgt_site", get_RTLS_site()
							),
					"TAL",LEXICON(
							"boundary",341,
							"active",FALSE,
							"tgt_site", LATLNG(0,0)
							),
					"ATO",LEXICON(
							"boundary",421,
							"active",FALSE
							),
					"MECO",LEXICON(
							"boundary",1000,
							"active",FALSE
							)
							
).



//abort monitor and trigger 

FUNCTION monitor_abort {

	LOCAL abort_detect IS SSME_out().
	
	
	LOCAL current_t IS TIME:SECONDS - vehicle["ign_t"].
	
	IF abort_detect {
		addMessage("ENGINE OUT DETECTED.").
		SET abort_modes["triggered"] TO TRUE.
		SET abort_modes["t_abort"] TO MAX( current_t + 1, vehicle["handover"]["time"] + 6 ).
		SET abort_modes["abort_v"] TO SHIP:VELOCITY:ORBIT:MAG.
	}


	IF (NOT abort_modes["triggered"]) {
		//set the correct abort mode to active and print information
		
		IF abort_modes["RTLS"]["active"] {
			IF ( current_t >= abort_modes["RTLS"]["boundary"]  ) {
				SET abort_modes["RTLS"]["active"] TO FALSE.
				SET abort_modes["TAL"]["active"] TO TRUE.
				addMessage("NEGATIVE RETURN").
			}
		} ELSE IF abort_modes["TAL"]["active"] {
			IF ( current_t >= abort_modes["TAL"]["boundary"]  ) {
				SET abort_modes["TAL"]["active"] TO FALSE.
				SET abort_modes["ATO"]["active"] TO TRUE.
				addMessage("PRESS TO ATO.").
			}
		} ELSE IF abort_modes["ATO"]["active"] {
			IF ( current_t >= abort_modes["ATO"]["boundary"]  ) {
				SET abort_modes["ATO"]["active"] TO FALSE.
				SET abort_modes["MECO"]["active"] TO TRUE.
				addMessage("PRESS TO MECO.").
			}
		}
	
	} ELSE {
		//check if conditions are right to setup the abort
		IF (abort_modes["RTLS"]["active"] ) {
			IF abort_detect {
				addMessage("ABORT RTLS AT "+sectotime(abort_modes["t_abort"])).
			}
		
			//need to check the time becase we wait for second stage for RTLS
			IF ( current_t >= abort_modes["t_abort"] ) {
				SET abort_modes["RTLS"]["active"] TO FALSE.
				setup_RTLS().
			}
		} ELSE IF (abort_modes["TAL"]["active"] ) {
			IF abort_detect {
				addMessage("ABORT TAL AT "+sectotime(abort_modes["t_abort"])).
			}
			//no need to check the time for TAL
			SET abort_modes["TAL"]["active"] TO FALSE.
			setup_TAL().
		}  ELSE IF (abort_modes["ATO"]["active"] ) {
			IF abort_detect {
				addMessage("ABORT ATO / AOA AT "+sectotime(abort_modes["t_abort"])).
			}
			//no need to check the time for ATO / AOA
			SET abort_modes["ATO"]["active"] TO FALSE.
			setup_ATO().
		
		}  ELSE IF (abort_modes["MECO"]["active"] ) {
			IF abort_detect {
				addMessage("PRESSING TO MECO").
			}

			SET abort_modes["MECO"]["active"] TO FALSE.
			setup_MECO_ENGOUT().
		
		}
	
	}

}







//		RTLS FUNCTIONS 


//to be called before launch, will fin the closest landing site 
//to the launchpad
FUNCTION get_RTLS_site {
	LOCAL reduced_sites_lex IS LEXICON(
									"KSC",ldgsiteslex["KSC"],
									"Edwards",ldgsiteslex["Edwards"]
	
	).
	LOCAL closest_out IS get_closest_site(reduced_sites_lex).
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


//dissipation angle as function of abort time
FUNCTION RTLS_dissip_theta_time {
	PARAMETER abort_t.
	
	LOCAL theta_vs_t IS LIST(
						LIST(0,66),
						LIST(30,64),
						LIST(60,62),
						LIST(90,58),
						LIST(120,52),
						LIST(150,42),
						LIST(180,39),
						LIST(210,35),
						LIST(240,31)
	).
	
	LOCAL theta_bias IS 0.
	
	RETURN INTPLIN( theta_vs_t, abort_t) + theta_bias. 

}

//dissipation angle as function of inertial velocity at abort
FUNCTION RTLS_dissip_theta_vel {
	PARAMETER vel.
	
	LOCAL theta_bias IS 0.
	
	RETURN 0.474 + 0.039698*vel - 7.8684*(1e-06)*(vel^2) + thetabias.
}




//c1 vector in upfg coordinates
//takes velocity in upfg coordinates
FUNCTION RTLS_C1 {
	PARAMETER abort_t.
	PARAMETER vel_vec.
	
	local thetabias is 0.
	
	local vel is vel_vec:MAG.
	
	LOCAL theta IS RTLs_dissip_theta_time(abort_t).
	//LOCAL theta IS RTLs_dissip_theta_vel(vel).
	
	
	LOCAL tgtsitevec IS RTLS_tgt_site_vector().
	
	LOCAL iz IS VCRS(vel_vec, tgtsitevec):NORMALIZED.
	LOCAL C1 IS VCRS(tgtsitevec,iz ):NORMALIZED.
	RETURN rodrigues(C1, iz, theta).
	

}

//RV-line , takes range to target site in metres
FUNCTION RTLS_rvline {
	PARAMETER rng.
	
	RETURN 0.0032*rng + 600.
	//RETURN 0.00250*rng + 1000.
	//RETURN 0.00242*rng + 768.7.
	
}


//normal vector to the plane containing current pos vector and target vector
FUNCTION RTLS_normal {

	LOCAL cur_pos IS -vecYZ(SHIP:ORBIT:BODY:POSITION:NORMALIZED).
		
	//find the current position vector of the target site
	LOCAL tgtsitevec IS RTLS_tgt_site_vector().
	
	//construct the plane of rtls
	LOCAL dr IS cur_pos - tgtsitevec.
	LOCAL tgtnorm IS -VCRS(tgtsitevec,dr):NORMALIZED.
	
	RETURN tgtnorm.

}

FUNCTION RTLS_cutoff_params {
	PARAMETER tgt_orb.
	PARAMETER cutoff_r.
	
	//first calculate the cutoff reference frame pointing to the target site right now
	LOCAL iy IS RTLS_normal().
	LOCAL ix IS cutoff_r:NORMALIZED.
	LOCAL iz IS VCRs(ix,iy).
	
	SET tgt_orb["radius"] TO ix* tgt_orb["radius"]:MAG.
	
	//calculate range to go and cutoff velocity using the RV line 
	//need to use a fixed position to make the calculation stable
	
	//this is the launch site position when the abort was triggered
	LOCAL tgtsitevec IS RTLS_shifted_tgt_site_vector(RTLSAbort["t_abort"] - TIME:SECONDS).
	
	LOCAL newrange IS VANG(tgtsitevec,ix).
	SET newrange TO newrange*(constant:pi*SHIP:BODY:RADIUS)/180 + RTLSAbort["MECO_range_shift"].
	SET tgt_orb["range"] TO newrange.
	
	LOCAL rv_vel IS RTLS_rvline(newrange).
	
	LOCAL vel IS rodrigues(iz,iy, tgt_orb["angle"]):NORMALIZED*rv_vel.
	LOCAL vEarth IS (constant:pi/43200)*VCRS( v(0,0,1),tgtsitevec).
	SET vel TO vel + vEarth.
	
	
	//set new normal to normal of the plane containing current pos and target vel
	SET tgt_orb["normal"] TO VCRS( -vecYZ(SHIP:ORBIT:BODY:POSITION:NORMALIZED) , vel:NORMALIZED  ).
	

	SET tgt_orb["velocity"] TO vel:MAG.	
	
	RETURN LIST(tgt_orb,vel).

}


FUNCTION RTLS_burnout_mass {

	SET vehicle["mbod"] TO vehicle["stages"][vehicle["stages"]:LENGTH - 1]["m_final"] + 10000.
}

//compare abort time with negative return to see if we should flyback immediately
FUNCTION RTLS_immediate_flyback {
	RETURN (RTLSAbort["t_abort"] + 10 > abort_modes["RTLS"]["boundary"]).
}

FUNCTION setup_RTLS {

	IF (DEFINED RTLSAbort) {
		RETURN.
	}
	
	//need to do the vehicle performance recalculations first because we need to know the time to burnout
	
	SET vehicle["stages"] TO vehicle["stages"]:SUBLIST(0,3).
	
	SET vehicle["stages"][2]["staging"]["type"] TO "depletion".
	SET vehicle["stages"][2]["mode"] TO 1.
	SET vehicle["stages"][2]["Throttle"] TO 1.
	vehicle["stages"][2]:REMOVE("glim").
	vehicle["stages"][2]:REMOVE("minThrottle").
	SET vehicle["stages"][2]["engines"] TO build_ssme_lex().
	
	LOCAL current_m IS SHIP:MASS*1000.
	local res_left IS get_prop_mass(vehicle["stages"][2]).
	
	update_stage2(current_m, res_left).
	
	
	vehicle:ADD("mbod",0).
	
	RTLS_burnout_mass().				 
	
	//so that downrange distance calculations are correct
	SET launchpad TO abort_modes["RTLS"]["tgt_site"].
	
	LOCAL t_abort IS TIME:SECONDS.
	
	LOCAL flyback_immediate IS (t_abort - vehicle["ign_t"] + 10 > abort_modes["RTLS"]["boundary"]).
		
	IF (flyback_immediate) {
		addMessage("POWERED PITCH-AROUND TRIGGERED").
		SET STEERINGMANAGER:MAXSTOPPINGTIME TO 1.2.
	}
	
	GLOBAL RTLSAbort IS LEXICON (
								"t_abort",t_abort,
								"C1",v(0,0,0),
								"Tc",0,
								"MECO_range_shift",0,
								"pitcharound",LEXICON(
													"refvec",v(0,0,0),
													"triggered",flyback_immediate,
													"complete",FALSE,
													"target",v(0,0,0)
													
												),
								"flyback_iter",-2,
								"flyback_conv",-2,
								"flyback_flag",flyback_immediate
	).
	
	LOCAL normvec IS RTLS_normal().
	
	LOCAL abort_v IS abort_modes["abort_v"]*vecYZ(SHIP:VELOCITY:ORBIT:NORMALIZED).
	SET RTLSAbort["C1"] TO VXCL(normvec,RTLS_C1(abort_modes["t_abort"],abort_v)).
	
	SET RTLSAbort["pitcharound"]["refvec"] TO VCRS( RTLSAbort["C1"], vecYZ(-SHIP:ORBIT:BODY:POSITION:NORMALIZED)).
	SET RTLSAbort["pitcharound"]["target"] TO rodrigues(RTLSAbort["C1"],RTLSAbort["pitcharound"]["refvec"],2.5*VANG(RTLSAbort["C1"],vecYZ(-SHIP:ORBIT:BODY:POSITION:NORMALIZED))).
	
	
	//calculate the range shift to use for RVline calculations
	//predict the time to desired cutoff mass, shift the target site forward to that point, measure distance and correct for inclination
	LOCAL dmbo_t IS (vehicle["stages"][2]["m_initial"] - vehicle["mbod"])/red_flow.
	
	LOCAL tgt_site_now IS RTLS_tgt_site_vector().
	LOCAL tgt_site_meco IS RTLS_shifted_tgt_site_vector(dmbo_t).
	LOCAL range_dist IS VANG(tgt_site_now,tgt_site_meco)*(constant:pi*SHIP:BODY:RADIUS)/180.
	
	LOCAL delta_tgt_pos IS tgt_site_meco - tgt_site_now.
	
	//should be negative if we're moving east (the taget site will move towards us during flyback) and positive if west (tgtsite will be moving away)
	SET RTLSAbort["MECO_range_shift"] TO -VDOT(SHIP:VELOCITY:SURFACE:NORMALIZED,delta_tgt_pos:NORMALIZED)*range_dist.
	
	
	
	LOCAL curR IS orbitstate["radius"].
	LOCAL cutoff_alt IS 80*1000 + SHIP:BODY:RADIUS.
	SET target_orbit TO LEXICON(
							"mode",5,
							"normal",normvec,
							"radius",(curR:NORMALIZED)*cutoff_alt,
							"velocity",2200,
							"angle",172,
							"range",500*1000,
							"Periapsis",0,
							"Apoapsis",0,
							"inclination",target_orbit["inclination"],
							"eta",0,
							"LAN",target_orbit["LAN"]
	
	).
	
	
	SET upfgConvergenceTgo TO 1.5.
	SET upfgFinalizationTime TO 15.
	SET upfgInternal["flyback_flag"] TO flyback_immediate.


	SET upfgInternal TO resetUPFG(upfgInternal).
	
	
	SET vehicle["roll"] TO 0.
	
	
	OMS_dump("oms","start").
	WHEN ( TIME:SECONDS > (RTLSAbort["t_abort"] + 540) ) THEN {
		OMS_dump("oms","stop").
		addMessage("OMS DUMP COMPLETE").
	}

	drawUI().
	

}













FUNCTION GRTLS_dataViz {

	
	PRINT "         GLIDE-RTLS GUIDANCE    "  AT (0,1).
	PRINT "                          "  AT (0,2).
	
	IF (vehiclestate["ops_mode"] = 5) {
	PRINT "            ALPHA RECOVERY    " AT (0,3).
	} ELSE IF (vehiclestate["ops_mode"] = 6) {
	PRINT "            HOLDING PITCH    " AT (0,3).
	} ELSE IF (vehiclestate["ops_mode"] = 7) {
	PRINT "               NZ HOLD      " AT (0,3).
	}
				   
								   
	PRINT "                     "  AT (0,4).
	PRINT "       M.E.T.      : "  AT (0,5).
	PRINT "                     "  AT (0,6).
	PRINT "    SURFACE ALT    : "  AT (0,7).
	PRINT "    VERTICAL SPD   : "  AT (0,8).
	PRINT "    VERTICAL AOA   : "  AT (0,9).
	PRINT "                     "  AT (0,10).
	PRINT "    DOWNRANGE DST  : "  AT (0,11).
	PRINT "      AZ ERROR     : "  AT (0,12).
	PRINT "                     "  AT (0,13).
	PRINT "    CUR G FORCE    : "  AT (0,14).
	PRINT "    TGT G FORCE    : "  AT (0,15).
	PRINT "                     "  AT (0,16).
	PRINT "     CUR PITCH     : "  AT (0,17).
	PRINT "     CUR ROLL      : "  AT (0,18).
   





	PRINTPLACE(sectotime(TIME:SECONDS - vehicle["ign_t"]),12,22,5).


	PRINTPLACE(ROUND(SHIP:ALTITUDE/1000 , 2) + " km",12,22,7).
	PRINTPLACE(ROUND(SHIP:VERTICALSPEED,1) + " m/s",12,22,8).
	PRINTPLACE(ROUND(VANG(SHIP:SRFPROGRADE:VECTOR, SHIP:UP:VECTOR) - VANG(SHIP:FACING:VECTOR, SHIP:UP:VECTOR),3) + " deg",12,22,9).

	PRINTPLACE(ROUND(downrangedist(abort_modes["RTLS"]["tgt_site"],SHIP:GEOPOSITION ),2) + " km",12,22,11).

	PRINTPLACE(ROUND(az_error(SHIP:GEOPOSITION,abort_modes["RTLS"]["tgt_site"],SHIP:VELOCITY:SURFACE),1) + " deg",12,22,12).


	PRINTPLACE(ROUND(NZHOLD["cur_nz"],1) ,12,22,14).
	PRINTPLACE(ROUND(NZHOLD["tgt_nz"],1) ,12,22,15).
	
	PRINTPLACE(ROUND(get_pitch(),1) + " deg",12,22,17).
	PRINTPLACE(ROUND(get_roll(),1) + " deg",12,22,18).
	
	

}




FUNCTION set_target_nz {
	
	IF (NZHOLD["cur_nz"] >= 1) {
		SET NZHOLD["tgt_nz"] TO 0.7 + ABS(NZHOLD["cur_hdot"]/655.32).
		SET NZHOLD["tgt_nz"] TO MAX(2.5,MIN( 3.9, NZHOLD["tgt_nz"])).
	}

}


FUNCTION nz_update_pitch {
	PARAMETER cur_pch.


	LOCAL deltapch IS - 0.9*NZHOLD["dt"].
	SET cur_pch TO cur_pch + deltapch.
	
	 
	RETURN cur_pch.
}


FUNCTION update_g_force {
	PARAMETER nz.
	
	LOCAL g0 IS 9.80665.
	LOCAL cur_t IS TIME:SECONDS.
	LOCAL cur_hdot IS SHIP:VERTICALSPEED.
	SET nz["dt"] TO cur_t - nz["cur_t"].
	
	IF nz["dt"]>0 {
		SET nz["cur_nz"] TO (cur_hdot- nz["cur_hdot"])/(g0*nz["dt"]) + 1.
	}
	SET nz["cur_hdot"] TO cur_hdot.
	SET nz["cur_t"] TO cur_t.

	RETURN nz.
}




FUNCTION GRTLS {

	CLEARSCREEN.
	RUNPATH("0:/Shuttle_entrysim/src/entry_utility").
	
	STEERINGMANAGER:RESETPIDS().
	STEERINGMANAGER:RESETTODEFAULT().

	SET STEERINGMANAGER:MAXSTOPPINGTIME TO 20.
	SET STEERINGMANAGER:PITCHTS TO 10.0.
	SET STEERINGMANAGER:YAWTS TO 10.0.
	SET STEERINGMANAGER:ROLLTS TO 8.0.
	SET STEERINGMANAGER:PITCHPID:KD TO 0.05.
	SET STEERINGMANAGER:YAWPID:KD TO 0.05.
	SET STEERINGMANAGER:ROLLPID:KD TO 0.05.

	LOCAL pitch0 IS 40.
	LOCAL pitchf IS 10.
	LOCAL tgt_hdot IS -150.
	LOCAL firstroll IS 28.
	
	LOCAL t_grtls IS TIME:SECONDS.

	LOCAL aimvec IS SHIP:VELOCITY:SURFACE:NORMALIZED.
	LOCAL upvec IS -SHIP:ORBIT:BODY:POSITION:NORMALIZED.
	LOCAL rightvec IS VCRS(upvec, aimvec ).
	
	LOCAL flap_control IS LEXICON(
						"deflection",0,
						"pitch_control",average_value_factory(4),
						"parts", LIST(
							LEXICON(
									"flapmod",SHIP:PARTSDUBBED("ShuttleElevonL")[0]:getmodule("FARControllableSurface"),
									"min_defl",-14,
									"max_defl",20
							),
							LEXICON(
									"flapmod",SHIP:PARTSDUBBED("ShuttleElevonR")[0]:getmodule("FARControllableSurface"),
									"min_defl",-25,
									"max_defl",20
							),
							LEXICON(
									"flapmod",SHIP:PARTSDUBBED("ShuttleBodyFlap")[0]:getmodule("FARControllableSurface"),
									"min_defl",-25,
									"max_defl",20
							)
						)
	).
	
	LISt ENGINES IN englist.
	LOCAL gimbals IS 0.
	FOR e IN englist {
		IF e:HASSUFFIX("gimbal") {
			SET gimbals TO e:GIMBAL.
			BREAK.
		}
	}
	gimbals:DOACTION("free gimbal", TRUE).
	//gg:DOEVENT("Show actuation toggles").
	gimbals:DOACTION("toggle gimbal roll", TRUE).
	gimbals:DOACTION("toggle gimbal yaw", TRUE).
	
	activate_flaps(flap_control["parts"]).

	SET aimvec TO rodrigues(aimvec,rightvec,-pitch0).
	
	RCS ON.
	SAS OFF.
	SET vehiclestate["ops_mode"] TO 5.
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
	
	
	//run the control loop 
	//faster than the main loop 
	LOCAL attitude_time_upd IS TIME:SECONDS.
	WHEN TIME:SECONDS>attitude_time_upd + 0.5 THEN {
		SET attitude_time_upd TO TIME:SECONDS.
		//steer to the new pitch and roll 
		SET P_att TO update_attitude(P_att,pitchv,rollv).

		
		PRESERVE.
	}
	
	
	LOCAL deltanz IS  -  NZHOLD["tgt_nz"].
	
	WHEN ( SHIP:ALTITUDE < 60000 OR (NZHOLD["cur_nz"] <0 AND NZHOLD["cur_nz"] > -2 AND SHIP:ALTITUDE < 70000) ) THEN {
		SET vehiclestate["ops_mode"] TO 6.
	}
	
	

	
	
	UNTIL FALSE {
		
	
		LOCAL prev_nz IS NZHOLD["cur_nz"].
		
		SET NZHOLD TO update_g_force(NZHOLD).
		
		IF (vehiclestate["ops_mode"] >= 6 ) {
		
			flap_control["pitch_control"]:update(-gimbals:PITCHANGLE).
			SET flap_control TO flaptrim_control( flap_control).
		
			IF (NZHOLD["tgt_nz"] = 0) {
				set_target_nz().
			}
					
			//want to switch to mode 7 when the current nz starts to decrease or when it beomes greater than the target nz
			IF ( vehiclestate["ops_mode"]=6 AND NZHOLD["cur_nz"]>0 AND (prev_nz >= NZHOLD["cur_nz"] OR (NZHOLD["tgt_nz"]>0 AND NZHOLD["cur_nz"] >= NZHOLD["tgt_nz"])) ) {
				SET vehiclestate["ops_mode"] TO 7.
			}
			
			
			SET deltanz TO NZHOLD["cur_nz"] -  NZHOLD["tgt_nz"].
			
			IF (vehiclestate["ops_mode"] = 7 ) {
				SET pitchv TO MAX(pitchf,MIN(pitch0,nz_update_pitch(pitchv))).
				SET NZHOLD["cmd_pch"] TO pitchv.
			}
			
			
			IF ((NZHOLD["cur_hdot"]> tgt_hdot ) AND (pitchv < pitchf*1.1)) {
				BREAK.
			}
			
		}
		

		GRTLS_dataViz().
		WAIT 0.05.
	}
	
	
	//prepare entry guidance
	GLOBAL pitchprof_segments IS LIST(
								LIST(350,5),
								LIST(1800,10)
								).
								
	GLOBAL sim_end_conditions IS LEXICON(
						"altitude",12000,
						"surfvel",350,
						"range_bias",0
	).
	GLOBAL start_guid_flag IS TRUE.
	
	SET TERMINAL:WIDTH TO 50.
	SET TERMINAL:HEIGHT TO 30.
	
	SAS ON.
	UNLOCK STEERING.
	RUN "0:/entry".

}




//		TAL FUNCTIONS 




//hard-coded selector of TAL sites based on inclination
FUNCTION get_TAL_site {

	LOCAL talsite IS LATLNG(0,0).

	IF TAL_site = "Nearest" {
		LOCAL orbplanevec IS VCRS(orbitstate["radius"],orbitstate["velocity"]):NORMALIZED.
		
		LOCAL lowestproj IS 0.
	
		FOR s in ldgsiteslex:KEYS {
			LOCAL sitepos IS vecYZ(pos2vec(s["position"])).
			
			LOCAL siteproj IS VDOT(sitepos,orbplanevec).
			
			IF (lowestproj=0 OR (lowestproj > 0 AND siteproj < lowestproj)) {
				SET lowestproj TO siteproj.
				SET talsite tO s.
			}
		}
	
	} ELSE {
		SET talsite TO ldgsiteslex[TAL_site]["position"].
	}
	
	RETURN talsite.

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
	LOCAL dr IS cur_pos - TALAbort["tgt_vec"].
	LOCAL tgtnorm IS VCRS(TALAbort["tgt_vec"],dr):NORMALIZED.
	
	//clearvecdraws().
	//arrow_body(vecYZ(cur_pos),"cur_pos").
	//arrow_body(vecYZ(TALAbort["tgt_vec"]),"tgt_vec").
	//arrow_body(vecYZ(tgtnorm*BODY:RADIUS),"tgtnorm").
	
	RETURN tgtnorm.
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
		LOCAL abeam_pos IS rodrigues(VXCL(target_orbit["normal"],posvec),-target_orbit["normal"],d_eta).
		
		SET shifted_site TO vecYZ(pos2vec(shift_pos(vecYZ(sitevec),-abeam_dt))).
		
		SET shifted_site_proj TO VXCL(target_orbit["normal"],shifted_site).
		
		LOCAL eta_error IS signed_angle(abeam_pos,shifted_site_proj,-target_orbit["normal"],0).
		
		IF (ABS(eta_error) < 0.01) {
			BREAK.
		}
		
		SET d_eta TO d_eta + eta_error.
	}
			
	//now find the plane containing both the shifted site and its projection onto the orbital plane 
	LOCAL abeam_norm IS VCRS(shifted_site:NORMALIZED,shifted_site_proj:NORMALIZED):NORMALIZED.
	
	//get the angle between the two
	//clamp this angle so that the crossrange is within a certain value
	LOCAL max_xrange IS 600.	//in km
	LOCAL abeam_angle IS signed_angle(shifted_site,shifted_site_proj,abeam_norm,0).
	SET abeam_angle TO SIGN(abeam_angle)*CLAMP(ABS(abeam_angle),0,max_xrange*1000*180/(CONSTANT:PI*BODY:RADIUS)).
	
	LOCAL tgtvec IS rodrigues(shifted_site,abeam_norm,abeam_angle):NORMALIZED*BODY:RADIUS.
	
	RETURN tgtvec.
}

//cutoff altitude is set as the apoapsis
//cutoff velocity is calculated so that the ballistic trajectory intersects the surface at the target vector
FUNCTION TAL_cutoff_params {
	PARAMETER tgt_orb.
	PARAMETER cutoff_r.

	SET tgt_orb["normal"] TO TAL_normal().
	SET tgt_orb["Inclination"] TO VANG(-tgt_orb["normal"],v(0,0,1)).
	
	SET tgt_orb["radius"] TO VXCL(tgt_orb["normal"],cutoff_r):NORMALIZED*cutoff_r:MAG.
	
	//shifts underground the radius of the ballistic impact point
	LOCAL radius_bias IS 15.	//in km
	
	
	LOCAL AP is tgt_orb["radius"]:MAG.
	LOCAL tgt_vec_radius IS BODY:RADIUS - radius_bias*1000.
	SET tgt_orb["angle"] TO 0.
	SET tgt_orb["eta"] TO 180.
	
	LOCAL tgt_eta IS 180 + signed_angle(tgt_orb["radius"],TALAbort["tgt_vec"],-tgt_orb["normal"],0).
	
	SET tgt_orb["ecc"] TO (AP - tgt_vec_radius)/(AP + tgt_vec_radius*COS(tgt_eta)).
	SET tgt_orb["SMA"] TO AP/(1 + tgt_orb["ecc"] ).
	SET tgt_orb["velocity"] TO SQRT(SHIP:BODY:MU * (2/AP - 1/tgt_orb["SMA"])).
	SET tgt_orb["periapsis"] TO (2*tgt_orb["SMA"] - AP - BODY:RADIUS)/1000.
	
	//clearscreen.
	//print target_orbit.
	//print tgt_eta at (0,40).
	//print greatcircledist(vecYZ(cutoff_r),vecYZ(TALAbort["tgt_vec"])).
	//until false{}
	
	RETURN tgt_orb.
}



FUNCTION setup_TAL {

	IF (DEFINED TALAbort) {
		RETURN.
	}
	
	SET abort_modes["TAL"]["tgt_site"] TO get_TAL_site().
	
	// declare it to signal that TAL has bene initialised
	//make an initial guess for the target vector, shift it 1000 seconds ahead, project it onto the orbital plane
	
	LOCAL tgtvec_guess IS VXCL(target_orbit["normal"], vecYZ(pos2vec(shift_pos(vecYZ(TAL_tgt_site_vector()),-1000))) ). 
	
	GLOBAL TALAbort IS LEXICON (
		"t_abort",TIME:SECONDS,
		"tgt_vec", tgtvec_guess
	).
	
	//here we make the assumption that UPFG is already running and so we have a reasonable prediction for the cutoff radius 
	SET target_orbit["mode"] TO 6.
	SET target_orbit["apoapsis"] TO (target_orbit["radius"]:MAG - BODY:RADIUS)/1000.
	
	SET target_orbit TO TAL_cutoff_params(target_orbit, target_orbit["radius"]).
	SET TALAbort["tgt_vec"] TO TAL_tgt_vec(orbitstate["radius"]).
	
	
	SET vehicle["stages"] TO vehicle["stages"]:SUBLIST(0,3).
	
	SET vehicle["stages"][2]["staging"]["type"] TO "depletion".
	SET vehicle["stages"][2]["mode"] TO 1.
	SET vehicle["stages"][2]["Throttle"] TO 1.
	vehicle["stages"][2]:REMOVE("glim").
	vehicle["stages"][2]:REMOVE("minThrottle").
	SET vehicle["stages"][2]["engines"] TO build_ssme_lex().
	
	LOCAL current_m IS SHIP:MASS*1000.
	local res_left IS get_prop_mass(vehicle["stages"][2]).
	
	update_stage2(current_m, res_left).
	
	SET upfgInternal TO resetUPFG(upfgInternal).
	
	//the dump will actually stop at MECO
	OMS_dump("oms","start").
	WHEN ( TIME:SECONDS > (TALAbort["t_abort"] + 540) ) THEN {
		OMS_dump("oms","stop").
		addMessage("OMS DUMP COMPLETE").
	}
	
	//trigger the roll to heads-up if it hasn't already, important for reentry 
	WHEN ( TIME:SECONDS > (TALAbort["t_abort"] + 20) ) THEN {
		roll_heads_up().
	}
	

	drawUI().
}

	
	
	
	
//		ATO / AOA FUNCTIONS 

//find new normal vector for ATO targeting
//find the plane containing the current pos vector ( in UPFG coords!!) and vel vector
FUNCTION ATO_normal {

	LOCAL cur_pos IS -vecYZ(SHIP:ORBIT:BODY:POSITION).
	LOCAL cur_vel IS vecYZ(SHIP:VELOCITY:ORBIT).

	LOCAL tgtnorm IS -VCRS(cur_pos,cur_vel):NORMALIZED.
	
	//clearvecdraws().
	//arrow_body(vecYZ(cur_pos),"cur_pos").
	//arrow_body(vecYZ(cur_vel),"cur_vel").
	//arrow_body(vecYZ(tgtnorm*BODY:RADIUS),"tgtnorm").
	
	RETURN tgtnorm.
}

//basically the same as normal cutoff params for mode 1 except we force normal vector in-plane
FUNCTION ATO_cutoff_params {
	PARAMETER target.
	PARAMETER cutoff_r.
	
	SET target["normal"] TO ATO_normal().
	SET target["Inclination"] TO VANG(- target["normal"],v(0,0,1)).

	LOCAL etaa IS 0.
	local r is cutoff_r:MAG.
	IF target["ecc"]=0 {set etaa to  0.}
	ELSE {		
		set etaa to (target["SMA"]*(1-target["ecc"]^2)/r - 1)/target["ecc"].
		set etaa to ARCCOS(limitarg(etaa)).
	}
	local x is  1 + target["ecc"]*COS(etaa).
	
	local v is SQRT(SHIP:BODY:MU * (2/r - 1/target["SMA"])).
		
	local phi is target["ecc"]*sin(etaa)/x.
	set phi to ARCTAN(phi).
	
	set target["velocity"] to v.
	set target["angle"] to phi.
	set target["eta"] to etaa.
	
	RETURN target.
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
	//lower cutoff altitude
	SET target_orbit["radius"] TO target_orbit["radius"]:NORMALIZED*((target_orbit["radius"]:MAG - SHIP:BODY:RADIUS)*0.985 + SHIP:BODY:RADIUS).
	//force cutoff altitude, free true anomaly
	SET target_orbit["mode"] TO 7.
	
	
	LOCAL pe IS target_orbit["periapsis"]*1000 + SHIP:BODY:RADIUS.
	LOCAL ap IS target_orbit["apoapsis"]*1000 + SHIP:BODY:RADIUS.
	SET target_orbit["SMA"] TO (pe+ap) / 2.
	SET target_orbit["ecc"] TO (ap - pe)/(ap + pe).
	
	SET target_orbit TO ATO_cutoff_params(target_orbit,target_orbit["radius"]).
	
	
	//need to take care of the stages, contrary to RTLS and TAL we might already be in constant-G mode
	IF (vehiclestate["cur_stg"]=2) {
		
		SET vehicle["stages"] TO vehicle["stages"]:SUBLIST(0,3).
	
		SET vehicle["stages"][2]["staging"]["type"] TO "depletion".
		SET vehicle["stages"][2]["mode"] TO 1.
		SET vehicle["stages"][2]["Throttle"] TO 1.
		vehicle["stages"][2]:REMOVE("glim").
		vehicle["stages"][2]:REMOVE("minThrottle").
		
		LOCAL current_m IS SHIP:MASS*1000.
		local res_left IS get_prop_mass(vehicle["stages"][2]).
		
		
		
		SET vehicle["stages"][2]["engines"] TO build_ssme_lex().
		
		update_stage2(current_m, res_left).
		
	} ELSE IF (vehiclestate["cur_stg"]=3) {
	
		SET vehicle["stages"] TO vehicle["stages"]:SUBLIST(0,4).
	
		SET vehicle["stages"][3]["staging"]["type"] TO "depletion".
		SET vehicle["stages"][3]["mode"] TO 1.
		SET vehicle["stages"][3]["Throttle"] TO 1.
		vehicle["stages"][3]:REMOVE("glim").
		vehicle["stages"][3]:REMOVE("minThrottle").
		vehicle["stages"][3]:REMOVE("throt_mult").
		SET vehicle["stages"][3]["engines"] TO build_ssme_lex().
		
		LOCAL current_m IS SHIP:MASS*1000.
		local res_left IS get_prop_mass(vehicle["stages"][3]).
		
		update_stage3(current_m, res_left).
		
	} 
	
	SET upfgInternal TO resetUPFG(upfgInternal).
	
	
	//the dump will actually stop at MECO
	OMS_dump("oms","start").
	WHEN ( TIME:SECONDS > (ATOAbort["t_abort"] + 540) ) THEN {
		OMS_dump("oms","stop").
		addMessage("OMS DUMP COMPLETE").
	}
	
	drawUI().
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
		vehicle["stages"][3]:REMOVE("minThrottle").
		vehicle["stages"][3]:REMOVE("throt_mult").
		SET vehicle["stages"][3]["engines"] TO build_ssme_lex().
		
		LOCAL current_m IS SHIP:MASS*1000.
		local res_left IS get_prop_mass(vehicle["stages"][3]).
		
		update_stage3(current_m, res_left).
		
	} ELSE IF (vehiclestate["cur_stg"]=4) {
	
		SET vehicle["stages"][4]["staging"]["type"] TO "depletion".
		SET vehicle["stages"][4]["mode"] TO 1.
		SET vehicle["stages"][4]["Throttle"] TO 1.
		vehicle["stages"][4]:REMOVE("minThrottle").
		SET vehicle["stages"][4]["engines"] TO build_ssme_lex().
		
		LOCAL current_m IS SHIP:MASS*1000.
		local res_left IS get_prop_mass(vehicle["stages"][4]).
		
		update_stage4(current_m, res_left).
		
	} 
	
	SET upfgInternal TO resetUPFG(upfgInternal).
	
	
	//the dump will actually stop at MECO
	OMS_dump("oms","start").
	WHEN ( TIME:SECONDS > (MECO_ENGOUT["t_abort"] + 540) ) THEN {
		OMS_dump("oms","stop").
		addMessage("OMS DUMP COMPLETE").
	}
	
	drawUI().


}