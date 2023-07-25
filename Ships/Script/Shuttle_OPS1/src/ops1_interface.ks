GLOBAL P_vizMsg IS LIST().


FUNCTION addGUIMessage {
	PARAMETER msg.
	
	LOCAL clear_ is false.
	//if (vehiclestate["ops_mode"] < 1) {
	//	set clear_ to TRUE.
	//}
	
	local t_msg is TIME:SECONDS - vehicle["ign_t"].
	
	local t_str IS "".
	
	if (t_msg >= 0) {
		SET t_str TO t_str + "+".
	}
	
	ascent_add_scroll_msg(
						t_str + sectotime(TIME:SECONDS - vehicle["ign_t"],"") + ": " + msg,
						clear_
	).

}

FUNCTION addMessage {
	DECLARE PARAMETER msg.
	LOCAL tt IS TIME:SECONDS.
	LOCAL ttl IS 4.

	local rem_list IS LIST().
	FROM {LOCAL k IS 0.} UNTIL k = P_vizMsg:LENGTH  STEP { SET k TO k+1.} DO{
		IF tt >= P_vizMsg[k]["ttl"] {
			rem_list:ADD(k).
		}
	}
	FROM {LOCAL k IS rem_list:LENGTH-1.} UNTIL k <0  STEP { SET k TO k-1.} DO{
		P_vizMsg:REMOVE(rem_list[k]).
	}
	
	P_vizMsg:INSERT(
					0,
					LEXICON(
							"msg","T+" + sectotime(TIME:SECONDS - vehicle["ign_t"],"") + ": " + msg,
							"ttl",tt + ttl
					)
	) .
}

FUNCTION dataViz {
	if (vehiclestate["ops_mode"] =0) {return.}
	
	log_telemetry().
	
	
	//gui update
	
	local roll_prog is get_roll_prograde().
	local pitch_prog is get_pitch_prograde().
	
	
	local thrvec is vehiclestate["thr_vec"]/1000.
	
	//predict 30 seconds into the future, 2 steps
	//keep roll and pitch fixed 
	LOCAL pred_simstate IS current_simstate().
	LOCAL sim_dt IS 15.
	
	IF (vehiclestate["ops_mode"] =1) {
		SET sim_dt TO 7.5.
	}
	
	FROM {local k is 1.} UNTIL k > 2 STEP {set k to k + 1.} DO { 
		SET pred_simstate TO rk3(sim_dt,pred_simstate,LIST(pitch_prog,roll_prog), thrvec).
	}	
	SET pred_simstate["altitude"] TO bodyalt(pred_simstate["position"]).
	SET pred_simstate["surfvel"] TO surfacevel(pred_simstate["velocity"],pred_simstate["position"]).
	SET pred_simstate["latlong"] TO shift_pos(pred_simstate["position"],pred_simstate["simtime"]).

	LOCAL pred_vi IS pred_simstate["velocity"]:MAG.
	LOCAL pred_ve IS pred_simstate["surfvel"]:MAG.
	LOCAL pred_alt IS pred_simstate["altitude"]/1000.
	
	local tgo is 0.
	local vgo is 0.
	
	if (vehiclestate["ops_mode"] > 1) {
		set tgo to upfgInternal["Tgo"].
		set vgo to upfgInternal["vgo"]:MAG.
	}	
	
	LOCAL gui_data IS lexicon(
				"met", TIME:SECONDS - vehicle["ign_t"],
				"ops_mode", vehiclestate["ops_mode"],
				"hdot", SHIP:VERTICALSPEED,
				"roll", unfixangle(control["roll_angle"] - get_roll_lvlh()),
				"pitch", pitch_prog,
				"yaw", get_yaw_prograde(),
				"vi", SHIP:VELOCITY:ORBIT:MAG,
				"ve", SHIP:VELOCITY:SURFACE:MAG,
				"alt", SHIP:ALTITUDE/1000,
				"pred_vi", pred_vi,
				"pred_ve", pred_ve,
				"pred_alt", pred_alt,
				"twr", get_TWR(),
				"ssme_thr", 100*get_ssme_throttle(),
				"et_prop", 100*get_et_prop_fraction(),
				"tgo", tgo,
				"vgo", vgo,
				"converged", (usc["conv"]=1 AND NOT usc["terminal"])
	).
	
	//do the rtls gui update 
	IF (DEFINED RTLSAbort) {
	
		gui_data:ADD("dwnrg_ve", current_horiz_dwnrg_speed(SHIP:GEOPOSITION, SHIP:VELOCITY:SURFACE)).
		gui_data:ADD("dwnrg_pred_ve", current_horiz_dwnrg_speed(pred_simstate["latlong"], pred_simstate["surfvel"])).
		gui_data:ADD("rtls_cutv", target_orbit["rtls_cutv"]).
		gui_data:ADD("rtls_tc", RTLSAbort["Tc"]).
		
		update_rtls_traj_disp(gui_data).
		
		//v_dwnrng", ,
		//"pred_v_dwnrg", pred_v_dwnrg,
	
	} else {
		
					
		update_ascent_traj_disp(gui_data).
	
	} 
	

}


function print_ascent_report {
		
	local orbit_str is "AP: " + ROUND(APOAPSIS/1000,1) + " km | PE: " + ROUND(PERIAPSIS/1000,1) + " km".
	
	set orbit_str to orbit_str + " | INCL:  " + ROUND(ORBIT:INCLINATION,3) + " ° | TRUE AN.: " + ROUND(ORBIT:TRUEANOMALY,2) + " °".

	addGUIMessage(orbit_str).
	
	local orbit_err_str is "Ap err: " + ROUND(abs((1 - APOAPSIS/(1 + target_orbit["Apoapsis"]*1000))*100),2) + "%".
	set orbit_err_str to orbit_err_str + " | Pe err: " + ROUND(abs((1 - PERIAPSIS/(1 + target_orbit["Periapsis"]*1000))*100),2) + "%".
	
	set orbit_err_str to orbit_err_str + " | Incl err: " + ROUND(abs((1 - ORBIT:INCLINATION/(0.001 + target_orbit["Inclination"]))*100),3) + "%".
	set orbit_err_str to orbit_err_str + " | True An err: " + ROUND(abs(ORBIT:TRUEANOMALY - target_orbit["eta"] ),2) + "° ".

	addGUIMessage(orbit_err_str).

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
	
	log_telemetry().

}



FUNCTION prepare_telemetry {
	if logdata=TRUE {	
		GLOBAL loglex IS LEXICON(
										"Time",0,
										"Lat",0,
										"Lng",0,
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
										"Vspeed",0,
										"Dwnrg surfvel",0,
										"Incl",0,
										"Ecctr",0
		).
		log_data(loglex,"./Shuttle_OPS1/LOGS/" + vehicle["name"] + "_log", TRUE).
	}
}

FUNCTION log_telemetry {
	if logdata=TRUE {
		LOCAL stg IS get_stage().
		
		//prepare list of values to log.
		
		SET loglex["Time"] TO TIME:SECONDS - vehicle["ign_t"].
		SET loglex["Lat"] TO SHIP:GEOPOSITION:LAT.
		SET loglex["Lng"] TO SHIP:GEOPOSITION:LNG.
		SET loglex["Altitude"] TO SHIP:ALTITUDE/1000.
		SET loglex["Dwnrg Dst"] TO downrangedist(launchpad,SHIP:GEOPOSITION ).
		SET loglex["Stage"] TO vehiclestate["cur_stg"].
		SET loglex["Mass"] TO stg["m_initial"].
		SET loglex["TWR"] TO get_TWR().
		SET loglex["Throt"] TO stg["Throttle"]*100.
		SET loglex["AZ(cmd)"] TO surfacestate["hdir"].
		SET loglex["HAOA"] TO get_yaw_prograde().
		SET loglex["Pitch"] TO surfacestate["vdir"].
		SET loglex["VAOA"] TO get_pitch_prograde().
		SET loglex["Surfvel"] TO SHIP:VELOCITY:SURFACE:MAG.
		SET loglex["Orbvel"] TO SHIP:VELOCITY:ORBIT:MAG.
		SET loglex["Vspeed"] TO SHIP:VERTICALSPEED.
		SET loglex["Dwnrg surfvel"] TO current_horiz_dwnrg_speed(SHIP:GEOPOSITION, SHIP:VELOCITY:SURFACE).
		SET loglex["Incl"] TO ORBIT:INCLINATION.
		SET loglex["Ecctr"] TO ORBIT:ECCENTRICITY.

		log_data(loglex).
	}
}
 