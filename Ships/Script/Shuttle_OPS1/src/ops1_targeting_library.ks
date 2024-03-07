
//GLOBAL NAV VARIABLES 

GLOBAL launchpad IS SHIP:GEOPOSITION.
GLOBAL surfacestate IS  LEXICON("MET",0,"az",0,"pitch",0,"alt",0,"vs",0,"hs",0,"vdir",0,"hdir",0,"q",0, "maxq", 0).
GLOBAL orbitstate IS  LEXICON("radius",0,"velocity",0). 


//			VARIOUS TARGETING FUNCTIONS

//for logging data in case of RTLS
//calculates the horizontal component of surface velocity 
//projected along the plane of current position and launchpad
FUNCTION current_horiz_dwnrg_speed {
	PARAMETER pos.
	PARAMETER srfvel.

	LOCAL launchvec IS pos2vec(launchpad).
	LOCAL posvec IS pos2vec(pos).
	
	LOCAL norm is VCRS(posvec, launchvec):NORMALIZED.
	LOCAL dwnrgvec IS posvec - launchvec.
	
	LOCAL horizvel IS VXCL(posvec, srfvel).
	SET horizvel TO VXCL(norm, horizvel).
	
	RETURN VDOT(dwnrgvec:NORMALIZED, horizvel).

}

// normal vector of the current instantaneous orbital plane
//to be compatible with TargetNormal, it's the left-handed cross prod of position and velocity
FUNCTION currentNormal{
	
	RETURN -VCRS(orbitstate["radius"],orbitstate["velocity"]):NORMALIZED.

}


FUNCTION nominal_cutoff_params {
	PARAMETER tgt_orb.
	PARAMETER cutoff_r.
	
	SET tgt_orb["normal"] TO upfg_normal(tgt_orb["inclination"], tgt_orb["LAN"]).
	
	set tgt_orb["radius"] to cutoff_r.
	
	local cut_alt is tgt_orb["radius"]:MAG.
	set tgt_orb["eta"] to orbit_alt_eta(cut_alt, tgt_orb["SMA"], tgt_orb["ecc"]).
	
	set tgt_orb["velocity"] to orbit_alt_vel(cut_alt, tgt_orb["SMA"]).
	
	set tgt_orb["fpa"] to orbit_eta_fpa(tgt_orb["eta"], tgt_orb["SMA"], tgt_orb["ecc"]).

	RETURN tgt_orb.
}


//only called if hastarget=true
//propagates the current target orbital parameters forward in time 
//given a delta-t and applies secular variations
//given by j2 perturbation
FUNCTION tgt_j2_timefor {
	PARAMETER tgt_orb.
	PARAMETER deltat.
	
	LOCAL j2LNG is -1.5*1.08262668e-3*rad2deg((BODY:RADIUS/(TARGET:ORBIT:SEMIMAJORAXIS*(1 - TARGET:ORBIT:ECCENTRICITY^2)))^2*SQRT(BODY:MU/TARGET:ORBIT:SEMIMAJORAXIS^3)*COS(TARGET:ORBIT:INCLINATION)).
	
	SET tgt_orb["LAN"] TO  fixangle(TARGET:ORBIT:LAN +  j2LNG*deltat ).
	SET tgt_orb["inclination"] TO TARGET:ORBIT:INCLINATION.	

	RETURN tgt_orb.

}							   

FUNCTION warp_window{
	parameter liftofftime.
	
	LOCAL timetolaunch IS liftofftime - TIME:SECONDS.
	addGUIMessage("TIME TO WINDOW : " + sectotime(timetolaunch)).

	UNTIL FALSE {
	
		LOCAL timetolaunch IS liftofftime - TIME:SECONDS.
		
		warp_controller(timetolaunch, TRUE, 2).
		
		IF (timetolaunch <=0.1) {BREAK.}
		
		Wait 0.
	}
	set warp to 0.
}

// calculate target orbital parameters 
//calculate launch azimuth and window and handle warp 
FUNCTION prepare_launch {

	clearvecdraws().

	target_orbit:ADD("direction", "nearest").
	target_orbit:ADD("sma", 0).
	target_orbit:ADD("ecc", 0).
	target_orbit:ADD("eta", 0).
	target_orbit:ADD("LAN", 0).
	target_orbit:ADD("radius", 0).
	target_orbit:ADD("velocity", 0).
	target_orbit:ADD("normal", V(0,0,0)).
	target_orbit:ADD("fpa", 0).
	target_orbit:ADD("launch_az", 0).
	target_orbit:ADD("warp_dt", 0).
	target_orbit:ADD("mode", 1).
	
	//first compute in-plane orbital parameters
	
	//check altitudes
	
	IF target_orbit["periapsis"]>target_orbit["apoapsis"] {
		local ap_ is target_orbit["apoapsis"].
		set target_orbit["apoapsis"] to target_orbit["periapsis"].
		set target_orbit["periapsis"] to ap_.
	}
	
	IF NOT target_orbit:HASKEY("cutoff alt") {
		target_orbit:ADD("cutoff alt", target_orbit["periapsis"]).
	} ELSE {
		IF (target_orbit["cutoff alt"] < target_orbit["periapsis"]) {
			SET target_orbit["cutoff alt"] TO target_orbit["periapsis"].
		} ELSE IF (target_orbit["cutoff alt"] > target_orbit["apoapsis"]) {
			SET target_orbit["cutoff alt"] TO target_orbit["apoapsis"].
		}
	}
	
	SET target_orbit["sma"] TO orbit_appe_sma(target_orbit["apoapsis"], target_orbit["periapsis"]).
	SET target_orbit["ecc"] TO orbit_appe_ecc(target_orbit["apoapsis"], target_orbit["periapsis"]).
	
	LOCAL cutoff_r IS target_orbit["cutoff alt"]*1000 + SHIP:BODY:RADIUS.
	
	//compute cutoff orbital parameters
	
	SET target_orbit["eta"] TO orbit_alt_eta(cutoff_r, target_orbit["sma"], target_orbit["ecc"]).
	set target_orbit["velocity"] to orbit_alt_vel(cutoff_r, target_orbit["sma"]).


	// now compute orbital plane

	// check inclination 
	//overridden in case of targeted launch
	//negative for southerly launches
	IF NOT target_orbit:HASKEY("inclination") {
		target_orbit:ADD("inclination", SHIP:GEOPOSITION:LAT).
	} ELSE {
		IF ABS(target_orbit["inclination"])<SHIP:GEOPOSITION:LAT {
			SET target_orbit["inclination"] TO SIGN(target_orbit["inclination"])*SHIP:GEOPOSITION:LAT.
		}
	}
	
	
	//compute lan given the various options
	
	//the second check only filters targets in earth orbit e.g. no interplanetary targets
	IF (HASTARGET = TRUE AND TARGET:BODY = SHIP:BODY) {
		SET target_orbit["LAN"] TO TARGET:ORBIT:LAN.
		SET target_orbit["inclination"] TO TARGET:ORBIT:INCLINATION.	
		
	} ELSE {
		IF target_orbit["inclination"] < 0 {
			SET target_orbit["direction"] TO "south".
			SET target_orbit["inclination"] TO ABS(target_orbit["inclination"]).	
		} ELSE {
			SET target_orbit["direction"] TO "north".
		}
		SET target_orbit["LAN"] TO LAN_orbit_overhead(target_orbit["inclination"], (target_orbit["direction"]="south"), vehicle["launchTimeAdvance"] + vehicle_countdown + 1).
	}
	
	//handle nearest launch window
	//compute nearest launch direction
	IF (target_orbit["direction"]="nearest") {
			
		LOCAL shiplngvec IS VXCL(V(0,1,0), -SHIP:ORBIT:BODY:POSITION):NORMALIZED.
		
		LOCAL dlng IS get_a_bBB(SHIP:GEOPOSITION:LAT, target_orbit["inclination"]).
		
		LOCAL north_launch_vec IS rodrigues(SOLARPRIMEVECTOR, V(0,1,0), -(target_orbit["LAN"] + dlng)).
		LOCAL south_launch_vec IS rodrigues(SOLARPRIMEVECTOR, V(0,1,0), -(target_orbit["LAN"] + 180 - dlng)).
		
		//arrow_body(north_launch_vec, "north").
		//arrow_body(south_launch_vec, "south").
		//arrow_body(shiplngvec, "ship").
		
		LOCAL north_dlan IS signed_angle(north_launch_vec, shiplngvec, V(0,1,0), 1).
		LOCAL south_dlan IS signed_angle(south_launch_vec, shiplngvec, V(0,1,0), 1).
		
		IF (south_dlan < north_dlan) {
			SET target_orbit["direction"] TO "south". 
		} ELSE {
			SET target_orbit["direction"] TO "north". 
		}
	}
	
	//time to window
	LOCAL time2window IS orbitInterceptTime(target_orbit["inclination"], target_orbit["LAN"], (target_orbit["direction"]="south")).
	
	//if launching into plane of target account for J2 nodal precession
	IF HASTARGET = TRUE AND (TARGET:BODY = SHIP:BODY) {
		LOCAL t2w IS time2window.
		LOCAL j2LNG is -1.5*1.08262668e-3*rad2deg((BODY:RADIUS/(TARGET:ORBIT:SEMIMAJORAXIS*(1 - TARGET:ORBIT:ECCENTRICITY^2)))^2*SQRT(BODY:MU/TARGET:ORBIT:SEMIMAJORAXIS^3)*COS(TARGET:ORBIT:INCLINATION)).
		LOCAL lan_old IS target_orbit["LAN"].
		UNTIL FALSE {
			//print ltt_old AT (0,54).
			SET target_orbit["LAN"] TO  fixangle(lan_old +  j2LNG*t2w ).
			LOCAL t2w_new IS orbitInterceptTime(target_orbit["inclination"], target_orbit["LAN"], (target_orbit["direction"]="south")).
			
			//print ltt_new AT (0,55).
			
			IF ABS(t2w - t2w_new)<0.05 {
				BREAK.
			}
			SET t2w TO t2w_new.
			SET target_orbit["inclination"] TO TARGET:ORBIT:INCLINATION.	
		}
		SET time2window TO t2w.
	}
	
	SET time2window TO time2window - vehicle["launchTimeAdvance"].
	
	IF (time2window < vehicle_countdown) {
		SET time2window TO time2window + SHIP:BODY:ROTATIONPERIOD.
	}
	
	set target_orbit["warp_dt"] to TIME:SECONDS + time2window  - vehicle_countdown.
	
	//this is for message logging
	SET vehicle["ign_t"] TO TIME:SECONDS + time2window. 
	
	set target_orbit["launch_az"] to launchAzimuth(target_orbit["inclination"], target_orbit["velocity"], (target_orbit["direction"]="south")).	
	
	warp_window(target_orbit["warp_dt"]).	
	
	//print target_orbit:dump.
	//arrow_body(targetLANvec(target_orbit["LAN"]), "lan").
	//arrow_body(targetNormal(target_orbit["inclination"], target_orbit["LAN"]), "norm").
	//until false{}

}






//		NAVIGATION FUNCTIONS 



FUNCTION update_navigation {
	
	SET surfacestate["MET"] TO TIME:SECONDS. 
	
	
	//measure position and orbit parameters
	
	LOCAL progv IS v(0,0,0).
	

	
	IF (vehiclestate["phase"] > 1) {set progv to SHIP:PROGRADE:VECTOR.}
	ELSE {set progv to SHIP:SRFPROGRADE:VECTOR.}
	
	SET surfacestate["hdir"] TO compass_for(progv,SHIP:GEOPOSITION ).
	SET surfacestate["vdir"] TO 90 - VANG(progv, SHIP:UP:VECTOR).
	SET surfacestate["pitch"] TO 90 - VANG(SHIP:FACING:VECTOR, SHIP:UP:VECTOR).	
	SET surfacestate["az"] TO compass_for(SHIP:FACING:VECTOR,SHIP:GEOPOSITION ).
	SET surfacestate["alt"] TO SHIP:ALTITUDE.
	SET surfacestate["vs"] TO SHIP:VERTICALSPEED.
	SET surfacestate["hs"] TO SHIP:VELOCITY:SURFACE:MAG.
	
	SET orbitstate["velocity"] TO vecYZ(SHIP:ORBIT:VELOCITY:ORBIT).
	SET orbitstate["radius"] TO vecYZ(SHIP:ORBIT:BODY:POSITION)*-1.

}