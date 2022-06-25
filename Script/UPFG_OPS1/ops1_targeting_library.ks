
//GLOBAL NAV VARIABLES 

GLOBAL launchpad IS SHIP:GEOPOSITION.
GLOBAL surfacestate IS  LEXICON("MET",0,"az",0,"pitch",0,"alt",0,"vs",0,"hs",0,"vdir",0,"hdir",0,"q",0).
GLOBAL orbitstate IS  LEXICON("radius",0,"velocity",0). 


//			VARIOUS TARGETING FUNCTIONS



//	Target plane normal vector in MATLAB coordinates, UPFG compatible direction
FUNCTION targetNormal {
	DECLARE PARAMETER targetInc.	//	Expects a scalar
	DECLARE PARAMETER targetLan.	//	Expects a scalar
	
	//	First create a vector pointing to the highest point in orbit by rotating the prime vector by a right angle.
	LOCAL highPoint IS rodrigues(SOLARPRIMEVECTOR, V(0,1,0), 90-targetLan).
	//	Then create a temporary axis of rotation (short form for 90 deg rotation).
	LOCAL rotAxis IS V(-highPoint:Z, highPoint:Y, highPoint:X).
	//	Finally rotate about this axis by a right angle to produce normal vector.
	LOCAL normalVec IS rodrigues(highPoint, rotAxis, 90-targetInc).
	
	RETURN -vecYZ(normalVec).
}

//computes periapsis vector (normalised) given target orbit and longitude of periapsis
FUNCTION target_perivec {
	LOCAL peri IS v(0,0,0).
	
	IF target_orbit["mode"] = 2 {
		set peri to rodrigues(vecYZ(SOLARPRIMEVECTOR), V(0,0,1), target_orbit["LAN"]).
		set peri to rodrigues(peri, -target_orbit["normal"], target_orbit["periarg"]).
	}
	ELSE {
		SET peri TO target_orbit["radius"].
	}
	
	return peri:NORMALIZED.

}



FUNCTION cutoff_params {
	PARAMETER target.
	PARAMETER cutoff_r.
	PARAMETER etaa.
	
	LOCAL mode IS target["mode"].
	
	SET target["normal"] TO targetNormal(target["inclination"], target["LAN"]).
	
	local x is 1 + target["ecc"]*COS(etaa).
	
	local r is cutoff_r:MAG.
	
	IF mode=2{ //given sma, ecc and eta, compute r
		set r to target["SMA"]*(1-target["ecc"]^2)/x.	
	}
	ELSE IF mode=1 {	//given sma, ecc and r, compute eta
		IF target["ecc"]=0 {set etaa to  0.}
		ELSE {		
			set etaa to (target["SMA"]*(1-target["ecc"]^2)/r - 1)/target["ecc"].
			set etaa to ARCCOS(limitarg(etaa)).
		}
		set x to 1 + target["ecc"]*COS(etaa).
	}
	
	
	//ELSE IF mode=1 {	//given r, ecc and eta, compute sma
	//	SET target["SMA"] TO x*r/(1-target["ecc"]^2).
	//}

	local v is SQRT(SHIP:BODY:MU * (2/r - 1/target["SMA"])).
		
	local phi is target["ecc"]*sin(etaa)/x.
	set phi to ARCTAN(phi).
	
	set target["radius"] to cutoff_r:NORMALIZED*r.
	set target["velocity"] to v.
	set target["angle"] to phi.
	set target["eta"] to etaa.
	
	
	
	
	RETURN target.
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


// calculate target orbital parameters 
//calculate launch azimuth and window and handle warp 
FUNCTION prepare_launch {

	FUNCTION compute_periarg {
	
		local periarg is 0.
		local vnorm IS -target_orbit["normal"]:NORMALIZED.
		local lanvec is rodrigues(vecYZ(SOLARPRIMEVECTOR), V(0,0,1), target_orbit["LAN"]).
		local peri is V(0,0,0).
		
		IF target_orbit["mode"] = "orbit" {
		
			LOCAL tgtlong IS convert_long(fixangle(target_orbit["Longitude of Periapsis"]),1).		

			SET peri TO rodrigues(vecYZ(SOLARPRIMEVECTOR), V(0,0,1), tgtlong):NORMALIZED.
			LOCAL nvec1 IS VCRS(peri,V(0,0,1)):NORMALIZED.
			LOCAL nvec2 IS VXCL(nvec1,vnorm):NORMALIZED.
			SET peri TO VXCL(nvec2,peri):NORMALIZED.
			
			//LOCAL nvec2 IS (vnorm - VDOT(vnorm,nvec1)*nvec1):NORMALIZED.
			//SET peri TO (peri - VDOT(peri,nvec2)*nvec2).
		
		}
		ELSE {
			SET peri TO target_orbit["radius"].
		} 
		//set periarg to signed_angle(peri,lanvec,vnorm,1).
		set periarg to signed_angle(lanvec,peri,vnorm,1).
		return periarg.
	}
	
	
	FUNCTION LAN_orbit_overhead {
	
		IF target_orbit["direction"] = "nearest" { SET target_orbit["direction"] TO "north". }
		LOCAL currentNode IS nodeVector(target_orbit["inclination"], target_orbit["direction"]).
		LOCAL currentLan IS VANG(currentNode, SOLARPRIMEVECTOR).
		IF VDOT(V(0,1,0), VCRS(currentNode, SOLARPRIMEVECTOR)) < 0 { SET currentLan TO 360 - currentLan. }
		
		LOCAL LAN_out IS currentLan + (vehicle["launchTimeAdvance"]+ 10.1)*360/SHIP:ORBIT:BODY:ROTATIONPERIOD.
		
		RETURN LAN_out.
	
	}
	
	
	//	Ascending node vector of the orbit passing right over the launch site
	FUNCTION nodeVector {
		DECLARE PARAMETER inc.				//	Inclination of the desired orbit. Expects a scalar.
		DECLARE PARAMETER dir IS "north".	//	Launch direction. Expects a string, either "north" or "south".
		
		//	From right spherical triangle composed of inclination, latitude and "b",
		//	which is angular difference between the desired node vector and projection
		//	of the vector pointing at the launch site onto the equatorial plane.
		LOCAL b IS TAN(90-inc)*TAN(SHIP:GEOPOSITION:LAT).
		SET b TO ARCSIN( MIN(MAX(-1, b), 1) ).
		LOCAL longitudeVector IS VXCL(V(0,1,0), -SHIP:ORBIT:BODY:POSITION):NORMALIZED.
		IF dir = "north" {
			RETURN rodrigues(longitudeVector, V(0,1,0), b).
		} ELSE IF dir = "south" {
			//	This can be easily derived from spherical triangle if one draws a half
			//	of an orbit, from node to node. It is obvious that distance from node to
			//	peak equals 90 degrees, and from that the following results.
			RETURN rodrigues(longitudeVector, V(0,1,0), 180-b).
		} ELSE {
			RETURN nodeVector(inc, "north").
		}
	}
	

	
	

	//	Time to next launch opportunity in given direction
	FUNCTION orbitInterceptTime {
		DECLARE PARAMETER launchDir IS target_orbit["direction"].	//	Passing as parameter for recursive calls.
		
		//	Expects a global variable "target_orbit" as lexicon
		LOCAL targetInc IS target_orbit["inclination"].
		LOCAL targetLan IS target_orbit["LAN"].
		
		//	For "nearest" launch opportunity:
		IF launchDir = "nearest" {
			LOCAL timeToNortherly IS orbitInterceptTime("north").
			LOCAL timeToSoutherly IS orbitInterceptTime("south").
			IF timeToSoutherly < timeToNortherly {
				SET target_orbit["direction"] TO "south".
				RETURN timeToSoutherly.
			} ELSE {
				SET target_orbit["direction"] TO "north".
				RETURN timeToNortherly.
			}
		} ELSE {
			//	Tind the ascending node vector of an orbit of the desired inclination,
			//	that passes above the launch site right now.
			SET currentNode TO nodeVector(targetInc, launchDir).
			//	Then find the ascending node vector of the target orbit.
			LOCAL targetNode IS rodrigues(SOLARPRIMEVECTOR, V(0,1,0), -targetLan).
			//	Find the angle between them, minding rotation direction, and return as time.
			LOCAL nodeDelta IS VANG(currentNode, targetNode).
			LOCAL deltaDir IS VDOT(V(0,1,0), VCRS(targetNode, currentNode)).
			IF deltaDir < 0 { SET nodeDelta TO 360 - nodeDelta. }
			LOCAL deltaTime IS SHIP:ORBIT:BODY:ROTATIONPERIOD * nodeDelta/360.
			
			RETURN deltaTime.
		}
	}

	//	Launch azimuth to a given orbit
	FUNCTION launchAzimuth {
	
		//	Expects global variables "target_orbit" as lexicons
		
		LOCAL targetInc IS target_orbit["inclination"].
		//internal flag for retrograde launches, northerly or southerly alike
		LOCAL retro IS FALSE.
		IF (targetInc > 90) {
			SET retro TO TRUE.
		}
		
		LOCAL targetAlt IS target_orbit["radius"]:MAG.
		//LOCAL targetVel IS SQRT( SHIP:ORBIT:BODY:MU/(SHIP:BODY:RADIUS+targetAlt*1000) ).		//	This is a normal calculation for a circular orbit
		LOCAL targetVel IS target_orbit["velocity"]*COS(target_orbit["angle"]).				//	But we already have our desired velocity, however we must correct for the flight path angle (only the tangential component matters here)
		
		LOCAL siteLat IS SHIP:GEOPOSITION:LAT.
		IF targetInc < siteLat { //pushUIMessage( "Target inclination below launch site!", 5, PRIORITY_HIGH ).

		}
		
		//get "orbit heading angle" between north and orbit plane at launch latitude
		LOCAL cincl IS COS(targetInc).
		IF (retro) {
			SET cincl TO COS(180 - targetInc).
		}
		LOCAL Binertial IS ABS(cincl/COS(siteLat)).
		IF Binertial < -1 { SET Binertial TO -1. }
		IF Binertial > 1 { SET Binertial TO 1. }
		SET Binertial TO ARCSIN(Binertial).		//	In case of an attempt at launch to a lower inclination than reachable
		
		//get launch azimuth angle wrt due east=0
		LOCAL Vbody IS (2*CONSTANT:PI*SHIP:BODY:RADIUS/SHIP:BODY:ROTATIONPERIOD)*COS(siteLat).
		LOCAL VrotX IS targetVel*SIN(Binertial)-Vbody.
		LOCAL VrotY IS targetVel*COS(Binertial).
		LOCAL azimuth IS ARCTAN2(VrotY, VrotX).
		//if retrograde, correct
		IF (retro) {
			SET azimuth TO 180 - azimuth.
		}
		
		//	In MATLAB an azimuth of 0 is due east, while in KSP it's due north.
		//	Return the valid value depending on the launch direction:
		LOCAL out IS 0.
		IF target_orbit["direction"] = "north" {
			SET out TO 90-azimuth.
		} ELSE IF target_orbit["direction"] = "south" {
			SET out TO  90+azimuth.
		} ELSE {
			SET out TO  90-azimuth.
		}
		
		//if retrograde, the result will be negative. correct
		SET azimuth TO fixangle(azimuth).
		
		//implement range azimuth limitation
		LOCAL site_azrange IS LEXICON(
							"KSC",LEXICON(
									"position",LATLNG(28.61938,-80.70092),
									"min_az",35,
									"max_az",120
							),
							"Vandenberg",LEXICON(
									"position",LATLNG(34.67974,-120.53102),
									"min_az",147,
									"max_az",201	//250
							)
		
		).
		LOCAL shippos IS SHIP:GEOPOSITION.
		FOR s IN site_azrange:VALUES{
			LOCAL sitepos IS s["position"].
			
			//if the launchsite is within 100km of a known site
			//apply its range restrictions
			IF downrangedist(sitepos,shippos) < 100 {
				SET out TO CLAMP(out,s["min_az"],s["max_az"]).
				BREAK.
			}
		
		}
		
		RETURN out.
	}

	FUNCTION warp_window{
	
		parameter liftofftime.
		
		LOCAL timetolaunch IS liftofftime - TIME:SECONDS.
		

		UNTIL timetolaunch <=0.1 {
		
			SET timetolaunch TO liftofftime - TIME:SECONDS.
			
			IF timetolaunch > 3600 or timetolaunch < 0    {set warp to 4.}
			ELSE IF timetolaunch > 400     {set warp to 3.}
			ELSE IF timetolaunch > 60  {set warp to 2.}
			ELSE IF timetolaunch > 1  {set warp to 1.}
			ELSE 							{set warp to 0.}
			
			PRINT "                                                               " at (1,23).
			PRINT "	TIME TO WINDOW : " + sectotime(timetolaunch) at (1,23).
			Wait 0.
		}
		set warp to 0.
	
	
	}


	
	
	PRINT " PREPARING TO LAUNCH " AT (0,5).
	
	target_orbit:ADD("radius", 0) .
	target_orbit:ADD("velocity", 0) .
	target_orbit:ADD("normal", V(0,0,0)) .
	target_orbit:ADD("angle", 0) .
	target_orbit:ADD("periarg", 0) .
	target_orbit:ADD("mode", 0) .
	
	
	PRINT " COMPUTING IN-PLANE TARGET ORBITAL PARAMETERS" AT (0,7).
	
	IF target_orbit["periapsis"]>target_orbit["apoapsis"] {
		local x is target_orbit["apoapsis"].
		set target_orbit["apoapsis"] to target_orbit["periapsis"].
		set target_orbit["periapsis"] to x.
	}
	
	LOCAL pe IS target_orbit["periapsis"]*1000 + SHIP:BODY:RADIUS.
	LOCAL ap IS target_orbit["apoapsis"]*1000 + SHIP:BODY:RADIUS.
	target_orbit:ADD("SMA", (pe+ap) / 2) .
	
	target_orbit:ADD("ecc", (ap - pe)/(ap + pe)).
	
	
	
	PRINT " COMPUTING TARGET ORBITAL PLANE" AT (0,9).
	
	
	IF NOT target_orbit:HASKEY("inclination") {
		target_orbit:ADD("inclination", ABS(SHIP:GEOPOSITION:LAT)).
	}
	
	
	//	Set default launch direction
	IF NOT target_orbit:HASKEY("direction") {
		target_orbit:ADD("direction", "nearest").
	}
	
	
	PRINT " COMPUTING TARGET LAN" AT (0,11).

	//the second check only filters targets in earth orbit e.g. no interplanetary targets
	IF HASTARGET = TRUE AND (TARGET:BODY = SHIP:BODY) {
		IF NOT target_orbit:HASKEY("LAN") {
			target_orbit:ADD("LAN",0).
		}
		SET target_orbit["LAN"] TO TARGET:ORBIT:LAN.
		SET target_orbit["inclination"] TO TARGET:ORBIT:INCLINATION.	
		
	}
	ELSE {
			IF ABS(target_orbit["inclination"])<SHIP:GEOPOSITION:LAT {
				SET target_orbit["inclination"] TO SIGN(target_orbit["inclination"])*SHIP:GEOPOSITION:LAT.
			}
			
			IF target_orbit["inclination"] >= 0 {
					SET target_orbit["direction"] TO "north".
			}
			ELSE IF target_orbit["inclination"] < 0 {
				SET target_orbit["direction"] TO "south".
				SET target_orbit["inclination"] TO 	ABS(target_orbit["inclination"]).	
			}
			
		//SET target_orbit["LAN"] TO .
		IF NOT target_orbit:HASKEY("LAN") {
			target_orbit:ADD("LAN",LAN_orbit_overhead()).
		}
		ELSE {
			SET target_orbit["LAN"] TO convert_long(fixangle(target_orbit["LAN"]),1).
		}
		
	}
	SET target_orbit["normal"] TO targetNormal(ABS(target_orbit["inclination"]), target_orbit["LAN"]).

	

	
	local vnorm is -target_orbit["normal"]:NORMALIZED.
	local cutvec is (vecYZ(SHIP:BODY:POSITION)*-1):NORMALIZED.
	set cutvec to (cutvec - VDOT(cutvec,vnorm)*vnorm):NORMALIZED.
	
	//arbitrarily set cutoff point at 30 degrees ahead of the launch position.
	set cutvec to rodrigues(cutvec,vnorm, 30):NORMALIZED.
	
	set target_orbit["radius"] TO cutvec.
	
	PRINT " COMPUTING TARGET ARGUMENT OF PERIAPSIS" AT (0,13).
	
	
	IF target_orbit:HASKEY("Longitude of Periapsis") {
			SET target_orbit["mode"] TO 2.
			SET target_orbit["periarg"] TO compute_periarg().
			target_orbit:REMOVE("Longitude of Periapsis").
	}
	ELSE {
		SET target_orbit["mode"] TO 1.
		LOCAL cut_alt IS target_orbit["periapsis"].
		IF target_orbit:HASKEY("Cutoff Altitude") {
			SET cut_alt TO target_orbit["Cutoff Altitude"].
			target_orbit:REMOVE("Cutoff Altitude").
		}
		IF cut_alt<target_orbit["periapsis"] {
			SET cut_alt TO target_orbit["periapsis"].
		}
		SET cut_alt TO (cut_alt*1000 + SHIP:BODY:RADIUS).
		set target_orbit["radius"] TO cutvec:NORMALIZED*cut_alt.
	}
	
	
	

	
	PRINT " COMPUTING PERIAPSIS VECTOR" AT (0,15).
	target_orbit:ADD("perivec", target_perivec()) .
	
	PRINT " ESTIMATE CUTOFF CONDITIONS" AT (0,17).
	local etaa is 0.

	IF target_orbit["mode"] = 2 {
		set etaa to signed_angle(target_orbit["perivec"],cutvec,vnorm,1).
	}
	IF target_orbit["mode"] = 1 {
		IF NOT target_orbit["ecc"]=0 {
			set etaa to (target_orbit["SMA"]*(1-target_orbit["ecc"]^2)/target_orbit["radius"]:MAG - 1)/target_orbit["ecc"].
			set etaa to ARCCOS(etaa).		
		}
	}
	target_orbit:ADD("eta", etaa) .
	SET target_orbit TO cutoff_params(target_orbit,target_orbit["radius"],etaa).
	
	PRINT " CALCULATING TIME TO LAUNCH " AT (0,19).	
	
	//	Calculate time to launch
	LOCAL timeToOrbitIntercept IS orbitInterceptTime().
	LOCAL liftoffTime IS TIME:SECONDS + timeToOrbitIntercept - vehicle["launchTimeAdvance"].
	
	
	//if launching into plane of target account for J2 nodal precession
	IF HASTARGET = TRUE AND (TARGET:BODY = SHIP:BODY) {
		LOCAL ltt_old IS liftoffTime.
		LOCAL j2LNG is -1.5*1.08262668e-3*rad2deg((BODY:RADIUS/(TARGET:ORBIT:SEMIMAJORAXIS*(1 - TARGET:ORBIT:ECCENTRICITY^2)))^2*SQRT(BODY:MU/TARGET:ORBIT:SEMIMAJORAXIS^3)*COS(TARGET:ORBIT:INCLINATION)).
		LOCAL lan_old IS target_orbit["LAN"].
		UNTIL FALSE {
			print ltt_old AT (0,54).
			SET target_orbit["LAN"] TO  fixangle(lan_old +  j2LNG*ltt_old ).
			LOCAL ltt_new IS orbitInterceptTime().
			print ltt_new AT (0,55).
			IF ABS(ltt_old - ltt_new)<0.05 {
				SET ltt_old TO ltt_new.
				BREAK.
			}
			SET ltt_old TO ltt_new.
			SET target_orbit["inclination"] TO TARGET:ORBIT:INCLINATION.	
		}
		SET liftoffTime TO TIME:SECONDS + ltt_old - vehicle["launchTimeAdvance"].
	}
	IF timeToOrbitIntercept < vehicle["launchTimeAdvance"] { SET liftoffTime TO liftoffTime + SHIP:BODY:ROTATIONPERIOD. }
	PRINT " CALCULATING LAUNCH AZIMUTH" AT (0,21).		
	set control["launch_az"] to launchAzimuth().	
		
	warp_window(liftoffTime).
	
		
	PRINT " COMPLETE. STARTING COUNTDOWN." AT (0,25).	
	
}	






//		NAVIGATION FUNCTIONS 



FUNCTION update_navigation {
	
	SET surfacestate["MET"] TO TIME:SECONDS. 
	
	
	//measure position and orbit parameters
	
	IF vehiclestate["ops_mode"] >1 {set v to SHIP:PROGRADE:VECTOR.}
	ELSE {set v to SHIP:SRFPROGRADE:VECTOR.}
	
	SET surfacestate["hdir"] TO compass_for(v,SHIP:GEOPOSITION ).
	SET surfacestate["vdir"] TO 90 - VANG(v, SHIP:UP:VECTOR).
	SET surfacestate["pitch"] TO 90 - VANG(SHIP:FACING:VECTOR, SHIP:UP:VECTOR).	
	SET surfacestate["az"] TO compass_for(SHIP:FACING:VECTOR,SHIP:GEOPOSITION ).
	SET surfacestate["alt"] TO SHIP:ALTITUDE.
	SET surfacestate["vs"] TO SHIP:VERTICALSPEED.
	SET surfacestate["hs"] TO SHIP:VELOCITY:SURFACE:MAG.
	
	SET orbitstate["velocity"] TO vecYZ(SHIP:ORBIT:VELOCITY:ORBIT).
	SET orbitstate["radius"] TO vecYZ(SHIP:ORBIT:BODY:POSITION)*-1.

}