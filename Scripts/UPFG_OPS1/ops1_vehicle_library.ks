//Global vars for running average of thrust calculation
GLOBAL count IS 1.
GLOBAL len IS 1.
GLOBAL time_stg IS 0.
GLOBAL thrust IS LIST().
GLOBAL j IS 1.  //stage counter
GLOBAL m_burn_left IS 0.
FROM {LOCAL k IS 1.} UNTIL k > 6 STEP { SET k TO k+1.} DO{thrust:ADD(0).}

GLOBAL g0 IS 9.80665. 


//VEHICLE INITIALISATION FUNCTION 

declare function initialise_vehicle{

	RUNPATH("0:/Libraries/resources_library").	

	
	FUNCTION add_resource {
		parameter lexx.
		parameter reslist.
		
		
		for res in reslist {
			IF NOT lexx:HASKEY(res) {lexx:ADD(res,0).}
		}
		RETURN lexx.
	}
	
	
	
	FUNCTION fix_mass_params {
		parameter stg.
		
		IF stg:HASKEY("m_burn") {
			IF stg:HASKEY("m_initial") {
				IF NOT stg:HASKEY("m_final") {
					stg:ADD("m_final",0).
					SET stg["m_final"] TO stg["m_initial"] - stg["m_burn"].
				}
			}
			ELSE IF stg:HASKEY("m_final") {
				IF NOT stg:HASKEY("m_initial") {
					stg:ADD("m_initial",0).
					SET stg["m_initial"] TO stg["m_final"] + stg["m_burn"].
				}
			}
		}
		ELSE IF stg:HASKEY("m_initial") AND stg:HASKEY("m_final") {
			IF NOT stg:HASKEY("m_burn") {
				stg:ADD("m_burn",0).
				SET stg["m_burn"] TO stg["m_initial"] - stg["m_final"].
			}
		}
		ELSE {
			PRINT ("ERROR! VEHICLE MASS PARAMETERS ILL-DEFINED") AT (1,40).
			LOCAL X IS 1/0.
		}
		
	
	}
	
	
	
	
	//local m_bias IS ship:mass.
	//SET m_bias TO m_bias - vehicle["stages"][1]["m_initial"].
	//IF m_bias<0.5 {
	
	//set m_bias to 0.
	//}

	//IF NOT vehicle:HASKEY("offaxis_thrust") {vehicle:ADD("offaxis_thrust",v(0,0,0)).}

	FROM {LOCAL k IS 1.} UNTIL k > (vehicle["stages"]:LENGTH - 1) STEP { SET k TO k+1.} DO{
	
		local stg IS vehicle["stages"][k].
	
	
		LOCAL iisspp IS 0.
		LOCAL tthrust IS 0.
		LOCAL fflow IS 0.
		local stage_res IS LEXICON().
		
		FOR v in stg["engines"] {
			SET tthrust TO tthrust + v["thrust"].
			SET iisspp TO iisspp + v["isp"]*v["thrust"].
			SET fflow TO fflow + v["flow"].
			SET stage_res TO add_resource(stage_res,v["resources"]).
		}
		SET stg["engines"] TO LEXICON("thrust", tthrust, "isp", iisspp/tthrust, "flow", fflow).
		

		SET stage_res TO res_dens_init(stage_res).
		
		stg:ADD("resources",stage_res).
		IF NOT (stg:HASKEY("Throttle")) {stg:ADD("Throttle",1).}
	
		IF NOT (stg:HASKEY("Tstage")) {stg:ADD("Tstage",0).}
		//stg:ADD("ign_t", 0).
		
		fix_mass_params(stg).
		
		SET stg["m_initial"] 			TO stg["m_initial"]*1000.
		SET stg["m_final"] 			TO stg["m_final"]*1000.
		SET stg["m_burn"] 				TO stg["m_burn"]*1000.
		SET stg["engines"]["thrust"] 	TO stg["engines"]["thrust"]*1000.
		
				
		//SET stg["m_initial"] TO stg["m_initial"] + m_bias.
		//SET stg["m_final"] TO stg["m_final"] + m_bias.

		IF NOT stg:HASKEY("mode") {	
			stg:ADD("mode", 1).	
		}
		
		local stg_stagingtype IS stg["staging"]["type"].
		
		IF stg_stagingtype="time" {
			SET stg["Tstage"] TO stg["Tstage"].
		} 
		ELSE IF (stg_stagingtype="depletion" OR stg_stagingtype="m_burn") {
			SET stg["Tstage"] TO (stg["m_burn"])/stg["engines"]["flow"].
		
		}
		ELSE IF stg_stagingtype="glim" {
		
			//will the stage exceed the g limit?
			LOCAL x IS glim_t_m(stg).

			
			If x[0] > 0 {
				//yes it will exceed the limit

				IF stg:HASKEY("minThrottle") {
					//the stage will be followed by a constant-accel stage which has
					//to be created from scratch
			
					//	In case user accidentally entered throttle as percentage instead of a fraction
					IF stg["minThrottle"] > 1.0	{ SET stg["minThrottle"] TO stg["minThrottle"]/100. }
					
					
					LOCAL new_stg  IS LEXICON(
												"m_initial",	x[1],
												"m_final",	stg["m_final"],
												"m_burn", x[1] - stg["m_final"],
												"staging", LEXICON (
															"stg_action",{},
															"type","depletion",
															"ignition",	FALSE,
															"ullage", "none",
															"ullage_t",	0
												),
												"engines",	stg["engines"],
												"resources",stg["resources"],
												"Tstage",0,
												"mode", 2,
												"glim",stg["glim"],
												"Throttle",stg["minThrottle"],
												"minThrottle",stg["minThrottle"],
												"throt_mult",0
										).
					
					SET new_stg["Tstage"] TO glim_stg_time(new_stg).
					vehicle["stages"]:INSERT(k+1, new_stg).

					SET k TO k+1.
				}
				ELSE {
					//the stage will be followed by a different stage in the sequence
					//which is however already present and only needs different mass parameters
					
					local nextstg IS vehicle["stages"][k+1].
					
					fix_mass_params(nextstg).
					
					SET nextstg["m_initial"] TO x[1]/1000.
					SET nextstg["m_final"] TO stg["m_final"]/1000.
					SET nextstg["m_burn"] TO nextstg["m_initial"] - nextstg["m_final"].
					SET nextstg["staging"]["type"] TO "depletion".
				
				}	
				
				SET stg["mode"] TO 1.
				SET stg["Tstage"] TO x[0].
				SET stg["m_final"] TO x[1].
				SET stg["m_burn"] TO stg["m_initial"] - x[1].
			
			}
			ELSE {
				//no it will never exceed the limit.
				//convert it to a depletion stage.
				
				SET stg["staging"]["type"] TO "depletion".
				SET stg["Tstage"] TO (stg["m_burn"])/stg["engines"]["flow"].
			}
		
		}
		
		
		
	}
	SET m_burn_left TO vehicle["stages"][1]["m_burn"].
	
	vehicle:ADD("ign_t", 0).

	WHEN vehicle["stages"][j]["Tstage"] <= 3 THEN {STAGING().}
	
	PRINT " INITIALISATION COMPLETE" AT (0,3).
	

	
	
}




									//CONTROL FUNCTIONS



//open-loop pitch profile for first stage ascent
FUNCTION pitch {
	PARAMETER v.
	PARAMETER v0.
	PARAMETER scale.			 
	
	LOCAL default IS 90.

	LOCAL out IS default.
	
	IF v>v0 {
		
		LOCAL p1 IS -0.0048.
		LOCAL p2 IS 28.8.
		LOCAL p3 IS 26300.
		LOCAL q1 IS 3.923.
		
		LOCAL x IS v + 400.391 - v0.
	
		SET out TO (p1*x^2 + p2*x + p3)/(x + q1).
		
		SET out TO out*(1 + scale*(1 - out/default)).
		
		LOCAL bias IS out - surfacestate["vdir"].
		
		SET out TO out + 0.8*bias.
		
		
	}

	RETURN CLAMP(out,0,default).

}



//compute net thrust vector as thrust-weighted average of engines position 
//relative to the ship raw frame
//obtain the difference between fore vector and thrust vector.
//then, given input reference "fore" and "up" vectors, rotate in that frame
FUNCTION thrustrot {

	PARAMETER ref_fore.
	PARAMETER ref_up.

	LOCAL thrvec IS v(0,0,0).
	local offs is v(0,0,0).
	LOCAL thr is 0.
	
	list ENGINES in all_eng.
	FOR e IN all_eng {
		IF e:ISTYPE("engine") {
			IF e:IGNITION {
				SET thr TO thr + (e:THRUST).
				//set x to x + 1.
				local v is -e:POSITION:NORMALIZED*e:THRUST.
				set offs to offs + v.
			}
		}
	}	
	set thrvec to (offs/thr):NORMALIZED .
	local ship_fore IS SHIP:FACING:VECTOR:NORMALIZED.

	//local ship_up IS SHIP:FACING:TOPVECTOR:NORMALIZED.
	//
	//SET thrvec TO ship_fore - thrvec.
	//
	////RETURN thrvec.
	//
	////SET thrvec TO thrvec - ship_fore.
	////SET thrvec TO rodrigues(thrvec, ship_fore, 180).
	//
	//LOCAL theta IS signed_angle(ship_fore, thrvec, VCRS(ship_fore, ship_up ), 1).
	//LOCAL phi IS signed_angle(ship_fore, thrvec, ship_up , 1).
	//
	//LOCAL out IS rodrigues(ref_fore, VCRS(ref_fore, ref_up ), theta).
	//SET out TO rodrigues(out,  ref_up , phi).
	//SET out TO out:NORMALIZED*thrvec:MAG.
	
	local out is ship_fore - thrvec.
	
	RETURN out.
}




//	Returns a kOS direction for given aim vector, reference up vector and roll angle.
//corrects for thrust offset
FUNCTION aimAndRoll {
	DECLARE PARAMETER aimVec.	//	Expects a vector
	DECLARE PARAMETER upVec.	//	Expects a vector
	DECLARE PARAMETER rollAng.	//	Expects a scalar
	
	
	//clearvecdraws().
	//arrow(upVec,"upvec",v(0,0,0),10,0.05).
	
	SET upVec TO VXCL(aimVec,upVec):NORMALIZED.
	SET upVec TO rodrigues(upVec, aimVec, rollAng).
	
	//print "      " at (2,45).
	//print rollAng at (2,45).
	
	
	//arrow(aimVec,"forevec",v(0,0,0),10,0.05).
	//arrow(upVec,"topvec",v(0,0,0),10,0.05).
	
	LOCAL current_up IS VXCL(aimVec,SHIP:FACING:TOPVECTOR):NORMALIZED.
	//arrow(current_up,"current_up",v(0,0,0),10,0.05).

	SET aimVec TO aimVec + thrustrot(aimVec,upVec).
	

	RETURN LOOKDIRUP(aimVec, upVec).
}

//given current vehicle fore vector, computes where the thrust is pointing
FUNCTION thrust_vec {
	RETURN SHIP:FACING:VECTOR:NORMALIZED - thrustrot(v(0,0,0),v(0,0,0)).
}




//	Throttle controller
FUNCTION throttleControl {

	local stg IS vehicle["stages"][j].
	local throtval is stg["Throttle"].
	
	LOCAL minthrot IS 0.
	IF stg:HASKEY("minThrottle") {
		SET minthrot TO stg["minThrottle"].
	}
	
	IF stg["mode"] = 2   {
		//SET minthrot TO stg["minThrottle"].
		SET throtval TO stg["throt_mult"]*SHIP:MASS*1000.
		SET usc["lastthrot"] TO throtval.
		SET stg["Throttle"] tO throtval.
	}
	
	SET throtval TO MIN(1,(throtval - minthrot)/(1 - minthrot)).			
	
	IF throtval < 0.005 {
		SET throtval TO 0.005.
		
		IF stg["mode"] = 2 {
			SET stg["mode"] TO 1.
			SET usc["lastthrot"] TO stg["minThrottle"].
			
		}
	}
	
	RETURN throtval.
}











									//VEHICLE PERFORMANCE & STAGING FUNCTIONS


FUNCTION get_stage {
	RETURN vehicle["stages"][j].
}



FUNCTION events_handler {

	local met is TIME:SECONDS - vehicle["ign_t"].

	local x IS events:LENGTH.
	
	local rem_list IS LIST().

	FROM {LOCAL k IS 0.} UNTIL k >= x STEP { SET k TO k+1.} DO{
		
		IF met>events[k]["time"]  {
			IF events[k]["type"]="jettison" {
				TOGGLE AG8.
				IF events[k]:HASKEY("mass") {
					FROM { LOCAL i IS j. } UNTIL i > (vehicle["stages"]:LENGTH - 1)  STEP { SET i TO i+1. } DO {
						SET vehicle["stages"][i]["m_initial"] TO vehicle["stages"][i]["m_initial"] - events[k]["mass"].
						SET vehicle["stages"][i]["m_final"] TO vehicle["stages"][i]["m_final"] - events[k]["mass"].
					}
				}
				rem_list:ADD(k).
				SET x TO x-1.
				SEt k TO k-1.
			}
			ELSE IF events[k]["type"]="roll" {
				//SET STEERINGMANAGER:MAXSTOPPINGTIME TO 0.4.
								
				IF ABS(events[k]["angle"] - vehicle["roll"])<5 {
					set vehicle["roll"] TO events[k]["angle"].
					rem_list:ADD(k).
					SET x TO x-1.
					SEt k TO k-1.
					
				} ELSE {
					//local rollsign is SIGN(events[k]["angle"] - vehicle["roll"]).
					
					local rollsign is SIGN( unfixangle( events[k]["angle"] - vehicle["roll"] ) ).
					set vehicle["roll"] TO fixangle(vehicle["roll"] + rollsign*5).
				} 
			}

			ELSE IF events[k]["type"]="action" { 
				IF events[k]:HASKEY("action") {
					events[k]["action"]:call().
				}
				rem_list:ADD(k).
			}
				
			
			
		}
	}
	
	FOR j IN rem_list {
		events:REMOVE(rem_list[j]).
	}
}

FUNCTION get_mass_bias {

	LOCAL stg IS vehicle["stages"][1].
		
	IF NOT stg:HASKEY("tankparts") {get_stg_tanks(stg).}
	local res_left IS get_prop_mass(stg).

	local dm IS m_burn_left - res_left.
	
	local m_bias IS SHIP:MASS*1000.
	SET m_bias TO m_bias - stg["m_initial"] + dm.
	IF m_bias<0.5 {
	
	set m_bias to 0.
	}

	FROM {LOCAL k IS 1.} UNTIL k > (vehicle["stages"]:LENGTH - 1) STEP { SET k TO k+1.} DO{	
		local stg IS vehicle["stages"][k].
		SET stg["m_initial"] TO stg["m_initial"] + m_bias.
		SET stg["m_final"] TO stg["m_final"] + m_bias.
	}

}



//calculates when the g limit will be violated and the vehicle mass at that moment
FUNCTION glim_t_m {
		PARAMETER stg.
		local out is LIST(0,0).
		
		local mbreak is stg["engines"]["thrust"]/(stg["glim"]*g0).
		IF mbreak > stg["m_final"]  {
			SET out[1] TO mbreak.
			SET out[0] TO (stg["m_initial"] - mbreak)/stg["engines"]["flow"].
		}
		RETURN out.
	}

//calculates new stage burn time  as a sum of constant g burn time
//and constant t burn time at minimum throttle
FUNCTION glim_stg_time {
	PARAMETER stg_lex.
	
	local glim is stg_lex["glim"].
	LOCAL tt Is 0.
	
	//compute burn time until  we deplete the stage.	
	
	LOCAL maxtime IS (stg_lex["engines"]["isp"]/glim) * LN(1 + stg_lex["m_burn"]/stg_lex["m_final"] ).

	//compute burn time until  we reach minimum throttle.	
	LOCAL limtime IS - stg_lex["engines"]["isp"]/glim * LN(stg_lex["minThrottle"]).
	LOCAL constThrustTime IS 0.
	IF limtime < maxtime {
		//	First we calculate mass of the fuel burned until violation
		LOCAL burnedFuel IS stg_lex["m_initial"]*(1 - CONSTANT:E^(-glim*limtime/stg_lex["engines"]["isp"])).
		//	Then, time it will take to burn the rest on constant minimum throttle
		SET constThrustTime TO (stg_lex["m_burn"] - burnedFuel  )/(stg_lex["minThrottle"]*stg_lex["engines"]["flow"]).
		SET tt TO limtime + constThrustTime.
	}
	ELSE {
		SET tt TO maxtime.
	}
	SET stg_lex["throt_mult"] TO glim*g0/stg_lex["engines"]["thrust"].
	
	RETURN tt.								
}



//measures current total engine thrust an isp, as well as theoretical max engine thrust at this altitude

FUNCTION get_thrust_isp {

	//calculating thrust as an average of current and previous readings.	
	FUNCTION avgThrust {
		LOCAL  thr IS 0.
		FROM {LOCAL k IS 1.} UNTIL k > len STEP { SET k TO k+1.} DO{ SET thr TO thr + thrust[k]/len. }	
		SET count TO count + 1.
		IF (len<4) {SET len TO len+1.}
		IF count>len {SET count TO 1.}
		
		RETURN thr.
	} 


	LOCAL thr is 0.
	LOCAL iisspp IS 0.
	LOCAL maxthr is 0.			   
	
	list ENGINES in all_eng.
	FOR e IN all_eng {
		IF e:ISTYPE("engine") {
			IF e:IGNITION {
				SET thr TO thr + (e:THRUST * 1000).
				SET iisspp TO iisspp + e:isp*(e:THRUST * 1000).
				SET maxthr TO maxthr + (e:AVAILABLETHRUST*1000).									
			}
		}
	}	
	SET thrust[count] TO thr.
	SET thr TO avgThrust().
	
	RETURN LIST(thr,iisspp,maxthr).
}






FUNCTION get_prop_mass {
	PARAMETER stg.
	
	local tanklist is stg["tankparts"].
	local reslist is stg["resources"].
	local prop_mass IS 0.
	
	FOR tk IN tanklist {
		FOR tkres In tk:RESOURCES {
			FOR res IN reslist:KEYS {
				IF tkres:NAME = res {
					set prop_mass TO prop_mass + tkres:amount*reslist[res].
				}
		
			}
		}
	}
	set prop_mass to prop_mass*1000.
    RETURN prop_mass.
}


//measures everything about the current state of the vehicle, including instantaneous thrust
//thrust only averaged over if staging is not in progress
FUNCTION getState {

	LOCAL deltat IS surfacestate["MET"].
	SET surfacestate["MET"] TO TIME:SECONDS. 
	SET deltat TO surfacestate["MET"] - deltat.

	//measure position and orbit parameters
	
	IF ops_mode >1 {set v to SHIP:PROGRADE:VECTOR.}
	ELSE {set v to SHIP:SRFPROGRADE:VECTOR.}
	
	SET surfacestate["hdir"] TO compass_for(v,SHIP:GEOPOSITION ).
	SET surfacestate["vdir"] TO 90 - VANG(v, SHIP:UP:VECTOR).
	SET surfacestate["pitch"] TO 90 - VANG(SHIP:FACING:VECTOR, SHIP:UP:VECTOR).	
	SET surfacestate["az"] TO compass_for(SHIP:FACING:VECTOR,SHIP:GEOPOSITION ).
	SET surfacestate["alt"] TO SHIP:ALTITUDE.
	SET surfacestate["vs"] TO SHIP:VERTICALSPEED.
	SET surfacestate["hs"] TO SHIP:VELOCITY:SURFACE:MAG.
	
	IF (surfacestate:HASKEY("q") ) {
		check_maxq(SHIP:Q).
	}
	
	SET orbitstate["velocity"] TO vecYZ(SHIP:ORBIT:VELOCITY:ORBIT).
	SET orbitstate["radius"] TO vecYZ(SHIP:ORBIT:BODY:POSITION)*-1.
	
	IF DEFINED events {	events_handler().}
	
	//measure and compute vehicle performance parameters
	
	local stg IS get_stage().
	local stg_staginginfo IS stg["staging"]["type"].
	
	LOCAL x IS get_thrust_isp().
	SET thrust[0] TO x[0].

	
	IF NOT (stagingInProgress) {
		
		IF NOT stg:HASKEY("tankparts") {get_stg_tanks(stg).}
		
		LOCAL m_old IS stg["m_initial"].
		
	
		SET stg["m_initial"] TO SHIP:MASS*1000.
		
		LOCAL dm IS m_old - stg["m_initial"].
		
		local res_left IS get_prop_mass(stg).

		local dmburn IS m_burn_left - res_left.
		SET m_burn_left to res_left.
		
		SET stg["m_final"] TO stg["m_initial"] - res_left.
		
		SET stg["m_final"] TO stg["m_final"] - ( dm - dmburn ).
								 
		IF stg_staginginfo="m_burn" {
			SET stg["m_burn"] TO stg["m_burn"] - dmburn.
		}
		ELSE IF (stg_staginginfo="time") {
		    	SET stg["Tstage"] TO stg["Tstage"] - deltat.
		}
		ELSE IF (stg_staginginfo="glim"){
			
			LOCAL nextstg IS vehicle["stages"][j+1].
			LOCAL stg_thr IS stg.
			//SET  stg_thr["engines"]["thrust"] TO stg["engines"]["thrust"]*stg["Throttle"].
			//SET  stg_thr["engines"]["flow"] TO stg["engines"]["flow"]*stg["Throttle"].			
			
			//SET stg["m_final"] TO stg["m_initial"] - res_left.
			
			
			LOCAL y IS glim_t_m(stg_thr).
			
			SET nextstg["m_initial"] TO y[1].
			SET nextstg["m_final"] TO stg["m_final"].
			SET stg["m_final"] TO y[1].
			SET stg["m_burn"] TO stg["m_initial"] - y[1].
			SET nextstg["m_burn"] TO res_left - stg["m_burn"].
			SET stg["Tstage"] TO y[0].
			
			IF nextstg["mode"]=1 {
				SET nextstg["Tstage"] TO (nextstg["m_burn"])/nextstg["engines"]["flow"].
			}
			ELSE IF nextstg["mode"]=2 {
				SET nextstg["Tstage"] TO glim_stg_time(nextstg).
			}
		}
		ELSE IF (stg_staginginfo="depletion") {
			
			SET stg["m_burn"] TO res_left.
			
			
			
			local thrustfortime IS stg["engines"]["thrust"].
			
			IF NOT (ops_mode=2) AND  (thrust[0]>0) {
					SET stg["engines"]["isp"] TO x[1]/thrust[0].
					SET thrustfortime TO thrust[0].
			}
			SET thrustfortime TO thrustfortime*stg["Throttle"].
			
			IF thrustfortime>0 AND stg["engines"]["isp"]>0 {	
							
				local tt is 0.
				IF stg["mode"]=1 {
					SET stg["engines"]["flow"] TO  thrustfortime/(stg["engines"]["isp"]*g0).
					SET stg["Tstage"] TO (stg["m_burn"])/stg["engines"]["flow"].
				}
				ELSE IF stg["mode"]=2 {
					SET stg["Tstage"] TO glim_stg_time(stg).
				}
			}
		}
	}	
}





//Staging function.
FUNCTION STAGING{
	
	local stg_staginginfo IS vehicle["stages"][j]["staging"]["type"].
	
	local flameout IS (stg_staginginfo="depletion").
	
	IF flameout {
		SET staginginprogress TO TRUE.
		SET P_steer TO "kill".
	}
	
	addMessage("CLOSE TO STAGING").
	SET time_stg TO TIME:SECONDS+100.		//bias of 100 seconds to avoid premature triggering of the staging actions
	
	
	

	WHEN (flameout AND maxthrust=0) or ((NOT flameout) AND vehicle["stages"][j]["Tstage"] <=0.0005 ) THEN {	
		SET staginginprogress TO TRUE.
		addMessage("STAGING").
		SET time_stg TO TIME:SECONDS.
		IF (vehicle["stages"][j]["staging"]["type"]="depletion") {set time_stg to time_stg + 1.5 .}
	}
	
	
	local stagingaction IS {STAGE.}.
	IF vehicle["stages"][j+1]["staging"]:HASKEY("stg_action") {
		SET stagingaction TO vehicle["stages"][j+1]["staging"]["stg_action"].
	}
	
	WHEN TIME:SECONDS > time_stg THEN {
		staging_reset(stagingaction).	
	}
	
	IF vehicle["stages"][j+1]["staging"]["ignition"]=TRUE {
		WHEN TIME:SECONDS > (time_stg + vehicle["stages"][j]["staging"]["ullage_t"]) THEN { 
			wait until stage:ready.
			
			STAGE.
			
			addMessage("SPOOLING UP").
			WHEN thrust[count]>= 0.95*vehicle["stages"][j]["engines"]["thrust"] THEN {
				addMessage("STAGING SEQUENCE COMPLETE").
				SET staginginprogress TO FALSE.
			}
		}
	}
	ELSE {
		WHEN TIME:SECONDS > time_stg + 0.5 THEN {
			addMessage("STAGING SEQUENCE COMPLETE").
			SET staginginprogress TO FALSE.
		}
	}
}


FUNCTION staging_reset {

	FUNCTION handle_ullage {
		PARAMETER stg.
	
		IF stg["staging"]["ullage"]="none" OR stg["staging"]["ullage_t"]=0 {
			RETURN.
		}
		addMessage("ULLAGE THRUST").
		IF stg["staging"]["ullage"]="srb"{
			RETURN.
		}
		ELSE IF stg["staging"]["ullage"]="rcs"{
			RCS ON. 
			SET SHIP:CONTROL:FORE TO 1.0.
			WHEN TIME:SECONDS > (time_stg + stg["staging"]["ullage_t"]+1) THEN {
				SET SHIP:CONTROL:FORE TO 0.0.
			}
		}
	}

	PARAMETER stagingaction.
	wait until stage:ready.
	
	stagingaction:call().
		
	SET j TO j+1.
	local stg is get_stage().
	SET stg["ign_t"] TO TIME:SECONDS.
	SET count TO 1.
	SET len TO 1.	
	FROM {LOCAL k IS 1.} UNTIL k=thrust:LENGTH STEP { SET k TO k+1.} DO{ SET thrust[k] to 0. }
	SET m_burn_left TO stg["m_burn"].
	handle_ullage(stg).
	WHEN ( (vehicle["stages"][j]["Tstage"] <= 3) AND ( j< (vehicle["stages"]:LENGTH - 1)) ) THEN {STAGING().}
	IF ops_mode=2 {SET usc["lastthrot"] TO stg["Throttle"].	}
}



//small function that, given time, computes how much burned mass that equates to
//used for pre-convergence of upfg
FUNCTION decrease_mass {
	parameter stage.
	parameter timespan.
	
	local deltam is 0.
	
	IF stage["mode"]=1 {
		set deltam to timespan*stage["engines"]["flow"].
	}
	ELSE IF stage["mode"]=2 {
		set deltam to stage["m_initial"]*(1 - CONSTANT:E^(-stage["glim"]*timespan/stage["engines"]["isp"])).
	}
	
	set stage["m_initial"] to stage["m_initial"] - deltam.
	set stage["m_burn"] to stage["m_burn"] - deltam.
	SET stage["Tstage"] TO stage["Tstage"] - timespan.
} 
	





//simple function to check if vehicle is past maxq
FUNCTION check_maxq {
	PARAMETER newq.
	
	IF (newq >=  surfacestate["q"] ) {
		SET surfacestate["q"] TO newq.
	} ELSE {
		addMessage("VEHICLE HAS REACHED MAX-Q").
		surfacestate:REMOVE("q").
	}

}





//		SHUTTLE-SPECIFIC FUNCTIONS 




//return the ET part to read fuel quantities
FUNCTION get_stg_tanks {
	PARAMETER stg.

	local tanklist IS SHIP:PARTSDUBBED("ShuttleExtTank").
	
	stg:ADD("tankparts", tanklist).	
	
}



FUNCTION disable_TVC {
	FOR ssme IN SHIP:PARTSDUBBED("ShuttleSSME") {
		IF ssme:ISTYPE("engine") {
			ssme:GIMBAL:DOACTION("lock gimbal", TRUE).
		}
	}
}




//close umbilical doors
FUNCTION close_umbilical {
	SHIP:PARTSDUBBED("ShuttleEngMount")[0]:GETMODULE("ModuleAnimateGeneric"):doaction("toggle et door", true).
}




FUNCTION toggle_dump {
	PARAMETER type.
	
	IF (type = "oms") {
		FOR oms IN SHIP:PARTSDUBBED("ShuttleEngineOMS") {
			IF (oms:IGNITION) {
				oms:SHUTDOWN.
			} ELSE {
				oms:ACTIVATE.
			}
		}
	} ELSE IF (type = "valve") {
		FOR p IN SHIP:PARTSDUBBED("km.valve2") {
		p:GETMODULE("valve"):DOEVENT("toggle").
	}
	}

}

//coudn active SSMEs and compare with expected number.
FUNCTION SSME_out {

	//get SSME parameters from vehicle struct snd known number of engines 
	LOCAL SSMEcount IS 0.
	
	FOR e IN SHIP:PARTSDUBBED("ShuttleSSME") {
		IF e:IGNITION {
			SET SSMEcount TO SSMEcount + 1.
		}
	}
	
	IF (SSMEcount < vehicle["SSME"]["active"]) {
		SET diff TO  vehicle["SSME"]["active"] - SSMEcount.
	
		FROM {LOCAL k IS 1.} UNTIL k > (vehicle["stages"]:LENGTH - 1) STEP { SET k TO k+1.} DO{
			SET vehicle["stages"][k]["engines"]["thrust"] TO SSMEcount*vehicle["SSME"]["thrust"]*1000.
			SET vehicle["stages"][k]["engines"]["flow"] TO SSMEcount*vehicle["SSME"]["flow"].
		}
		
		SET vehicle["SSME"]["active"] TO SSMEcount.
		
		RETURN TRUE.
	}

	RETURN FALSE.
}

