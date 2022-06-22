
GLOBAL g0 IS 9.80665. 


GLOBAL vehiclestate IS LEXICON(
	"cur_stg", 1,
	"staging_time", 0,
	"staging_in_progress", FALSE,
	"m_burn_left", 0,
	"avg_thr", average_value_factory(6)
).


//VEHICLE INITIALISATION FUNCTION 

declare function initialise_shuttle{

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
	
	//hard-coded initialisation of shuttle vehicle
	
	
	//not necessary per se but a useful check to see if we're flying a DECQ shuttle
	LOCAL ssme_count IS SHIP:PARTSDUBBED("ShuttleSSME"):LENGTH.
	IF (ssme_count<>3) {
		PRINT ("ERROR! THE VEHICLE SEEMS TO HAVE THE WRONG NUMBER OF SSMEs") AT (1,40).
		LOCAL X IS 1/0.
	}
	
	vehicle["SSME"]:ADD(
		"active",ssme_count
	).
	
	//	In case user accidentally entered throttle as percentage instead of a fraction
	IF vehicle["SSME"]["minThrottle"] > 1.0	{ SET vehicle["SSME"]["minThrottle"] TO vehicle["SSME"]["minThrottle"]/100. }
	
	local veh_res IS res_dens_init(
		add_resource(
			LEXICON(),
			LIST("LqdHydrogen","LqdOxygen")
		)
	).
	
	//stage1 - SRB
	local stg1 IS vehicle["stages"][1].
	fix_mass_params(stg1).
	
	LOCAL new_stg_1 IS LEXICON(
		"m_initial",	stg1["m_initial"]*1000,
		"m_final",	stg1["m_final"]*1000,
		"m_burn",	stg1["m_burn"]*1000,
		"staging", LEXICON (
			"type","time",
			"ignition",	TRUE,
			"ullage", "none",
			"ullage_t",	0	
		),
		"ign_t", 0,
		"Tstage",stg1["Tstage"],
		"Throttle",1,
		"minThrottle",vehicle["SSME"]["minThrottle"],
		"engines",	
			LEXICON(
				"thrust", ssme_count*vehicle["SSME"]["thrust"]*1000, 
				"isp", vehicle["SSME"]["isp"], 
				"flow",ssme_count*vehicle["SSME"]["flow"], 
				"resources",LIST("LqdHydrogen","LqdOxygen")
		),
		"resources",veh_res,
		"mode", 1
	).
	
	SET vehicle["stages"][1] TO new_stg_1.
	
	
	//stage 2 - SSME CONSTANT T
	local stg2 IS vehicle["stages"][2].
	fix_mass_params(stg2).
	
	LOCAL new_stg_2 IS LEXICON(
		"m_initial",	stg2["m_initial"]*1000,
		"m_final",	stg2["m_final"]*1000,
		"m_burn",	stg2["m_burn"]*1000,
		"staging", LEXICON (
			"type","glim",
			"ignition",	FALSE,
			"ullage", "none",
			"ullage_t",	0	
		),
		"glim", 3,
		"ign_t", 0,
		"Tstage",0,
		"Throttle",1,
		"minThrottle",vehicle["SSME"]["minThrottle"],
		"engines", new_stg_1["engines"],
		"resources",veh_res,
		"mode", 1
	).
	
	
	//will the stage exceed the g limit?
	LOCAL x IS glim_t_m(new_stg_2).
	If x[0] <= 0 {
		PRINT ("ERROR! THE VEHICLE WILL NEVER EXCEED THE 3G ACCELERATION LIMIT. VERIFY PAYLOAD MASS WITHIN LIMITS AND CHECK VEHICLE DEFINITION") AT (1,40).
		LOCAL X IS 1/0.
	}
	
	SET new_stg_2["Tstage"] TO x[0].
	SET new_stg_2["m_final"] TO x[1].
	SET new_stg_2["m_burn"] TO new_stg_2["m_initial"] - x[1].
	
	
	
	LOCAL new_stg_3 IS LEXICON(
		"m_initial",	x[1],
		"m_final",	stg2["m_final"]*1000,
		"m_burn", x[1] - stg2["m_final"]*1000,
		"staging", LEXICON (
					"stg_action",{},
					"type","depletion",
					"ignition",	FALSE,
					"ullage", "none",
					"ullage_t",	0
		),
		"glim",new_stg_2["glim"],
		"ign_t", 0,
		"Tstage",0,
		"Throttle",1,
		"minThrottle",new_stg_2["minThrottle"],
		"throt_mult",0,
		"engines",	new_stg_2["engines"],
		"resources",veh_res,
		"mode", 2
	).
	
	SET new_stg_3["Tstage"] TO glim_stg_time(new_stg_3).
	SET vehicle["stages"][2] TO new_stg_2.								   
	vehicle["stages"]:INSERT(3, new_stg_3).


	SET vehiclestate["m_burn_left"] TO vehicle["stages"][1]["m_burn"].
	
	
	//final vehicle parameters
	
	vehicle:ADD("ign_t", 0).
	vehicle:ADD("launchTimeAdvance", 300).
	vehicle:ADD("roll",180).
	vehicle:ADD("preburn",5.1).
	vehicle:ADD(
		"handover",
		LEXICON("time", vehicle["stages"][1]["Tstage"] + 5)
	).

	WHEN vehicle["stages"][vehiclestate["cur_stg"]]["Tstage"] <= 3 THEN {STAGING().}
	
	WHEN (SHIP:Q > 0.28) THEN {
		addMessage("THROTTLING DOWN").
		SET vehicle["stages"][1]["Throttle"] TO 0.75.
	}
	
	PRINT " INITIALISATION COMPLETE" AT (0,3).
	
	//debug_vehicle().
	
}

FUNCTION debug_vehicle {
	IF EXISTS(vehicledump.txt)=TRUE {
		DELETEPATH(vehicledump.txt).
	}
	log vehicle to vehicledump.txt.
	
	until false{
		wait 0.1.
	}
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
	
	local norm is VCRS(ref_fore,ref_up).

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
	
	LOCAL newthrvec IS rodrigues(ref_fore,norm,-VANG(ship_fore,thrvec)):NORMALIZED*thrvec:MAG.
	
	RETURN ref_fore - newthrvec.
}




//	Returns a kOS direction for given aim vector, reference up vector and roll angle.
//corrects for thrust offset
FUNCTION aimAndRoll {
	DECLARE PARAMETER aimVec.	//	Expects a vector
	DECLARE PARAMETER upVec.	//	Expects a vector
	DECLARE PARAMETER rollAng.	//	Expects a scalar
			 
	LOCAL steerVec IS aimVec.
	
	LOCAL topVec IS VXCL(steerVec,upVec):NORMALIZED.
	SET topVec TO rodrigues(topVec, steerVec, rollAng).
	
	//if the target aiming vector is too far away in yaw calculate an intermediate vector
	//project the target vector in the ship vertical plane, rotate this towards the target by 3 degrees in the yaw plane
	//LOCAL thrustvec IS thrust_vec().
	//IF VANG(thrustvec,steerVec) > 5 {
	//	LOCAL steerVecproj IS VXCL(SHIP:FACING:STARVECTOR,steerVec).
	//	
	//	LOCAL aimnorm Is VCRS(steerVec, steerVecproj).
	//	
	//	SET steerVec TO rodrigues(steerVecproj,aimnorm,-5).
	//
	//}
	
	LOCAL thrustCorr IS thrustrot(steerVec,topVec).
	
	LOCAL outdir IS LOOKDIRUP(steerVec + thrustCorr, topVec).


	//clearvecdraws().
	//arrow(topVec,"topVec",v(0,0,0),30,0.05).
	//arrow(aimVec,"aimVec",v(0,0,0),30,0.05).
	//arrow(steerVec,"steerVec",v(0,0,0),30,0.05).
	//arrow(thrustCorr,"thrustCorr",v(0,0,0),30,0.05).

	RETURN outdir.
}

//given current vehicle fore vector, computes where the thrust is pointing
FUNCTION thrust_vec {
	RETURN SHIP:FACING:VECTOR:NORMALIZED - thrustrot(SHIP:FACING:FOREVECTOR,SHIP:FACING:TOPVECTOR).
}




//	Throttle controller
FUNCTION throttleControl {

	local stg IS vehicle["stages"][vehiclestate["cur_stg"]].
	local throtval is stg["Throttle"].
	
	LOCAL minthrot IS 0.
	IF stg:HASKEY("minThrottle") {
		SET minthrot TO stg["minThrottle"].
	}
	
	IF stg["mode"] = 2   {
		SET throtval TO stg["throt_mult"]*SHIP:MASS*1000.
		SET usc["lastthrot"] TO throtval.
	}
	
	SET throtval TO MIN(1,(throtval - minthrot)/(1 - minthrot)).			
	
	IF throtval < 0.005 {
		SET throtval TO 0.005.
		
		IF stg["mode"] = 2 {
			SET stg["mode"] TO 1.
			SET usc["lastthrot"] TO stg["minThrottle"].
			SET stg["Throttle"] TO stg["minThrottle"].
		}
	}
	
	RETURN throtval.
}











									//VEHICLE PERFORMANCE & STAGING FUNCTIONS


FUNCTION get_stage {
	RETURN vehicle["stages"][vehiclestate["cur_stg"]].
}



FUNCTION events_handler {

	local met is TIME:SECONDS - vehicle["ign_t"].

	local x IS events:LENGTH.
	
	local rem_list IS LIST().

	//FROM {LOCAL k IS 0.} UNTIL k >= x STEP { SET k TO k+1.} DO{
	
	LOCAL k IS 0.
	FOR evt IN events {
		
		IF met>evt["time"]  {
			IF evt["type"]="jettison" {
				TOGGLE AG8.
				IF evt:HASKEY("mass") {
					FROM { LOCAL i IS vehiclestate["cur_stg"]. } UNTIL i > (vehicle["stages"]:LENGTH - 1)  STEP { SET i TO i+1. } DO {
						SET vehicle["stages"][i]["m_initial"] TO vehicle["stages"][i]["m_initial"] - evt["mass"].
						SET vehicle["stages"][i]["m_final"] TO vehicle["stages"][i]["m_final"] - evt["mass"].
					}
				}
				rem_list:ADD(k).
			}
			ELSE IF evt["type"]="roll" {
								
				IF ABS(evt["angle"] - vehicle["roll"])<5 {
					set vehicle["roll"] TO evt["angle"].
					rem_list:ADD(k).
					SET STEERINGMANAGER:MAXSTOPPINGTIME TO 0.2.
					
				} ELSE {
					SET STEERINGMANAGER:MAXSTOPPINGTIME TO 0.8.
					local rollsign is SIGN( unfixangle( evt["angle"] - vehicle["roll"] ) ).
					set vehicle["roll"] TO fixangle(vehicle["roll"] + rollsign*5).
				} 
			}
			ELSE IF evt["type"]="action" { 
				IF evt:HASKEY("action") {
					evt["action"]:call().
				}
				rem_list:ADD(k).
			}
				
			
			
		}
		SET k TO k+1.
	}
	
	LOCAL items_removed IS 0.
	FOR e IN rem_list {
		events:REMOVE(e - items_removed).
		SET items_removed TO items_removed + 1.
	}
}

FUNCTION get_mass_bias {

	LOCAL stg IS vehicle["stages"][1].
		
	IF NOT stg:HASKEY("tankparts") {get_stg_tanks(stg).}
	local res_left IS get_prop_mass(stg).

	local dm IS vehiclestate["m_burn_left"] - res_left.
	
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
	
	vehiclestate["avg_thr"]:update(thr).
	
	RETURN LIST(vehiclestate["avg_thr"]:average(),iisspp,maxthr).
}



FUNCTION get_TWR {
	RETURN vehiclestate["avg_thr"]:average()/(1000*SHIP:MASS*g0).
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
	
	IF (surfacestate:HASKEY("q") AND surfacestate["alt"] > 100 ) {
		check_maxq(SHIP:Q).
	}
	
	SET orbitstate["velocity"] TO vecYZ(SHIP:ORBIT:VELOCITY:ORBIT).
	SET orbitstate["radius"] TO vecYZ(SHIP:ORBIT:BODY:POSITION)*-1.
	
	IF DEFINED events {	events_handler().}
	
	//measure and compute vehicle performance parameters
	
	local stg IS get_stage().
	local stg_staginginfo IS stg["staging"]["type"].
	
	LOCAL x IS get_thrust_isp().
	LOCAL avg_thrust is x[0].
	LOCAL avg_isp is x[1].

	
	IF NOT (vehiclestate["staging_in_progress"]) {
		
		IF NOT stg:HASKEY("tankparts") {get_stg_tanks(stg).}
		
		LOCAL m_old IS stg["m_initial"].
		
	
		SET stg["m_initial"] TO SHIP:MASS*1000.
		
		LOCAL dm IS m_old - stg["m_initial"].
		
		local res_left IS get_prop_mass(stg).

		local dmburn IS vehiclestate["m_burn_left"] - res_left.
		SET vehiclestate["m_burn_left"] to res_left.
		
		SET stg["m_final"] TO stg["m_initial"] - res_left.
		
		SET stg["m_final"] TO stg["m_final"] - ( dm - dmburn ).
								 
		IF stg_staginginfo="m_burn" {
			SET stg["m_burn"] TO stg["m_burn"] - dmburn.
		}
		ELSE IF (stg_staginginfo="time") {
		    	SET stg["Tstage"] TO stg["Tstage"] - deltat.
		}
		ELSE IF (stg_staginginfo="glim"){
			
			LOCAL nextstg IS vehicle["stages"][vehiclestate["cur_stg"]+1].
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
			
			IF stg["mode"]=1 {
				LOCAL red_flow IS stg["engines"]["thrust"]*stg["throttle"]/(stg["engines"]["isp"]*g0).
				SET stg["Tstage"] TO stg["m_burn"]/red_flow.					 
			}
			ELSE IF stg["mode"]=2 {
				SET stg["Tstage"] TO glim_stg_time(stg).
	 
			}
		}
	}	
}





//Staging function.
FUNCTION STAGING{
	IF vehiclestate["staging_in_progress"] {RETURN.}
	
	local stg_staginginfo IS vehicle["stages"][vehiclestate["cur_stg"]]["staging"]["type"].
	
	local flameout IS (stg_staginginfo="depletion").
	
	IF flameout {
		SET vehiclestate["staging_in_progress"] TO TRUE.
		SET control["steerdir"] TO "kill".
	}
	
	addMessage("CLOSE TO STAGING").
	SET vehiclestate["staging_time"] TO TIME:SECONDS+1000.		//bias of 1000 seconds to avoid premature triggering of the staging actions
	
	
	

	WHEN (flameout AND maxthrust=0) or ((NOT flameout) AND vehicle["stages"][vehiclestate["cur_stg"]]["Tstage"] <=0.0005 ) THEN {	
		SET vehiclestate["staging_in_progress"] TO TRUE.
		addMessage("STAGING").
		SET vehiclestate["staging_time"] TO TIME:SECONDS.
		IF (vehicle["stages"][vehiclestate["cur_stg"]]["staging"]["type"]="depletion") {set vehiclestate["staging_time"] to vehiclestate["staging_time"] + 1.5 .}
	}
	
	
	local stagingaction IS {STAGE.}.
	IF vehicle["stages"][vehiclestate["cur_stg"]+1]["staging"]:HASKEY("stg_action") {
		SET stagingaction TO vehicle["stages"][vehiclestate["cur_stg"]+1]["staging"]["stg_action"].
	}
	
	WHEN TIME:SECONDS > vehiclestate["staging_time"] THEN {
		staging_reset(stagingaction).	
	}
	
	IF vehicle["stages"][vehiclestate["cur_stg"]+1]["staging"]["ignition"]=TRUE {
		WHEN TIME:SECONDS > (vehiclestate["staging_time"] + vehicle["stages"][vehiclestate["cur_stg"]]["staging"]["ullage_t"]) THEN { 
			wait until stage:ready.
			
			STAGE.
			
			addMessage("SPOOLING UP").
			WHEN vehiclestate["avg_thr"]:list:list[0]>= 0.95*vehicle["stages"][vehiclestate["cur_stg"]]["engines"]["thrust"] THEN {
				addMessage("STAGING SEQUENCE COMPLETE").
				SET vehiclestate["staging_in_progress"] TO FALSE.
			}
		}
	}
	ELSE {
		WHEN TIME:SECONDS > vehiclestate["staging_time"] + 0.5 THEN {
			addMessage("STAGING SEQUENCE COMPLETE").
			SET vehiclestate["staging_in_progress"] TO FALSE.
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
			WHEN TIME:SECONDS > (vehiclestate["staging_time"] + stg["staging"]["ullage_t"]+1) THEN {
				SET SHIP:CONTROL:FORE TO 0.0.
			}
		}
	}

	PARAMETER stagingaction.
	wait until stage:ready.
	
	stagingaction:call().
		
	SET vehiclestate["cur_stg"] TO vehiclestate["cur_stg"]+1.
	local stg is get_stage().
	SET stg["ign_t"] TO TIME:SECONDS.
	vehiclestate["avg_thr"]:reset().
	SET vehiclestate["m_burn_left"] TO stg["m_burn"].
	handle_ullage(stg).
	IF vehiclestate["cur_stg"] < (vehicle["stages"]:LENGTH - 1) {
		WHEN ( (vehicle["stages"][vehiclestate["cur_stg"]]["Tstage"] <= 3) AND (vehiclestate["cur_stg"] < (vehicle["stages"]:LENGTH - 1)) ) THEN {STAGING().}
	}
	
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
		WHEN (SHIP:Q < 0.95*newq) THEN {
			addMessage("THROTTLING UP").
			SET vehicle["stages"][1]["Throttle"] TO 1.
		}
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




FUNCTION OMS_dump {
	PARAMETER type.
	PARAMETER state.
	
	IF (type = "oms") {
		FOR oms IN SHIP:PARTSDUBBED("ShuttleEngineOMS") {
			IF (state="start") {
				oms:ACTIVATE.
			} ELSE IF (state="stop") {
				oms:SHUTDOWN.
			}
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

FUNCTION SSME_flameout {
	FOR e IN SHIP:PARTSDUBBED("ShuttleSSME") {
		IF e:FLAMEOUT {
			RETURN true.
		}
	}
	RETURN FALSE.
}