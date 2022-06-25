GLOBAL g0 IS 9.80665. 


GLOBAL vehiclestate IS LEXICON(
	"ops_mode",0,
	"cur_stg", 1,
	"staging_time", 0,
	"staging_in_progress", FALSE,
	"m_burn_left", 0,
	"avg_thr", average_value_factory(6)
).


GLOBAL control Is LEXICON(
	"launch_az",0,
	"steerdir", LOOKDIRUP(SHIP:FACING:FOREVECTOR, SHIP:FACING:TOPVECTOR),
	"roll_angle",0,
	"refvec", v(0,0,0)
).



GLOBAL events IS LIST().


//VEHICLE INITIALISATION FUNCTION 


function initialise_shuttle {

	RUNPATH("0:/Libraries/resources_library").	

	
	FUNCTION add_resource {
		parameter lexx.
		parameter reslist.
		
		
		for res in reslist {
			IF NOT lexx:HASKEY(res) {lexx:ADD(res,0).}
		}
		RETURN lexx.
	}

	//this must be changed if the RO config for the SRBs ever changes
	LOCAL srb_time IS 120.

	
	//hard-coded initialisation of shuttle vehicle
	
	
	//prepare the main struct 
	
	GLOBAL vehicle IS LEXICON(
						"name",SHIP:NAME,
						"ign_t", 0,
						"launchTimeAdvance", 300,
						"preburn",5.1,
						"roll",180,
						"handover", LEXICON("time", srb_time + 5),
						"stages",LIST(),
						"SSME",0
	).
	
	SET vehicle["SSME"] TO parse_ssme().
	
	//add the ssme type to the vessel name 
	
	SET vehicle["name"] TO vehicle["name"] + " " + vehicle["SSME"]["type"].

	
	local veh_res IS res_dens_init(
		add_resource(
			LEXICON(),
			LIST("LqdHydrogen","LqdOxygen")
		)
	).
	
	
	//measure total mass less the SRBs and clamps
	
	LOCAL et_part IS get_ext_tank_part().
	
	
	LOCAL stack_mass IS getShuttleStackMass().
	
	LOCAL total_prop_mass IS get_prop_mass(
		LEXICON(
			"resources",veh_res,
			"ext_tank",et_part
		)
	).
	
	LOCAL stack_empty_mass IS stack_mass - total_prop_mass.	
	
	
	//prepare stages list
	
	LOCAL engines_lex IS build_ssme_lex().
	
	
	//zeroth stage 
	vehicle["stages"]:ADD(0).
	
	//stage1 - SRB
	
	LOCAL stage1_burned_mass IS srb_time * engines_lex["flow"].
	
	LOCAL stage2InitialMass IS stack_mass - stage1_burned_mass.
	
	
	LOCAL new_stg_1 IS LEXICON(
		"m_initial",	stack_mass,
		"m_final",	stage2InitialMass,
		"m_burn",	stage1_burned_mass,
		"staging", LEXICON (
			"type","time",
			"ignition",	TRUE
		),
		"ign_t", 0,
		"Tstage",srb_time,
		"Throttle",1,
		"minThrottle",vehicle["SSME"]["minThrottle"],	//needed for the max q throttle down
		"engines",	engines_lex,
		"ext_tank",et_part,
		"resources",veh_res,
		"mode", 1
	).

	vehicle["stages"]:ADD(new_stg_1).
	
	
	//stage 2 - SSME CONSTANT T
	
	LOCAL new_stg_2 IS LEXICON(
		"m_initial",	stage2InitialMass,
		"m_final",	stack_empty_mass,
		"m_burn",	stage2InitialMass - stack_empty_mass,
		"staging", LEXICON (
			"type","glim",
			"ignition",	FALSE
		),
		"glim", 3,
		"ign_t", 0,
		"Tstage",0,
		"Throttle",1,
		"minThrottle",vehicle["SSME"]["minThrottle"],
		"engines",	engines_lex,
		"ext_tank",et_part,
		"resources",veh_res,
		"mode", 1
	).
	
	//will the stage exceed the g limit?
	LOCAL x IS glim_t_m(new_stg_2).
	If x[0] <= 0 {
		PRINT ("ERROR! THE VEHICLE WILL NEVER EXCEED THE 3G ACCELERATION LIMIT. VERIFY PAYLOAD MASS WITHIN LIMITS") AT (1,40).
		LOCAL X IS 1/0.
	}
	
	LOCAL stage3InitialMass IS x[1].
	SET new_stg_2["Tstage"] TO x[0].
	SET new_stg_2["m_final"] TO stage3InitialMass.
	SET new_stg_2["m_burn"] TO new_stg_2["m_initial"] - stage3InitialMass.
	
	
	vehicle["stages"]:ADD(new_stg_2).
	
	//stage 3 - SSME CONSTANT G until depletion or violation
	
	LOCAL new_stg_3 IS LEXICON(
		"m_initial",	stage3InitialMass,
		"m_final",	stack_empty_mass,
		"m_burn", stage3InitialMass - stack_empty_mass,
		"staging", LEXICON (
					"type","minthrot",
					"ignition",	FALSE
		),
		"glim",new_stg_2["glim"],
		"ign_t", 0,
		"Tstage",0,
		"Throttle",1,
		"minThrottle",vehicle["SSME"]["minThrottle"],
		"throt_mult",0,
		"engines",	engines_lex,
		"ext_tank",et_part,
		"resources",veh_res,
		"mode", 2
	).
	
	SET new_stg_3["throt_mult"] TO new_stg_3["glim"]*g0/engines_lex["thrust"].
	
	LOCAL y IS const_G_t_m(new_stg_3).
	SET new_stg_3["Tstage"] TO y[0].
	LOCAL stage4InitialMass IS y[1].
	
	//we don't want to ad a fourth stage unless it burns for at least 3 seconds
	//to avoid problems with mass uncertainties
	//if it's zero already because there is no need for a fourth stage at all we fall in the same condition 
	
	LOCAL min_stage4InitialMass IS stack_empty_mass + 3 * engines_lex["flow"] * vehicle["SSME"]["minThrottle"] .
	
	If stage4InitialMass <= min_stage4InitialMass {
		//no fourth stage to be added
		SET new_stg_3["staging"]["type"] TO "depletion".
		SET stage4InitialMass TO 0.
	} ELSE {
		SET new_stg_3["m_final"] TO stage4InitialMass.
		SET new_stg_3["m_burn"] TO new_stg_3["m_initial"] - stage4InitialMass.
	}
	
	vehicle["stages"]:ADD(new_stg_3).
	
	
	IF (stage4InitialMass>0) {
	
		//stage 4 - SSME CONSTANT T at minimum throttle
	
		LOCAL new_stg_4 IS LEXICON(
			"m_initial",	stage4InitialMass,
			"m_final",	stack_empty_mass,
			"m_burn",	stage4InitialMass - stack_empty_mass,
			"staging", LEXICON (
				"type","depletion",
				"ignition",	FALSE
			),
			"ign_t", 0,
			"Tstage",0,
			"Throttle",vehicle["SSME"]["minThrottle"],
			"minThrottle",vehicle["SSME"]["minThrottle"],
			"engines",	engines_lex,
			"ext_tank",et_part,
			"resources",veh_res,
			"mode", 1
		).
		
		SET new_stg_4["Tstage"] TO const_f_t(new_stg_4).
	
		vehicle["stages"]:ADD(new_stg_4).
	} 
	

	
	SET control["roll_angle"] TO vehicle["roll"].


	setup_engine_failure().
	
	WHEN (SHIP:Q > 0.28) THEN {
		addMessage("THROTTLING DOWN").
		SET vehicle["stages"][1]["Throttle"] TO 0.75.
	}
	
	PRINT " INITIALISATION COMPLETE" AT (0,3).
	
}

FUNCTION debug_vehicle {
	IF EXISTS("0:/vehicledump.txt") {
		DELETEPATH("0:/vehicledump.txt").
	}
	
	log vehicle:dump() to "0:/vehicledump.txt".
	
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


FUNCTION roll_heads_up {

	LOCAL tgt_roll IS 0.
	
	IF (vehicle["roll"] <> tgt_roll) {
		SET vehicle["roll"] TO tgt_roll.
		SET STEERINGMANAGER:MAXSTOPPINGTIME TO 0.8.
		addMessage("ROLL TO HEADS-UP ATTITUDE").
		
		WHEN (control["roll_angle"] = vehicle["roll"]) THEN {
			SET STEERINGMANAGER:MAXSTOPPINGTIME TO 0.2.
		}
	}
	
}



//	Returns a kOS direction for given aim vector, reference up vector and roll angle.
//corrects for thrust offset
FUNCTION aimAndRoll {
	DECLARE PARAMETER aimVec.
	DECLARE PARAMETER tgtRollAng.
			 
	LOCAL steerVec IS aimVec.
	
	LOCAL newRollAng IS tgtRollAng.
	
	IF ABS(tgtRollAng - control["roll_angle"])>5 {
		local rollsign is SIGN( unfixangle( tgtRollAng - control["roll_angle"] ) ).
		set control["roll_angle"] TO fixangle(control["roll_angle"] + rollsign*5).
		SET newRollAng TO control["roll_angle"].
	}
	
	
	LOCAL topVec IS VXCL(steerVec,control["refvec"]):NORMALIZED.
	SET topVec TO rodrigues(topVec, steerVec, newRollAng).
	
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

	local stg IS get_stage().
	local throtval is stg["Throttle"].
	
	LOCAL minthrot IS 0.
	IF stg:HASKEY("minThrottle") {
		SET minthrot TO stg["minThrottle"].
	}
	
	IF stg["mode"] = 2   {
		SET throtval TO stg["throt_mult"]*SHIP:MASS*1000.
		SET usc["lastthrot"] TO throtval.
	}

	RETURN CLAMP((throtval - minthrot)/(1 - minthrot),0.005,1).
}






									//VEHICLE PERFORMANCE & STAGING FUNCTIONS
									

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


FUNCTION get_stage {
	RETURN vehicle["stages"][vehiclestate["cur_stg"]].
}


FUNCTION add_action_event{
	PARAMETER time.
	PARAMETER callable.
	
	
	events:ADD(
		LEXICON(
				"time",time,
				"type", "action",
				"action",callable
		)
	).
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



//compute the total mass of all the parts composing orbiter, payload and ET excluding SRBs and clamps
function getShuttleStackMass {

	function removeChildrenPartsRecursively {
		parameter partslist.
		parameter part.
		
		local partchildren is part:children:copy.
		
		if partchildren:length > 0 {
			for p in partchildren {
			
				removeChildrenPartsRecursively(
					partslist,
					p 
				).
			
			}
		}
		
		partslist:remove(partslist:find(part)).
		return.

	}
	
	LOCAL et_part IS get_ext_tank_part().

	local shuttleParts is ship:parts:copy.

	removeChildrenPartsRecursively(
		shuttleParts,
		et_part
	).

	shuttleParts:add(et_part).
	
	LOCAL stackmass IS 0.
	FOR p IN shuttleParts {
		set stackmass to stackmass + p:mass*1000.
	}
	
	RETURN stackmass.
}


//calculates burn time for a constant thrust stage 
FUNCTION const_f_t {
	PARAMETER stg.

	LOCAL red_flow IS stg["engines"]["flow"] * stg["throttle"].
	RETURN stg["m_burn"]/red_flow.	
}


//calculates when the g limit will be violated and the vehicle mass at that moment
//returns (0,0) if the g-lim is never reached
FUNCTION glim_t_m {
	PARAMETER stg.
	local out is LIST(0,0).
	
	local mbreak is stg["engines"]["thrust"]/(stg["glim"]*g0).
	IF mbreak > stg["m_final"]  {
		SET out[1] TO mbreak.
		SET out[0] TO (stg["m_initial"] - mbreak)/(stg["engines"]["flow"] * stg["Throttle"]).
	}
	
	RETURN out.
}


//given a constant g stage calculates the burn time until the lower throttle limit will be reached and the vehicle mass at that moment
FUNCTION const_G_t_m {
	PARAMETER stg.
	local out is LIST(0,0).
	
	//calculate mass of the vehicle at throttle violation 
	LOCAL mviol IS stg["engines"]["thrust"] * stg["minThrottle"]/( stg["glim"] * g0 ).
	
	//initialise final mass to stage final mass
	LOCAL m_final IS stg["m_final"].
	
	IF mviol > m_final  {
		SET out[1] TO mviol.
		SET m_final TO mviol.
	}
	
	local red_isp is stg["engines"]["isp"]/stg["glim"].
		
	//calculate burn time until we reach the final mass 
	SET out[0] TO red_isp * LN( stg["m_initial"]/m_final ).
		
	RETURN out.
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
	
	local reslist is stg["resources"].
	local prop_mass IS 0.
	
	FOR tkres In stg["ext_tank"]:RESOURCES {
		FOR res IN reslist:KEYS {
			IF tkres:NAME = res {
				set prop_mass TO prop_mass + tkres:amount*reslist[res].
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
	
	update_navigation().
	
	SET deltat TO surfacestate["MET"] - deltat.

	
	IF DEFINED events {	events_handler().}
	
	//measure and compute vehicle performance parameters
	
	LOCAL cur_stg IS get_stage().
	
	LOCAL x IS get_thrust_isp().
	LOCAL avg_thrust is x[0].
	LOCAL avg_isp is x[1].

	IF NOT (vehiclestate["staging_in_progress"]) {
		
		LOCAL current_m IS SHIP:MASS*1000.
		local res_left IS get_prop_mass(cur_stg).
		
		SET vehiclestate["m_burn_left"] to res_left.
		
		
		IF (vehiclestate["cur_stg"]=1) {
		
			//do it here so we bypass the check during later stages
			IF (surfacestate:HASKEY("q") AND surfacestate["alt"] > 100 ) {
				check_maxq(SHIP:Q).
			}
		
			SET cur_stg["Tstage"] TO cur_stg["Tstage"] - deltat.
			
		} ELSE IF (vehiclestate["cur_stg"]=2) {
		
			update_stage2(current_m, res_left).
	
		} ELSE IF (vehiclestate["cur_stg"]=3) {
			
			update_stage3(current_m, res_left).
		
		} ELSE IF (vehiclestate["cur_stg"]=4) {
			
			update_stage4(current_m, res_left).
		}
	
	}
}

FUNCTION update_stage2 {
	PARAMETER m_initial.
	PARAMETER res_left.
	PARAMETER stg2 IS vehicle["stages"][2].

	SET stg2["m_initial"] TO m_initial.
	SET stg2["m_burn"] TO res_left.
	SET stg2["m_final"] TO m_initial - res_left.
	
	IF (stg2["staging"]["type"]="depletion") {
		
		SET stg2["Tstage"] TO const_f_t(stg2).	
		
	} ELSE IF (stg2["staging"]["type"]="glim") {
		//assume the g limit will be exceeded 
		
		LOCAL x IS glim_t_m(stg2).
		
		SET stg2["Tstage"] TO x[0]. 
		SET stg2["m_final"] TO x[1]. 
		SET stg2["m_burn"] TO m_initial - x[1].
		
		LOCAL stg3_m_initial IS x[1].
		LOCAL stg3_m_burn IS res_left - stg2["m_burn"].
		
		update_stage3(stg3_m_initial, stg3_m_burn).
	}

}



FUNCTION update_stage3 {
	PARAMETER m_initial.
	PARAMETER res_left.
	PARAMETER stg3 IS vehicle["stages"][3].
	
	SET stg3["m_initial"] TO m_initial.
	SET stg3["m_burn"] TO res_left.
	SET stg3["m_final"] TO m_initial - res_left.
	
	IF stg3["mode"]=1 {
		//constant thrust depletion stage, only used for late ATO aborts
		
		SET stg3["Tstage"] TO const_f_t(stg3).	
	
	} ELSE IF stg3["mode"]=2 {
	
		LOCAL x IS const_G_t_m(stg3).
		SET stg3["Tstage"] TO x[0].

		IF (stg3["staging"]["type"]="minthrot") {
			//there is a fourth stage, assume there will be a violation of minimum throttle 
			
			SET stg3["m_final"] TO x[1].
			LOCAL stg4_m_initial IS x[1].
			
			SET stg3["m_burn"] TO m_initial - x[1].
			LOCAL stg4_m_burn IS res_left - stg3["m_burn"].

			update_stage4(stg4_m_initial, stg4_m_burn).
			
		}
	}
}


FUNCTION update_stage4 {
	PARAMETER m_initial.
	PARAMETER res_left.
	PARAMETER stg4 IS vehicle["stages"][4].
	
	//assume it's a depletion stage
	
	
	SET stg4["m_initial"] TO m_initial.
	SET stg4["m_burn"] TO res_left.
	SET stg4["m_final"] TO m_initial - res_left.
	
	SET stg4["Tstage"] TO const_f_t(stg4).	
	
}



FUNCTION increment_stage {
	
	SET vehiclestate["staging_time"] TO TIME:SECONDS.
	
	LOCAL j IS vehiclestate["cur_stg"].
	
	SET vehiclestate["cur_stg"] TO j + 1.
			
	SET vehicle["stages"][j] TO 0.
	
	SET vehicle["stages"][j+1]["ign_t"] TO vehiclestate["staging_time"].
	
	IF vehiclestate["ops_mode"]=2 {
		SET usc["lastthrot"] TO vehicle["stages"][j+1]["Throttle"].
	}
	
	vehiclestate["avg_thr"]:reset().
	
	WHEN TIME:SECONDS > vehiclestate["staging_time"] + 0.5 THEN {
		addMessage("STAGING SEQUENCE COMPLETE").
		SET vehiclestate["staging_in_progress"] TO FALSE.
	}

}


FUNCTION srb_staging {
	IF vehiclestate["staging_in_progress"] {RETURN.}

	IF (vehicle["stages"][vehiclestate["cur_stg"]]["Tstage"] <= 3 ) {
		SET vehiclestate["staging_in_progress"] TO TRUE.
		SET control["steerdir"] TO "kill".
		addMessage("STAND-BY FOR SRB SEP").
		
		
		WHEN (get_TWR()<1) THEN {
		
			wait until stage:ready.
			STAGE.
		
			increment_stage().
			
			SET vehicle["handover"]["time"] TO vehiclestate["staging_time"] - vehicle["ign_t"] + 5.
		}
	}
	

}


//combined function that takes care of staging during ssme burnign phase 
//returns true when we're at the last ssme phase and close to depletion
FUNCTION ssme_staging_flameout {
	IF vehiclestate["staging_in_progress"] {RETURN.}

	LOCAL j IS vehiclestate["cur_stg"].
	
	IF j = (vehicle["stages"]:LENGTH - 1) {
		RETURN (vehicle["stages"][j]["Tstage"] <= 3).
	} ELSE {
		
		IF (vehicle["stages"][j]["Tstage"] <=0.1) {
			SET vehiclestate["staging_in_progress"] TO TRUE.
			increment_stage().
		}
		RETURN FALSE.
	}

}









//		SHUTTLE-SPECIFIC FUNCTIONS 


//if a global flag is set, sets up an event to shutdown one of the SSMEs
FUNCTION setup_engine_failure {

	IF (DEFINED engine_failure_time) {
		
		events:ADD(	
			LEXICON(
					"time",engine_failure_time,
					"type", "action",
					"action",{
								LOCAL englist IS SHIP:PARTSDUBBED("ShuttleSSME").
								englist[FLOOR(3*RANDOM())]:SHUTDOWN.
					}
			)
		).
	
	}

}



//return the ET part to read fuel quantities
//designed to crash if there is no part with this name
FUNCTION get_ext_tank_part {
	RETURN SHIP:PARTSDUBBED("ShuttleExtTank")[0].
}


FUNCTION activate_fuel_cells {
	
	for m in SHIP:PARTSDUBBED("ShuttleEngMount")[0]:MODULESNAMED("ModuleResourceConverter") {
		for an in m:ALLACTIONNAMES {
			IF an:CONTAINS("start fuel cell") {
				m:DOACTION(an, true).
			}
		}
	}
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

//check that the three ssme are present and the same type, calculate all the parameters needed
FUNCTION parse_ssme {
	
	LOCAL ssmelex IS LEXICON(
		"type","",
		"active",0,
		"isp",0,
		"thrust",0,
		"flow",0,
		"minThrottle",0
	
	).
	
	//count SSMEs, not necessary per se but a useful check to see if we're flying a DECQ shuttle
	LOCAL ssme_count IS 0. 
	SET ssmelex["type"] TO SHIP:PARTSDUBBED("ShuttleSSME")[0]:CONFIG.
	
	FOR ssme IN SHIP:PARTSDUBBED("ShuttleSSME") {
		IF ssme:ISTYPE("engine") {
			SET ssme_count TO ssme_count + 1.
			
			IF (ssme:CONFIG <> ssmelex["type"]) {
				PRINT ("ERROR! THE VEHICLE SEEMS TO HAVE MISMATCHED SSME TYPES") AT (1,40).
				LOCAL X IS 1/0.
			}
			
			LOCAL ssme_thr IS ssme:POSSIBLETHRUSTAT(0.0).
			LOCAL ssme_isp IS ssme:VACUUMISP.
			LOCAL ssme_flow IS ssme:MAXMASSFLOW*1000.
			LOCAL ssme_minthr IS ssme:MINTHROTTLE.
	
			SET ssmelex["thrust"] TO ssmelex["thrust"] + ssme_thr.
			SET ssmelex["isp"] TO ssmelex["isp"] + ssme_isp*ssme_thr.
			SET ssmelex["flow"] TO ssmelex["flow"] + ssme_flow.
			SET ssmelex["minThrottle"] TO ssmelex["minThrottle"] + ssme_minthr*ssme_thr.
			
		}
	}
	
	IF (ssme_count <> 3 ) {
		PRINT ("ERROR! THE VEHICLE SEEMS TO HAVE THE WRONG NUMBER OF SSMEs") AT (1,40).
		LOCAL X IS 1/0.
	}
	
	SET ssmelex["active"] TO ssme_count.
	
	SET ssmelex["isp"] TO ssmelex["isp"]/ssmelex["thrust"].
	SET ssmelex["minThrottle"] TO ssmelex["minThrottle"]/ssmelex["thrust"].
	
	SET ssmelex["thrust"] TO ssmelex["thrust"]/ssme_count.
	SET ssmelex["flow"] TO ssmelex["flow"]/ssme_count.

	
	
	RETURN ssmelex.
	
}


FUNCTION build_ssme_lex {

	RETURN LEXICON(
				"thrust", vehicle["SSME"]["active"]*vehicle["SSME"]["thrust"]*1000, 
				"isp", vehicle["SSME"]["isp"], 
				"flow",vehicle["SSME"]["active"]*vehicle["SSME"]["flow"], 
				"resources",LIST("LqdHydrogen","LqdOxygen")
	).


}


//count active SSMEs and compare with expected number.
FUNCTION SSME_out {

	//get SSME parameters from vehicle struct snd known number of engines 
	LOCAL SSMEcount IS 0.
	
	FOR e IN SHIP:PARTSDUBBED("ShuttleSSME") {
		IF e:IGNITION {
			SET SSMEcount TO SSMEcount + 1.
		}
	}
	
	IF (SSMEcount < vehicle["SSME"]["active"]) {
		
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
