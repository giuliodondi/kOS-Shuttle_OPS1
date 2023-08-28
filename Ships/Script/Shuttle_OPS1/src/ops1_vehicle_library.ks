GLOBAL g0 IS 9.80665. 
GLOBAL vehicle_countdown IS 10.

GLOBAL vehiclestate IS LEXICON(
	"ops_mode",0,
	"cur_stg", 1,
	"staging_time", 0,
	"staging_in_progress", FALSE,
	"m_burn_left", 0,
	"thr_vec", v(0,0,0),
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
	LOCAL srb_time IS 119.

	
	//hard-coded initialisation of shuttle vehicle
	
	
	//prepare the main struct 
	
	GLOBAL vehicle IS LEXICON(
						"name",SHIP:NAME,
						"ign_t", 0,
						"launchTimeAdvance", 300,
						"trajectory_scale",0,
						"preburn",5.1,
						"roll",180,
						"handover", LEXICON("time", srb_time + 5),
						"maxThrottle",0,	
						"stages",LIST(),
						"SSME",0
	).
	
	SET vehicle["SSME"] TO parse_ssme().
	
	//add the ssme type to the vessel name 
	
	SET vehicle["name"] TO vehicle["name"] + " " + vehicle["SSME"]["type"].
	
	//limit upper throttle in nominal case
	SET vehicle["maxThrottle"] TO vehicle["SSME"]["maxThrottle"].

	
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
		"Throttle",vehicle["maxThrottle"],
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
		"Throttle",vehicle["maxThrottle"],
		"minThrottle",vehicle["SSME"]["minThrottle"],
		"engines",	engines_lex,
		"ext_tank",et_part,
		"resources",veh_res,
		"mode", 1
	).
	
	//will the stage exceed the g limit?
	LOCAL x IS glim_t_m(new_stg_2).
	If x[0] <= 0 {
		PRINT ("ERROR! THE VEHICLE WILL NEVER EXCEED THE 3G ACCELERATION LIMIT. VERIFY PAYLOAD MASS WITHIN LIMITS") AT (1,5).
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
		"Throttle",vehicle["maxThrottle"],
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
	
	SET vehicle["traj_steepness"] TO vehicle_traj_steepness().
	
	SET control["roll_angle"] TO vehicle["roll"].


	setup_engine_failure().
	
	//initialise first stage thrust at 100% rpl 
	SET vehicle["stages"][1]["Throttle"] TO convert_ssme_throt_rpl(1).
	
	//prepare launch triggers 
	add_action_event(1, activate_fuel_cells@ ).
	add_action_event(350, roll_heads_up@ ).
	
	WHEN (SHIP:Q > 0.28) THEN {
		IF NOT (abort_modes["triggered"]) {
			addGUIMessage("THROTTLING DOWN").
			SET vehicle["stages"][1]["Throttle"] TO convert_ssme_throt_rpl(0.7).
		}
	}
	
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

//default trajectory steepness factor 
//bias given deltas of cutoff altitude and engine thrust with respect to reference
//engine thrust with reference to 104% rpl
FUNCTION vehicle_traj_steepness {

	//reference thrust is rs-25D
	LOCAL ssme_thr_fac IS (1.045*get_rpl_thrust())/(vehicle["SSME"]["thrust"]*vehicle["SSME"]["maxThrottle"]).
	
	//reference alt is 112 km.
	LOCAL cut_alt_fac IS target_orbit["cutoff alt"]/112.
	
	//base + alt correction + thrust correction
	LOCAL steep_ IS 1 + 0.14*cut_alt_fac + 0.05*ssme_thr_fac.
	
	RETURN steep_.
}


//open-loop pitch profile for first stage ascent
FUNCTION open_loop_pitch {
	PARAMETER curv.	 

	LOCAL v0 IS 25.
	
	LOCAL refv IS 400.
	
	LOCAL steep_fac IS vehicle["traj_steepness"].
	
	//bias trajectory in case of first-stage rtls
	IF (abort_modes["triggered"] ) {
		SET steep_fac TO steep_fac + RTLS_first_stage_lofting_bias(abort_modes["t_abort_true"]).
	}
	
	IF curv<=v0 {
		RETURN 90.
	} ELSE {
		
		LOCAL p1 IS -0.0068.
		LOCAL p2 IS 28.8.
		LOCAL p3 IS 26300.
		LOCAL q1 IS 3.923.
		
		LOCAL x IS curv + refv - v0.
	
		LOCAL out IS CLAMP((p1*x^2 + p2*x + p3)/(x + q1), 0, 90).
		
		SET out TO out + (steep_fac - 1)*(90 - out)^0.7.
		
		LOCAL bias IS out - surfacestate["vdir"].
		
		RETURN CLAMP(out + 0.8*bias,0,90).
	}
}




//manage roll to heads-up manoeuvre
FUNCTION roll_heads_up {

	//skip if rtls is in progress
	IF (DEFINED RTLSAbort) {
		RETURN.
	}
	
	//setup the new roll and steering
	if (vehicle["roll"] <> 0) {
		SET STEERINGMANAGER:MAXSTOPPINGTIME TO 0.8.
		addGUIMessage("ROLL TO HEADS-UP ATTITUDE").
		SET vehicle["roll"] TO 0.
	}
	
	//set a new control roll angle until we reach zero
	IF (control["roll_angle"] > 3) {
		SET control["roll_angle"] TO MAX(control["roll_angle"] - 7,0).
		
		local tnext is TIME:SECONDS +1.
		WHEN(TIME:SECONDS > tnext) THEN {
			roll_heads_up().
		}
	
	} ELSE {
		SET control["roll_angle"] TO 0.
	}
}




//	Throttle controller
FUNCTION throttleControl {

	local stg IS get_stage().
	local throtval is stg["Throttle"].
	
	IF stg["mode"] = 2   {
		SET throtval TO stg["throt_mult"]*SHIP:MASS*1000.
		SET usc["lastthrot"] TO throtval.
	}
	
	set throtval to min(vehicle["maxThrottle"],throtval).
	set stg["Throttle"] to throtval.

	LOCAL minthrot IS 0.
	IF stg:HASKEY("minThrottle") {
		SET minthrot TO stg["minThrottle"].
	}
	
	RETURN throtteValueConverter(throtval, minthrot).
}


//for terminal guidance, fix the throttle at minimum
FUNCTION fix_minimum_throttle {
	local stg IS get_stage().

	set vehicle["maxThrottle"] to stg["minThrottle"].

}



									//VEHICLE PERFORMANCE & STAGING FUNCTIONS
									

//simple function to check if vehicle is past maxq
FUNCTION check_maxq {
	PARAMETER newq.
	
	IF (newq >=  surfacestate["q"] ) {
		SET surfacestate["q"] TO newq.
	} ELSE {
		addGUIMessage("VEHICLE HAS REACHED MAX-Q").
		surfacestate:REMOVE("q").
		WHEN (SHIP:Q < 0.95*newq) THEN {
			IF (vehicle["stages"][1]["Throttle"] < 1) {
				addGUIMessage("GO AT THROTTLE-UP").
				SET vehicle["stages"][1]["Throttle"] TO 1.
			}
		}
	}

}


FUNCTION get_stage {
	RETURN vehicle["stages"][vehiclestate["cur_stg"]].
}


FUNCTION add_action_event{
	PARAMETER tt.
	PARAMETER callable.
	
	
	events:ADD(
		LEXICON(
				"time",tt,
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
	
	local mbreak is stg["engines"]["thrust"] * stg["Throttle"]/(stg["glim"]*g0).
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
	
	LOCAL x IS get_current_thrust_isp().
	
	SET vehiclestate["thr_vec"] TO x[0].
	
	vehiclestate["avg_thr"]:update(vehiclestate["thr_vec"]:MAG).
	
	LOCAL avg_thrust is vehiclestate["avg_thr"]:average().
	LOCAL avg_isp is x[1].

	IF NOT (vehiclestate["staging_in_progress"]) {
		
		LOCAL current_m IS SHIP:MASS*1000.
		local res_left IS get_prop_mass(cur_stg).
		
		SET vehiclestate["m_burn_left"] to res_left.
		
		
		IF (vehiclestate["cur_stg"]=1) {
		
			//do it here so we bypass the check during later stages
			IF (surfacestate:HASKEY("q") AND surfacestate["vs"] > 50 ) {
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
		addGUIMessage("STAGING SEQUENCE COMPLETE").
		SET vehiclestate["staging_in_progress"] TO FALSE.
	}

}


FUNCTION srb_staging {
	IF vehiclestate["staging_in_progress"] {RETURN.}

	IF (vehicle["stages"][vehiclestate["cur_stg"]]["Tstage"] <= 4 ) {
		SET vehiclestate["staging_in_progress"] TO TRUE.
		//SET control["steerdir"] TO SHIP:FACING.
		addGUIMessage("STAND-BY FOR SRB SEP").
		
		
		//WHEN (get_TWR()<0.98) THEN {
		WHEN (get_srb_thrust()<400) THEN {	//try srb thrust triggering
			
			wait until stage:ready.
			STAGE.
			
			//measure conditions at staging 
			LOCAL v_stg IS SHIP:VELOCITY:SURFACE:MAG.
			SET abort_modes["abort_v"] TO v_stg.	//if an abort hasn't been triggered yet it wil be overwritten
			SET abort_modes["staging"]["v"] TO v_stg.
			SET abort_modes["staging"]["alt"] TO SHIP:ALTITUDE.
			
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
		RETURN (vehicle["stages"][j]["Tstage"] <= upfgFinalizationTime).
	} ELSE {
		
		IF (vehicle["stages"][j]["Tstage"] <=0.1) {
			SET vehiclestate["staging_in_progress"] TO TRUE.
			increment_stage().
		}
		RETURN FALSE.
	}

}









//		SHUTTLE-SPECIFIC FUNCTIONS 


FUNCTION get_srb_thrust {

	local srb is ship:partsdubbed("ShuttleRocketBooster")[0].
	
	return srb:thrust.

}


//if a global flag is set, sets up an event to shutdown one of the SSMEs
FUNCTION setup_engine_failure {

	IF (DEFINED engine_failure_time) {
		
		events:ADD(	
			LEXICON(
					"time",engine_failure_time,
					"type", "action",
					"action",{
								LOCAL englist IS SHIP:PARTSDUBBED("ShuttleSSME"):SUBLIST(0,2).
								select_rand(englist):SHUTDOWN.
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

//return the ET propellant fraction left
function get_et_prop_fraction {
	return vehiclestate["m_burn_left"]/730874.
}


FUNCTION activate_fuel_cells {

	LOCAL partslist IS LIST().
	for p in SHIP:PARTSDUBBEDPATTERN("ShuttleOrbiter*") {
		partslist:ADD(p).
	}
	for p in SHIP:PARTSDUBBED("ShuttleEngMount") {
		partslist:ADD(p).
	}
	
	for p in partslist {
		for m in p:MODULESNAMED("ModuleResourceConverter") {
			for an in m:ALLACTIONNAMES {
				IF an:CONTAINS("start fuel cell") {
					m:DOACTION(an, true).
				}
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

	LOCAL partslist IS LIST().
	for p in SHIP:PARTSDUBBEDPATTERN("ShuttleOrbiter*") {
		partslist:ADD(p).
	}
	for p in SHIP:PARTSDUBBED("ShuttleEngMount") {
		partslist:ADD(p).
	}
	
	for p in partslist {
		for m in p:MODULESNAMED("ModuleAnimateGeneric") {
			if m:hasaction("toggle et door") {
				m:doaction("toggle et door", true).
			}
		}
	}
}




FUNCTION start_oms_dump {
	RCS ON.
	SET SHIP:CONTROL:FORE TO 1.
	SET SHIP:CONTROL:TOP TO 1.
	FOR oms IN SHIP:PARTSDUBBED("ShuttleEngineOMS") {
		oms:ACTIVATE.
	}
	SET abort_modes["oms_dump"] TO TRUE.
}

FUNCTION stop_oms_dump {
	PARAMETER force IS FALSE.
	IF abort_modes["oms_dump"] {
		IF (OMS_quantity()< 0.2 OR force) {

			SET SHIP:CONTROL:NEUTRALIZE TO TRUE.
			FOR oms IN SHIP:PARTSDUBBED("ShuttleEngineOMS") {
				oms:SHUTDOWN.
			}
			addGUIMessage("OMS DUMP STOPPED").
			SET abort_modes["oms_dump"] TO FALSE.
		}
	}
}


FUNCTION OMS_quantity {
	
	LOCAL quant IS 0.
	
	LOCAL i IS 0.
	FOR tank IN ship:PARTSNAMEDPATTERN("ShuttleOMSPod*") {
		SET quant TO quant + (tank:mass - tank:drymass)/(tank:wetmass - tank:drymass).
		SET i TO i + 1.
	}
	SET quant TO quant/i.
	
	return quant.

}

//check that the three ssme are present and the same type, calculate all the parameters needed
FUNCTION parse_ssme {
	
	LOCAL ssmelex IS LEXICON(
		"type","",
		"active",0,
		"isp",0,
		"thrust",0,
		"flow",0,
		"minThrottle",0,
		"maxThrottle",0
	).
	
	//count SSMEs, not necessary per se but a useful check to see if we're flying a DECQ shuttle
	LOCAL ssme_count IS 0. 
	SET ssmelex["type"] TO SHIP:PARTSDUBBED("ShuttleSSME")[0]:CONFIG.
	
	FOR ssme IN SHIP:PARTSDUBBED("ShuttleSSME") {
		IF ssme:ISTYPE("engine") {
			SET ssme_count TO ssme_count + 1.
			
			IF (ssme:CONFIG <> ssmelex["type"]) {
				PRINT ("ERROR! THE VEHICLE SEEMS TO HAVE MISMATCHED SSME TYPES") AT (1,5).
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
		PRINT ("ERROR! THE VEHICLE SEEMS TO HAVE THE WRONG NUMBER OF SSMEs") AT (1,5).
		LOCAL X IS 1/0.
	}
	
	SET ssmelex["active"] TO ssme_count.
	
	SET ssmelex["isp"] TO ssmelex["isp"]/ssmelex["thrust"].
	SET ssmelex["minThrottle"] TO ssmelex["minThrottle"]/ssmelex["thrust"].
	
	SET ssmelex["thrust"] TO ssmelex["thrust"]/ssme_count.
	SET ssmelex["flow"] TO ssmelex["flow"]/ssme_count.

	//calculate max throttle given nominal power level of 104.5%
	SET ssmelex["maxThrottle"] TO min(1.045*get_rpl_thrust()/ssmelex["thrust"], 1).
	
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

//100 percent rated power level in kn in vacuum (1ssme)
function get_rpl_thrust {
	return 2090.
}

//given a throttle value as a percentage of rpl, converts it into absolute percentage
FUNCTION convert_ssme_throt_rpl {
	parameter rpl_throt.
	
	local stg IS get_stage().
	local ssme_thr IS stg["engines"]["thrust"]/3000.
	
	//first work out the rpl level of the engines' max thrust 
	
	local max_rpl_thr is ssme_thr/get_rpl_thrust().
	
	//then do the proportion with 100% commanded throttle 
	
	return rpl_throt/max_rpl_thr.

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
		
		LOCAL cur_stg IS get_stage().

		SET cur_stg["engines"] TO build_ssme_lex().
		
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

//get the ssme instantaneous vacuum thrust as percentage
function get_ssme_throttle {
	
	local stg IS get_stage().
	local throtval is stg["Throttle"].
	
	local ssme_thr IS throtval* stg["engines"]["thrust"]/(vehicle["SSME"]["active"] * 1000).
	
	return ssme_thr/get_rpl_thrust().
}
