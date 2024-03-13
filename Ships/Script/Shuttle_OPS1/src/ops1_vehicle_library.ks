GLOBAL g0 IS 9.80665. 
GLOBAL vehicle_countdown IS 10.

GLOBAL vehiclestate IS LEXICON(
	"phase",0,
	"cur_stg", 1,
	"staging_time", 0,
	"staging_in_progress", FALSE,
	"m_burn_left", 0,
	"thr_vec", v(0,0,0),
	"avg_thr", average_value_factory(6)
).


GLOBAL control Is LEXICON(
	"launch_az",0,
	"aimvec",SHIP:FACING:FOREVECTOR,
	"steerdir", LOOKDIRUP(-SHIP:ORBIT:BODY:POSITION, SHIP:FACING:TOPVECTOR),
	"roll_angle",0,
	"refvec", v(0,0,0)
).



GLOBAL events IS LIST().


//VEHICLE INITIALISATION FUNCTION 


function initialise_shuttle {

	//RUNPATH("0:/Libraries/resources_library").	

	
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
						"roll_v0",35,
						"pitch_v0",38.7096,
						"trajectory_scale",0,
						"preburn",5.1,
						"roll",180,
						"handover", LEXICON("time", srb_time + 5),
						"maxThrottle",0,	
						"SSME_prop_0", 0,
						"SSME_prop", 0,
						"OMS_prop_0", 0,
						"OMS_prop", 0,
						"stages",LIST(),
						"SSME",0,
						"OMS",0
	).
	
	SET vehicle["SSME"] TO parse_ssme().
	SET vehicle["OMS"] TO parse_oms().
	
	//measure initial propellants 
	SET vehicle["SSME_prop"] TO get_ssme_prop().
	SET vehicle["SSME_prop_0"] TO vehicle["SSME_prop"].
	
	SET vehicle["OMS_prop"] TO get_oms_prop().
	SET vehicle["OMS_prop_0"] TO vehicle["OMS_prop"].
	
	//now reset number of engines to default : 3 active SSME, 0 active OMS
	SET vehicle["SSME"]["active"] TO 3.
	SET vehicle["OMS"]["active"] TO 0.
	
	//add the ssme type to the vessel name 
	
	SET vehicle["name"] TO vehicle["name"] + " " + vehicle["SSME"]["type"].
	
	//limit upper throttle in nominal case
	SET vehicle["maxThrottle"] TO vehicle["SSME"]["maxThrottle"].
	
	//measure total mass less the SRBs and clamps
	LOCAL stack_mass IS getShuttleStackMass().
	
	//nominally we just burn SSME propellant
	LOCAL total_prop_mass IS vehicle["SSME_prop"].
	
	LOCAL stack_empty_mass IS stack_mass - total_prop_mass.	
	
	
	//prepare stages list
	
	LOCAL engines_lex IS build_engines_lex().
	
	
	//zeroth stage 
	vehicle["stages"]:ADD(0).
	
	//stage1 - SRB
	
	LOCAL new_stg_1 IS LEXICON(
		"m_initial",	stack_mass,
		"m_final",	0,
		"m_burn",	0,
		"staging", LEXICON (
			"type","time",
			"ignition",	TRUE
		),
		"ign_t", 0,
		"Tstage",srb_time,
		"Throttle",vehicle["maxThrottle"],
		"engines",	engines_lex,
		"mode", 1
	).
	
	LOCAL stage1_final_mass IS const_f_dt_mfinal(new_stg_1).
	
	SET new_stg_1["m_final"] TO stack_mass - stage1_final_mass.
	SET new_stg_1["m_burn"] TO stage1_final_mass. 

	vehicle["stages"]:ADD(new_stg_1).
	
	
	//stage 2 - SSME CONSTANT T
	
	LOCAL stage2InitialMass IS stage1_final_mass.
	
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
		"engines",	engines_lex,
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
		"throt_mult",0,
		"engines",	engines_lex,
		"mode", 2
	).
	
	SET new_stg_3["throt_mult"] TO new_stg_3["glim"]*g0/engines_lex["thrust"].
	
	LOCAL y IS const_G_t_m(new_stg_3).
	SET new_stg_3["Tstage"] TO y[0].
	LOCAL stage4InitialMass IS y[1].
	
	//we don't want to ad a fourth stage unless it burns for at least 3 seconds
	//to avoid problems with mass uncertainties
	//if it's zero already because there is no need for a fourth stage at all we fall in the same condition 
	
	LOCAL min_stage4InitialMass IS stack_empty_mass + 3 * engines_lex["flow"] * engines_lex["minThrottle"] .
	
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
			"Throttle",engines_lex["minThrottle"],
			"engines",	engines_lex,
			"mode", 1
		).
		
		SET new_stg_4["Tstage"] TO const_f_t(new_stg_4).
	
		vehicle["stages"]:ADD(new_stg_4).
	} 
	
	SET vehicle["traj_steepness"] TO vehicle_traj_steepness().
	
	SET control["roll_angle"] TO vehicle["roll"].

	//initialise first stage thrust at 100% rpl 
	SET vehicle["stages"][1]["Throttle"] TO convert_ssme_throt_rpl(1).
	
	//debug_vehicle().
	
	//prepare launch triggers 
	add_action_event(1, activate_fuel_cells@ ).
	add_action_event(350, roll_heads_up@ ).
	
	setup_engine_failure().
	
	WHEN (SHIP:Q > 0.28) THEN {
		IF NOT (abort_modes["triggered"]) {
			addGUIMessage("THROTTLING DOWN").
			SET vehicle["stages"][1]["Throttle"] TO convert_ssme_throt_rpl(0.75).
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

fUNCTION nominal_pitch_profile {
	PARAMETER rel_v.
	PARAMETER steep_fac.
	
	LOCAL p1 IS -0.0088.
	LOCAL p2 IS 30.5.
	LOCAL p3 IS 28000.
	LOCAL q1 IS 3.923.
	
	LOCAL x IS rel_v + 400.

	LOCAL out IS CLAMP((p1*x^2 + p2*x + p3)/(x + q1), 0, 90).
	
	return out + (steep_fac - 1)*(90 - out)^0.7.

}

//open-loop pitch profile for first stage ascent
FUNCTION open_loop_pitch {
	PARAMETER curv.	 

	LOCAL v0 IS vehicle["pitch_v0"].
	
	LOCAL v_match IS 110.
	
	LOCAL steep_fac IS vehicle["traj_steepness"].
	
	//bias trajectory in case of first-stage rtls
	IF (abort_modes["triggered"] ) {
		SET steep_fac TO steep_fac + RTLS_first_stage_lofting_bias(abort_modes["t_abort_true"]).
	}
	
	IF (curv<=v0) {
		RETURN 90.
	} ELSE {
	
		LOCAL vrel IS curv - v0.
	
		LOCAL pitch_prof IS 0.
		
		//quadratic matching to the pitch profile
		IF (curv<=v_match) {
			
			LOCAL vrel_match IS v_match - v0.
			
			LOCAL prof_match IS nominal_pitch_profile(vrel_match, steep_fac).
			
			LOCAL dv IS 2.
			LOCAL dp_dv IS (nominal_pitch_profile(vrel_match + dv, steep_fac) - prof_match) / dv.
			
			local c_ is (2*(prof_match - 90) - dp_dv * vrel_match) / vrel_match^3.
			local b_ is (-dp_dv/vrel_match - 3*c_*vrel_match)/2.
			
			SET pitch_prof tO 90 - (b_ + c_ * vrel) * vrel^2.
			
		} ELSE {
			SET pitch_prof TO nominal_pitch_profile(vrel, steep_fac).
			
			LOCAL bias IS pitch_prof - surfacestate["vdir"].
		
			LOCAL bias_gain IS CLAMP((curv - v_match)/ 200, 0, 0.5).
			
			LOCAL prof_corr IS ABS(bias_gain * bias).
			
			//don't deviate too much from prograde
			LOCAL max_prograde_dev IS 12.
			LOCAL max_prof_dev IS MAX(max_prograde_dev - ABS((bias_gain + 1) * bias), 0).
			
			set pitch_prof to surfacestate["vdir"] + SIGN(bias) * MIN(max_prof_dev, prof_corr).
			
		}
		
		
		
		RETURN CLAMP(pitch_prof, 0, 90).
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
		addGUIMessage("ROLL TO HEADS-UP ATTITUDE").
		SET vehicle["roll"] TO 0.
		set_steering_med().
	}
	
	LOCAL aimv IS control["aimvec"].
	LOCAL refv IS VXCL(aimv, control["refvec"]):NORMALIZED.
	
	LOCAL topv IS VXCL(aimv, SHIP:FACING:TOPVECTOR):NORMALIZED.
	LOCAL cur_roll IS VANG(refv, topv).
	LOCAL angle_err IS vehicle["roll"] -  cur_roll.
	
	IF (ABS(angle_err) > 0.5) {
		
		LOCAL delta IS SIGN(angle_err) * MIN(ABS(angle_err), 15).
		
		SET control["roll_angle"] TO cur_roll + delta.
		
		local tnext is TIME:SECONDS + 0.2.
		WHEN(TIME:SECONDS > tnext) THEN {
			roll_heads_up().
		}
		
	} ELSE {
		SET control["roll_angle"] TO vehicle["roll"].
		set_steering_low().
	}
	
}


//Steering controller
FUNCTION steeringControl {
	RETURN aimAndRoll(control["aimvec"], control["refvec"], control["roll_angle"]).	
}


//	Throttle controller
FUNCTION throttleControl {

	local stg IS get_stage().
	local throtval is stg["Throttle"].
	
	IF stg["mode"] = 2   {
		SET throtval TO stg["throt_mult"]*SHIP:MASS*1000.
		SET upfgInternal["throtset"] TO throtval.
	}
	
	set throtval to min(vehicle["maxThrottle"],throtval).
	set stg["Throttle"] to throtval.

	LOCAL minthrot IS stg["engines"]["minThrottle"].
	
	RETURN throtteValueConverter(throtval, minthrot).
}


//for terminal guidance, fix the throttle at minimum
FUNCTION fix_minimum_throttle {
	local stg IS get_stage().

	set vehicle["maxThrottle"] to stg["engines"]["minThrottle"].

}

FUNCTION set_steering_high {
	SET STEERINGMANAGER:MAXSTOPPINGTIME TO 1.5.
}

FUNCTION set_steering_med {
	SET STEERINGMANAGER:MAXSTOPPINGTIME TO 0.7.
}

FUNCTION set_steering_low {
	SET STEERINGMANAGER:MAXSTOPPINGTIME TO 0.1.
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

FUNCTION get_TWR {
	RETURN vehiclestate["avg_thr"]:average()/(1000*SHIP:MASS*g0).
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
		
		local res_left IS get_shuttle_res_left().
		
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
	
	//moved this here bc we must update oms propellant once before checking if it's time to stop
	stop_oms_dump().
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
		
		IF (x[0] > 0) {
			SET stg2["Tstage"] TO x[0]. 
			SET stg2["m_final"] TO x[1]. 
			SET stg2["m_burn"] TO m_initial - x[1].
		} ELSE {
			SET stg2["Tstage"] TO 0. 
			SET stg2["m_burn"] TO m_initial - stg2["m_final"].
		}
		
		
		
		LOCAL stg3_m_initial IS x[1].
		LOCAL stg3_m_burn IS res_left - stg2["m_burn"].
		
		update_stage3(stg3_m_initial, stg3_m_burn).
	} ELSE IF (stg2["staging"]["type"]="time") {
		//only ever entered during aborts 
		
		SET stg2["m_final"] TO const_f_dt_mfinal(stg2).
		SET stg2["m_burn"] TO stg2["m_initial"] - stg2["m_final"].
		
		IF (vehicle["stages"]:LENGTH > 3) {
			LOCAL stg3_m_initial IS stg2["m_final"].
			LOCAL stg3_m_burn IS MAX(0, res_left - stg2["m_burn"]).
			
			update_stage3(stg3_m_initial, stg3_m_burn).
		}
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
		//constant thrust depletion stage, only used for aborts
		
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
	
	IF (vehiclestate["phase"]=2) {
		SET upfgInternal["throtset"] TO vehicle["stages"][j+1]["Throttle"].
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
		RETURN (vehicle["stages"][j]["Tstage"] <= upfgInternal["terminal_time"]).
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
								LOCAL englist IS SHIP:PARTSDUBBED("ShuttleSSME").
								LOCAL zpos IS 0.
								LOCAL ze_ IS 0.
								FROM {local e_ is 0.} UNTIL e_ >= englist:LENGTH STEP {set e_ to e_+1.} DO { 
									
									LOCAL eng IS englist[e_].
									LOCAL z_ IS VDOT(VXCL(SHIP:FACING:STARVECTOR, eng:POSITION), SHIP:FACING:TOPVECTOR).
									IF (z_ > zpos) {
										set zpos to z_.
										set ze_ to e_.
									}
								}
								englist:REMOVe(ze_).
								select_rand(englist):SHUTDOWN.
					}
			)
		).
	
	}

}



//measure both ssme and oms fuel 
FUNCTION get_shuttle_res_left {

	SET vehicle["SSME_prop"] TO get_ssme_prop().
	SET vehicle["OMS_prop"] TO get_oms_prop().

	//default and to prevent divisions by zero
	LOCAL total_prop IS vehicle["SSME_prop"].
	//IF (vehicle["SSME"]["active"] > 0) {
	//	SET total_prop TO total_prop + vehicle["SSME_prop"].
	//}
	
	IF (vehicle["OMS"]["active"] > 0) {
		SET total_prop TO total_prop + vehicle["OMS_prop"].
	}
		
	return total_prop.
}

//return the ET part to read fuel quantities
//designed to crash if there is no part with this name
FUNCTION get_ext_tank_part {
	RETURN SHIP:PARTSDUBBED("ShuttleExtTank")[0].
}

//get propellants for SSMEs
FUNCTION get_ssme_prop {
	return get_prop_mass(
		LEXICON(
			"resources",vehicle["SSME"]["resources"],
			"tankparts",LIST(get_ext_tank_part())
		)
	).
}

//return the ET propellant fraction left
function get_et_prop_fraction {
	return vehicle["SSME_prop"] / vehicle["SSME_prop_0"].
}

//return a list ocntaining parts with OMS fuel (i.e. oms pods)
//designed to crash if there are less than two matching parts
FUNCTION get_oms_tanks_parts {
	LOCAL partslist IS ship:PARTSNAMEDPATTERN("ShuttleOMSPod*").
	RETURN LIST(partslist[0], partslist[1]).
}

//get propellants for OMSs
FUNCTION get_oms_prop {
	return get_prop_mass(
		LEXICON(
			"resources",vehicle["OMS"]["resources"],
			"tankparts",get_oms_tanks_parts()
		)
	).
}

//return the ET propellant fraction left
function get_oms_prop_fraction {
	return vehicle["OMS_prop"] / vehicle["OMS_prop_0"].
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

	IF ((NOT abort_modes["triggered"]) OR (abort_modes["oms_dump"])) {
		RETURN.
	}

	RCS ON.
	//SET SHIP:CONTROL:FORE TO 1.
	//SET SHIP:CONTROL:TOP TO 1.
	FOR oms IN SHIP:PARTSDUBBED("ShuttleEngineOMS") {
		oms:ACTIVATE.
	}
	SET abort_modes["oms_dump"] TO TRUE.
}

FUNCTION stop_oms_dump {
	PARAMETER force IS FALSE.
	
	IF (NOT (abort_modes["triggered"] AND abort_modes["oms_dump"])) {
		RETURN.
	}

	IF (get_oms_prop_fraction() <= 0.2 OR force) {

		SET SHIP:CONTROL:NEUTRALIZE TO TRUE.
		FOR oms IN SHIP:PARTSDUBBED("ShuttleEngineOMS") {
			oms:SHUTDOWN.
		}
		addGUIMessage("OMS DUMP STOPPED").
		SET abort_modes["oms_dump"] TO FALSE.
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
		"minThrottle",0,
		"maxThrottle",0,
		"resources",0
	).
	
	//count SSMEs, not necessary per se but a useful check to see if we're flying a DECQ shuttle
	LOCAL ssme_count IS 0. 
	SET ssmelex["type"] TO SHIP:PARTSDUBBED("ShuttleSSME")[0]:CONFIG.
	
	LOCAL ssme_reslex IS LEXICON().
	
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
			LOCAL ssme_res IS ssme:consumedresources:VALUES.
	
			SET ssmelex["thrust"] TO ssmelex["thrust"] + ssme_thr.
			SET ssmelex["isp"] TO ssmelex["isp"] + ssme_isp*ssme_thr.
			SET ssmelex["flow"] TO ssmelex["flow"] + ssme_flow.
			SET ssmelex["minThrottle"] TO ssmelex["minThrottle"] + ssme_minthr*ssme_thr.
			
			FOR res IN ssme_res {
				IF NOT ssme_reslex:HASKEY(res:name) {
					ssme_reslex:ADD(res:name, res).
				}
			}
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
	
	SET ssmelex["resources"] TO ssme_reslex.
	
	RETURN ssmelex.
	
}


//check that the two oms are present and the same type, calculate all the parameters needed
FUNCTION parse_oms {
	
	LOCAL omslex IS LEXICON(
		"type","",
		"active",0,
		"isp",0,
		"thrust",0,
		"flow",0,
		"minThrottle",0,
		"maxThrottle",0,
		"resources",0
	).
	
	//count OMS, not necessary per se but a useful check to see if we're flying a DECQ shuttle
	LOCAL oms_count IS 0. 
	SET omslex["type"] TO SHIP:PARTSDUBBED("ShuttleEngineOMS")[0]:CONFIG.
	
	LOCAL oms_reslex IS LEXICON().
	
	FOR oms IN SHIP:PARTSDUBBED("ShuttleEngineOMS") {
		IF oms:ISTYPE("engine") {
			SET oms_count TO oms_count + 1.
			
			IF (oms:CONFIG <> omslex["type"]) {
				PRINT ("ERROR! THE VEHICLE SEEMS TO HAVE MISMATCHED OMS TYPES") AT (1,5).
				LOCAL X IS 1/0.
			}
			
			LOCAL oms_thr IS oms:POSSIBLETHRUSTAT(0.0).
			LOCAL oms_isp IS oms:VACUUMISP.
			LOCAL oms_flow IS oms:MAXMASSFLOW*1000.
			LOCAL oms_minthr IS oms:MINTHROTTLE.
			LOCAL oms_res IS oms:consumedresources:VALUES.
	
			SET omslex["thrust"] TO omslex["thrust"] + oms_thr.
			SET omslex["isp"] TO omslex["isp"] + oms_isp*oms_thr.
			SET omslex["flow"] TO omslex["flow"] + oms_flow.
			SET omslex["minThrottle"] TO omslex["minThrottle"] + oms_minthr*oms_thr.
			
			FOR res IN oms_res {
				IF NOT oms_reslex:HASKEY(res:name) {
					oms_reslex:ADD(res:name, res).
				}
			}
		}
	}
	
	IF (oms_count <> 2 ) {
		PRINT ("ERROR! THE VEHICLE SEEMS TO HAVE THE WRONG NUMBER OF OMSs") AT (1,5).
		LOCAL X IS 1/0.
	}
	
	SET omslex["active"] TO oms_count.
	
	SET omslex["isp"] TO omslex["isp"]/omslex["thrust"].
	SET omslex["minThrottle"] TO omslex["minThrottle"]/omslex["thrust"].
	
	SET omslex["thrust"] TO omslex["thrust"]/oms_count.
	SET omslex["flow"] TO omslex["flow"]/oms_count.

	//calculate max throttle given nominal power level of 104.5%
	SET omslex["maxThrottle"] TO 1.
	
	SET omslex["resources"] TO oms_reslex.
	
	RETURN omslex.
	
}

FUNCTION build_ssme_lex {

	RETURN LEXICON(
				"thrust", vehicle["SSME"]["active"]*vehicle["SSME"]["thrust"]*1000, 
				"isp", vehicle["SSME"]["isp"], 
				"flow",vehicle["SSME"]["active"]*vehicle["SSME"]["flow"]
	).
}

FUNCTION build_engines_lex {

	LOCAL tot_ssme_thrust IS vehicle["SSME"]["active"]*vehicle["SSME"]["thrust"].
	LOCAL tot_oms_thrust IS vehicle["OMS"]["active"]*vehicle["OMS"]["thrust"].
	LOCAL tot_thrust IS tot_ssme_thrust + tot_oms_thrust.

	LOCAL ssme_flow IS vehicle["SSME"]["active"]*vehicle["SSME"]["flow"].
	LOCAL oms_flow IS vehicle["OMS"]["active"]*vehicle["OMS"]["flow"].
	LOCAL tot_flow IS ssme_flow + oms_flow.
	
	LOCAL tot_isp IS 0.
	
	IF (tot_flow > 0) {
		SET tot_isp TO (vehicle["SSME"]["isp"] * ssme_flow + vehicle["OMS"]["isp"] * oms_flow) / tot_flow.
	}
	
	
	LOCAL tot_minThrot IS vehicle["SSME"]["minThrottle"].
	IF (tot_thrust > 0) {
		SET tot_minThrot TO (tot_ssme_thrust * vehicle["SSME"]["minThrottle"] + tot_oms_thrust * vehicle["OMS"]["minThrottle"]) / tot_thrust.
	}
	
	RETURN LEXICON(
				"thrust", tot_thrust*1000, 
				"isp", tot_isp, 
				"flow",tot_flow,
				"minThrottle", tot_minThrot
	).
}

//100 percent rated power level in kn in vacuum (1ssme)
function get_rpl_thrust {
	return 2090.
}

//given a throttle value as a percentage of rpl, converts it into absolute percentage
FUNCTION convert_ssme_throt_rpl {
	parameter rpl_throt.
	
	//first work out the rpl level of the engines' max thrust 
	
	local max_rpl_thr is vehicle["SSME"]["thrust"]/get_rpl_thrust().
	
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
		
		measure_update_engines().
		
		RETURN TRUE.
	}

	RETURN FALSE.
}

//measure running engines of both kinds and rebuild the engines LEXICON
FUNCTION measure_update_engines {
	
	//measure engines 
	LOCAL SSMEcount IS 0.
	FOR e IN SHIP:PARTSDUBBED("ShuttleSSME") {
		IF e:IGNITION {
			SET SSMEcount TO SSMEcount + 1.
		}
	}
	
	
	SET vehicle["SSME"]["active"] TO SSMEcount.
	
	LOCAL OMScount IS 0.
	FOR e IN SHIP:PARTSDUBBED("ShuttleEngineOMS") {
		IF e:IGNITION {
			SET OMScount TO OMScount + 1.
		}
	}
	
	SET vehicle["OMS"]["active"] TO OMScount.
	
	LOCAL cur_stg IS get_stage().
	SET cur_stg["engines"] TO build_engines_lex().
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
	
	local ssme_thr IS throtval* vehicle["SSME"]["thrust"].
	
	return ssme_thr/get_rpl_thrust().
}
