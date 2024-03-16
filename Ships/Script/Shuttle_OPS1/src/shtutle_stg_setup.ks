
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
						"qbucketval", 0.28,
						"qbucket", FALSE,
						"max_q_reached", FALSE,
						"handover", LEXICON("time", srb_time + 5),
						"glim", 3,
						"maxThrottle",0,	
						"minThrottle",0,	
						"nominalThrottle",0,	
						"qbucketThrottle",0,	
						"SSME_prop_0", 0,
						"SSME_prop", 0,
						"OMS_prop_0", 0,
						"OMS_prop", 0,
						"stages",LIST(),
						"SSME",0,
						"OMS",0,
						"ssme_out_detected", FALSE
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
	SET vehicle["minThrottle"] TO vehicle["SSME"]["minThrottle"].
	SET vehicle["nominalThrottle"] TO min(1, convert_ssme_throt_rpl(1.045)).
	SET vehicle["qbucketThrottle"] TO convert_ssme_throt_rpl(0.75).
	
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
	
	LOCAL new_stg_1 IS empty_stg_template().
	
	set new_stg_1["m_initial"] to stack_mass.
	set new_stg_1["glim"] to vehicle["glim"].
	set new_stg_1["Throttle"] to vehicle["nominalThrottle"].
	set new_stg_1["Tstage"] to srb_time.
	set new_stg_1["engines"] to engines_lex.
	set new_stg_1["mode"] to 1.
	SET new_stg_1["staging"]["type"] TO "time".
	
	LOCAL stage1_final_mass IS const_f_dt_mfinal(new_stg_1).
	
	SET new_stg_1["m_final"] TO stack_mass - stage1_final_mass.
	SET new_stg_1["m_burn"] TO stage1_final_mass. 

	vehicle["stages"]:ADD(new_stg_1).
	
	setup_shuttle_stages(
	
	).
	
	SET vehicle["traj_steepness"] TO vehicle_traj_steepness().
	
	//debug_vehicle().
	
	//prepare launch triggers 
	add_action_event(1, activate_fuel_cells@ ).
	add_action_event(350, roll_heads_up@ ).
	
	setup_engine_failure().
	
}

function setup_shuttle_stages {
	parameter initial_mass.
	parameter stack_empty_mass.
	parameter max_throtval.
	
	local stg_1 is vehicle["stages"][1].
	
	LOCAL engines_lex IS build_engines_lex().
	
	// ssme constant F stage
	
	local new_stg_2 is empty_stg_template().
	
	set new_stg_2["m_initial"] to initial_mass.
	set new_stg_2["m_final"] to stack_empty_mass.
	set new_stg_2["m_burn"] to initial_mass - stack_empty_mass.
	set new_stg_2["glim"] to vehicle["glim"].
	set new_stg_2["Throttle"] to max_throtval.
	set new_stg_2["engines"] to engines_lex.
	set new_stg_2["mode"] to 1.
	
	LOCAL stage3InitialMass IS 0.
	LOCAL stage3mode IS 0.
	
	LOCAL gl_out IS glim_t_m(new_stg_2).
	
	If gl_out[0] <= 0 {
		// won't violate the glim
		set stage3InitialMass to new_stg_2["m_final"].
		set stage3mode to 1.
		SET new_stg_2["Tstage"] TO const_f_t(new_stg_2).
		SET new_stg_2["staging"]["type"] TO "depletion".
		
	} else {
		// will violate the glim 
		set stage3InitialMass to gl_out[1].
		set stage3mode to 2.
		set new_stg_2["m_final"] to gl_out[1].
		SET new_stg_2["m_burn"] TO new_stg_2["m_initial"] - new_stg_2["m_final"].
		SET new_stg_2["Tstage"] TO gl_out[0].
		SET new_stg_2["staging"]["type"] TO "glim".
	}
	
	local new_stg_3 is empty_stg_template().
	stg_list:add(new_stg_3).
	
	set new_stg_3["m_initial"] to stage3InitialMass.
	set new_stg_3["m_final"] to stack_empty_mass.
	set new_stg_3["m_burn"] to stage3InitialMass - stack_empty_mass.
	set new_stg_3["glim"] to vehicle["glim"].
	set new_stg_3["Throttle"] to max_throtval.
	set new_stg_3["engines"] to engines_lex.
	set new_stg_3["mode"] to stage3mode.
	
	LOCAL stage4InitialMass IS 0.
	
	if (stage3mode = 1) {
		// only if we never reach the glim 
		
		SET new_stg_3["Tstage"] TO const_f_t(new_stg_3).
		SET new_stg_3["staging"]["type"] TO "depletion".
		
	} else if (stage3mode = 2) {
		LOCAL gtm_out IS const_G_t_m(new_stg_3).
		
		set stage4InitialMass to gtm_out[1].
		SET new_stg_3["m_final"] TO gtm_out[1].
		SET new_stg_3["m_burn"] TO new_stg_3["m_initial"] - new_stg_3["m_final"].
		SET new_stg_3["Tstage"] TO gtm_out[0].
		
		
		
		if (stage4InitialMass > stack_empty_mass) {
			SET new_stg_3["staging"]["type"] TO "minthrot".
		} else {
			SET new_stg_3["staging"]["type"] TO "depletion".
		}
		
	}	
	
	
	local new_stg_4 is empty_stg_template().
	stg_list:add(new_stg_4).
	
	set new_stg_4["m_initial"] to stage4InitialMass.
	set new_stg_4["m_final"] to stack_empty_mass.
	set new_stg_4["m_burn"] to stage4InitialMass - stack_empty_mass.
	set new_stg_4["glim"] to vehicle["glim"].
	set new_stg_4["Throttle"] to vehicle["minThrottle"].
	set new_stg_4["engines"] to engines_lex.
	set new_stg_4["mode"] to 1.
	set new_stg_4["staging"]["type"] TO "depletion".
	SET new_stg_4["Tstage"] TO const_f_t(new_stg_4).
	
	set vehicle["stages"] to list(
									0,
									stg_1,
									new_stg_2,
									new_stg_3,
									new_stg_4
	).

}




function empty_stg_template {

	return LEXICON(
		"m_initial", 0,
		"m_final", 0,
		"m_burn", 0,
		"staging", LEXICON (
					"type","",
					"ignition",	FALSE
		),
		"glim", 0,
		"ign_t", 0,
		"Throttle", 0,
		"Tstage",0,
		"engines", 0,
		"mode", 0
	).

}