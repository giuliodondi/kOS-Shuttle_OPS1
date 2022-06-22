


GLOBAL g0 IS 9.80665. 


GLOBAL vehicle IS LEXICON(
					"name","Space Shuttle - RS25D",
					"Ext_Tank_Part","ShuttleExtTank",
					"SRB_time",120.8,
					"SSME",LEXICON(
							"isp",453,
							"thrust",2319.9,
							"flow",522.2162,
							"minThrottle",0.65
						)
).



initialise_shuttle().



clearscreen.
debug_vehicle().





function initialise_shuttle {
	CLEARSCREEN.

	RUNPATH("0:/Libraries/resources_library").	

	
	FUNCTION add_resource {
		parameter lexx.
		parameter reslist.
		
		
		for res in reslist {
			IF NOT lexx:HASKEY(res) {lexx:ADD(res,0).}
		}
		RETURN lexx.
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
	
	
	//measure total mass less the SRBs and clamps
	
	LOCAL et_part IS get_ext_tank_part().
	
	
	LOCAL stack_mass IS 0.
	FOR p IN getShuttleParts() {
		set stack_mass to stack_mass + p:mass*1000.
	}
	
	LOCAL total_prop_mass IS get_prop_mass(
		LEXICON(
			"resources",veh_res,
			"ext_tank",et_part
		)
	).
	
	LOCAL stack_empty_mass IS stack_mass - total_prop_mass.	
	
	
	//prepare stages list
	
	
	vehicle:ADD("stages",LIST()).
	
	//zeroth stage 
	vehicle["stages"]:ADD(0).
	
	//stage1 - SRB
	
	LOCAL stage1_burned_mass IS vehicle["SRB_time"] * ssme_count * vehicle["SSME"]["flow"].
	
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
		"Tstage",vehicle["SRB_time"],
		"Throttle",1,
		"minThrottle",vehicle["SSME"]["minThrottle"],
		"engines",	
			LEXICON(
				"thrust", ssme_count*vehicle["SSME"]["thrust"]*1000, 
				"isp", vehicle["SSME"]["isp"], 
				"flow",ssme_count*vehicle["SSME"]["flow"], 
				"resources",LIST("LqdHydrogen","LqdOxygen")
		),
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
		"engines", new_stg_1["engines"],
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
		"minThrottle",new_stg_2["minThrottle"],
		"throt_mult",0,
		"engines",	new_stg_2["engines"],
		"ext_tank",et_part,
		"resources",veh_res,
		"mode", 2
	).
	
	SET new_stg_3["throt_mult"] TO new_stg_3["glim"]*g0/new_stg_3["engines"]["thrust"].
	
	LOCAL y IS const_G_t_m(new_stg_3).
	SET new_stg_3["Tstage"] TO y[0].
	LOCAL stage4InitialMass IS y[1].
	
	If stage4InitialMass <= 0 {
		//no fourth stage to be added
		SET new_stg_3["staging"]["type"] TO "depletion".
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
			"engines", new_stg_1["engines"],
			"tankparts",et_part,
			"resources",veh_res,
			"mode", 1
		).
		
		SET new_stg_4["Tstage"] TO new_stg_4["m_burn"]/(new_stg_4["engines"]["flow"] * new_stg_4["Throttle"]).
	
		vehicle["stages"]:ADD(new_stg_4).
	} 



	//final vehicle parameters
	
	vehicle:ADD("ign_t", 0).
	vehicle:ADD("launchTimeAdvance", 300).
	vehicle:ADD("roll",180).
	vehicle:ADD("preburn",5.1).
	vehicle:ADD(
		"handover",
		LEXICON("time", vehicle["stages"][1]["Tstage"] + 5)
	).
	vehicle:REMOVE("Ext_Tank_Part").
	vehicle:REMOVE("SRB_time").

	
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

	local glim is stg["glim"].
	
	//compute burn time until  we deplete the stage.	
	
	LOCAL maxtime IS (stg["engines"]["isp"]/glim) * LN(1 + stg["m_burn"]/stg["m_final"] ).
	
	//compute burn time until  we reach minimum throttle.	
	LOCAL limtime IS - stg["engines"]["isp"]/glim * LN(stg["minThrottle"]).

	//calculate mass of the fuel burned until violation
	LOCAL mviol IS stg["m_initial"]*CONSTANT:E^(-glim*limtime/stg["engines"]["isp"]).
	
	IF mviol > stg["m_final"]  {
		SET out[1] TO mviol.
		SET out[0] TO limtime.
	} ELSE {
		SET out[0] TO maxtime.
	}
	
		
	RETURN out.
}













//return the ET part to read fuel quantities
FUNCTION get_ext_tank_part {
	return SHIP:PARTSDUBBED(vehicle["Ext_Tank_Part"])[0].
}

FUNCTION get_prop_mass {
	PARAMETER stg.
	
	local tankpart is stg["ext_tank"].
	local reslist is stg["resources"].
	local prop_mass IS 0.
	
	FOR tkres In tankpart:RESOURCES {
		FOR res IN reslist:KEYS {
			IF tkres:NAME = res {
				set prop_mass TO prop_mass + tkres:amount*reslist[res].
			}
	
		}
	}
	set prop_mass to prop_mass*1000.
    RETURN prop_mass.
}



//returns the list of parts making up the Orbiter and External Tank
function getShuttleParts {

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
	
	
	local et is get_ext_tank_part().

	local shuttleParts is ship:parts:copy.

	removeChildrenPartsRecursively(
		shuttleParts,
		et
	).

	shuttleParts:add(et).
	
	return shuttleParts.
}