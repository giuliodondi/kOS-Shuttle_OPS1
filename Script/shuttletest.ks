clearscreen.


GLOBAL vehicle IS LEXICON(
					"name","Space Shuttle - RS25D",
					"Ext_Tank_Part","ShuttleExtTank",
					"SRB_time",120.8,
					"stages",LIST(0,		//never remove this zero		
					
								LEXICON(
										"m_initial",	1993.889,
										"m_final",	842.995,
										"Tstage",
								),
								
								LEXICON(
										"m_initial",	702.334,
										"m_final",	112.354			
								)
								
							),
					"SSME",LEXICON(
							"isp",453,
							"thrust",2319.9,
							"flow",522.2162,
							"minThrottle",0.65
						)
).



initialise_shuttle().



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
		set stack_mass to stack_mass + p:mass.
	}
	
	LOCAL total_prop_mass IS get_prop_mass(
		LEXICON(
			"resources",veh_res,
			"tankparts",et_part
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
			"ignition",	TRUE,
			"ullage", "none",
			"ullage_t",	0	
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
		"tankparts",et_part,
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
		"tankparts",et_part,
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





//return the ET part to read fuel quantities
FUNCTION get_ext_tank_part {
	return SHIP:PARTSDUBBED(vehicle["Ext_Tank_Part"])[0].
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