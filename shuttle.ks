@LAZYGLOBAL OFF.

//Launch Settings

//GLOBAL vesselfilename is "Columbia".     //this is the name of the vessel file to load
//GLOBAL vesselfilename is "Discovery".     //this is the name of the vessel file to load
GLOBAL vesselfilename is "Discovery - RTLS".     //this is the name of the vessel file to load
//LOBAL vesselfilename is "Discovery - TAL".     //this is the name of the vessel file to load

GLOBAL target_orbit IS LEXICON (	
								"periapsis",30,
								"apoapsis",220,
								"inclination",40,
								"Cutoff Altitude",120,
								//"Longitude of Periapsis",-120		//if circular orbit will be set to arbitrary value
																	// set it 2 or 3 degrees to the EAST of its intended long because of the earth's rotation.
								"end",0								//don't remove this
).







GLOBAL logdata Is false.

//CD("0:/UPFG_latest").
//run upfg_launch.

CD("0:/UPFG_OPS1").
run ops1_launch.