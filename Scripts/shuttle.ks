@LAZYGLOBAL OFF.

//Launch Settings

//GLOBAL vesselfilename is "Discovery".     //this is the name of the vessel file to load
GLOBAL vesselfilename is "Discovery - RTLS".     //this is the name of the vessel file to load
//GLOBAL vesselfilename is "Discovery - TAL".     //this is the name of the vessel file to load

GLOBAL target_orbit IS LEXICON (	
								"periapsis",30,
								"apoapsis",220,
								"inclination",40,
								"Cutoff Altitude",112,
								//"Longitude of Periapsis",-120		//if circular orbit will be set to arbitrary value
																	// set it 2 or 3 degrees to the EAST of its intended long because of the earth's rotation.
								"end",0								//don't remove this
).

//change this to the best suitable site based on launch inclination
GLOBAL TAL_site is "Zaragoza".





GLOBAL logdata Is true.

//CD("0:/UPFG_latest").
//run upfg_launch.

CD("0:/UPFG_OPS1").
run ops1_launch.