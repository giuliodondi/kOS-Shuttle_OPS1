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
								"end",0								//don't remove this
).

//change this to the best suitable site based on launch inclination
GLOBAL TAL_site is "Zaragoza".

GLOBAL logdata Is false.

CD("0:/UPFG_OPS1").
run ops1_launch.
