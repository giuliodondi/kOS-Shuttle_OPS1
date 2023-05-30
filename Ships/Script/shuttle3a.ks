@LAZYGLOBAL OFF.


//Launch Settings

//for Vandenberg launches

GLOBAL target_orbit IS LEXICON (	
								"periapsis",30,
								"apoapsis",195,
								"inclination",-104,
								"Cutoff Altitude",120,
								"end",0	
).

//GLOBAL target_orbit IS LEXICON (	
//								"periapsis",30,
//								"apoapsis",220,
//								"inclination",-87,
//								"Cutoff Altitude",115,
//								"end",0								//don't remove this
//).


// uncomment this line to trigger automatically an engine failure. Alternatively shutdown manually one of the engines 
//GLOBAL engine_failure_time IS 150.


//TAL site selection is now automatic. Uncomment this to select a specific TAL site
//GLOBAL TAL_site is "Mataveri".


GLOBAL logdata Is false.


RUNPATH("0:/UPFG_OPS1/ops1_launch").
