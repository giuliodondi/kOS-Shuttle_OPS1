@LAZYGLOBAL OFF.


//Launch Settings

GLOBAL target_orbit IS LEXICON (	
								"periapsis",30,
								"apoapsis",220,
								"inclination",52,
								"Cutoff Altitude",112,
								"end",0								//don't remove this
).


// uncomment this line to trigger automatically an engine failure. Alternatively shutdown manually one of the engines 
//GLOBAL engine_failure_time IS 150.
//GLOBAL engine_failure_time IS 230.


//TAL site selection is now automatic. Uncomment this to select a specific TAL site
//GLOBAL TAL_site is "Zaragoza".


GLOBAL logdata Is false.


RUNPATH("0:/UPFG_OPS1/ops1_launch").