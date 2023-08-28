@LAZYGLOBAL OFF.


//Launch Settings

//for Vandenberg launches

//FWC SRBs, RS-25A, 11.4 ton payload
GLOBAL target_orbit IS LEXICON (	
								"periapsis",0,
								"apoapsis",195,
								"cutoff alt",115,
								"inclination",-104
).


// uncomment this line to trigger automatically an engine failure. Alternatively shutdown manually one of the engines 
//GLOBAL engine_failure_time IS 150.


//TAL site selection is now automatic. Uncomment this to select a specific TAL site
//GLOBAL TAL_site is "Mataveri".


GLOBAL logdata Is false.


RUNPATH("0:/Shuttle_OPS1/src/ops1_launch").
