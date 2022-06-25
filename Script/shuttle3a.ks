@LAZYGLOBAL OFF.


//Launch Settings

//for Vandenberg launches

//GLOBAL target_orbit IS LEXICON (	
//								"periapsis",140,
//								"apoapsis",190,
//								"inclination",-104,
//								"Cutoff Altitude",160,
//								"end",0	
//).

GLOBAL target_orbit IS LEXICON (	
								"periapsis",30,
								"apoapsis",220,
								"inclination",-87,
								"Cutoff Altitude",115,
								"end",0								//don't remove this
).


// uncomment this line to trigger automatically an engine failure. Alternatively shutdown manually one of the engines 
//GLOBAL engine_failure_time IS 150.


//change this to the best suitable site based on launch inclination
GLOBAL TAL_site is "Mataveri".


GLOBAL logdata Is false.


RUNPATH("0:/UPFG_OPS1/ops1_launch").
