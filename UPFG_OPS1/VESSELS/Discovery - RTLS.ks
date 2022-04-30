
GLOBAL vehicle IS LEXICON(
					"name","Space Shuttle - RS25D",
					"launchTimeAdvance", 300,
					"gravturninc",22,
					"roll",180,
					"handover",LEXICON(			//CHOOSE ONLY ONE OF THE FOLLOWING OPTIONS
										//"stage",2			
										"time",130
					),
					"preburn",5.1,			//time to spool up engines at liftoff
					"stages",LIST(0,		//never remove this zero		
					
								LEXICON(
										"m_initial",	1993.889,
										"m_final",	842.995,
										"staging", LEXICON (
											"type","time",
											"ignition",	TRUE,
											"ullage", "none",
											"ullage_t",	0	
										),
										"Tstage",120.8,
										"minThrottle",0.65,
										"engines",	LIST(
														LEXICON("thrust", 2319.9*3, "isp", 453, "flow",522.2162*3, "resources",LIST("LqdHydrogen","LqdOxygen")),	//3xrs25D
														LEXICON("thrust", 14234.3*2, "isp", 263, "flow",5518.99*2, "resources",LIST("PBAN"))	//2xssrb
										)					
								),
								
								LEXICON(
										"m_initial",	702.334,
										"m_final",	112.354,
										"staging", LEXICON (
											"type","glim",
											"ignition",	FALSE,
											"ullage", "none",
											"ullage_t",	0	
										),
										"glim",3,
										"minThrottle",0.65,
										"engines",	LIST(
														LEXICON("thrust", 2319.9*3, "isp", 453, "flow",522.2162*3, "Resources",LIST("LqdHydrogen","LqdOxygen"))	//3xrs25D
										)					
								)
								
							),
					"SSME",LEXICON(
							"isp",453,
							"thrust",2319.9,
							"flow",522.2162,
							"active",3
					
						)
).
GLOBAL events IS LIST(
					LEXICON("time",1,"type", "action","action",{TOGGLE AG1.}),	//activates fuel cells
					LEXICON("time",35,"type", "action","action",{ addMessage("THROTTLING DOWN").SET vehicle["stages"][1]["Throttle"] TO 0.75.}),
					LEXICON("time",60,"type", "action","action",{addMessage("THROTTLING UP"). SET vehicle["stages"][1]["Throttle"] TO 1.}),
					//LEXICON("time",62,"type", "action","action",{LOCAL englist IS SHIP:PARTSDUBBED("ShuttleSSME"). englist[ROUND(RANDOM(),0)]:SHUTDOWN.}),
					LEXICON("time",80,"type", "action","action",{LOCAL englist IS SHIP:PARTSDUBBED("ShuttleSSME"). englist[1]:SHUTDOWN.}),
					LEXICON("time",122,"type", "action","action",{ SET STEERINGMANAGER:MAXSTOPPINGTIME TO 0.2.}),
					//LEXICON("time",180,"type", "action","action",{LOCAL englist IS SHIP:PARTSDUBBED("ShuttleSSME"). englist[1]:SHUTDOWN.}),
					LEXICON("time",350,"type", "roll","angle",0)
).

					






