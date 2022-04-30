
GLOBAL vehicle IS LEXICON(
					"name","Space Shuttle - RS25A",
					"launchTimeAdvance", 300,
					"gravturninc",22,
					"roll",180,
					"handover",LEXICON(			//CHOOSE ONLY ONE OF THE FOLLOWING OPTIONS
										"stage",2			
										//"time",140
					),
					"preburn",5.1,			//time to spool up engines at liftoff
					"stages",LIST(0,		//never remove this zero		
					
								LEXICON(
										"m_initial",	1997.171,
										"m_final",	823.315,
										"staging", LEXICON (
											"type","time",
											"ignition",	TRUE,
											"ullage", "none",
											"ullage_t",	0	
										),
										"Tstage",123,
										"minThrottle",0.65,
										"engines",	LIST(
														LEXICON("thrust", 2173.6*3, "isp", 453, "flow",489.2837*3, "resources",LIST("LqdHydrogen","LqdOxygen")),	//3xrs25a
														LEXICON("thrust", 14234.3*2, "isp", 263, "flow",5518.99*2, "resources",LIST("PBAN"))	//2xssrb
										)					
								),
								
								LEXICON(
										"m_initial",	689.153,
										"m_final",	115.636,
										"staging", LEXICON (
											"type","glim",
											"ignition",	FALSE,
											"ullage", "none",
											"ullage_t",	0	
										),
										"glim",3,
										"minThrottle",0.65,
										"engines",	LIST(
														LEXICON("thrust", 2173.6*3, "isp", 453, "flow",489.2837*3, "Resources",LIST("LqdHydrogen","LqdOxygen"))	//3xrs25a
										)					
								)
								
							),
					"SSME",LEXICON(
							"isp",453,
							"thrust",2173.6,
							"flow",489.2837,
							"active",3
					
						)
).
GLOBAL events IS LIST(
					LEXICON("time",1,"type", "action","action",{TOGGLE AG1.}),	//activates fuel cells
					LEXICON("time",35,"type", "action","action",{ addMessage("THROTTLING DOWN").SET vehicle["stages"][j]["Throttle"] TO 0.75.}),
					LEXICON("time",60,"type", "action","action",{addMessage("THROTTLING UP"). SET vehicle["stages"][j]["Throttle"] TO 1.}),
					LEXICON("time",122,"type", "action","action",{ SET STEERINGMANAGER:MAXSTOPPINGTIME TO 0.2.}),
					LEXICON("time",350,"type", "roll","angle",0)
).

					






