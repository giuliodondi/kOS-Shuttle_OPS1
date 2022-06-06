
GLOBAL vehicle IS LEXICON(
					"name","Space Shuttle - RS25D",
					"stages",LIST(0,		//never remove this zero		
					
								LEXICON(
										"m_initial",	1993.889,
										"m_final",	842.995,
										"Tstage",120.8
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
GLOBAL events IS LIST(
					LEXICON("time",1,"type", "action","action",{TOGGLE AG1.}),	//activates fuel cells
					LEXICON("time",262,"type", "action","action",{LOCAL englist IS SHIP:PARTSDUBBED("ShuttleSSME"). englist[ROUND(RANDOM(),0)]:SHUTDOWN.}),
					LEXICON("time",350,"type", "roll","angle",0)
).

					






