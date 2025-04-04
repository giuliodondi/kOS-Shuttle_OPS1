

//general parameters to control main executive

GLOBAL ops1_parameters is LEXICON(
						"dap_debug", false,
						"debug_mode", false,
						"random_failures", false,
						"control_loop_dt",0.15,	//DO NOT CHANGE
						"launchTimeAdvance", 300,
						"preburn",5.1,
						"roll_v0",35,
						"pitch_v0",38.7,
						"qbucketval", 0.28,
						"yaw_steer_lim", 45,
						"roll_headsup_vi", 3800,	//SET THIS TO >10000 TO INHIBIT
						"low_level_burnt", 10,
						"RTLS_prop_frac", 0.01,
						"OMS_prop_dump_frac", 0.3,
						"ATO_max_underspd", 235,
						"ECAL_dz_lim", 40,
						
						"dummy", 0
).