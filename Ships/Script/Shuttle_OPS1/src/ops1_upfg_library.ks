
//conic state extrapolation function / gravity integrator
FUNCTION cse {
	 PARAMETER r0.
	PARAMETER v0.
	PARAMETER tgo.
	PARAMETER dummy_ IS 0.
		
	LOCAL nstep IS 30.
	LOCAL dT Is tgo/nstep.
	
	LOCAl simstate IS blank_simstate(
		LEXICON(
				"position", r0,
				"velocity", v0
		)
	).
	
	FROM { LOCAL i IS 1. } UNTIL i>nstep STEP { SET i TO i+1. } DO {
		SET simstate TO clone_simstate(coast_rk3(dT, simstate)).
	}
	
	RETURN LISt(
				simstate["position"],
				simstate["velocity"],
				dummy_
	).

}


//global UPFG variables 

// dummy lexicon for gui printout before mode 103
GLOBAL upfgInternal IS LEXICON(
							"tgo", 1,
							"vgo", v(0,0,0),
							"s_meco", FALSE,
							"s_init", FALSE,
							"s_conv", FALSE
).
	

									//	UPFG HANDLING FUNCTIONS






//	empty lexicon
FUNCTION setupUPFG {
	LOCAL init_steervec IS vecYZ(thrust_vec()).
	LOCAL stg IS get_stage().
	local init_throt is stg["Throttle"].
	LOCAL min_throt IS stg["engines"]["minThrottle"].

	SET upfgInternal TO  LEXICON(
		"r_cur", V(0, 0, 0),
		"v_cur", V(0, 0, 0),
		"t_cur", 0,
		"m_cur", 0,
		"tb_cur", 0,
		"s_meco", FALSE,
		"s_init", FALSE,
		"s_conv", FALSE,
		"iter_conv", 0,
		"iter_unconv", 0,
		"itercount", 0,
		"terminal_time", 5,
		"tgo_conv", 1,
		"steer_conv", 20,
		"constraint_release_t", 40,
		"cser", 0,
		"rbias", V(0, 0, 0),
		"rd", V(0, 0, 0),
		"rdmag", 0,
		"vd", V(0, 0, 0),
		"v", V(0, 0, 0),
		"ix", V(0, 0, 0),
		"iy", V(0, 0, 0),
		"iy_tvr", V(0, 0, 0),
		"iz", V(0, 0, 0),
		"rgrav", V(0, 0, 0),
		"time", 0,
		"dt", 0,
		"tgo", 1,
		"vgo", v(0,0,0),
		"vgodmag", 0,
		"lambda", V(1,0,0),
		"lambdadot", V(0,0,0),
		"lambdy", 0,
		"omega", V(0,0,0),
		"steering", init_steervec,
		"s_alt", FALSE,
		"s_plane", FALSE,
		"s_throt", FALSE,
		"throtset", init_throt,
		"rtls_Kkmin", min_throt * 100,			//rtls throttling works in percentages not fractions
		"rtls_throt_release_t", 60,
		"s_flyback",FALSE,
		"mbod",0,
		"mbo_T",0
	).
}	


FUNCTION upfg_normal {
	PARAMETER tgtIncl.
	PARAMETER tgtLAN.

	RETURN vecYZ(targetNormal(tgtIncl, tgtLAN)). 
}

FUNCTION resetUPFG {
	addGUIMessage("RESETTING UPFG").
	setupUPFG().
}


FUNCTION upfg_standard_initialise {
	PARAMETER tgt_orb.
	PARAMETER internal.
	
	LOCAL curT IS surfacestate["time"].
	LOCAL curR IS orbitstate["radius"].
	LOCAL curV IS orbitstate["velocity"].
	
	local cutvec is VXCL(tgt_orb["normal"], curR):NORMALIZED.
	
	//arbitrarily set cutoff point at 10 degrees ahead of the current position.
	set cutvec to rodrigues(cutvec, tgt_orb["normal"], 10).
	
	LOCAL rdmag IS tgt_orb["cutoff alt"]*1000 + SHIP:BODY:RADIUS.
	
	set tgt_orb["radius"] TO cutvec:NORMALIZED * rdmag.	
	
	local rgrav IS -SHIP:ORBIT:BODY:MU * curR / curR:MAG^3.
	
	SET internal["time"] tO curT.
	SET internal["rd"] tO tgt_orb["radius"].
	SET internal["rdmag"] tO rdmag.
	SET internal["v"] tO curV.
	SET internal["rgrav"] tO 0.5 * rgrav.
	
	
	SET internal["s_plane"] TO TRUE.
	SET internal["s_alt"] TO TRUE.
	
	SET internal["ix"] tO tgt_orb["radius"]:NORMALIZED.
	SET internal["iy"] tO -tgt_orb["normal"].
	
	SET internal["vd"] tO cutoff_velocity_vector(
		internal["ix"],
		internal["iy"],
		tgt_orb["velocity"],
		tgt_orb["fpa"]
	).
	
	SET internal["vgo"] tO internal["vd"] - internal["v"].
}

FUNCTION upfg_rtls_initialise {
	PARAMETER tgt_orb.
	PARAMETER internal.
	
	LOCAL curT IS surfacestate["time"].
	LOCAL curR IS orbitstate["radius"].
	LOCAL curV IS orbitstate["velocity"].
	
	SET internal["time"] tO curT.
	SET internal["rd"] tO tgt_orb["radius"].
	SET internal["rdmag"] tO tgt_orb["radius"]:MAG.
	
	SET internal["ix"] tO tgt_orb["radius"]:NORMALIZED.
	SET internal["iy"] tO -tgt_orb["normal"].
	
	LOCAL rgravmag IS (SHIP:ORBIT:BODY:MU / curR:MAG^2).
	//vel gain due to gravity alone until mbo_T
	LOCAL vgor IS rgravmag * internal["mbo_T"] - VDOT(curV, internal["ix"]).
	SET vgor TO MIN(vgor, internal["vgodmag"]).
	
	SET internal["vgo"] TO vgor * internal["ix"] + SQRT(internal["vgodmag"]^2 - vgor^2) * VCRS(internal["ix"], internal["iy"]).
	
	SET internal["s_plane"] TO FALSE.
	SET internal["s_alt"] TO TRUE.
}

FUNCTION upfg_sense_current_state {
	PARAMETER internal.
	
	LOCAL stg IS get_stage().
	
	SET internal["t_cur"] TO surfacestate["time"].
	SET internal["r_cur"] TO orbitstate["radius"].
	SET internal["v_cur"] TO orbitstate["velocity"].
	SET internal["m_cur"] TO stg["m_initial"].
	SET internal["tb_cur"] TO stg["Tstage"].

	IF (target_orbit["mode"] = 5) {
		SET internal["mbod"] tO vehicle["rtls_mbod"].
	}
}


//		UPFG MAIN ROUTINE

FUNCTION upfg {
	PARAMETER vehicle.
	PARAMETER tgt_orb.
	PARAMETER internal.
	
	LOCAL s_mode Is tgt_orb["mode"].
	
	LOCAL g0 IS 9.80665. 
	
	//	measure vehicle parameters
	LOCAL n IS vehicle:LENGTH.
	LOCAL SM IS LIST().
	LOCAL aL IS LIST().
	LOCAL mT IS LIST().
	LOCAL md IS LIST().
	LOCAL ve IS LIST().
	LOCAL fT IS LIST().
	LOCAL aT IS LIST().
	LOCAL tu IS LIST().
	LOCAL tb IS LIST().
	LOCAL kklist IS LIST().
	
	local kk_cmd is internal["throtset"].
	local kk_gl is kk_cmd.
  
	FROM { LOCAL i IS 0. } UNTIL i>=n STEP { SET i TO i+1. } DO {
		SM:ADD(vehicle[i]["mode"]).
		IF vehicle[i]:HASKEY("glim") {
			aL:ADD(vehicle[i]["glim"]*g0).
		} ELSE {
			aL:ADD(0).
		}
		fT:ADD(vehicle[i]["engines"]["thrust"]).
		md:ADD(vehicle[i]["engines"]["flow"]).
		ve:ADD(vehicle[i]["engines"]["isp"]*g0).
		tb:ADD(vehicle[i]["Tstage"]).
		kklist:ADD(vehicle[i]["Throttle"]).
		mT:ADD(vehicle[i]["m_initial"]).

		IF (i=0) {
			SET kklist[i] TO internal["throtset"].	
			SET mT[i] TO internal["m_cur"].	
			SET tb[i] TO internal["tb_cur"].	
			
			if (SM[i]=2) {
				set kk_gl to vehicle[i]["glim"] * g0 * mT[i] / fT[i].
			}
		}
		SET fT[i] TO fT[i]*kklist[i].
		SET md[i] TO md[i]*kklist[i].
		aT:ADD(fT[i]/mT[i]).
		tu:ADD(ve[i]/aT[i]).
	}
	
	//desired vgo for rtls
	IF (s_mode = 5) {
		SET internal["vgodmag"] tO ve[0] * LN(internal["m_cur"] / internal["mbod"]).
		SET internal["mbo_T"] tO (internal["m_cur"] - internal["mbod"]) / md[0].
	}
	
	//initialise or update
	LOCAL dt IS 0.
	IF (internal["s_init"]) {
		SET dt TO internal["t_cur"] - internal["time"].	
		SET internal["time"] TO internal["t_cur"].
		SET internal["dt"] TO dt.
		
		//vgo update subtask
		SET internal["vgo"] TO internal["vgo"] - (internal["v_cur"] - internal["v"]).
		SET internal["v"] TO internal["v_cur"].
	} ELSE {
		IF (s_mode = 5) {
			upfg_rtls_initialise(tgt_orb, internal).
		} eLSE {
			
			upfg_standard_initialise(tgt_orb, internal).
			
			IF (s_mode=7) {
				SET internal["s_plane"] TO FALSE.
			}
			
		}
		
		SET internal["s_init"] tO TRUE.
		
		//modification - reset convergence 
		SET internal["iter_conv"] TO 0.
		SET internal["s_conv"] tO FALSE.
	}
	
	
	//	tgo subtask
	IF SM[0]=1 {
		SET aT[0] TO fT[0] / internal["m_cur"].
	} ELSE IF SM[0]=2 {
		SET aT[0] TO aL[0].
	}
	SET tu[0] TO ve[0] / aT[0].
	
	LOCAL Li IS LIST().
	LOCAL Lsum IS 0.
	FROM { LOCAL i IS 0. } UNTIL i>=(n-1) STEP { SET i TO i+1. } DO {
		
		LOCAL Li_ IS 0.
		
		IF SM[i]=1 {
			SET Li_ TO ve[i] * LN(tu[i]/(tu[i]-tb[i])).
		} ELSE IF SM[i]=2 {
			SET Li_ TO  aL[i]*tb[i].
		} ELSE  {
			SET Li_ TO 0.
		}
		
		IF ((Lsum + Li_) > internal["vgo"]:MAG) {
			SET n to (i + 1).
			BREAK.
		} ELSE {
			Li:ADD(Li_).
			SET Lsum TO Lsum + Li_.
		}
	}
	
	Li:ADD(internal["vgo"]:MAG - Lsum).
	
	
	LOCAL tgoi IS LIST().
	FROM { LOCAL i IS 0. } UNTIL i>=n STEP { SET i TO i+1. } DO {
		IF SM[i]=1 {
			SET tb[i] TO tu[i] * (1-CONSTANT:E^(-Li[i]/ve[i])).
		} ELSE IF SM[i]=2 {
			SET tb[i] TO Li[i] / aL[i].
		}
		IF i=0 {
			tgoi:ADD(tb[i]).
		} ELSE {
			tgoi:ADD(tgoi[i-1] + tb[i]).
		}
	}
	
	LOCAL tgop IS internal["tgo"].
	SET internal["tgo"] TO tgoi[n-1].
	
	//	thrust integrals subtask
	LOCAL L_ IS 0.
	LOCAL J_ IS 0.
	LOCAL S_ IS 0.
	LOCAL Q_ IS 0.
	LOCAL H_ IS 0.
	LOCAL P_ IS 0.
	LOCAL Ji IS LIST().
	LOCAL Si IS LIST().
	LOCAL Qi IS LIST().
	LOCAL Pi IS LIST().
	LOCAL tgoi1 IS 0.
	
	FROM { LOCAL i IS 0. } UNTIL i>=n STEP { SET i TO i+1. } DO {
		IF i>0 {
			SET tgoi1 TO tgoi[i-1].
		}
		IF SM[i]=1 {
			Ji:ADD( tu[i]*Li[i] - ve[i]*tb[i] ).
			Si:ADD( -Ji[i] + tb[i]*Li[i] ).
			Qi:ADD( Si[i]*(tu[i]+tgoi1) - 0.5*ve[i]*tb[i]^2 ).
			Pi:ADD( Qi[i]*(tu[i]+tgoi1) - 0.5*ve[i]*tb[i]^2 * (tb[i]/3+tgoi1) ).
		} ELSE IF SM[i]=2 {
			Ji:ADD( 0.5*Li[i]*tb[i] ).
			Si:ADD( Ji[i] ).
			Qi:ADD( Si[i]*(tb[i]/3+tgoi1) ).
			Pi:ADD( (1/6)*Si[i]*(tgoi[i]^2 + 2*tgoi[i]*tgoi1 + 3*tgoi1^2) ).
		}
		
		SET Ji[i] TO Ji[i] + Li[i]*tgoi1.
		SET Si[i] TO Si[i] + L_*tb[i].
		SET Qi[i] TO Qi[i] + J_*tb[i].
		SET Pi[i] TO Pi[i] + H_*tb[i].
		
		SET L_ TO L_+Li[i].
		SET J_ TO J_+Ji[i].
		SET S_ TO S_+Si[i].
		SET Q_ TO Q_+Qi[i].
		SET P_ TO P_+Pi[i].
		SET H_ TO J_*tgoi[i] - Q_.
	}
	LOCAL JOL IS J_/L_.
	LOCAL Qprime IS Q_ - S_*JOL.
	
	
	//	reference thrust vector subtask
	IF internal["vgo"]:MAG <>0 { SET internal["lambda"] TO internal["vgo"]:NORMALIZED.}
	
	// range-to-go subtask
	
	IF (tgop>0) {
		SET internal["rgrav"] TO (internal["tgo"]/tgop)^2 * internal["rgrav"].
	}
	LOCAL rgo IS internal["rd"] - (internal["r_cur"] + internal["v_cur"]*internal["tgo"] + internal["rgrav"]) + internal["rbias"].
	
	IF (s_mode = 5) {
		//unit normal which defines the xrange component of rgo (because s_plane is always false during rtls)
		SET internal["iy"] TO VCRS(internal["lambda"], internal["ix"]):NORMALIZED.
		SET internal["iy_tvr"] TO internal["iy"].
		
		IF (internal["s_alt"]) {
			SET internal["omega"] TO internal["lambdy"] * VCRS(internal["lambda"], internal["iy"]).
		}
	}
	
	LOCAL rgoprime IS Qprime * VCRS(internal["omega"], internal["lambda"]) + S_ * internal["lambda"].
	
	LOCAL rgox IS 0.
	IF (internal["s_alt"]) {
		SET rgox TO VDOT(internal["ix"], rgo).
	} ELSE {
		SET rgox tO VDOT(internal["ix"], rgoprime).
	}
	
	LOCAL rgoy IS 0.
	IF (internal["s_plane"]) {
		SET rgoy TO VDOT(internal["iy"],rgo).
	} ELSE {
		SET rgoy tO VDOT(internal["iy"],rgoprime).
	}
	
	LOCAL rgoxy IS rgox * internal["ix"] + rgoy * internal["iy"].
	
	SET internal["iz"] TO VCRS(internal["ix"], internal["iy"]).
	LOCAL rgoz IS (S_ - VDOT(internal["lambda"], rgoxy)) / VDOT(internal["lambda"], internal["iz"]).
	SET rgo TO rgoxy + rgoz * internal["iz"].
	
	//turning rate vector subtask
	
	SET internal["lambdadot"] TO (rgo - S_ * internal["lambda"]) / Qprime.
	
	//steering inputs update subtask
	LOCAL steering_prev IS internal["steering"].
	SET internal["steering"] TO  internal["lambda"] - internal["lambdadot"]*JOL.
	
	//burnout state vector prediction + thrust time integrals
	LOCAL phi IS VANG(internal["steering"], internal["lambda"]) * CONSTANT:DEGTORAD.
	LOCAL phidot IS - phi / JOL.
	LOCAL vthrust IS (L_ - 0.5 * L_ * phi^2 - J_ * phi * phidot - 0.5 * H_ * phidot^2).
	SET vthrust TO vthrust * internal["lambda"] - (L_ * phi + J_ * phidot) * internal["lambdadot"]:NORMALIZED.
	LOCAL rthrust IS S_ - 0.5 * S_ * phi^2 - Q_ * phi * phidot - 0.5 * P_ * phidot^2.
	SET rthrust TO rthrust * internal["lambda"] - (S_ * phi + Q_ * phidot) * internal["lambdadot"]:NORMALIZED.
	SET internal["rbias"] TO rgo - rthrust.
	
	
	LOCAL rc1 IS internal["r_cur"] - 0.1 * rthrust - (internal["tgo"] / 30) * vthrust.
	LOCAL vc1 IS internal["v_cur"] + 1.2 * rthrust / internal["tgo"] - 0.1 * vthrust.
	LOCAL pack IS cse(rc1, vc1, internal["tgo"], internal["cser"]).
	SET internal["cser"] TO pack[2].
	SET internal["rgrav"] TO pack[0] - rc1 - vc1 * internal["tgo"].
	LOCAL vgrav IS pack[1] - vc1.
	

	LOCAL rp IS internal["r_cur"] + internal["v_cur"]*internal["tgo"] + internal["rgrav"] + rthrust.
	LOCAL vp IS internal["v_cur"] + vgrav + vthrust.
	
	//desired orbit plane correction subtask
	IF (internal["s_plane"]) {
		SET internal["rd"] TO VXCL(internal["iy"],rp).	
	} ELSE {
		//always land in this for rtls
		set internal["rd"] to rp.
	}
	
	//desired position subtask
	SET internal["ix"] TO internal["rd"]:NORMALIZED.
	
	IF (internal["s_alt"]) {
		SET internal["rd"] TO tgt_orb["radius"]:MAG * internal["ix"].	
	}
	
	//corrector portion
	LOCAL dvgo IS 0.
	IF (s_mode=5) {
		//rtls desired plane
		LOCAL rt_meco IS RTLS_shifted_tgt_site_vector(internal["tgo"]).
		SET tgt_orb["rtls_tgt"] TO rt_meco.
		
		//plane from cutoff to target
		SET internal["iy"] TO VCRS(rt_meco, internal["rd"]):NORMALIZED.
		
		IF (internal["s_alt"]) {
			//the downrange component of the final LTG steering vector
			LOCAL lambda_x_T IS VDOT(internal["lambda"] + (internal["tgo"] - JOL) * internal["lambdadot"], internal["iz"]).
			//calculate out-of-plane turn rate vector to zero sideslip at ppd (or when we release the cutoff constraint?)
			LOCAL lambdyJOL IS  lambda_x_T * TAN(-SIGN(VDOT(internal["lambda"], internal["iy"])) * ARCCOS(VDOT(internal["iy_tvr"], internal["iy"]))).
			//my modification, replaced t_release_constraint with terminal_t
			SET internal["lambdy"] TO (internal["terminal_time"] / internal["tgo"]) * (lambdyJOL / (internal["tgo"] - JOL)).
		}
		
		//distance to site IN METRES
		LOCAL rtheta IS  greatcircledist(rt_meco, internal["rd"]) * 1000.
		SET tgt_orb["range"] TO rtheta.
		
		SET internal["iz"] TO VCRS(internal["ix"], internal["iy"]):NORMALIZED.
		
		//desired vel 
		LOCAL rvline_coefs IS RTLS_rvline_coefs().
		local rvline_vd IS rvline_coefs[0] * rtheta + rvline_coefs[1].
		SET internal["vd"] TO rodrigues(internal["iz"], internal["iy"], tgt_orb["fpa"]):NORMALIZED * rvline_vd.
		SET internal["vd"] TO internal["vd"] + VCRS((constant:pi/43200) * v(0,0,1), internal["rd"]).
		
		IF (internal["s_flyback"]) {
			//for display, this value is unreliable before flyback
			SET tgt_orb["rtls_cutv"] TO rvline_vd.
		}
		SET tgt_orb["velocity"] TO internal["vd"]:MAG.
		
		//plane of final radius and desired velocity 
		SET internal["iy"] TO VCRS(internal["vd"], internal["rd"]):NORMALIZED.
		SET internal["iz"] TO VCRS(internal["ix"], internal["iy"]):NORMALIZED.
		SET tgt_orb["normal"] TO -internal["iy"].
		
		//rho matrix subtask
		LOCAL vmiss IS vp - internal["vd"].
		
		LOCAL vmiss_g1 IS VDOT(vmiss, internal["ix"]).
		LOCAL vmiss_g2 IS VDOT(vmiss, internal["iy"]).
		LOCAL vmiss_g3 IS VDOT(vmiss, internal["iz"]).
		
		LOCAL lam_g1 IS VDOT(internal["lambda"], internal["ix"]).
		LOCAL lam_g2 IS VDOT(internal["lambda"], internal["iy"]).
		LOCAL lam_g3 IS VDOT(internal["lambda"], internal["iz"]).
		
		//final acceleration (not limited)
		LOCAL atrf IS aT[0] * CONSTANT:E^(internal["vgodmag"] / ve[0]).
		
		LOCAL vgravx IS VDOT(vgrav, internal["ix"]).
		
		LOCAL KkP IS kk_cmd * 100.
		
		LOCAL beta1 IS 1 + rvline_coefs[0] * (0.5*internal["tgo"] + VDOT((internal["v_cur"] + 0.5*internal["vgo"]), internal["iz"]) * lam_g3 / atrf).
		LOCAL beta2 IS 1 + vgravx * lam_g1 / (atrf * internal["tgo"]).
		LOCAL beta3 IS 1 + internal["vgodmag"] * internal["tgo"] / (2 * rtheta).
		LOCAL beta4 IS KkP / (vgravx * lam_g1).
		
		LOCAL rho1 IS V(0, -lam_g2/beta3, -lam_g3/beta1) / lam_g1.
		LOCAL p_rho IS V(lam_g1, lam_g2*beta2/beta3, lam_g3/beta1) * beta4.
		
		LOCAL vmiss_g IS V(vmiss_g1, vmiss_g2, vmiss_g3).
		LOCAL beta5 IS VDOT(p_rho, vmiss_g).
		
		LOCAL vgo_err IS 0.
		LOCAL delKk IS 0.
		IF (internal["s_flyback"] AND internal["s_throt"]) {
			SET vgo_err TO internal["vgo"]:MAG - internal["vgodmag"].
			SET delKk TO beta5 - beta4 * beta2 * vgo_err.
			
			SET delKk TO SIGN(delKk) * MIN(2, ABS(delKk)).
			
			//print round(delKk,2) + "   "  at (0,20).
			
			LOCAL Kknew IS  CLAMP(KkP + delKk, internal["rtls_Kkmin"], 100).	//should cover all throttle limits for all ssme variants, actual min throttle is calculated outside
			SET delKk TO (Kknew - KkP).
			
			//print round(Kknew,2) + "   "  at (0,21).
			
			SET kk_cmd TO CLAMP(Kknew / 100, 0, 1).
		}
		
		SET vgo_err TO (beta5 - delKk) / (beta4 * beta2).
		
		SET dvgo TO -((VDOT(rho1, vmiss_g) + vgo_err/lam_g1)*internal["ix"] + (vmiss_g2/beta3)*internal["iy"] + (vmiss_g3/beta1)*internal["iz"]).
	
	} ELSE {
		//desired velocity
	
		IF (s_mode=6) {
			SET tgt_orb TO TAL_cutoff_params(tgt_orb, internal["r_cur"], internal["rd"]).
		} ELSE {
			SET tgt_orb TO nominal_cutoff_params(tgt_orb, internal["rd"]).
		}
		
		IF (internal["s_plane"]) {
			SET internal["iy"] TO -tgt_orb["normal"].
		} ELSE {
			SET internal["iy"] TO VCRS(internal["vd"], internal["rd"]):NORMALIZED.
			SET tgt_orb["normal"] TO -internal["iy"].
		}
		
		SET internal["iz"] TO VCRS(internal["ix"], internal["iy"]):NORMALIZED.
		
		SET internal["vd"] TO cutoff_velocity_vector(
			internal["ix"],
			internal["iy"],
			tgt_orb["velocity"],
			tgt_orb["fpa"]
		).
	
		SET dvgo TO (internal["vd"] - vp).
	}
	

	//vgo correction subtask
	SET internal["vgo"] TO internal["vgo"] + dvgo.
	
	//my addition: throttle controller for constant g in any case 
	if (SM[0]=2) {
		SET kk_cmd TO CLAMP(kk_gl, 0, 1).
	}
	
	//convergence check subtask 
	LOCAL wasconv IS internal["s_conv"].
	LOCAL tgo_expected IS tgop - dt.
	
	IF (NOT internal["s_conv"]) {
		SET internal["itercount"] TO internal["itercount"]+1.
	}
		
	IF (ABS(tgo_expected - internal["tgo"]) < internal["tgo_conv"]) { //first criterion for convergence
		IF (VANG(steering_prev, internal["steering"]) < internal["steer_conv"]) { //second criterion for convergence
			SET internal["iter_unconv"] TO 0.
			IF (internal["iter_conv"] < 3) {
				SET internal["iter_conv"] TO internal["iter_conv"] + 1.
			} ELSE {
				if (NOT internal["s_conv"]) {
					//moved here from main executive
					addGUIMessage("GUIDANCE CONVERGED IN " + internal["itercount"] + " ITERATIONS").
					SET internal["s_conv"] tO TRUE.
				}
			}
		} ELSE { //something is wrong, reset
			resetUPFG().
			RETURN.
		}
	} ELSE {
		//if we were converged and twice in a row we break the convergence criterion, go unconverged
		IF (wasconv AND internal["iter_unconv"] < 2) {
			SET internal["iter_unconv"] TO internal["iter_unconv"] + 1.
		} ELSE {
			SET internal["iter_conv"] TO 0.
			SET internal["s_conv"] tO FALSE.
		}
	}	

	IF wasconv AND (NOT internal["s_conv"]) {
		//in this case we had convergence and we lost it, reset itercount
		SET internal["itercount"] TO 0.
	}
	
	//rtls throttling release 
	if (s_mode = 5) {
		IF (internal["s_flyback"]) AND (internal["tgo"] < internal["rtls_throt_release_t"]) {
			SET internal["s_throt"] TO FALSE.
		}
	}
	
	//constraint release
	If (internal["s_conv"] AND internal["tgo"] < internal["constraint_release_t"]) OR 
		((NOT internal["s_init"]) AND (internal["tgo"] < (internal["constraint_release_t"] - 10))) {
		//release cutoff position
		IF (internal["s_alt"] OR internal["s_plane"]) {
			SET internal["omega"] TO VCRS(internal["lambda"], internal["lambdadot"]).
			SET internal["s_plane"] TO FALSE.
			SET internal["s_alt"] TO FALSE.
		}
	}
	
	//throttle command 
	SET internal["throtset"] TO CLAMP(kk_cmd, 0, 1).
	
	//terminal count 	
	local tgt_v_close_flag is (internal["v_cur"]:MAG >= 0.985*tgt_orb["velocity"]).
	local tgo_terminal_flag IS (internal["tgo"] <= internal["terminal_time"]).
	
	local unguided_meco_flag is (NOT internal["s_conv"]) AND tgt_v_close_flag.
	local guided_meco_flag is internal["s_conv"] AND tgo_terminal_flag.
	
	if (s_mode = 5) {
		set guided_meco_flag to internal["s_flyback"] AND guided_meco_flag.
	    set unguided_meco_flag to internal["s_flyback"] AND unguided_meco_flag. 
	}

	set internal["s_meco"] TO (guided_meco_flag OR unguided_meco_flag).
		
}












//my own implementation of rtls, for posterity
FUNCTION upfg_own {
	PARAMETER vehicle.
	PARAMETER tgt_orb.
	PARAMETER internal.
	
	LOCAL s_mode Is tgt_orb["mode"].
	
	LOCAL g0 IS 9.80665. 
	
	//	measure vehicle parameters
	LOCAL n IS vehicle:LENGTH.
	LOCAL SM IS LIST().
	LOCAL aL IS LIST().
	LOCAL mT IS LIST().
	LOCAL md IS LIST().
	LOCAL ve IS LIST().
	LOCAL fT IS LIST().
	LOCAL aT IS LIST().
	LOCAL tu IS LIST().
	LOCAL tb IS LIST().
	LOCAL kklist IS LIST().
  
	FROM { LOCAL i IS 0. } UNTIL i>=n STEP { SET i TO i+1. } DO {
		SM:ADD(vehicle[i]["mode"]).
		IF vehicle[i]:HASKEY("glim") {
			aL:ADD(vehicle[i]["glim"]*g0).
		} ELSE {
			aL:ADD(0).
		}
		fT:ADD(vehicle[i]["engines"]["thrust"]).
		md:ADD(vehicle[i]["engines"]["flow"]).
		ve:ADD(vehicle[i]["engines"]["isp"]*g0).
		tb:ADD(vehicle[i]["Tstage"]).
		kklist:ADD(vehicle[i]["Throttle"]).
		mT:ADD(vehicle[i]["m_initial"]).
		
		//for RTLS overwrite throttle for all stages
		IF (i=0) OR (s_mode = 5) {
			SET kklist[i] TO internal["throtset"].	
		}
		
		IF (i=0) {
			SET mT[i] TO internal["m_cur"].	
			SET tb[i] TO internal["tb_cur"].	
		}
		SET fT[i] TO fT[i]*kklist[i].
		SET md[i] TO md[i]*kklist[i].
		aT:ADD(fT[i]/mT[i]).
		tu:ADD(ve[i]/aT[i]).
	}
	
	//desired vgo for rtls
	IF (s_mode = 5) {
		SET internal["vgodmag"] tO ve[0] * LN(internal["m_cur"] / internal["mbod"]).
		SET internal["mbo_T"] tO (internal["m_cur"] - internal["mbod"]) / md[0].
	}
	
	//initialise or update
	LOCAL dt IS 0.
	IF (internal["s_init"]) {
		SET dt TO internal["t_cur"] - internal["time"].	
		SET internal["time"] TO internal["t_cur"].
		SET internal["dt"] TO dt.
		
		//vgo update subtask
		SET internal["vgo"] TO internal["vgo"] - (internal["v_cur"] - internal["v"]).
		SET internal["v"] TO internal["v_cur"].
	} ELSE {
		IF (s_mode = 5) {
			upfg_rtls_initialise(tgt_orb, internal).
		} eLSE {
			
			upfg_standard_initialise(tgt_orb, internal).
			
			IF (s_mode=7) {
				SET internal["s_plane"] TO FALSE.
			}
			
		}
		
		SET internal["s_init"] tO TRUE.
	}
	
	
	//	tgo subtask
	IF SM[0]=1 {
		SET aT[0] TO fT[0] / internal["m_cur"].
	} ELSE IF SM[0]=2 {
		SET aT[0] TO aL[0].
	}
	SET tu[0] TO ve[0] / aT[0].
	
	LOCAL Li IS LIST().
	LOCAL Lsum IS 0.
	FROM { LOCAL i IS 0. } UNTIL i>=(n-1) STEP { SET i TO i+1. } DO {
		
		LOCAL Li_ IS 0.
		
		IF SM[i]=1 {
			SET Li_ TO ve[i] * LN(tu[i]/(tu[i]-tb[i])).
		} ELSE IF SM[i]=2 {
			SET Li_ TO  aL[i]*tb[i].
		} ELSE  {
			SET Li_ TO 0.
		}
		
		IF ((Lsum + Li_) > internal["vgo"]:MAG) {
			SET n to (i + 1).
			BREAK.
		} ELSE {
			Li:ADD(Li_).
			SET Lsum TO Lsum + Li_.
		}
	}
	
	Li:ADD(internal["vgo"]:MAG - Lsum).
	
	
	LOCAL tgoi IS LIST().
	FROM { LOCAL i IS 0. } UNTIL i>=n STEP { SET i TO i+1. } DO {
		IF SM[i]=1 {
			SET tb[i] TO tu[i] * (1-CONSTANT:E^(-Li[i]/ve[i])).
		} ELSE IF SM[i]=2 {
			SET tb[i] TO Li[i] / aL[i].
		}
		IF i=0 {
			tgoi:ADD(tb[i]).
		} ELSE {
			tgoi:ADD(tgoi[i-1] + tb[i]).
		}
	}
	
	LOCAL tgop IS internal["tgo"].
	SET internal["tgo"] TO tgoi[n-1].
	
	//	thrust integrals subtask
	LOCAL L_ IS 0.
	LOCAL J_ IS 0.
	LOCAL S_ IS 0.
	LOCAL Q_ IS 0.
	LOCAL H_ IS 0.
	LOCAL P_ IS 0.
	LOCAL Ji IS LIST().
	LOCAL Si IS LIST().
	LOCAL Qi IS LIST().
	LOCAL Pi IS LIST().
	LOCAL tgoi1 IS 0.
	
	FROM { LOCAL i IS 0. } UNTIL i>=n STEP { SET i TO i+1. } DO {
		IF i>0 {
			SET tgoi1 TO tgoi[i-1].
		}
		IF SM[i]=1 {
			Ji:ADD( tu[i]*Li[i] - ve[i]*tb[i] ).
			Si:ADD( -Ji[i] + tb[i]*Li[i] ).
			Qi:ADD( Si[i]*(tu[i]+tgoi1) - 0.5*ve[i]*tb[i]^2 ).
			Pi:ADD( Qi[i]*(tu[i]+tgoi1) - 0.5*ve[i]*tb[i]^2 * (tb[i]/3+tgoi1) ).
		} ELSE IF SM[i]=2 {
			Ji:ADD( 0.5*Li[i]*tb[i] ).
			Si:ADD( Ji[i] ).
			Qi:ADD( Si[i]*(tb[i]/3+tgoi1) ).
			Pi:ADD( (1/6)*Si[i]*(tgoi[i]^2 + 2*tgoi[i]*tgoi1 + 3*tgoi1^2) ).
		}
		
		SET Ji[i] TO Ji[i] + Li[i]*tgoi1.
		SET Si[i] TO Si[i] + L_*tb[i].
		SET Qi[i] TO Qi[i] + J_*tb[i].
		SET Pi[i] TO Pi[i] + H_*tb[i].
		
		SET L_ TO L_+Li[i].
		SET J_ TO J_+Ji[i].
		SET S_ TO S_+Si[i].
		SET Q_ TO Q_+Qi[i].
		SET P_ TO P_+Pi[i].
		SET H_ TO J_*tgoi[i] - Q_.
	}
	LOCAL JOL IS J_/L_.
	LOCAL Qprime IS Q_ - S_*JOL.
	
	
	//	reference thrust vector subtask
	IF internal["vgo"]:MAG <>0 { SET internal["lambda"] TO internal["vgo"]:NORMALIZED.}
	
	// range-to-go subtask
	
	IF (tgop>0) {
		SET internal["rgrav"] TO (internal["tgo"]/tgop)^2 * internal["rgrav"].
	}
	LOCAL rgo IS internal["rd"] - (internal["r_cur"] + internal["v_cur"]*internal["tgo"] + internal["rgrav"]) + internal["rbias"].
	
	LOCAL rgoprime IS Qprime * VCRS(internal["omega"], internal["lambda"]) + S_ * internal["lambda"].
	
	LOCAL rgox IS 0.
	IF (internal["s_alt"]) {
		SET rgox TO VDOT(internal["ix"], rgo).
	} ELSE {
		SET rgox tO VDOT(internal["ix"], rgoprime).
	}
	
	LOCAL rgoy IS 0.
	IF (internal["s_plane"]) {
		SET rgoy TO VDOT(internal["iy"],rgo).
	} ELSE {
		SET rgoy tO VDOT(internal["iy"],rgoprime).
	}
	
	LOCAL rgoxy IS rgox * internal["ix"] + rgoy * internal["iy"].
	
	SET internal["iz"] TO VCRS(internal["ix"], internal["iy"]).
	LOCAL rgoz IS (S_ - VDOT(internal["lambda"], rgoxy)) / VDOT(internal["lambda"], internal["iz"]).
	SET rgo TO rgoxy + rgoz * internal["iz"].
	
	//turning rate vector subtask
	
	SET internal["lambdadot"] TO (rgo - S_ * internal["lambda"]) / Qprime.
	
	//steering inputs update subtask
	LOCAL steering_prev IS internal["steering"].
	SET internal["steering"] TO  internal["lambda"] - internal["lambdadot"]*JOL.
	
	//burnout state vector prediction + thrust time integrals
	LOCAL phi IS VANG(internal["steering"], internal["lambda"]) * CONSTANT:DEGTORAD.
	LOCAL phidot IS - phi / JOL.
	LOCAL vthrust IS (L_ - 0.5 * L_ * phi^2 - J_ * phi * phidot - 0.5 * H_ * phidot^2).
	SET vthrust TO vthrust * internal["lambda"] - (L_ * phi + J_ * phidot) * internal["lambdadot"]:NORMALIZED.
	LOCAL rthrust IS S_ - 0.5 * S_ * phi^2 - Q_ * phi * phidot - 0.5 * P_ * phidot^2.
	SET rthrust TO rthrust * internal["lambda"] - (S_ * phi + Q_ * phidot) * internal["lambdadot"]:NORMALIZED.
	SET internal["rbias"] TO rgo - rthrust.
	
	
	LOCAL rc1 IS internal["r_cur"] - 0.1 * rthrust - (internal["tgo"] / 30) * vthrust.
	LOCAL vc1 IS internal["v_cur"] + 1.2 * rthrust / internal["tgo"] - 0.1 * vthrust.
	LOCAL pack IS cse(rc1, vc1, internal["tgo"], internal["cser"]).
	SET internal["cser"] TO pack[2].
	SET internal["rgrav"] TO pack[0] - rc1 - vc1 * internal["tgo"].
	LOCAL vgrav IS pack[1] - vc1.
	

	LOCAL rp IS internal["r_cur"] + internal["v_cur"]*internal["tgo"] + internal["rgrav"] + rthrust.
	LOCAL vp IS internal["v_cur"] + vgrav + vthrust.
	
	//desired orbit plane correction subtask
	IF (internal["s_plane"]) {
		SET internal["rd"] TO VXCL(internal["iy"],rp).	
	} ELSE {
		//always land in this for rtls
		set internal["rd"] to rp.
	}
	
	//desired position subtask
	SET internal["ix"] TO internal["rd"]:NORMALIZED.
	
	IF (internal["s_alt"]) {
		SET internal["rd"] TO tgt_orb["radius"]:MAG * internal["ix"].	
	}
	
	//corrector portion
	LOCAL dvgo IS 0.
	IF (s_mode=5) {
		//rtls desired plane
		LOCAL rt_meco IS RTLS_shifted_tgt_site_vector(internal["tgo"]).
		SET tgt_orb["rtls_tgt"] TO rt_meco.
		
		//plane from cutoff to target
		SET internal["iy"] TO VCRS(rt_meco, internal["rd"]):NORMALIZED.
		
		//distance to site IN METRES
		LOCAL rtheta IS  greatcircledist(rt_meco, internal["rd"]) * 1000.
		SET tgt_orb["range"] TO rtheta.
		
		SET internal["iz"] TO VCRS(internal["ix"], internal["iy"]):NORMALIZED.
		
		//desired vel 
		LOCAL rvline_coefs IS RTLS_rvline_coefs().
		local rvline_vd IS rvline_coefs[0] * rtheta + rvline_coefs[1].
		SET internal["vd"] TO rodrigues(internal["iz"], internal["iy"], tgt_orb["fpa"]):NORMALIZED * rvline_vd.
		SET internal["vd"] TO internal["vd"] + VCRS((constant:pi/43200) * v(0,0,1), internal["rd"]).
		
		IF (internal["s_flyback"]) {
			//for display, this value is unreliable before flyback
			SET tgt_orb["rtls_cutv"] TO rvline_vd.
		}
		SET tgt_orb["velocity"] TO internal["vd"]:MAG.
		
		//plane of final radius and desired velocity 
		SET internal["iy"] TO VCRS(internal["vd"], internal["rd"]):NORMALIZED.
		SET internal["iz"] TO VCRS(internal["ix"], internal["iy"]):NORMALIZED.
		SET tgt_orb["normal"] TO -internal["iy"].
		
		LOCAL KkP IS internal["throtset"] * 100.
		
		//throttling
		LOCAL delKk IS 0.
		IF (internal["s_flyback"] AND internal["s_throt"]) {
			LOCAL Tc IS internal["mbo_T"] - internal["tgo"].
			LOCAL throtgain IS -internal["dt"] * 0.25.
			SET delKk TO throtgain * Tc.
			
			print round(Tc,2) + " " at (0,20).
			print round(delKk,2) + " " at (0,21).
			
			LOCAL Kknew IS  CLAMP(KkP + delKk, internal["rtls_Kkmin"], 100).	//should cover all throttle limits for all ssme variants, actual min throttle is calculated outside
			SET delKk TO (Kknew - KkP).
			
			print round(Kknew,2) + " " at (0,22).
			
			SET internal["throtset"] TO CLAMP(Kknew/100, 0, 1).
		}
		
		SET dvgo TO (internal["vd"] - vp).
		
	} ELSE {
		//desired velocity
	
		IF (s_mode=6) {
			SET tgt_orb TO TAL_cutoff_params(tgt_orb, internal["rd"]).
		} ELSE {
			SET tgt_orb TO nominal_cutoff_params(tgt_orb, internal["rd"]).
		}
		
		IF (internal["s_plane"]) {
			SET internal["iy"] TO -tgt_orb["normal"].
		} ELSE {
			SET internal["iy"] TO VCRS(internal["vd"], internal["rd"]):NORMALIZED.
			SET tgt_orb["normal"] TO -internal["iy"].
		}
		
		SET internal["iz"] TO VCRS(internal["ix"], internal["iy"]):NORMALIZED.
		
		SET internal["vd"] TO rodrigues(internal["iz"], internal["iy"], tgt_orb["fpa"]):NORMALIZED * tgt_orb["velocity"].	
	
		SET dvgo TO (internal["vd"] - vp).
	}

	//vgo correction subtask
	SET internal["vgo"] TO internal["vgo"] + dvgo.
	
	//convergence check subtask 
	LOCAL wasconv IS internal["s_conv"].
	LOCAL tgo_expected IS tgop - dt.
	
	IF (NOT internal["s_conv"]) {
		SET internal["itercount"] TO internal["itercount"]+1.
	}
		
	IF (ABS(tgo_expected - internal["tgo"]) < internal["tgo_conv"]) { //first criterion for convergence
		IF (VANG(steering_prev, internal["steering"]) < internal["steer_conv"]) { //second criterion for convergence
			SET internal["iter_unconv"] TO 0.
			IF (internal["iter_conv"] < 3) {
				SET internal["iter_conv"] TO internal["iter_conv"] + 1.
			} ELSE {
				SET internal["s_conv"] tO TRUE.
			}
		} ELSE { //something is wrong, reset
			resetUPFG().
			RETURN.
		}
	} ELSE {
		//if we were converged and twice in a row we break the convergence criterion, go unconverged
		IF (wasconv AND internal["iter_unconv"] < 2) {
			SET internal["iter_unconv"] TO internal["iter_unconv"] + 1.
		} ELSE {
			SET internal["iter_conv"] TO 0.
			SET internal["s_conv"] tO FALSE.
		}
	}	

	IF wasconv AND (NOT internal["s_conv"]) {
		//in this case we had convergence and we lost it, reset itercount
		SET internal["itercount"] TO 0.
	}
	
	//rtls throttling release 
	if (s_mode = 5) {
		IF (internal["s_flyback"]) AND (internal["tgo"] < internal["rtls_throt_release_t"]) {
			SET internal["s_throt"] TO FALSE.
		}
	}
	
	//constraint release
	If (internal["s_conv"] AND internal["tgo"] < internal["constraint_release_t"]) OR 
		((NOT internal["s_init"]) AND (internal["tgo"] < (internal["constraint_release_t"] - 10))) {
		//release cutoff position
		IF (internal["s_alt"] OR internal["s_plane"]) {
			SET internal["omega"] TO VCRS(internal["lambda"], internal["lambdadot"]).
			SET internal["s_plane"] TO FALSE.
			SET internal["s_alt"] TO FALSE.
		}
	}
}