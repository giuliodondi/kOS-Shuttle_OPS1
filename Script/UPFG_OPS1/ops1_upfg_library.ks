
//conic state extrapolation function / gravity integrator
//RUNPATH("0:/UPFG_OPS1/upfg__cser_new").
RUNPATH("0:/UPFG_OPS1/upfg__cser_sg_simple").


//global UPFG variables 

GLOBAL upfgFinalizationTime IS 5.		//	When time-to-go gets below that, keep attitude stable and simply count down time to cutoff.
GLOBAL upfgConvergenceTgo IS 1.	//	Maximum difference between consecutive UPFG T-go predictions that allow accepting the solution.
GLOBAL upfgConvergenceVec IS 15.	//	Maximum angle between guidance vectors calculated by UPFG between stages that allow accepting the solution.
	
	

									//	UPFG HANDLING FUNCTIONS






//	Creates and initializes UPFG internal struct
FUNCTION setupUPFG {
	parameter target.
	
	LOCAL curR IS orbitstate["radius"].
	LOCAL curV IS orbitstate["velocity"].

	//SET target_orbit["normal"] TO targetNormal(target_orbit["inclination"], target_orbit["LAN"]).

	LOCAL tgoV IS v(0,0,0).

	SET tgoV TO VCRS(-target_orbit["normal"], target_orbit["radius"]):NORMALIZED.	
	SET tgoV TO rodrigues(tgoV,target_orbit["normal"], target_orbit["angle"]).	
	SET tgoV TO target_orbit["velocity"]* tgoV - curV.


	
	local rgrav IS -SHIP:ORBIT:BODY:MU/2 * curR / curR:MAG^3.
	
	local stg is get_stage().
	
	local LEX1 IS LEXICON(
		"cser", 4,
		"rbias", V(0, 0, 0),
		"rd", target_orbit["radius"],
		"rgrav", rgrav,
		"vgrav", 2*rgrav,
		"time", surfacestate["MET"],
		"tgo", 100,
		"v", curV,
		"vgo", tgoV,
		"lambda", V(1,0,0),
		"lambdadot", V(0,0,0),
		"t_lambda",surfacestate["MET"],
		"steering",V(1,0,0),
		"throtset",stg["Throttle"],
		"flyback_flag",FALSE,
		"mbod",0,
		"dmbo",0,
		"Tc",0
	).
	local LEX2 IS LEXICON(
		"iter",-2,
		"conv",-2,
		"itercount",0,
		"lastvec",V(1,0,0),
		"lastiter",surfacestate["MET"],
		"lastthrot",stg["Throttle"],
		"terminal",FALSE
	).
	

	RETURN LIST(LEX1,LEX2).
}	

FUNCTION upfg_framerot {
	DECLARE PARAMETER upfg_in.
	
	LOCAL newref IS SOLARPRIMEVECTOR.
	
	LOCAL rota is signed_angle(usc["lastref"],SOLARPRIMEVECTOR,-v(0,1,0),1).
	
	SET upfg_in["rd"] TO rodrigues(upfg_in["rd"],V(0,0,1),rota).
	SET upfg_in["rgrav"] TO rodrigues(upfg_in["rgrav"],V(0,0,1),rota).
	SET upfg_in["v"] TO rodrigues(upfg_in["v"],V(0,0,1),rota).
	SET upfg_in["vgo"] TO rodrigues(upfg_in["vgo"],V(0,0,1),rota).
	SET upfg_in["lambda"] TO rodrigues(upfg_in["lambda"],V(0,0,1),rota).
	SET upfg_in["lambdadot"] TO rodrigues(upfg_in["lambdadot"],V(0,0,1),rota).

	SET usc["lastref"] TO newref.
	
	RETURN upfg_in.

}

FUNCTION upfg_wrapper {

	DECLARE PARAMETER upfgInternal.
	

	LOCAL currentIterationTime IS surfacestate["MET"].
	
	//clearvecdraws().
	//arrow(vecyz(upfgOutput["steering"]),"iF1",v(0,0,0),50,0.05)
	
	LOCAL wasconv IS usc["conv"]=1.
	
	SET upfgInternal["throtset"] TO usc["lastthrot"].
	
	IF (DEFINED RTLSAbort) {
		SET upfgInternal["mbod"] TO vehicle["mbod"].
	}
	
	
	LOCAL out IS upfg(
		currentIterationTime,
		vehicle["stages"]:SUBLIST(vehiclestate["cur_stg"],vehicle:LENGTH-vehiclestate["cur_stg"]),
		target_orbit,
		upfgInternal
	).
	
	
	
	LOCAL upfgOutput IS out[0].
	SET target_orbit TO out[1].
	
	
	IF NOT usc["terminal"] {
		IF usc["conv"]<1 {SET usc["itercount"] TO usc["itercount"]+1.}
		
		LOCAL iterationDeltaTime IS ABS(currentIterationTime - usc["lastiter"]).
		IF vehiclestate["staging_in_progress"] {
			SET iterationDeltaTime TO 0.
			SET upfgOutput["time"] TO upfgInternal["time"].
		}

	
		SET usc["lastiter"] TO currentIterationTime.
		LOCAL expectedTgo IS upfgInternal["tgo"]- iterationDeltaTime.
			
		IF ABS(expectedTgo-upfgOutput["tgo"]) < upfgConvergenceTgo { //first criterion for convergence
			IF VANG(upfgOutput["steering"], upfgInternal["steering"]) < upfgConvergenceVec { //second criterion for convergence
				IF usc["conv"]<1 { //score one good hit, increment until conv is 1
					SET usc["conv"] TO usc["conv"]+1.
				}
			} ELSE { //something is wrong, reset
				IF NOT vehiclestate["staging_in_progress"] {
					SET upfgOutput TO resetUPFG(upfgOutput).
		    	}
		    }
		} ELSE {SET usc["conv"] TO usc["iter"].}	

		IF wasconv AND usc["conv"]<1 {
			//in this case we had convergence and we lost it, reset itercount
			SET usc["itercount"] TO 0.
		}
	}
	
	
	IF (DEFINED RTLSAbort) {
		//clearvecdraws().
		//arrow(vecyz(upfgOutput["steering"]),"iF2",v(0,0,0),50,0.05).
		//arrow(vecYZ(target_orbit["normal"]:NORMALIZED),"norm",v(0,0,0),50,0.05).
		
		//dissipation and flyback trigger logic
		IF (NOT (RTLSAbort["flyback_flag"] AND RTLSAbort["pitcharound"]["complete"] )) {
			IF ( NOT RTLSAbort["pitcharound"]["triggered"] ) {
			
				//dissipation thrust vector
				//SET RTLSAbort["C1"] TO  VXCL(target_orbit["normal"],RTLSAbort["C1"]).
				SET usc["lastvec"] TO RTLSAbort["C1"].
				
				SET RTLSAbort["pitcharound"]["refvec"] TO VCRS( RTLSAbort["C1"], vecYZ(-SHIP:ORBIT:BODY:POSITION:NORMALIZED)).
				
				SET RTLSAbort["pitcharound"]["target"] TO VXCL(RTLSAbort["pitcharound"]["refvec"],upfgOutput["steering"]).
				
				IF (upfgOutput["Tc"] > RTLSAbort["Tc"]) {
					SET RTLSAbort["flyback_conv"] TO RTLSAbort["flyback_iter"].
				} ELSE {
					SET RTLSAbort["flyback_conv"] TO MIN( 1, RTLSAbort["flyback_conv"] + 1).
				}
				
				SET control["refvec"] TO SHIP:ORBIT:BODY:POSITION:NORMALIZED.
				
				IF (upfgOutput["Tc"] <= 1 AND RTLSAbort["flyback_conv"] = 1) {
					addMessage("POWERED PITCH-AROUND TRIGGERED").
					SET STEERINGMANAGER:MAXSTOPPINGTIME TO 1.2.
					SET RTLSAbort["pitcharound"]["triggered"] TO TRUE.
					SET RTLSAbort["pitcharound"]["complete"] TO FALSE.
					SET RTLSAbort["flyback_flag"] TO TRUE.
					SET upfgOutput["flyback_flag"] TO TRUE.
					drawUI().
				} 
				
			} 
			ELSE {
					//powered pitcharound					
					//get the current thrust vector, project in the plane containing the usc vector (flyback guidance command) and C1,
					//rotate ahead by a few degrees
					
					SET control["refvec"] TO VXCL(vecYZ(RTLSAbort["pitcharound"]["refvec"]),SHIP:FACING:TOPVECTOR).
					
					LOCAL thrust_facing IS VXCL(RTLSAbort["pitcharound"]["refvec"],vecYZ(thrust_vec()):NORMALIZED).
								
					SET usc["lastvec"] TO rodrigues(thrust_facing, RTLSAbort["pitcharound"]["refvec"],15). 
					
					IF (VANG(thrust_facing, RTLSAbort["pitcharound"]["target"]) < 10) {
						SET STEERINGMANAGER:MAXSTOPPINGTIME TO 0.2.
						SET RTLSAbort["pitcharound"]["complete"] TO TRUE.
						//do it again for good measure
						SET RTLSAbort["flyback_flag"] TO TRUE.
						SET upfgOutput["flyback_flag"] TO TRUE.
						SET control["refvec"] TO -SHIP:ORBIT:BODY:POSITION:NORMALIZED.
						SET usc["lastthrot"] TO upfgOutput["throtset"].
						
					}	
			}
			//disable regular upfg steering by continually resetting the convergence flags
			SET usc["conv"] TO usc["iter"].
			//itercount must be reset so we don't end up with a huge iterations count at PPA
			//BUT DON'T RESET IT TO ZERO BC AT EVERY LOOP A WHEN CHECK WILL BE ADDED!
			SET usc["itercount"] TO 1.
		}
		
		SET RTLSAbort["Tc"] TO upfgOutput["Tc"].
	}
	
	
	IF usc["conv"]=1 { //converged and stable, accept result
			SET usc["lastvec"] TO upfgOutput["steering"].
			SET usc["lastthrot"] TO upfgOutput["throtset"].
	}
	ELSE{
		IF wasconv{//in this case we had convergence and we lost it, reset itercount
		SET usc["itercount"] TO 0.
		}
	}
	
	
	
	RETURN upfgOutput.
}


FUNCTION resetUPFG {
	PARAMETER upfgOutput.
	

	addMessage("RESETTING UPFG").
	LOCAL lastvec IS usc["lastvec"].
	LOCAL x IS setupUPFG(target_orbit).
	SET upfgOutput[0] TO x[0].
	SET usc TO x[1].
	SET usc["lastvec"] TO lastvec.
	local stg is get_stage().
	SET usc["lastthrot"] TO stg["Throttle"].
	
	RETURN upfgOutput.
}





//		UPFG MAIN ROUTINE

FUNCTION upfg {

	DECLARE FUNCTION compute_iF {
		PARAMETER time_.
		LOCAL out IS  lambda + lambdadot*time_.
		RETURN out:NORMALIZED.
	}

	PARAMETER t.
	PARAMETER vehicle.
	PARAMETER tgt_orb.
	PARAMETER previous.
	

	LOCAL dt IS t - previous["time"].
	LOCAL v IS orbitstate["velocity"].
	LOCAL vgo IS previous["vgo"] - (v - previous["v"]).
	LOCAL tgo IS previous["tgo"].
	LOCAL lambda IS previous["lambda"].
	LOCAL lambdadot IS previous["lambdadot"].
		
	LOCAL r IS orbitstate["radius"].
	LOCAL cser IS previous["cser"].
	LOCAL rd IS previous["rd"].
	LOCAL rbias IS previous["rbias"].
	LOCAL rgrav IS previous["rgrav"].
	LOCAL iy IS tgt_orb["normal"]:NORMALIZED.
	LOCAL iz IS VCRS(rd,iy):NORMALIZED.
	LOCAL m IS vehicle[0]["m_initial"].
	LOCAL Kk IS previous["throtset"].
	
	LOCAL t40flag IS tgo<40.
	
	LOCAL g0 IS 9.80665. 
	
	LOCAL s_mode IS tgt_orb["mode"].
	
	LOCAL flyback_flag IS FALSE.
	LOCAL mbod IS 0.
	LOCAL dmbo IS 0.
	LOCAL Tc IS 0.
	LOCAL burnout_m IS 0.
	LOCAL mbo_T IS 0.
	LOCAL RTLSthrotflag IS tgo>60.
	
	IF (s_mode = 5) {
		SET mbod TO  previous["mbod"].
		SET flyback_flag TO previous["flyback_flag"].
		IF (NOT flyback_flag ) {
			SET Kk TO 0.99.
		}
	}

	
	//	1
	LOCAL n IS vehicle:LENGTH.
	LOCAL SM IS LIST().
	LOCAL aL IS LIST().
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
		kklist:ADD(vehicle[i]["Throttle"]).
		IF (i=0) {
			SET kklist[i] TO Kk.	
		}
		SET fT[i] TO fT[i]*kklist[i].
		SET md[i] TO md[i]*kklist[i].
		ve:ADD(vehicle[i]["engines"]["isp"]*g0).
		aT:ADD(fT[i] / vehicle[i]["m_initial"]).
		tu:ADD(ve[i]/aT[i]).
		tb:ADD(vehicle[i]["Tstage"]).
	}
	
	
	//	3
	IF SM[0]=1 {
		SET aT[0] TO fT[0]*Kk / m.
	} ELSE IF SM[0]=2 {
		SET aT[0] TO aL[0].
	}
	SET tu[0] TO ve[0] / aT[0].
	
	LOCAL Li IS LIST().
		
	IF (s_mode = 5) {
		Li:ADD(vgo:MAG).
		SET burnout_m TO m*CONSTANT:E^(-Li[0]/ve[0]).
		SET mbo_T TO (m - mbod)/md[0].
	} ELSE {
		LOCAL Lsum IS 0.
		FROM { LOCAL i IS 0. } UNTIL i>=n-1 STEP { SET i TO i+1. } DO {
			IF SM[i]=1 {
				Li:ADD( ve[i]*LN(tu[i]/(tu[i]-tb[i])) ).
			} ELSE IF SM[i]=2 {
				Li:ADD( aL[i]*tb[i] ).
			} ELSE Li:ADD( 0 ).
			SET Lsum TO Lsum + Li[i].
			
			IF Lsum>vgo:MAG {
				RETURN upfg(
					t,
					vehicle:SUBLIST(0,vehicle:LENGTH-1),
					tgt_orb,
					previous
				).
			}
		}
		Li:ADD(vgo:MAG - Lsum).
	}
	
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
	
	SET tgo TO tgoi[n-1].
	
	//	4
	LOCAL L IS 0.
	LOCAL J IS 0.
	LOCAL S IS 0.
	LOCAL Q IS 0.
	LOCAL H IS 0.
	LOCAL P IS 0.
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
		SET Si[i] TO Si[i] + L*tb[i].
		SET Qi[i] TO Qi[i] + J*tb[i].
		SET Pi[i] TO Pi[i] + H*tb[i].
		
		SET L TO L+Li[i].
		SET J TO J+Ji[i].
		SET S TO S+Si[i].
		SET Q TO Q+Qi[i].
		SET P TO P+Pi[i].
		SET H TO J*tgoi[i] - Q.
	}
	LOCAL K IS J/L.
	
	
	//	5
	IF vgo:MAG <>0 { SET lambda TO vgo:NORMALIZED.}
	IF previous["tgo"]>0 {
		SET rgrav TO (tgo/previous["tgo"])^2 * rgrav.
	}
	
	LOCAL rgo IS rd - (r + v*tgo + rgrav).
	LOCAL iz IS VCRS(rd,iy):NORMALIZED.
	LOCAL rgoxy IS rgo - VDOT(iz,rgo)*iz.
	LOCAL rgoz IS (S - VDOT(lambda,rgoxy)) / VDOT(lambda,iz).
	SET rgo TO rgoxy + rgoz*iz + rbias.
	LOCAL lambdade IS Q - S*K.
	
	IF (NOT t40flag) {
		SET lambdadot TO (rgo - S*lambda) / lambdade.
	}
	
	
	LOCAL iF_ IS compute_iF(-K).
	LOCAL phi IS VANG(iF_,lambda)*CONSTANT:DEGTORAD.
	LOCAL phidot IS -phi/K.
	LOCAL vthrust IS (L - 0.5*L*phi^2 - J*phi*phidot - 0.5*H*phidot^2).
	SET vthrust TO vthrust*lambda - (L*phi + J*phidot)*lambdadot:NORMALIZED.
	LOCAL rthrust IS S - 0.5*S*phi^2 - Q*phi*phidot - 0.5*P*phidot^2.
	SET rthrust TO rthrust*lambda - (S*phi + Q*phidot)*lambdadot:NORMALIZED.
	SET vbias TO vgo - vthrust.
	SET rbias TO rgo - rthrust.
	
	
	//	7
	
	
	LOCAL rc1 IS r - 0.1*rthrust - (tgo/30)*vthrust.
	LOCAL vc1 IS v + 1.2*rthrust/tgo - 0.1*vthrust.
	LOCAL pack IS cse(rc1, vc1, tgo, cser).
	SET cser TO pack[2].
	SET rgrav TO pack[0] - rc1 - vc1*tgo.
	LOCAL vgrav IS pack[1] - vc1.
	
	
	//	8
	LOCAL rp IS r + v*tgo + rgrav + rthrust.
	
	IF (NOT t40flag) OR ( s_mode=5 ) {
		SET rp TO VXCL(iy,rp).
	}
	
	LOCAL vd IS v(0,0,0).
	
	//some code duplication but helps readability
	IF s_mode=5 {
		LOCAL out IS RTLS_cutoff_params(tgt_orb,rp).
		SET tgt_orb TO out[0].
		SET vd TO  out[1].
		
		IF (t40flag) {
			SET tgt_orb["radius"] TO rp.
		}
		
		SET rd TO tgt_orb["radius"].
	
	} ELSE IF s_mode=6 {
		LOCAL ix IS rp:NORMALIZED.
		SET iz TO VCRS(ix,iy):NORMALIZED.

		SET tgt_orb TO TAL_cutoff_params(tgt_orb,rd).
		SET rd TO tgt_orb["radius"]:MAG*ix.	
		SET vd TO iz*tgt_orb["velocity"].
	
	} ELSE IF s_mode=7 {
		LOCAL ix IS rp:NORMALIZED.
		SET iz TO VCRS(ix,iy):NORMALIZED.
		
		SET tgt_orb TO ATO_cutoff_params(tgt_orb,rd).
		SET rd TO tgt_orb["radius"]:MAG*ix.	
		
		SET vd TO rodrigues(iz,iy, tgt_orb["angle"]):NORMALIZED*tgt_orb["velocity"].	
	
	} ELSE {
		LOCAL ix IS rp:NORMALIZED.
		SET iz TO VCRS(ix,iy):NORMALIZED.
		
		LOCAL eta IS 0.
	 
		IF tgt_orb["mode"]=2 {								
			//recompute cutoff true anomaly
			SET tgt_orb["perivec"] TO target_perivec().
			SET  eta TO signed_angle(tgt_orb["perivec"],rp,-iy,1).	
		}
		 
		SET tgt_orb TO cutoff_params(tgt_orb,rd,eta).
		SET rd TO tgt_orb["radius"]:MAG*ix.	
		
		SET vd TO rodrigues(iz,iy, tgt_orb["angle"]):NORMALIZED*tgt_orb["velocity"].	
		
	}
	

	SET vgo TO vd - v - vgrav + vbias.
	
	IF (s_mode = 5) {
		LOCAL dmbo IS burnout_m - mbod.
		
		SET Tc TO mbo_T - tgo.
		print "Tc : " + Tc at (0,49).
		
		IF (flyback_flag AND RTLSthrotflag) {
			
			LOCAL throtgain IS -dt*2.1e-3.
			
			LOCAL newKk IS Kk + throtgain*Tc.
			SET Kk TO MAX(0,MIN(1,newKk)).
		}
		print "KK : " + Kk at (0,50).
	}
	
	//	RETURN - build new internal state instead of overwriting the old one
	LOCAL current IS LEXICON(
		"cser", cser,
		"rbias", rbias,
		"rd", rd,
		"rgrav", rgrav,
		"time", t,
		"tgo", tgo,
		"v", v,
		"vgo", vgo,
		"lambda", lambda,
		"lambdadot", lambdadot,
		"t_lambda",(t + K),
		"steering",iF_,
		"throtset",Kk,
		"flyback_flag",flyback_flag,
		"dmbo",dmbo,
		"mbod",mbod,
		"Tc",Tc
	).
	
	
	RETURN LIST(current,tgt_orb).
}