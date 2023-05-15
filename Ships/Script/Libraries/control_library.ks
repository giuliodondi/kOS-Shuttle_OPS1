
//compute net thrust vector as thrust-weighted average of engines position 
//relative to the ship raw frame
//obtain the difference between fore vector and thrust vector.
//then, given input reference "fore" and "up" vectors, rotate in that frame
//the "instantaneous" flag specifies if the thrust correction is calculated using 
//the engines runnign right now at their current thrust (TRUE)
//or all engines at their theoretical max thrust (FALSE)
//the latter is needed for on-orbit manoeuvres when engines are not yet running
FUNCTION thrustrot {
	PARAMETER ref_fore.
	PARAMETER ref_up.
	PARAMETER instantaneous IS TRUE.
	
	LOCAL thrvec IS v(0,0,0).
	
	IF instantaneous {
		set thrvec to get_current_thrust_isp()[0]:NORMALIZED.
	} ELSe {
		set thrvec to get_max_thrust_isp()[0]:NORMALIZED.
	}
	
	local norm is VCRS(ref_fore,ref_up).
	local ship_fore IS SHIP:FACING:VECTOR:NORMALIZED.
	
	LOCAL newthrvec IS rodrigues(ref_fore,norm,-VANG(ship_fore,thrvec)):NORMALIZED.
	
	RETURN ref_fore - newthrvec.
}


//	Returns a kOS direction for given aim vector, reference up vector and roll angle.
//corrects for thrust offset
FUNCTION aimAndRoll {
	PARAMETER aimVec.	//	Expects a vector
	PARAMETER upVec.	//	Expects a vector
	PARAMETER rollAng.	//	Expects a scalar
	PARAMETER instantaneous IS TRUE.
	
	
	LOCAL steerVec IS aimVec.
	
	LOCAL topVec IS VXCL(steerVec,upVec):NORMALIZED.
	SET topVec TO rodrigues(topVec, steerVec, rollAng).
	
	LOCAL thrustCorr IS thrustrot(steerVec,topVec, instantaneous).
	
	LOCAL outdir IS LOOKDIRUP(steerVec + thrustCorr, topVec).

	//clearvecdraws().
	//arrow(topVec,"topVec",v(0,0,0),40,0.05).
	//arrow(aimVec,"aimVec",v(0,0,0),40,0.05).
	//arrow(steerVec,"steerVec",v(0,0,0),40,0.05).
	//arrow(thrustCorr,"thrustCorr",v(0,0,0),40,0.05).

	RETURN outdir.
}


//measures current total engine thrust vector and isp of running engines

FUNCTION get_current_thrust_isp {
	
	LOCAL thrvec IS v(0,0,0).
	LOCAL thr IS 0.
	LOCAL isp_ IS 0.
	
	list ENGINES in all_eng.
	FOR e IN all_eng {
		IF e:ISTYPE("engine") {
			IF e:IGNITION {
				LOCAL e_thr IS (e:THRUST * 1000).
				SET thr TO thr + e_thr.
				SET isp_ TO isp_ + e:vacuumisp*e_thr.
				set thrvec to thrvec -e:POSITION:NORMALIZED*e_thr.
			}
		}
	}	
	
	RETURN LIST(thrvec, isp_).
}


//measures theoretical max engine thrust and isp at this altitude
FUNCTION get_max_thrust_isp{

	LOCAL thrvec IS v(0,0,0).
	LOCAL thr IS 0.
	LOCAL isp_ IS 0.
	
	list ENGINES in all_eng.
	FOR e IN all_eng {
		IF e:ISTYPE("engine") {
			IF e:IGNITION {
				LOCAL e_thr IS (e:AVAILABLETHRUST * 1000).
				SET thr TO thr + e_thr.
				SET isp_ TO isp_ + e:vacuumisp*e_thr.
				set thrvec to thrvec -e:POSITION:NORMALIZED*e_thr.
			}
		}
	}	
	
	RETURN LIST(thrvec, isp_).
}

//converts between absolute throttle value (percentage of max thrust)
//and throttle percentage relative to the range min-max which KSP uses
FUNCTION throtteValueConverter {
	PARAMETER abs_throt.
	PARAMETER minthrot IS 0.

	RETURN CLAMP((abs_throt - minthrot)/(1 - minthrot),0.005,1).
}
