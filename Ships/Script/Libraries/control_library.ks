RUNONCEPATH ("0:/Libraries/maths_library").
RUNONCEPATH ("0:/Libraries/navigation_library").
RUNONCEPATH("0:/Libraries/misc_library").	

FUNCTION airplane_DAP_factory {
	PARAMETER max_bank.
	PARAMETER max_pitch.

	LOCAL this IS lexicon().

	this:add("engaged", FALSE).
	
	this:add("engage", {
		SET this:engaged TO TRUE.
		SAS OFF.
		this:update_time().
		this:update_lvlh().
		this:reset_steering().
	}).
	this:add("disengage", {
		SET this:engaged TO FALSE.
	}).
	
	this:add("max_pitch",max_pitch).
	this:add("max_bank",max_bank).
	
	this:add("last_time", TIME:SECONDS).
	
	this:add("iteration_dt", 0).
	
	this:add("update_time",{
		LOCAL old_t IS this:last_time.
		SET this:last_time TO TIME:SECONDS.
		SET this:iteration_dt TO this:last_time - old_t.
	}).
	
	this:add("rollsteer", 0).
	this:add("pitchsteer", 0).
	this:add("yawsteer", 0).
	
	this:add("lvlh_roll", 0).
	this:add("lvlh_pitch", 0).
	
	this:add("update_lvlh", {
		SET this:lvlh_pitch TO get_pitch_lvlh().
		SET this:lvlh_roll TO get_roll_lvlh().
	
	}).
	
	this:add("reset_steering",{
		SET this:rollsteer TO this:lvlh_roll.
		SET this:pitchsteer TO this:lvlh_pitch.
	}).
	
	this:add("measure_inputs",{
		
		LOCAL rollgain IS 2.5.
		LOCAL pitchgain IS 0.8.
		LOCAL yawgain IS 5.
		LOCAL time_gain IS ABS(this:iteration_dt/0.03).
		
		LOCAL deltaroll IS time_gain*rollgain*(SHIP:CONTROL:PILOTROLL - SHIP:CONTROL:PILOTROLLTRIM).
		LOCAL deltapitch IS time_gain*pitchgain*COS(this:lvlh_roll)*(SHIP:CONTROL:PILOTPITCH - SHIP:CONTROL:PILOTPITCHTRIM).
		
		SET deltaroll TO CLAMP(deltaroll, -20, 20).
		SET deltapitch TO CLAMP(deltapitch, -20, 20).
		
		LOCAL deltaroll IS yawgain*SHIP:CONTROL:PILOTYAW.
		
		
		
		
		
		LOCAL roll_left_lim IS MAX(-this:max_bank, this:lvlh_roll - 20).
		LOCAL roll_right_lim IS MAX(this:max_bank, this:lvlh_roll + 20).
		
		SET this:rollsteer TO CLAMP(this:rollsteer + deltaroll, roll_left_lim, roll_right_lim) .
		
		LOCAL pitch_down_lim IS MAX(-this:max_pitch, this:lvlh_pitch - 20).
		LOCAL pitch_up_lim IS MAX(this:max_pitch, this:lvlh_pitch + 20).
		
		SET this:pitchsteer TO CLAMP(this:pitchsteer + deltapitch, pitch_down_lim, pitch_up_lim) .
		
		
		
		SET this:yawsteer TO 
	
	}).
	
	this:add("update_steering",{
	
		this:update_time().
		this:update_lvlh().
		
		IF NOT this:engaged {
			this:reset_steering().
			RETURN SHIP:FACING.
		} 
		
		this:measure_inputs().
		
		RETURN this:new_steering_dir().
		
	}).
	
	this:add("new_steering_dir",{
	
		LOCAL refv IS SHIP:VELOCITY:SURFACE:NORMALIZED.
		LOCAL upv IS -SHIP:ORBIT:BODY:POSITION:NORMALIZED.
		
		LOCAL upv_proj IS VXCL(SHIP:FACING:STARVECTOR, upv).
		SET refv TO VXCL(upv_proj,refv).
		
		LOCAL ship_upv IS SHIP:FACING:UPVECTOR.
		
		SET refv TO rodrigues(refv, ship_upv, this:yawsteer ).
		
		//create the pitch rotation vector
		LOCAL nv IS VCRS(refv,ship_upv).
		
		//rotate the prograde vector by the pitch angle
		LOCAL aimv IS rodrigues(refv,nv,this:pitchsteer * 0.9/ COS(this:lvlh_roll)).
		
		//rotate the up vector by the new roll anglwe 
		LOCAL new_ship_upv IS rodrigues(upv,aimv,-this:rollsteer).
		
		
		
		clearvecdraws().
		arrow(refv, "refvec", 20, 0.02).
		arrow(aimv, "aimvec", 20, 0.02).

		
		RETURN LOOKDIRUP(aimv, new_ship_upv).
	}).
	
	this:add("print_debug",{
		PARAMETER line.
		
		print "engaged : " + this:engaged + "  " at (0,line).
		
		print "lvlh pitch : " + round(this:lvlh_pitch,3) + "    " at (0,line + 2).
		print "lvlh roll : " + round(this:lvlh_roll,3) + "    " at (0,line + 3).
		
		print "loop dt : " + round(this:iteration_dt(),3) + "    " at (0,line + 5).
		print "steer pitch : " + round(this:pitchsteer,3) + "    " at (0,line + 6).
		print "steer roll : " + round(this:rollsteer,3) + "    " at (0,line + 7).
		print "steer yaw : " + round(this:yawsteer,3) + "    " at (0,line + 8).
		
	}).
	
	IF (DEFINED SASPITCHPID) {UNSET SASPITCHPID.}
	IF (DEFINED SASROLLPID) {UNSET SASROLLPID.}
	SET STEERINGMANAGER:ROLLCONTROLANGLERANGE TO 180.
	
	this:update_time().
	this:update_lvlh().
	
	RETURN this.
}
