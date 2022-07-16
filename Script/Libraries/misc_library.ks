



//fancy function to print stuff to screen in a consistent format
//prints a fixed width of screen in characters at a specified position 
//given by leftcolumn and line .
//pads with blank spaces as needed
FUNCTION PRINTPLACE{
	parameter string.
	parameter width.
	parameter leftcolumn.
	parameter line.
	
	set string to string + "".

	local x is width - string:LENGTH.
	
	IF x<0 {RETURN.}
	
	set x to FLOOR(x/2).
	if x=0 {set x to 0.}
	
	LOCAL blankstr IS "".
	
	FROM {LOCAL k IS 1.} UNTIL k > width STEP { SET k TO k+1.} DO{
		SET blankstr TO blankstr + " ".
	}
	
	PRINT blankstr AT (leftcolumn,line).
	PRINT string AT (leftcolumn + x,line).

}




//log handler function, takes a lexicon as imput
//creates a log file, deleting the file by the same name if it exists
//then creates the column headers by dumping the lexicon keys
//if the log file has already been created just logs the lexicon to file
FUNCTION log_data {
	
	//logs to a file specified by filename the values of a lexicon log_lex
	FUNCTION dataLog {
		DECLARE PARAMETER filename.
		PARAMETER log_lex.
		
		LOCAL str IS "".
		
		//append to the string the numbers in sequence separated by four spaces
		FOR val IN log_lex:VALUES {
			SET str TO str + val + "    ".
		}

		LOG str TO filename.
	}

	PARAMETER log_lex.
	PARAMETER logname_string IS "".
	PARAMETER overwrite IS FALSE.
	
	if not (defined logname) {
	
		IF overwrite {
			GLOBAL logname is logname_string  + ".txt".
			IF EXISTS(logname)=TRUE {
				MOVEPATH(logname,logname_string + "_old" + ".txt").
			}
		} ELSE {
			local logcount is 0.
			GLOBAL logname is logname_string + logcount + ".txt".
			until false {
				set logname to logname_string + logcount + ".txt".
				IF EXISTS(logname)=TRUE {
					set logcount to logcount + 1.
				}
				ELSE {break.}
				
				
			}
		}
		
		LOCAL titlestr IS "".
		
		FOR key IN log_lex:KEYS {
			SET titlestr TO titlestr + key + "    ".
		}
		
		log titlestr to logname.

	} ELSE { 	
		dataLog(logname,log_lex).
	}
}





//draw a vector  with label, by default its centered on the ship and scaled to 10 times its length
FUNCTION arrow {
	PARAMETER v.
	PARAMETER lab.
	PARAMETER v_centre IS v(0,0,0).
	PARAMETER scl IS 10.
	PARAMETER wdh IS 0.5.
	
	VECDRAW(
      v_centre,
      v,
      RGB(1,0,0),
      lab,
      scl,
      TRUE,
      wdh
    ).

}

//draw a vector  with label, by default its centered on the body and scaled to 2 times its length
FUNCTION arrow_body {
	PARAMETER v.
	PARAMETER lab.
	PARAMETER scl IS 2.
	PARAMETER wdh IS 0.5.
	
	VECDRAW(
      SHIP:ORBIT:BODY:POSITION,
      v,
      RGB(1,0,0),
      lab,
      scl,
      TRUE,
      wdh
    ).

}



//converts the universal clock in seconds
FUNCTION utc_time_seconds {
	LOCAL utc_time Is TIME.
	RETURN utc_time:SECOND + 60*utc_time:MINUTE + 3600*utc_time:HOUR.
}

//givne a longitude from 0 long. gives the local clock in seconds
FUNCTION local_time_seconds {
	PARAMETER long.

	LOCAL utc_time IS utc_time_seconds().
	
	LOCAL degree_shift IS BODY:ROTATIONPERIOD/360.

	RETURN wraparound(utc_time + long*degree_shift, 0, BODY:ROTATIONPERIOD).
}


//converts a time value into a hours,minutes,seconds string
declare function sectotime {
	parameter t.
	local string is "".
	if t<0 {
		set t to ABS(t).
		set string to string + "-".
	}
	if t<60 {
		set string to string + FLOOR(t) + " s".	
	}
	else {
		local min is FLOOR(t/60).
		if min<60 {
			local sec is FLOOR(t - min*60).
			set string to string + min + " m " + sec+ " s".
		}
		else {
			local sec is FLOOR(t - min*60).	
			local hr is FLOOR(min/60).
			set min to min - hr*60.
			set string to string + hr + " h " + min + " m " + sec+ " s".
		}
	}
	return string.
}


// object that implements a fixed-size list where elements are added at the front
FUNCTION fixed_list_factory {
	PARAMETER len.

	local this is lexicon().
	
	this:add(
			"list", LIST()
	).
	
	this:add(
			"maxlength", len
	).
	
	this:add("trim", {
		if (this:list:length > this:maxlength) {
			local sublist is this:list:sublist(0,this:maxlength).
			set this:list to sublist.
		}
	}
	).
	
	this:add("push", {
		parameter newval.
		this:list:insert(0, newval).
		this:trim().
	}
	).
	
	this:add("printat", {
		parameter col.
		parameter line.
		
		local c is col.
		for v in this:list {
			local str_v is v + ",".
			print v at (c,line).
			set c to c + str_v:length + 1.
		}
		//get rid of the last comma
		print " " at (c - 1,line).
	}
	).

	return this.
}

//object that keeps track of the last x values inserted and calculates the running average of them
FUNCTION average_value_factory {
	parameter avgcount.

	local this is lexicon().
	
	this:add(
			"numvalues", avgcount
	).
	
	this:add(
			"list", fixed_list_factory(this["numvalues"])
	).
	
	
	
	this:add("reset", {
		set this:list to fixed_list_factory(this["numvalues"]).
	}
	).
	
	
	this:add("update", {
		parameter newval.
		this:list:push(newval).
	}
	).
	
	this:add("average", {
		
		local avg is 0.
		
		local values is this:list:list.
		local len is values:length.
		
		if (len=0) {return 0.}
		
		for v in values {
			set avg to avg + v.
		}
		
		return avg/len.
	}
	).

	return this.
}



