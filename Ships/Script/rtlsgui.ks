 clearscreen.
 
 RUNPATH("0:/Libraries/misc_library").	
RUNPATH("0:/Libraries/maths_library").	
RUNPATH("0:/Libraries/navigation_library").	


RUNPATH("0:/Shuttle_OPS1/src/ops1_gui_library.ks").

RUNPATH("0:/Shuttle_OPS1/src/sample_rtls_traj_data.ks").

IF (DEFINED RTLSAbort) {
	UNSET RTLSAbort.
}

GLOBAL quit_program IS FALSE.

make_main_ascent_gui().


local alt_ is 0.
local vel_ is 0.
local rtls_cutv is 2000.

local sample_data is sample_rtls_traj_data().
local sample_data_count is 700.

make_ascent_traj1_disp().
ascent_gui_set_cutv_indicator(7654).

local phase is 1.

local tign is TIME:SECONDS + 10.

when (sample_data_count > 222) then  {
	global RTLSAbort is lexicon().
	make_rtls_traj2_disp().
}

until false {

	if (ship:control:pilotyaw > 0) {
		set sample_data_count to sample_data_count + 2.
	} else if (ship:control:pilotyaw < 0) {
		set sample_data_count to max(0, sample_data_count - 2).
	}
	
	set rtls_cutv to rtls_cutv + 10*ship:control:pilotpitch.

	
	set alt_ to sample_data[sample_data_count][0].
	set ve to sample_data[sample_data_count][1].
	set vi to sample_data[sample_data_count][2].
	
	print "ve " + ve + " " at (0,2).
	print "rtls_cutv " + rtls_cutv + " " at (0,3).

	LOCAL gui_data IS lexicon(
						"met", TIME:SECONDS - tign,
						"ops_mode", 1,
						"hdot", 100,
						"roll", 1,
						"pitch", 2,
						"yaw", 3,
						"vi", vi,
						"ve", ve,
						"alt", alt_,
						"pred_vi", vi + 100,
						"pred_ve", ve + 100,
						"pred_alt", alt_ + 1,
						"twr", 1.5,
						"ssme_thr", 100,
						"et_prop", 100,
						"tgo", 265,
						"vgo", 5432,
						"converged", false
			).

	IF (DEFINED RTLSAbort) {
	
		set vdwnrg to sample_data[sample_data_count][3].
	
		gui_data:ADD("dwnrg_ve", vdwnrg).
		gui_data:ADD("dwnrg_pred_ve", vdwnrg + sign(vdwnrg) * 100).
		gui_data:ADD("rtls_cutv", rtls_cutv).
		gui_data:ADD("rtls_tc", 110).
	
		update_rtls_traj_disp(gui_data).
	} else {
		update_ascent_traj_disp(gui_data).
	
	}
	
	

	if (quit_program) {
		BREAK.
	} 

	wait 0.1.
}

close_all_GUIs().