 clearscreen.
 
 RUNPATH("0:/Libraries/misc_library").	
RUNPATH("0:/Libraries/maths_library").	
RUNPATH("0:/Libraries/navigation_library").	


RUNPATH("0:/Shuttle_OPS1/src/ops1_gui_library.ks").

RUNPATH("0:/Shuttle_OPS1/src/sample_traj_data.ks").

GLOBAL quit_program IS FALSE.

make_main_ascent_gui().


local alt_ is 0.
local vel_ is 0.
local twr is 0.
local vi is 7000.

local sample_data is sample_traj_data().
local sample_data_count is 0.

make_ascent_traj1_disp(7654).

local phase is 1.

local tign is TIME:SECONDS + 10.

until false {

	if (ship:control:pilotyaw > 0) {
		set sample_data_count to sample_data_count + 2.
	} else if (ship:control:pilotyaw < 0) {
		set sample_data_count to max(0, sample_data_count - 2).
	}
	
	set twr to twr + 0.05*ship:control:pilotpitch.
	
	if (ship:control:pilotroll <>0 ) {
		add_scroll_msg("new message added").
	}
	
	
	print "vi " + vi + " " at (0,2).
	
	set alt_ to sample_data[sample_data_count][0].
	set vel_ to sample_data[sample_data_count][1].


	
	LOCAL gui_data IS lexicon(
					"met", TIME:SECONDS - tign,
					"ops_mode", 1,
					"hdot", 100,
					"roll", 1,
					"pitch", 2,
					"yaw", 3,
					"vi", vi,
					"ve", vel_,
					"alt", alt_,
					"pred_vi", vi + 100,
					"pred_ve", vel_ + 100,
					"pred_alt", alt_ + 1,
					"twr", 1.5,
					"ssme_thr", 100,
					"et_prop", 100,
					"tgo", 265,
					"vgo", 5432,
					"converged", true
		).

	update_ascent_traj_disp(gui_data).

	if (quit_program) {
		BREAK.
	} 

	wait 0.1.
}

close_all_GUIs().