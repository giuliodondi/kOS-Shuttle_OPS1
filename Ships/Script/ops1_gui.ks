@LAZYGLOBAL OFF.
CLEARSCREEN.
SET CONFIG:IPU TO 1200.	
GLOBAL quit_program IS FALSE.


RUNPATH("0:/Libraries/misc_library").	
RUNPATH("0:/Libraries/maths_library").	
RUNPATH("0:/Libraries/navigation_library").	
RUNPATH("0:/Libraries/vehicle_library").	
RUNPATH("0:/Libraries/aerosim_library").	

RUNPATH("0:/Shuttle_OPS1/src/ops1_interface").
RUNPATH("0:/Shuttle_OPS1/src/ops1_vehicle_library").
RUNPATH("0:/Shuttle_OPS1/src/ops1_targeting_library").
RUNPATH("0:/Shuttle_OPS1/src/ops1_upfg_library").
RUNPATH("0:/Shuttle_OPS1/src/ops1_abort_library").
RUNPATH("0:/Shuttle_OPS1/src/ops1_gui_library.ks").
RUNPATH("0:/Shuttle_OPS1/src/sample_traj_data.ks").


close_all_GUIs().
	
make_main_ascent_gui().
make_ascent_traj1_disp().

local sample_data is sample_traj_data().
local sample_data_count is 250.

local t0 is TIME:SECONDS.

ascent_traj_add_engout(1000).
ascent_traj_add_engout(2000).
ascent_traj_add_engout(3000).

until false {
	if (quit_program) {break.}
	
	
	if (ship:control:pilotpitch > 0) {
		set sample_data_count to sample_data_count + 2.
	} else if (ship:control:pilotpitch < 0) {
		set sample_data_count to max(0, sample_data_count - 2).
	}
	
	local alt_ is sample_data[sample_data_count][0].
	local ve is sample_data[sample_data_count][1].
	local vi is sample_data[sample_data_count][2].
	
	local traj_disp_alt_ref is 45.
	if (ascent_traj_disp_counter > 1) {
		set traj_disp_alt_ref to 115.
	}
	
	
	local tgo is upfgInternal["Tgo"].
	local vgo is upfgInternal["vgo"]:MAG.
	LOCAL converged IS (upfgInternal["s_conv"]) AND (NOT upfgInternal["s_meco"]).
	
	LOCAL gui_data IS lexicon(
				"major_mode", vehiclestate["major_mode"],
				"hdot", 100,
				"r_delta", 5,
				"p_delta", 4,
				"y_delta", 3,
				"t_delta", 7,
				"vi", vi,
				"ve", ve,
				"alt", alt_,
				"alt_ref", traj_disp_alt_ref,
				"pred_vi", vi + 100,
				"pred_ve", ve + 100,
				"pred_alt", alt_ + 2,
				"twr", 2,
				"ssme_thr", 104,
				"et_prop", 98,
				"tgo", tgo,
				"vgo", vgo,
				"rtls_tc", -2,
				"converged", converged
	).
	
	
	update_ascent_traj_disp(gui_data).
	
	wait (0.1).
}



CLEARSCREEN.
close_all_GUIs().