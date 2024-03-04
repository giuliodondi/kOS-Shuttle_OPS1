@LAZYGLOBAL OFF.
clearscreen.
clearvecdraws().
SET CONFIG:IPU TO 1200.	

RUNONCEPATH("0:/Libraries/misc_library").	
RUNONCEPATH("0:/Libraries/maths_library").	
RUNONCEPATH("0:/Libraries/navigation_library").
RUNPATH("0:/Libraries/vehicle_library").	

RUNPATH("0:/Shuttle_OPS1/src/ops1_vehicle_library").

LOCAL dap IS ascent_dap_factory().

LOCK STEERING TO dap:steer_dir.
LOCK THROTTLE to dap:thr_cmd.

GLOBAL ascent_dap_executor IS loop_executor_factory(
												0.2,
												{
													clearscreen.
													clearvecdraws().
													
													
													dap:steer_css().
													dap:thr_control_css().
													
													dap:print_debug(2).
													
													arrow_ship(2 * dap:steer_dir:forevector,"forevec").
													arrow_ship(2 * dap:steer_dir:topvector,"topvec").
													
													
												}
	).
	
	
	
until false{}