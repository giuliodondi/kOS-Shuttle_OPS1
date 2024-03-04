@LAZYGLOBAL OFF.
clearscreen.
clearvecdraws().
SET CONFIG:IPU TO 1200.	

RUNONCEPATH("0:/Libraries/misc_library").	
RUNONCEPATH("0:/Libraries/maths_library").	
RUNONCEPATH("0:/Libraries/navigation_library").

RUNPATH("0:/Shuttle_OPS1/src/ops1_vehicle_library").

LOCAL dap IS ascent_dap_factory().

GLOBAL ascent_dap_executor IS loop_executor_factory(
												0.2,
												{
													
													if (parameters["full_debug"]) {
														dap:print_debug(2).
													}
													
													
													
													
												}
	).
	
	
	
until false{}