GLOBAL guitextgreen IS RGB(20/255,255/255,21/255).
global trajbgblack IS RGB(5/255,8/255,8/255).

GLOBAL guitextgreenhex IS "14ff15".
GLOBAL guitextredhex IS "ff1514".
GLOBAL guitextyellowhex IS "fff600".

GLOBAL main_ascent_gui_width IS 550.
GLOBAL main_ascent_gui_height IS 525.


FUNCTION make_main_ascent_gui {
	

	//create the GUI.
	GLOBAL main_ascent_gui is gui(main_ascent_gui_width,main_ascent_gui_height).
	SET main_ascent_gui:X TO 200.
	SET main_ascent_gui:Y TO 540.
	SET main_ascent_gui:STYLe:WIDTH TO main_ascent_gui_width.
	SET main_ascent_gui:STYLe:HEIGHT TO main_ascent_gui_height.
	SET main_ascent_gui:STYLE:ALIGN TO "center".
	SET main_ascent_gui:STYLE:HSTRETCH  TO TRUE.

	set main_ascent_gui:skin:LABEL:TEXTCOLOR to guitextgreen.


	// Add widgets to the GUI
	GLOBAL title_box is main_ascent_gui:addhbox().
	set title_box:style:height to 35. 
	set title_box:style:margin:top to 0.


	GLOBAL text0 IS title_box:ADDLABEL("<b><size=20>SPACE SHUTTLE OPS1 ASCENT GUIDANCE</size></b>").
	SET text0:STYLE:ALIGN TO "center".

	GLOBAL minb IS  title_box:ADDBUTTON("-").
	set minb:style:margin:h to 7.
	set minb:style:margin:v to 7.
	set minb:style:width to 20.
	set minb:style:height to 20.
	set minb:TOGGLE to TRUE.
	function minimizecheck {
		PARAMETER pressed.
		
		IF pressed {
			main_ascent_gui:SHOWONLY(title_box).
			SET main_ascent_gui:STYLe:HEIGHT TO 50.
		} ELSE {
			SET main_ascent_gui:STYLe:HEIGHT TO main_gui_height.
			for w in main_ascent_gui:WIDGETS {
				w:SHOW().
			}
		}
		
	}
	SET minb:ONTOGGLE TO minimizecheck@.

	GLOBAL quitb IS  title_box:ADDBUTTON("X").
	set quitb:style:margin:h to 7.
	set quitb:style:margin:v to 7.
	set quitb:style:width to 20.
	set quitb:style:height to 20.
	function quitcheck {
	  SET quit_program TO TRUE.
	}
	SET quitb:ONCLICK TO quitcheck@.

	
	main_ascent_gui:addspacing(7).
	
	GLOBAL ascent_traj_disp_counter IS 1.				   			   
	
	GLOBAL ascent_traj_disp IS main_ascent_gui:addvlayout().
	SET ascent_traj_disp:STYLE:WIDTH TO main_ascent_gui_width - 16.
	SET ascent_traj_disp:STYLE:HEIGHT TO 380.
	SET ascent_traj_disp:STYLE:ALIGN TO "center".
	
	set ascent_traj_disp:style:BG to "Shuttle_OPS1/src/gui_images/ascent_traj_bg.png".

	GLOBAL ascent_traj_disp_titlebox IS ascent_traj_disp:ADDHLAYOUT().
	SET ascent_traj_disp_titlebox:STYLe:WIDTH TO ascent_traj_disp:STYLE:WIDTH.
	SET ascent_traj_disp_titlebox:STYLe:HEIGHT TO 1.
	GLOBAL ascent_traj_disp_title IS ascent_traj_disp_titlebox:ADDLABEL("").
	SET ascent_traj_disp_title:STYLE:ALIGN TO "center".
	
	GLOBAL ascent_traj_disp_clockbox IS ascent_traj_disp:ADDHLAYOUT().
	SET ascent_traj_disp_clockbox:STYLe:WIDTH TO ascent_traj_disp:STYLE:WIDTH.
	SET ascent_traj_disp_clockbox:STYLe:HEIGHT TO 1.
	GLOBAL ascent_traj_disp_clock IS ascent_traj_disp_clockbox:ADDLABEL("MET 00:00:00").
	SET ascent_traj_disp_clock:STYLE:ALIGN TO "right".
	SET ascent_traj_disp_clock:STYLE:margin:h to 20.
	
	ascent_traj_disp:addspacing(18).
	
	GLOBAL ascent_traj_disp_overlaiddata IS ascent_traj_disp:ADDVLAYOUT().
	SET ascent_traj_disp_overlaiddata:STYLE:ALIGN TO "Center".
	SET ascent_traj_disp_overlaiddata:STYLe:WIDTH TO ascent_traj_disp:STYLE:WIDTH.
	SET ascent_traj_disp_overlaiddata:STYLe:HEIGHT TO 1.
	
	GLOBAL ascent_traj_disp_mainbox IS ascent_traj_disp:ADDVLAYOUT().
	SET ascent_traj_disp_mainbox:STYLE:ALIGN TO "Center".
	SET ascent_traj_disp_mainbox:STYLe:WIDTH TO ascent_traj_disp:STYLE:WIDTH.
	SET ascent_traj_disp_mainbox:STYLe:HEIGHT TO ascent_traj_disp:STYLE:HEIGHT - 22.
	
	GLOBAL ascent_traj_disp_orbiter_box IS ascent_traj_disp_mainbox:ADDVLAYOUT().
	SET ascent_traj_disp_orbiter_box:STYLE:ALIGN TO "Center".
	SET ascent_traj_disp_orbiter_box:STYLe:WIDTH TO 1.
	SET ascent_traj_disp_orbiter_box:STYLe:HEIGHT TO 1.
	
	GLOBAL ascent_traj_disp_orbiter IS ascent_traj_disp_orbiter_box:ADDLABEL().
	SET ascent_traj_disp_orbiter:IMAGE TO "Shuttle_OPS1/src/gui_images/traj_pos_bug.png".
	SET ascent_traj_disp_orbiter:STYLe:WIDTH TO 12.
	
	local shut_bug_pos is set_ascent_traj_disp_pos(v(ascent_traj_disp_x_convert(0),ascent_traj_disp_y_convert(0), 0), 5).
	
	SET ascent_traj_disp_orbiter:STYLE:margin:v to shut_bug_pos[1].
	SET ascent_traj_disp_orbiter:STYLE:margin:h to shut_bug_pos[0].
	
	GLOBAL ascent_traj_disp_pred_box IS ascent_traj_disp_mainbox:ADDVLAYOUT().
	SET ascent_traj_disp_pred_box:STYLE:ALIGN TO "Center".
	SET ascent_traj_disp_pred_box:STYLe:WIDTH TO 1.
	SET ascent_traj_disp_pred_box:STYLe:HEIGHT TO 1.
	
	GLOBAL ascent_traj_disp_pred_bug_ IS ascent_traj_disp_pred_box:ADDLABEL().
	SET ascent_traj_disp_pred_bug_:IMAGE TO "Shuttle_OPS1/src/gui_images/traj_pred_bug.png".
	SET ascent_traj_disp_pred_bug_:STYLe:WIDTH TO 10.
	
	SET ascent_traj_disp_pred_bug_:STYLE:margin:v to shut_bug_pos[1] - 3.
	SET ascent_traj_disp_pred_bug_:STYLE:margin:h to shut_bug_pos[0].
	
	main_ascent_gui:addspacing(7).
	
	GLOBAL ascent_msg_scroll_box IS main_ascent_gui:addvlayout().
	SET ascent_msg_scroll_box:STYLE:WIDTH TO main_ascent_gui_width - 16.
	SET ascent_msg_scroll_box:STYLE:HEIGHT TO 80.
	SET ascent_msg_scroll_box:STYLE:ALIGN TO "center".
	
	global msgscroll is ascent_msg_scroll_box:addscrollbox().
	set msgscroll:valways to true.
	set msgscroll:style:margin:h to 0.
	set msgscroll:style:margin:v to 0.
	
	main_ascent_gui:SHOW().
	
}

FUNCTION close_all_GUIs{
	CLEARGUIS().
}

function ascent_add_scroll_msg {
	parameter msg.
	parameter clear_all is false.
	
	if (clear_all AND msgscroll:widgets:LENGTH > 0) {
		msgscroll:widgets[0]:dispose().
	}
	
	local newlab is msgscroll:addlabel(msg).
	set newlab:style:margin:v to -2.
	
	set msgscroll:position to v(0,1000,0).

}

FUNCTION clear_ascent_traj_data {
	ascent_traj_disp_overlaiddata:clear().
}

function make_ascent_traj1_disp {

	set ascent_traj_disp_counter to 1.
	
	clear_ascent_traj_data().
	
	local text_ht is ascent_traj_disp_titlebox:style:height*0.75.
	set ascent_traj_disp_title:text to "<b><size=" + text_ht + ">XXXXXX TRAJ X</size></b>".
	
	set ascent_traj_disp_mainbox:style:BG to "Shuttle_OPS1/src/gui_images/ascent_traj1_bg.png".
	
	GLOBAL ascent_traj_disp_upperdatabox IS ascent_traj_disp_overlaiddata:ADDHLAYOUT().
	
	SET ascent_traj_disp_upperdatabox:STYLE:WIDTH TO 125.
    SET ascent_traj_disp_upperdatabox:STYLE:HEIGHT TO 30.
	set ascent_traj_disp_upperdatabox:style:margin:h to 0.
	set ascent_traj_disp_upperdatabox:style:margin:v to 0.
	
	make_ascent_cutv_slider(ascent_traj_disp_upperdatabox).
	
	ascent_traj_disp_upperdatabox:HIDE().
	
	
	GLOBAL ascent_traj_disp_leftdatabox IS ascent_traj_disp_overlaiddata:ADDVLAYOUT().
	SET ascent_traj_disp_leftdatabox:STYLE:ALIGN TO "left".
	SET ascent_traj_disp_leftdatabox:STYLE:WIDTH TO 125.
    SET ascent_traj_disp_leftdatabox:STYLE:HEIGHT TO 115.
	set ascent_traj_disp_leftdatabox:style:margin:h to 20.
	set ascent_traj_disp_leftdatabox:style:margin:v to 12.
	
	GLOBAL ascent_trajleftdata1 IS ascent_traj_disp_leftdatabox:ADDLABEL("Ḣ   xxxxxx").
	set ascent_trajleftdata1:style:margin:v to -4.
	GLOBAL ascent_trajleftdata2 IS ascent_traj_disp_leftdatabox:ADDLABEL("").
	set ascent_trajleftdata2:style:margin:v to -4.
	GLOBAL ascent_trajleftdata3 IS ascent_traj_disp_leftdatabox:ADDLABEL("R  xxxxxx").
	set ascent_trajleftdata3:style:margin:v to -4.
	GLOBAL ascent_trajleftdata4 IS ascent_traj_disp_leftdatabox:ADDLABEL("P xxxxxx").
	set ascent_trajleftdata4:style:margin:v to -4.
	GLOBAL ascent_trajleftdata5 IS ascent_traj_disp_leftdatabox:ADDLABEL("Y xxxxxx").
	set ascent_trajleftdata5:style:margin:v to -4.
	
	GLOBAL ascent_traj_disp_rightdatabox IS ascent_traj_disp_overlaiddata:ADDHLAYOUT().
	SET ascent_traj_disp_rightdatabox:STYLE:ALIGN TO "left".
	SET ascent_traj_disp_rightdatabox:STYLE:WIDTH TO 125.
    SET ascent_traj_disp_rightdatabox:STYLE:HEIGHT TO 115.
	set ascent_traj_disp_rightdatabox:style:margin:h to 320.
	set ascent_traj_disp_rightdatabox:style:margin:v to 10.
	
	make_g_slider(ascent_traj_disp_rightdatabox).
	
	GLOBAL ascent_traj_disp_rightdatabox2 IS ascent_traj_disp_rightdatabox:ADDVLAYOUT().
	SET ascent_traj_disp_rightdatabox2:STYLE:ALIGN TO "left".
	SET ascent_traj_disp_rightdatabox2:STYLE:WIDTH TO 125.
    SET ascent_traj_disp_rightdatabox2:STYLE:HEIGHT TO 115.
	set ascent_traj_disp_rightdatabox2:style:margin:h to 15.
	set ascent_traj_disp_rightdatabox2:style:margin:v to 20.
	
	GLOBAL ascent_trajrightdata1 IS ascent_traj_disp_rightdatabox2:ADDLABEL("PROP xxxxxx").
	set ascent_trajrightdata1:style:margin:v to -4.
	GLOBAL ascent_trajrightdata2 IS ascent_traj_disp_rightdatabox2:ADDLABEL("THR   xxxxxx").
	set ascent_trajrightdata2:style:margin:v to -4.
	GLOBAL ascent_trajrightdata3 IS ascent_traj_disp_rightdatabox2:ADDLABEL("").
	set ascent_trajrightdata3:style:margin:v to -4.
	GLOBAL ascent_trajrightdata4 IS ascent_traj_disp_rightdatabox2:ADDLABEL("TGO  xxxxxx").
	set ascent_trajrightdata4:style:margin:v to -4.
	GLOBAL ascent_trajrightdata5 IS ascent_traj_disp_rightdatabox2:ADDLABEL("VGO  xxxxxx").
	set ascent_trajrightdata5:style:margin:v to -4.
	
	
}

function make_ascent_cutv_slider {
	parameter container_box.
	
	GLOBAL cutv_sliderbox IS container_box:ADDHLAYOUT().
	SET cutv_sliderbox:STYLe:HEIGHT TO 40.
	SET cutv_sliderbox:STYLe:width TO 400.
	set cutv_sliderbox:style:margin:h to 80.
	set cutv_sliderbox:style:margin:v to 10.
	
	GLOBAL cutv_tgt_bug_box IS cutv_sliderbox:ADDVLAYOUT().
	SET cutv_tgt_bug_box:STYLE:ALIGN TO "Center".
	SET cutv_tgt_bug_box:STYLe:WIDTH TO 1.
	SET cutv_tgt_bug_box:STYLe:HEIGHT TO 1.
	
	GLOBAL cutv_tgt_bug_ IS cutv_tgt_bug_box:ADDLABEL().
	SET cutv_tgt_bug_:IMAGE TO "Shuttle_OPS1/src/gui_images/cutoff_vel_bug.png".
	SET cutv_tgt_bug_:STYLe:WIDTH TO 25.
	SET cutv_tgt_bug_:STYLe:HEIGHT TO 25.
	
	set cutv_tgt_bug_:style:margin:h to 4.
	set cutv_tgt_bug_:style:margin:v to 5.
	
	SET main_ascent_gui:skin:horizontalslider:bg TO "Shuttle_OPS1/src/gui_images/cutv_slider_bg.png".
	set main_ascent_gui:skin:horizontalsliderthumb:BG to "Shuttle_OPS1/src/gui_images/hslider_thumb.png".
	set main_ascent_gui:skin:horizontalsliderthumb:HEIGHT to 11.
	set main_ascent_gui:skin:horizontalsliderthumb:WIDTH to 16.
	set main_ascent_gui:skin:horizontalsliderthumb:margin:v to -12.
	
	GLOBAL cutv_slider is cutv_sliderbox:addhslider(7,7,8).
	SET cutv_slider:style:vstretch to false.
	SET cutv_slider:style:hstretch to false.
	SET cutv_slider:STYLE:WIDTH TO 335.
	SET cutv_slider:STYLE:HEIGHT TO 20.

}

function ascent_gui_set_cutv_indicator {
	parameter cutv.

	set cutv_tgt_bug_:style:margin:h to 4 + (cutv/1000 - 7)*320.

}

function make_rtls_cutv_slider {
	parameter container_box.
	
	GLOBAL rtls_cutv_sliderbox IS container_box:ADDHLAYOUT().
	SET rtls_cutv_sliderbox:STYLe:HEIGHT TO 40.
	SET rtls_cutv_sliderbox:STYLe:width TO 500.
	set rtls_cutv_sliderbox:style:margin:h to 30.
	set rtls_cutv_sliderbox:style:margin:v to 10.
	
	GLOBAL cutv_tgt_bug_box IS rtls_cutv_sliderbox:ADDVLAYOUT().
	SET cutv_tgt_bug_box:STYLE:ALIGN TO "Center".
	SET cutv_tgt_bug_box:STYLe:WIDTH TO 1.
	SET cutv_tgt_bug_box:STYLe:HEIGHT TO 1.
	
	GLOBAL rtls_cutv_tgt_bug_ IS cutv_tgt_bug_box:ADDLABEL().
	SET rtls_cutv_tgt_bug_:IMAGE TO "Shuttle_OPS1/src/gui_images/cutoff_vel_bug.png".
	SET rtls_cutv_tgt_bug_:STYLe:WIDTH TO 25.
	SET rtls_cutv_tgt_bug_:STYLe:HEIGHT TO 25.
	
	set rtls_cutv_tgt_bug_:style:margin:h to 4.
	set rtls_cutv_tgt_bug_:style:margin:v to 5.
	
	SET main_ascent_gui:skin:horizontalslider:bg TO "Shuttle_OPS1/src/gui_images/rtls_cutv_slider_bg.png".
	set main_ascent_gui:skin:horizontalsliderthumb:BG to "Shuttle_OPS1/src/gui_images/hslider_thumb.png".
	set main_ascent_gui:skin:horizontalsliderthumb:HEIGHT to 11.
	set main_ascent_gui:skin:horizontalsliderthumb:WIDTH to 16.
	set main_ascent_gui:skin:horizontalsliderthumb:margin:v to -12.
	
	GLOBAL rtls_cutv_slider is rtls_cutv_sliderbox:addhslider(1.5,2.5,1.5).
	SET rtls_cutv_slider:style:vstretch to false.
	SET rtls_cutv_slider:style:hstretch to false.
	SET rtls_cutv_slider:STYLE:WIDTH TO 435.
	SET rtls_cutv_slider:STYLE:HEIGHT TO 20.

}

function rtls_gui_set_cutv_indicator {
	parameter cutv.

	set rtls_cutv_tgt_bug_:style:margin:h to 4 + ( 1 - (cutv/1000 - 1.5))*420.

}

function make_g_slider {
	parameter container_box.
	
	SET main_ascent_gui:skin:verticalslider:bg TO "Shuttle_OPS1/src/gui_images/g_slider_bg2.png".
	set main_ascent_gui:skin:verticalsliderthumb:BG to "Shuttle_OPS1/src/gui_images/vslider_thumb.png".
	set main_ascent_gui:skin:verticalsliderthumb:HEIGHT to 15.
	set main_ascent_gui:skin:verticalsliderthumb:WIDTH to 11.
	set main_ascent_gui:skin:verticalsliderthumb:margin:h to 22.
	
	GLOBAL g_sliderbox IS container_box:ADDHLAYOUT().
	SET g_sliderbox:STYLe:WIDTH TO 40.
	GLOBAL g_slider is g_sliderbox:addvslider(1,3.4,0.6).
	SET g_slider:STYLE:ALIGN TO "Center".
	SET g_slider:style:vstretch to false.
	SET g_slider:style:hstretch to false.
	SET g_slider:STYLE:WIDTH TO 27.
	SET g_slider:STYLE:HEIGHT TO 130.
	
}

function update_g_slider {
	parameter g_val.
	
	SET g_slider:VALUE TO CLAMP(g_val,g_slider:MIN,g_slider:MAX).
	
	if (g_val > 3.1) {
		set main_ascent_gui:skin:verticalsliderthumb:BG to "Shuttle_OPS1/src/gui_images/vslider_thumb_yellow.png".
	} else {
		set main_ascent_gui:skin:verticalsliderthumb:BG to "Shuttle_OPS1/src/gui_images/vslider_thumb.png".
	}

}

FUNCTION update_traj_disp_title {
	PARAMETER disp_counter.
	
	//based on abort modes
	LOCAL str is "ASCENT".
	IF (abort_modes["triggered"]) {
		IF (abort_modes["ATO"]["triggered"] = TRUE) {
			SET str TO "   ATO".
		} ELSE IF (abort_modes["TAL"]["triggered"] = TRUE) {
			SET str TO "   TAL".
		} ELSE IF (abort_modes["RTLS"]["triggered"] = TRUE) {
			SET str TO "  RTLS".
		}
	}
	
	local text_ht is ascent_traj_disp_titlebox:style:height*0.75.
	set ascent_traj_disp_title:text to "<b><size=" + text_ht + ">" + str + " TRAJ " + disp_counter + "</size></b>".

}

function make_ascent_traj2_disp {
	set ascent_traj_disp_counter to 2.
	
	set ascent_traj_disp_mainbox:style:BG to "Shuttle_OPS1/src/gui_images/ascent_traj2_bg.png".
	
	ascent_traj_disp_upperdatabox:SHOW().
	
	//ADD stuff to ascent_traj_disp_overlaiddata
}

//
function make_rtls_traj2_disp {
	
	local text_ht is ascent_traj_disp_titlebox:style:height*0.75.
	set ascent_traj_disp_title:text to "<b><size=" + text_ht + ">XXXXXX TRAJ X</size></b>".
	
	set ascent_traj_disp_mainbox:style:BG to "Shuttle_OPS1/src/gui_images/rtls_traj2_bg.png".
	
	clear_ascent_traj_data().
	
	GLOBAL rtls_traj_disp_upperdatabox IS ascent_traj_disp_overlaiddata:ADDHLAYOUT().
	
	SET rtls_traj_disp_upperdatabox:STYLE:WIDTH TO 300.
    SET rtls_traj_disp_upperdatabox:STYLE:HEIGHT TO 30.
	set rtls_traj_disp_upperdatabox:style:margin:h to 0.
	set rtls_traj_disp_upperdatabox:style:margin:v to 0.
	
	make_rtls_cutv_slider(rtls_traj_disp_upperdatabox).
	
	GLOBAL rtls_traj_disp_leftdatabox IS ascent_traj_disp_overlaiddata:ADDVLAYOUT().
	SET rtls_traj_disp_leftdatabox:STYLE:ALIGN TO "left".
	SET rtls_traj_disp_leftdatabox:STYLE:WIDTH TO 125.
    SET rtls_traj_disp_leftdatabox:STYLE:HEIGHT TO 115.
	set rtls_traj_disp_leftdatabox:style:margin:h to 80.
	set rtls_traj_disp_leftdatabox:style:margin:v to 12.
	
	GLOBAL rtls_trajleftdata1 IS rtls_traj_disp_leftdatabox:ADDLABEL("Ḣ   xxxxxx").
	set rtls_trajleftdata1:style:margin:v to -4.
	GLOBAL rtls_trajleftdata2 IS rtls_traj_disp_leftdatabox:ADDLABEL("THR   xxxxxx").
	set rtls_trajleftdata2:style:margin:v to -4.
	GLOBAL rtls_trajleftdata3 IS rtls_traj_disp_leftdatabox:ADDLABEL("PROP xxxxxx").
	set rtls_trajleftdata3:style:margin:v to -4.
	
	GLOBAL rtls_trajleftdata4 IS rtls_traj_disp_leftdatabox:ADDLABEL("").
	set rtls_trajleftdata4:style:margin:v to -4.
	
	GLOBAL rtls_trajleftdata5 IS rtls_traj_disp_leftdatabox:ADDLABEL("TGO  xxxxxx").
	set rtls_trajleftdata5:style:margin:v to -4.
	GLOBAL rtls_trajleftdata6 IS rtls_traj_disp_leftdatabox:ADDLABEL("VGO  xxxxxx").
	set rtls_trajleftdata6:style:margin:v to -4.
	GLOBAL rtls_trajleftdata7 IS rtls_traj_disp_leftdatabox:ADDLABEL("T_C  xxxxxx").
	set rtls_trajleftdata7:style:margin:v to -4.
	
	
	GLOBAL rtls_traj_disp_rightdatabox IS ascent_traj_disp_overlaiddata:ADDHLAYOUT().
	SET rtls_traj_disp_rightdatabox:STYLE:ALIGN TO "left".
	SET rtls_traj_disp_rightdatabox:STYLE:WIDTH TO 125.
    SET rtls_traj_disp_rightdatabox:STYLE:HEIGHT TO 115.
	set rtls_traj_disp_rightdatabox:style:margin:h to 410.
	set rtls_traj_disp_rightdatabox:style:margin:v to 45.
	
	make_g_slider(rtls_traj_disp_rightdatabox).
	
}

function update_ascent_traj_disp {
	parameter gui_data.
	
	if (ascent_traj_disp_counter = 1 AND gui_data["ve"] >= 1000) {
		make_ascent_traj2_disp().
	}
	
	update_traj_disp_title(ascent_traj_disp_counter).

	SET ascent_traj_disp_clock:text TO "MET " + sectotime_simple(gui_data["met"], true).
	
	set ascent_trajleftdata1:text to "Ḣ   " + round(gui_data["hdot"], 0). 
	
	local rolstr is "R   ".
	if (gui_data["roll"] >=0) {
		set rolstr to rolstr + "R".
	} else {
		set rolstr to rolstr + "L".
	}
	set rolstr to rolstr + round(abs(gui_data["roll"]),0).
	set ascent_trajleftdata3:text to rolstr. 
	
	local pchstr is "P   ".
	if (gui_data["pitch"] >=0) {
		set pchstr to pchstr + "U".
	} else {
		set pchstr to pchstr + "D".
	}
	set pchstr to pchstr + round(abs(gui_data["pitch"]),0).
	set ascent_trajleftdata4:text to pchstr. 
	
	local yawstr is "Y   ".
	if (gui_data["yaw"] >=0) {
		set yawstr to yawstr + "R".
	} else {
		set yawstr to yawstr + "L".
	}
	set yawstr to yawstr + round(abs(gui_data["yaw"]),0).
	set ascent_trajleftdata5:text to yawstr. 
	
	
	set ascent_trajrightdata1:text to "PROP " + round(gui_data["et_prop"],0). 
	set ascent_trajrightdata2:text to "THR  " + round(gui_data["ssme_thr"], 0). 

	local upfg_text_color is guitextgreenhex.
	if (NOT gui_data["converged"]) {
		set upfg_text_color to guitextyellowhex.
	}

	set ascent_trajrightdata4:text to "<color=#" + upfg_text_color + ">TGO " + sectotime_simple(gui_data["tgo"]) + "</color>". 
	set ascent_trajrightdata5:text to "<color=#" + upfg_text_color + ">VGO  " + round(gui_data["vgo"], 0) + "</color>". 
	
	update_g_slider(gui_data["twr"]).
	
	SET cutv_slider:VALUE TO CLAMP(gui_data["vi"]/1000,cutv_slider:MIN,cutv_slider:MAX).
	
	local xval is gui_data["ve"].
	local xpredval is gui_data["pred_ve"].
	
	if (ascent_traj_disp_counter = 2) {
		set xval to gui_data["vi"].
		set xpredval to gui_data["pred_vi"].
	}
	
	local shut_bug_pos is set_ascent_traj_disp_pos(v(ascent_traj_disp_x_convert(xval),ascent_traj_disp_y_convert(gui_data["alt"]), 0), 5).
	
	SET ascent_traj_disp_orbiter:STYLE:margin:v to shut_bug_pos[1].
	SET ascent_traj_disp_orbiter:STYLE:margin:h to shut_bug_pos[0].
	
	local shut_pred_pos is set_ascent_traj_disp_pos(v(ascent_traj_disp_x_convert(xpredval),ascent_traj_disp_y_convert(gui_data["pred_alt"]), 0), 5).
	SET ascent_traj_disp_pred_bug_:STYLE:margin:v to shut_pred_pos[1] - 3.
	SET ascent_traj_disp_pred_bug_:STYLE:margin:h to shut_pred_pos[0].

}


function update_rtls_traj_disp {
	parameter gui_data.
	
	update_traj_disp_title(ascent_traj_disp_counter).

	SET ascent_traj_disp_clock:text TO "MET " + sectotime_simple(gui_data["met"], true).
	
	set rtls_trajleftdata1:text to "Ḣ   " + round(gui_data["hdot"], 0). 
	
	set rtls_trajleftdata3:text to "PROP " + round(gui_data["et_prop"],0). 
	set rtls_trajleftdata2:text to "THR  " + round(gui_data["ssme_thr"], 0). 

	local upfg_text_color is guitextgreenhex.
	if (NOT gui_data["converged"]) {
		set upfg_text_color to guitextyellowhex.
	}
	
	LOCAL tc_sign IS " ".
	IF (gui_data["rtls_tc"] <= -1) {
		SET tc_sign TO "-".
	}

	set rtls_trajleftdata5:text to "<color=#" + upfg_text_color + ">TGO " + sectotime_simple(gui_data["tgo"]) + "</color>". 
	set rtls_trajleftdata6:text to "<color=#" + upfg_text_color + ">VGO  " + round(gui_data["vgo"], 0) + "</color>". 
	set rtls_trajleftdata7:text to "<color=#" + guitextgreenhex + ">T_C " + tc_sign + sectotime_simple(ABS(gui_data["rtls_tc"])) + "</color>". 
	

	update_g_slider(gui_data["twr"]).
	
	SET rtls_cutv_slider:VALUE TO CLAMP(-SIGN(gui_data["dwnrg_ve"]) * gui_data["ve"]/1000,rtls_cutv_slider:MIN,rtls_cutv_slider:MAX).

	rtls_gui_set_cutv_indicator(gui_data["rtls_cutv"]).

	local shut_bug_pos is set_ascent_traj_disp_pos(v(rtls_traj_disp_x_convert(gui_data["dwnrg_ve"]),rtls_traj_disp_y_convert(gui_data["alt"]), 0), 5).
	
	SET ascent_traj_disp_orbiter:STYLE:margin:v to shut_bug_pos[1].
	SET ascent_traj_disp_orbiter:STYLE:margin:h to shut_bug_pos[0].
	
	local shut_pred_pos is set_ascent_traj_disp_pos(v(rtls_traj_disp_x_convert(gui_data["dwnrg_pred_ve"]),rtls_traj_disp_y_convert(gui_data["pred_alt"]), 0), 5).
	SET ascent_traj_disp_pred_bug_:STYLE:margin:v to shut_pred_pos[1] - 3.
	SET ascent_traj_disp_pred_bug_:STYLE:margin:h to shut_pred_pos[0].

}

//rescale redo
function set_ascent_traj_disp_pos {
	parameter bug_pos.
	parameter bias is 0.
	
	local bug_margin is 10.
	
	local bounds_x is list(10, ascent_traj_disp_mainbox:STYLe:WIDTH - 32).
	local bounds_y is list(ascent_traj_disp_mainbox:STYLe:HEIGHT - 29, 0).
	
	//print "calc_x: " + bug_pos:X + " calc_y: " +  + bug_pos:Y  + "  " at (0, 4).
	
	local pos_x is 1.04693*bug_pos:X  - 8.133 + bias.
	local pos_y is 395.55 - 1.1685*bug_pos:Y + bias.
	
	//print "disp_x: " + pos_x + " disp_y: " + pos_y + "  " at (0, 5).
	
	set pos_x to clamp(pos_x, bounds_x[0], bounds_x[1] ).
	set pos_y to clamp(pos_y, bounds_y[0], bounds_y[1]).
	
	//print "disp_x: " + pos_x + " disp_y: " + pos_y + "  " at (0, 6).
	
	return list(pos_x,pos_y).
}

function ascent_traj_disp_x_convert {
	parameter val.
	
	local par is val*3.28084.
	local out is 0.
	
	if (ascent_traj_disp_counter = 1) {
		set out to (par/5000 * 0.8 + 0.1)*512.
	} else if (ascent_traj_disp_counter = 2) {
		set out to ((par - 5000)/21000 * 0.8 + 0.1)*512.
	}
	
	return out.
}

function ascent_traj_disp_y_convert {
	parameter val.
	
	local par is val*3280.84.
	local out is 0.
	
	if (ascent_traj_disp_counter = 1) {
		set out to 512.0 - (par / 170000.0 * 0.6 + 0.2) * 512.
	} else if (ascent_traj_disp_counter = 2) {
		set out to 512.0 - ((par - 140000.0) / 385000.0 * 0.6 + 0.2) * 512.
	}

	return (425 - out)*300/275 + 50.
}	


function rtls_traj_disp_x_convert {
	parameter val.
	
	local par is val*3.28084.
	
	local out is  ((par + 8000.0)/18000*0.8 + 0.1)*512.
	
	return out.
}

function rtls_traj_disp_y_convert {
	parameter val.
	
	local par is val*3280.84.
	
	local out is 512.0 - ((par - 150000)/450000 * 0.7 + 0.2) * 512.

	return (425 - out)*300/275 + 50.
}	
