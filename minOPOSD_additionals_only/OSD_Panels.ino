//
//
//

/*

refactoring started
there is more refactoring necessary because there are too much side effects

TODO:

	refactor:
		switchPanels
	
	implement usage of pal_ntsc
	
	maybe implement usage of panCallsign

*/


#include "OSD_Config.h"

#ifdef FLIGHT_BATT_ON_MINIMOSD
#include "FlightBatt.h"
#endif

#ifdef PACKETRXOK_ON_MINIMOSD
#include "PacketRxOk.h"
#endif


#define PWM_LO			1200	// [us]	PWM low value
#define PWM_HI			1800	// [us]	PWM high value
#define PWM_OFFSET		100	// [us]	PWM offset for detecting stick movement

#define SETUP_TIME		30000	// [ms]	the time after boot while we can enter the setup menu
#define SETUP_LOWEST_MENU	2	//	lowest shown setup menue item
#ifndef FLIGHT_BATT_ON_MINIMOSD
#define SETUP_HIGHEST_MENU	2	//	highest shown setup menue item
#else
#define SETUP_HIGHEST_MENU	11	//	highest shown setup menue item
#endif

#define WARN_FLASH_TIME		1000	// [ms]	the time with which the warnings are flashing
#define WARN_RECOVER_TIME	4000	// [ms]	the time we stay in the first panel after last warning
#define WARN_MAX		5	//	the number of implemented warnings

#define MODE_SWITCH_TIME	2000	// [ms]	the time for mode switching

#define TIME_RESET_AMPERE	2	// [A]	the current above which the on time is set to 00:00


/******* GLOBAL VARS *******/

static boolean		setup_menu_active = false;
static boolean		warning_active = false;

static float		convert_speed = 0;
static float		convert_length = 0;
static uint8_t		unit_speed = 0;
static uint8_t		unit_length = 0;

static int16_t		chan1_raw_middle = 0;
static int16_t		chan2_raw_middle = 0;


/******* MAIN FUNCTIONS *******/


/******************************************************************/
// Panel  : startPanels
// Output : Logo panel and initialization
/******************************************************************/
void startPanels() {
    osd.clear();
    panLogo();		// display logo
    set_converts();	// initialize the units
}


/******************************************************************/
// Panel  : writePanels
// Output : Write the panels or do other things
/******************************************************************/
void writePanels() {
    if (ch_toggle > 3) switchPanels();										// this must be first so you can always switch the panel
    if (!setup_menu_active) {											// setup is called in the else path
        if (panel < npanels) {											// first or second panel
            if (ISd(panel,Warn_BIT))		panWarn(panWarn_XY[0][panel], panWarn_XY[1][panel]);		// this must be here so warnings are always checked

	    // these GPS related panels are active under all circumstances
            if (ISa(panel,GPSats_BIT))		panGPSats(panGPSats_XY[0][panel], panGPSats_XY[1][panel]);	// number of visible sats
            if (ISa(panel,GPL_BIT))		panGPL(panGPL_XY[0][panel], panGPL_XY[1][panel]);		// sat fix type

	    // these GPS related panels are active if GPS was valid before
	    if (osd_got_home) {
		if (ISa(panel,GPS_BIT))		panGPS(panGPS_XY[0][panel], panGPS_XY[1][panel]);		// Lat & Lon
		if (ISb(panel,HDis_BIT))	panHomeDis(panHomeDis_XY[0][panel], panHomeDis_XY[1][panel]);
                if (ISb(panel,HDir_BIT))	panHomeDir(panHomeDir_XY[0][panel], panHomeDir_XY[1][panel]);
	    }

	    // these GPS related panels are active if GPS was valid before and we have a sat fix
	    if (osd_got_home && osd_fix_type > 1) {
                if (ISc(panel,Halt_BIT))	panHomeAlt(panHomeAlt_XY[0][panel], panHomeAlt_XY[1][panel]);
                if (ISc(panel,Alt_BIT))		panAlt(panAlt_XY[0][panel], panAlt_XY[1][panel]);
                if (ISc(panel,Vel_BIT))		panVel(panVel_XY[0][panel], panVel_XY[1][panel]);
                if (ISd(panel,Climb_BIT))	panClimb(panClimb_XY[0][panel], panClimb_XY[1][panel]);
                if (ISb(panel,Head_BIT))	panHeading(panHeading_XY[0][panel], panHeading_XY[1][panel]);
                if (ISb(panel,Rose_BIT))	panRose(panRose_XY[0][panel], panRose_XY[1][panel]);
	    }

	    if (ISd(panel,RSSI_BIT))		panRSSI(panRSSI_XY[0][panel], panRSSI_XY[1][panel]);
            if (ISa(panel,Rol_BIT))		panRoll(panRoll_XY[0][panel], panRoll_XY[1][panel]);
            if (ISa(panel,Pit_BIT))		panPitch(panPitch_XY[0][panel], panPitch_XY[1][panel]);
            if (ISc(panel,Thr_BIT))		panThr(panThr_XY[0][panel], panThr_XY[1][panel]);
            if (ISc(panel,FMod_BIT))		panFlightMode(panFMod_XY[0][panel], panFMod_XY[1][panel]);
            if (ISa(panel,BatA_BIT))		panBatt_A(panBatt_A_XY[0][panel], panBatt_A_XY[1][panel]);
            if (ISc(panel,CurA_BIT))		panCur_A(panCur_A_XY[0][panel], panCur_A_XY[1][panel]);
            if (ISa(panel,Bp_BIT))		panBatteryPercent(panBatteryPercent_XY[0][panel], panBatteryPercent_XY[1][panel]);
            if (ISb(panel,Time_BIT))		panTime(panTime_XY[0][panel], panTime_XY[1][panel]);
            if (ISc(panel,Hor_BIT))		panHorizon(panHorizon_XY[0][panel], panHorizon_XY[1][panel]);
	} else {												// panel off
            if (ISd(0,Warn_BIT))		panWarn(panWarn_XY[0][0], panWarn_XY[1][0]);			// this must be here so warnings are always checked
        }
    } else {													// setup menu is active
        panSetup();
    }

#ifdef membug
    // OSD debug for development
    osd.setPanel(13,4);
    osd.openPanel();
    osd.printf("%i", freeMem()); 
    osd.closePanel();
#endif
}


/******************************************************************/
// Panel  : switchPanels
// Output : Switch between panels
// TODO   : REFACTOR
/******************************************************************/
void switchPanels() {
    static uint8_t      	osd_off_switch = 0;
    static uint8_t      	osd_switch_last = 100;
    static unsigned long	osd_switch_time = 0;
    static uint16_t		ch_raw = 0;

    if (ch_toggle == 4) {
        if ((osd_mode != FLIGHTSTATUS_FLIGHTMODE_AUTOTUNE) && (osd_mode != FLIGHTSTATUS_FLIGHTMODE_POSITIONHOLD)) {
            if (osd_off_switch != osd_mode) { 
                osd_off_switch = osd_mode;
                osd_switch_time = millis();

                if (osd_off_switch == osd_switch_last) {
                    switch (panel) {
			case 0:
                            panel = 1;                                                        
                            if (millis() <= SETUP_TIME) {
                                setup_menu_active = true;
                            } else {
                                setup_menu_active = false;
                            }                            
                           break;
			case 1:
                            panel = npanels;
                            setup_menu_active = false;
                           break;
			case npanels:
                            panel = 0;
                            break;
                    }
                    osd.clear();
                }
            }
            if ((millis() - osd_switch_time) > MODE_SWITCH_TIME) {
                osd_switch_last = osd_mode;
            }
        }
    }
    else {
        if (ch_toggle == 5) ch_raw = osd_chan5_raw;
        else if (ch_toggle == 6) ch_raw = osd_chan6_raw;
        else if (ch_toggle == 7) ch_raw = osd_chan7_raw;
        else if (ch_toggle == 8) ch_raw = osd_chan8_raw;

        if (switch_mode == 0) {
            if (ch_raw > PWM_HI) {
                if (millis() <= SETUP_TIME) {
                    setup_menu_active = true;
                }
                else if (!setup_menu_active && !warning_active) {
                    osd.clear();
                }
                panel = npanels;				// off panel
            }
            else if (ch_raw < PWM_LO && panel != 0) {
                setup_menu_active = false;
                osd.clear();
                panel = 0;					// first panel
            }
            else if (ch_raw >= PWM_LO && ch_raw <= PWM_HI && panel != 1 && !warning_active) {
                setup_menu_active = false;
                osd.clear();
                panel = 1;					// second panel
            }        
        } else {
            if (ch_raw > PWM_LO) {
                if (millis() <= SETUP_TIME && !setup_menu_active) {
                    if (osd_switch_time + MODE_SWITCH_TIME / 2 < millis()) {
                        setup_menu_active = true;
                        osd_switch_time = millis();
                    }
                } else {
                    if (osd_switch_time + MODE_SWITCH_TIME / 2 < millis()) {
                        setup_menu_active = false;
                        osd.clear();
                        if (panel == npanels) {
                            panel = 0;
                        } else {
                            panel++;
                        }
                        if (panel > 1) panel = npanels;
                        osd_switch_time = millis();
                    }
                }
	    }
        }    
    }
}


/******* SPECIAL PANELS *******/


/******************************************************************/
// Panel  : panWarn
// Needs  : X, Y locations
// Output : Warnings if there are any but only if the setup menu is not active
/******************************************************************/
void panWarn(int first_col, int first_line) {
    static char* warning_string;
    static uint8_t last_warning_type = 1;
    static uint8_t warning_type = 0;
    static unsigned long warn_recover_timer = 0;
    static unsigned long warn_text_timer = 0;
    int cycle;

    if (!setup_menu_active && millis() > warn_text_timer) {	// if the setup menu is not active and the text or blank text has been shown for a while
        if (warning_type) {					// there was a warning, so we now blank it out for a while
            last_warning_type = warning_type;			// save the warning type for cycling
            warning_type = 0;
	    warning_string = "            ";			// blank the warning
	    warn_text_timer = millis() + WARN_FLASH_TIME;	// set clear warning time
        } else {
            cycle = last_warning_type;				// start the warning checks cycle where we left it last time
            while (!warning_type) {				// cycle through the warning checks
                if (++cycle > WARN_MAX) cycle = 1;
                switch (cycle) {
                case 1:						// DISARMED
		    if (!motor_armed) {
			warning_type = cycle;
			warning_string = "  DISARMED  ";
		    }
                    break;
                case 2:						// No telemetry communication
		    if (uavtalk_state() != TELEMETRYSTATS_STATE_CONNECTED) {
			warning_type = cycle;
			warning_string = " NO TEL COM ";
		    }
                    break;
		case 3:						// NO GPS FIX
                    if ((osd_fix_type) < 2 && osd_got_home) {	// to allow flying in the woods (what I really like) without this annoying warning,
			warning_type = cycle;			// this warning is only shown if GPS was valid before (osd_got_home)
			warning_string = " NO GPS FIX ";
		    }
                    break;
                case 4:						// BATT LOW
#ifdef FLIGHT_BATT_ON_MINIMOSD
                    if (osd_vbat_A < battv/10.0) {
#else
                    if (osd_vbat_A < float(battv)/10.0 || osd_battery_remaining_A < batt_warn_level) {
#endif
			warning_type = cycle;
			warning_string = "  BATT LOW  ";
		    }
                    break;
                case 5:						// RSSI LOW
#ifdef PACKETRXOK_ON_MINIMOSD
		    rssi = PacketRxOk_get();
#endif
                    if (rssi < rssi_warn_level && rssi != -99 && !rssiraw_on) {
			warning_type = cycle;
			warning_string = "  RSSI LOW  ";
		    }
                    break;
                }
                if (cycle == last_warning_type) break;		// we've done a full cycle
            }
	    if (warning_type) {					// if there a warning
		warning_active = true;				// then set warning active
		warn_text_timer = millis() + WARN_FLASH_TIME;	// set show warning time
		warn_recover_timer = millis() + WARN_RECOVER_TIME;            
	    } else {						// if not, we do not want the delay, so a new error shows up immediately
		if (millis() > warn_recover_timer) {		// if recover time over since last warning
		    warning_active = false;			// no warning active anymore
		}
	    }
        }

	osd.setPanel(first_col, first_line);
	osd.openPanel();
        if (warning_active) {
            if (panel > 0) osd.clear();
            panel = 0;						// switch to first panel if there is a warning                  
        }
        osd.printf("%s", warning_string);
	osd.closePanel();
    }
}


/******************************************************************/
// Panel  : panSetup
// Needs  : Nothing, uses whole screen
// Output : The settings menu
/******************************************************************/
void panSetup() {
    static int8_t setup_menu = 0;
    int delta = 100;

    osd.clear();
    osd.setPanel(5, 3);
    osd.openPanel();
	
    osd.printf_P(PSTR("Setup screen|||"));

    if (chan1_raw_middle == 0 || chan2_raw_middle == 0) {
        chan1_raw_middle = chan1_raw;
        chan2_raw_middle = chan2_raw;
    }

    if ((chan2_raw - PWM_OFFSET) > chan2_raw_middle ) setup_menu++;
    else if ((chan2_raw + PWM_OFFSET) < chan2_raw_middle ) setup_menu--;
	
    if (setup_menu < SETUP_LOWEST_MENU) setup_menu = SETUP_LOWEST_MENU;
    else if (setup_menu > SETUP_HIGHEST_MENU) setup_menu = SETUP_HIGHEST_MENU;

    switch (setup_menu) {
        case 2:
            osd.printf_P(PSTR("Battery warning "));
            osd.printf("%3.1f%c", float(battv)/10.0 , 0x76, 0x20);
            battv = change_val(battv, battv_ADDR);
            break;
#ifdef FLIGHT_BATT_ON_MINIMOSD
        case 5:
	    delta /= 10;
        case 4:
	    delta /= 10;
        case 3:
	    // volt_div_ratio
            osd.printf_P(PSTR("Calibrate||measured volt: "));
            osd.printf("%c%5.2f%c", 0xE2, (float)osd_vbat_A, 0x8E);
            osd.printf("||volt div ratio:  %5i", volt_div_ratio);
            volt_div_ratio = change_int_val(volt_div_ratio, volt_div_ratio_ADDR, delta);
            break;
	case 8:
	    delta /= 10;
	case 7:
	    delta /= 10;
	case 6:
	    // curr_amp_offset
            osd.printf_P(PSTR("Calibrate||measured amp:  "));
            osd.printf("%c%5.2f%c", 0xE2, osd_curr_A * .01, 0x8F);
            osd.printf("||amp offset:      %5i", curr_amp_offset);
            curr_amp_offset = change_int_val(curr_amp_offset, curr_amp_offset_ADDR, delta);
            break;
	case 11:
	    delta /= 10;
	case 10:
	    delta /= 10;
	case 9:
	    // curr_amp_per_volt
            osd.printf_P(PSTR("Calibrate||measured amp:  "));
            osd.printf("%c%5.2f%c", 0xE2, osd_curr_A * .01, 0x8F);
            osd.printf("||amp per volt:    %5i", curr_amp_per_volt);
            curr_amp_per_volt = change_int_val(curr_amp_per_volt, curr_amp_per_volt_ADDR, delta);
            break;
#endif
    }
    osd.closePanel();
}


/******* PANELS *******/


/******************************************************************/
// Panel  : panBoot
// Needs  : X, Y locations
// Output : Booting up text and empty bar after that
/******************************************************************/
void panBoot(int first_col, int first_line) {
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf_P(PSTR("Booting up:\xed\xf2\xf2\xf2\xf2\xf2\xf2\xf2\xf3")); 
    osd.closePanel();
}


/******************************************************************/
// Panel  : panLogo
// Needs  : X, Y locations
// Output : Startup OSD LOGO
/******************************************************************/
void panLogo() {
    osd.setPanel(3, 5);
    osd.openPanel();
    osd.printf_P(PSTR("\x20\x20\x20\x20\x20\xba\xbb\xbc\xbd\xbe|\x20\x20\x20\x20\x20\xca\xcb\xcc\xcd\xce|minOPOSD 1.2.1"));
#ifdef PACKETRXOK_ON_MINIMOSD
    osd.printf_P(PSTR(" PRxOk"));
#endif
#ifdef ANALOG_RSSI_ON_MINIMOSD
    osd.printf_P(PSTR(" ARSSI"));
#endif
#ifdef JR_SPECIALS
    osd.printf_P(PSTR(" JRS"));
#endif
    osd.closePanel();
}


/******************************************************************/
// Panel  : panGPSats
// Needs  : X, Y locations
// Output : 1 symbol and number of locked satellites
/******************************************************************/
void panGPSats(int first_col, int first_line) {
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf("%c%2i", 0x0f, osd_satellites_visible);
    osd.closePanel();
}


/******************************************************************/
// Panel  : panGPL
// Needs  : X, Y locations
// Output : 1 static symbol with changing FIX symbol
/******************************************************************/
void panGPL(int first_col, int first_line) {
    char* gps_str;
    
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    if (osd_fix_type == 0 || osd_fix_type == 1) gps_str = "\x10\x20";
    else if (osd_fix_type == 2 || osd_fix_type == 3) gps_str = "\x11\x20";
    osd.printf("%s", gps_str);
#ifdef JR_SPECIALS	// I use this place for debug info
    osd.printf("%02x", op_alarm);
#endif
    osd.closePanel();
}


/******************************************************************/
// Panel  : panGPS
// Needs  : X, Y locations
// Output : two row numeric value of current GPS location with LAT/LON symbols
/******************************************************************/
void panGPS(int first_col, int first_line) {
    osd.setPanel(first_col, first_line);
    osd.openPanel();
#ifdef JR_SPECIALS	// I like it more one row style
    osd.printf("%c%10.6f     %c%10.6f", 0x83, (double)osd_lat, 0x84, (double)osd_lon);
#else
    osd.printf("%c%11.6f|%c%11.6f", 0x83, (double)osd_lat, 0x84, (double)osd_lon);
#endif
    osd.closePanel();
}


/******************************************************************/
// Panel  : panHomeDis
// Needs  : X, Y locations
// Output : Distance to home
/******************************************************************/
void panHomeDis(int first_col, int first_line) {
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf("%c%5.0f%c", 0x1F, (double)((osd_home_distance) * convert_length), unit_length);
    osd.closePanel();
}


/******************************************************************/
// Panel  : panHomeDir
// Needs  : X, Y locations
// Output : 2 symbols that are combined as one arrow, shows direction to home
/******************************************************************/
void panHomeDir(int first_col, int first_line) {
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    showArrow((uint8_t)osd_home_direction);
    osd.closePanel();
}


/******************************************************************/
// Panel  : panHomeAlt
// Needs  : X, Y locations
// Output : Hom altitude
/******************************************************************/
void panHomeAlt(int first_col, int first_line) {
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf("%c%5.0f%c", 0xE7, (double)((osd_alt - osd_home_alt) * convert_length), unit_length);
    osd.closePanel();
}


/******************************************************************/
// Panel  : panAlt
// Needs  : X, Y locations
// Output : Altitude
/******************************************************************/
void panAlt(int first_col, int first_line) {
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf("%c%5.0f%c",0xE6, (double)(osd_alt * convert_length), unit_length);
    osd.closePanel();
}


/******************************************************************/
// Panel  : panVel
// Needs  : X, Y locations
// Output : Velocity 
/******************************************************************/
void panVel(int first_col, int first_line) {
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf("%c%3.0f%c", 0xE9, (double)(osd_groundspeed * convert_speed), unit_speed);
    osd.closePanel();
}


/******************************************************************/
// Panel  : panClimb
// Needs  : X, Y locations
// Output : Climb Rate
/******************************************************************/
void panClimb(int first_col, int first_line) {
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf("%c%3.0f%c", 0x16, (double)(osd_climb), 0x88);
    osd.closePanel();
}


/******************************************************************/
// Panel  : panHeading
// Needs  : X, Y locations
// Output : Symbols with numeric compass heading value
/******************************************************************/
void panHeading(int first_col, int first_line) {
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf("%4.0f%c", (double)osd_heading, 0xb0);
    osd.closePanel();
}


/******************************************************************/
// Panel  : panRose
// Needs  : X, Y locations
// Output : a dynamic compass rose that changes along the heading information
/******************************************************************/
void panRose(int first_col, int first_line) {
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    //osd_heading  = osd_yaw;
    //if (osd_yaw < 0) osd_heading = 360 + osd_yaw;
    osd.printf("%s|%c%s%c", "\x20\xc0\xc0\xc0\xc0\xc0\xc7\xc0\xc0\xc0\xc0\xc0\x20", 0xd0, buf_show, 0xd1);
    osd.closePanel();
}


/******************************************************************/
// Panel  : panRSSI
// Needs  : X, Y locations
// Output : RSSI %
/******************************************************************/
void panRSSI(int first_col, int first_line) {
    osd.setPanel(first_col, first_line);
    osd.openPanel();
#ifdef PACKETRXOK_ON_MINIMOSD
    PacketRxOk_print();
#else
    rssi = (int16_t)osd_rssi;
    if (!rssiraw_on) rssi = (int16_t)((float)(rssi - rssipersent)/(float)(rssical-rssipersent)*100.0f);
    if (rssi < -99) rssi = -99;
    osd.printf("%c%3i%c", 0xE1, rssi, 0x25);
#endif
    osd.closePanel();
}


/******************************************************************/
// Panel  : panRoll
// Needs  : X, Y locations
// Output : -+ value of current Roll from vehicle with degree symbols and roll symbol
/******************************************************************/
void panRoll(int first_col, int first_line) {
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf("%4i%c%c", osd_roll, 0xb0, 0xb2);
    osd.closePanel();
}


/******************************************************************/
// Panel  : panPitch
// Needs  : X, Y locations
// Output : -+ value of current Pitch from vehicle with degree symbols and pitch symbol
/******************************************************************/
void panPitch(int first_col, int first_line) {
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf("%4i%c%c", osd_pitch, 0xb0, 0xb1);
    osd.closePanel();
}

  
/******************************************************************/
// Panel  : panThr
// Needs  : X, Y locations
// Output : Throttle 
/******************************************************************/
void panThr(int first_col, int first_line) {
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf("%c%3.0i%c", 0x87, osd_throttle, 0x25);
    osd.closePanel();
}


/******************************************************************/
// Panel  : panFlightMode 
// Needs  : X, Y locations
// Output : current flight modes
/******************************************************************/
void panFlightMode(int first_col, int first_line) {
    char* mode_str="";
    
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    if      (osd_mode == FLIGHTSTATUS_FLIGHTMODE_MANUAL         ) mode_str = "man";	// MANUAL
    else if (osd_mode == FLIGHTSTATUS_FLIGHTMODE_STABILIZED1    ) mode_str = "st1";	// STABILIZED1
    else if (osd_mode == FLIGHTSTATUS_FLIGHTMODE_STABILIZED2    ) mode_str = "st2";	// STABILIZED2
    else if (osd_mode == FLIGHTSTATUS_FLIGHTMODE_STABILIZED3    ) mode_str = "st3";	// STABILIZED3
    else if (osd_mode == FLIGHTSTATUS_FLIGHTMODE_AUTOTUNE       ) mode_str = "at ";	// AUTOTUNE
    else if (osd_mode == FLIGHTSTATUS_FLIGHTMODE_ALTITUDEHOLD   ) mode_str = "ah ";	// ALTITUDEHOLD
    else if (osd_mode == FLIGHTSTATUS_FLIGHTMODE_VELOCITYCONTROL) mode_str = "vc ";	// VELOCITYCONTROL
    else if (osd_mode == FLIGHTSTATUS_FLIGHTMODE_POSITIONHOLD   ) mode_str = "ph ";	// POSITIONHOLD
    osd.printf("%c%s", 0xE0, mode_str);
    osd.closePanel();
}


/******************************************************************/
// Panel  : panBattery A (Voltage 1)
// Needs  : X, Y locations
// Output : Voltage value as in XX.X and symbol of over all battery status
/******************************************************************/
void panBatt_A(int first_col, int first_line) {
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf("%c%5.2f%c", 0xE2, (double)osd_vbat_A, 0x8E);
    osd.closePanel();
}


/******************************************************************/
// Panel  : panCur_A
// Needs  : X, Y locations
// Output : Current
/******************************************************************/
void panCur_A(int first_col, int first_line) {
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf("%c%5.2f%c", 0xE4, osd_curr_A * .01, 0x8F);
    osd.closePanel();
}


/******************************************************************/
// Panel  : panBatteryPercent
// Needs  : X, Y locations
// Output : Battery
//          (if defined FLIGHT_BATT_ON_MINIMOSD then not percent but consumed mAh)
/******************************************************************/
void panBatteryPercent(int first_col, int first_line) {
    osd.setPanel(first_col, first_line);
    osd.openPanel();
#ifdef FLIGHT_BATT_ON_MINIMOSD
    osd.printf("%6i%c", osd_total_A, 0x82);
#else
    osd.printf("%c%3.0i%c", 0xB9, osd_battery_remaining_A, 0x25);
#endif
    osd.closePanel();
}


/******************************************************************/
// Panel  : panTime
// Needs  : X, Y locations
// Output : Time from bootup or start
/******************************************************************/
void panTime(int first_col, int first_line) {
    int start_time;

#ifdef JR_SPECIALS	// Time restarts with 00:00 when measured current > TIME_RESET_AMPERE for the 1st time
    static unsigned long engine_start_time = 0;
    
    if (engine_start_time == 0 && osd_curr_A > TIME_RESET_AMPERE * 100) {
        engine_start_time = millis();
    }
    start_time = (int) ((millis() - engine_start_time) / 1000);
    
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf("%c%2i%c%02i", 0xB3, ((int)(start_time/60))%60, 0x3A, start_time%60);
    osd.closePanel();
#else
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    start_time = (int) (millis() / 1000);
    osd.printf("%c%2i%c%02i", 0xB3, ((int)(start_time/60))%60, 0x3A, start_time%60);
    osd.closePanel();
#endif
}


/******************************************************************/
// Panel  : panHorizon
// Needs  : X, Y locations
// Output : artificial horizon
/******************************************************************/
void panHorizon(int first_col, int first_line) {
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf_P(PSTR("\xc8\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\xc9|"));
    osd.printf_P(PSTR("\xc8\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\xc9|"));
    osd.printf_P(PSTR("\xd8\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\xd9|"));
    osd.printf_P(PSTR("\xc8\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\xc9|"));
    osd.printf_P(PSTR("\xc8\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\xc9"));
    osd.closePanel();
    showHorizon((first_col + 1), first_line);
}


#if 0
/******************************************************************/
// Panel  : panOtherUAV
// Needs  : X, Y locations
// Needs  : globals: oUAV_lat, oUAV_lon, oUAV_alt, osd_heading
// Output : shows some mystic info
// Size   : 3 x 6  (rows x chars)
// Status : just an idea and not to forget
//          not compiled
//          not tested
//          not ready
// ToDo   : refactor with function setHomeVars
/******************************************************************/
void panOtherUAV(int first_col, int first_line) {
    float dstlon, dstlat;
    long oUAV_distance;
    long oUAV_bearing;
    uint8_t oUAV_direction;
    
    // shrinking factor for longitude going to poles direction
    float rads = fabs(oUAV_lat) * 0.0174532925;
    double scaleLongDown = cos(rads);
    double scaleLongUp   = 1.0f/cos(rads);
   
    // DST to oUAV
    dstlat = fabs(oUAV_lat - osd_lat) * 111319.5;
    dstlon = fabs(oUAV_lon - osd_lon) * 111319.5 * scaleLongDown;
    oUAV_distance = sqrt(sq(dstlat) + sq(dstlon));
    
    // DIR to oUAV
    dstlon = (oUAV_lon - osd_lon);					// OffSet X
    dstlat = (oUAV_lat - osd_lat) * scaleLongUp;			// OffSet Y
    oUAV_bearing = 90 + (atan2(dstlat, -dstlon) * 57.295775);		// absolut oUAV direction
    if (oUAV_bearing < 0) oUAV_bearing += 360;				// normalization
    oUAV_bearing = oUAV_bearing - 180;					// absolut goal direction
    if (oUAV_bearing < 0) oUAV_bearing += 360;				// normalization
    oUAV_bearing = oUAV_bearing - osd_heading;				// relative oUAV direction
    if (oUAV_bearing < 0) oUAV_bearing += 360;				// normalization
    oUAV_direction = round((float)(oUAV_bearing/360.0f) * 16.0f) + 1;	// array of arrows
    if (oUAV_direction > 16) oUAV_direction = 0;
    
    osd.setPanel(first_col, first_line);
    osd.openPanel();
    osd.printf("D%4.0f%c|", (double)((float)(oUAV_distance) * convert_length), unit_length);
    osd.printf("A%4.0f%c|  ", (double)((oUAV_alt - osd_alt) * convert_length), unit_length);
    showArrow((uint8_t)oUAV_direction);
    osd.closePanel();
}
#endif


/******* HELPER FUNCTIONS *******/


// Setup change function
int change_int_val(int value, int address, int delta) {
    int value_old = value;
    
    osd.printf_P(PSTR("|                   "));
    switch (delta) {
        case 100:
            osd.printf_P(PSTR("\x5E"));
	break;
        case 10:
            osd.printf_P(PSTR(" \x5E"));
	break;
        case 1:
            osd.printf_P(PSTR("  \x5E"));
	break;
    }
		
    if (chan1_raw > chan1_raw_middle + PWM_OFFSET) value -= delta;
    else if (chan1_raw < chan1_raw_middle - PWM_OFFSET) value += delta;

    if (value != value_old && setup_menu_active) {
	EEPROM.write(address, value&0xff);
	EEPROM.write(address+1, (value>>8)&0xff);
    }
    return value;
}


// Setup change function
int change_val(int value, int address) {
    uint8_t value_old = value;
    
    if (chan1_raw > chan1_raw_middle + PWM_OFFSET) value--;
    else if (chan1_raw < chan1_raw_middle - PWM_OFFSET) value++;

    if (value != value_old && setup_menu_active) EEPROM.write(address, value);
    return value;
}


// Show those fancy 2 char arrows
void showArrow(uint8_t rotate_arrow) {  
    char arrow_set1 = 0x0;
    char arrow_set2 = 0x0;   
    switch (rotate_arrow) {
    case 0: 
        arrow_set1 = 0x90;
        arrow_set2 = 0x91;
        break;
    case 1: 
        arrow_set1 = 0x90;
        arrow_set2 = 0x91;
        break;
    case 2: 
        arrow_set1 = 0x92;
        arrow_set2 = 0x93;
        break;
    case 3: 
        arrow_set1 = 0x94;
        arrow_set2 = 0x95;
        break;
    case 4: 
        arrow_set1 = 0x96;
        arrow_set2 = 0x97;
        break;
    case 5: 
        arrow_set1 = 0x98;
        arrow_set2 = 0x99;
        break;
    case 6: 
        arrow_set1 = 0x9A;
        arrow_set2 = 0x9B;
        break;
    case 7: 
        arrow_set1 = 0x9C;
        arrow_set2 = 0x9D;
        break;
    case 8: 
        arrow_set1 = 0x9E;
        arrow_set2 = 0x9F;
        break;
    case 9: 
        arrow_set1 = 0xA0;
        arrow_set2 = 0xA1;
        break;
    case 10: 
        arrow_set1 = 0xA2;
        arrow_set2 = 0xA3;
        break;
    case 11: 
        arrow_set1 = 0xA4;
        arrow_set2 = 0xA5;
        break;
    case 12: 
        arrow_set1 = 0xA6;
        arrow_set2 = 0xA7;
        break;
    case 13: 
        arrow_set1 = 0xA8;
        arrow_set2 = 0xA9;
        break;
    case 14: 
        arrow_set1 = 0xAA;
        arrow_set2 = 0xAB;
        break;
    case 15: 
        arrow_set1 = 0xAC;
        arrow_set2 = 0xAd;
        break;
    case 16: 
        arrow_set1 = 0xAE;
        arrow_set2 = 0xAF;
        break;
    } 
    osd.printf("%c%c", arrow_set1, arrow_set2);
}


// Calculate and shows artificial horizon
void showHorizon(int start_col, int start_row) { 

    int x, nose, row, minval, hit, subval = 0;
    const int cols = 12;
    const int rows = 5;
    int col_hit[cols];
    float  pitch, roll;

    (abs(osd_pitch) == 90) ? pitch = 89.99 * (90/osd_pitch) * -0.017453293 : pitch = osd_pitch * -0.017453293;
    (abs(osd_roll)  == 90) ? roll =  89.99 * (90/osd_roll)  *  0.017453293 : roll =  osd_roll  *  0.017453293;

    nose = round(tan(pitch) * (rows * 9));
    for (int col=1; col<=cols; col++) {
        x = (col * 12) - (cols * 6) - 6;				// center X point at middle of each col
        col_hit[col-1] = (tan(roll) * x) + nose + (rows*9) - 1;		// calculating hit point on Y plus offset to eliminate negative values
    }

    for (int col=0; col<cols; col++) {
        hit = col_hit[col];
        if (hit > 0 && hit < (rows * 18)) {
            row = rows - ((hit-1) / 18);
            minval = rows * 18 - row * 18 + 1;
            subval = hit - minval;
            subval = round((subval * 9) / 18);
            if (subval == 0) subval = 1;
            printHit(start_col + col, start_row + row - 1, subval);
        }
    }
}


void printHit(byte col, byte row, byte subval) {
    osd.openSingle(col, row);
    char subval_char;
        switch (subval) {
        case 1:
            subval_char = 0x06;
            break;
        case 2:
            subval_char = 0x07; 
            break;
        case 3:
            subval_char = 0x08;
            break;
        case 4:
            subval_char = 0x09;
            break;
        case 5:
            subval_char = 0x0a; 
            break;
        case 6:
            subval_char = 0x0b;
            break;
        case 7:
            subval_char = 0x0c;
            break;
        case 8:
            subval_char = 0x0d;
            break;
        case 9:
            subval_char = 0x0e;
            break;
        }
        osd.printf("%c", subval_char);
}


void set_converts() {
    if (EEPROM.read(measure_ADDR) == 0) {
        convert_speed = 3.6;
        convert_length = 1.0;
        unit_speed = 0x81;
        unit_length = 0x8D;
    } else {
        convert_speed = 2.23;
        convert_length = 3.28;
        unit_speed = 0xfb;
        unit_length = 0x66;
    }
}
