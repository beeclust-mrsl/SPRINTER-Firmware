#include	"gcode_process.h"

/** \file
	\brief Work out what to do with received G-Code commands
*/

#include	<string.h>

#include	"gcode_parse.h"

#include 	"cpu.h"
#include	"dda.h"
#include	"dda_queue.h"
#include	"watchdog.h"
#include	"delay.h"
#include	"serial.h"
#include	"timer.h"
#include	"sersendf.h"
#include	"pinio.h"
#include	"debug.h"
#include	"clock.h"
#include	"config_wrapper.h"
#include	"home.h"
#include	"Arduino.h"

/// the current tool
uint8_t tool;
uint8_t led;

/// the tool to be changed when we get an M6
uint8_t next_tool;

uint16_t strip;
unsigned long last_inkjet;

/************************************************************************//**

  \brief Processes command stored in global \ref next_target.
  This is where we work out what to actually do with each command we
    receive. All data has already been scaled to integers in gcode_process.
    If you want to add support for a new G or M code, this is the place.


*//*************************************************************************/

void process_gcode_command() {
	uint32_t	backup_f;

	// convert relative to absolute
	if (next_target.option_all_relative) {
    next_target.target.axis[X] += startpoint.axis[X];
    next_target.target.axis[Y] += startpoint.axis[Y];
    next_target.target.axis[Z] += startpoint.axis[Z];
	}

	// E relative movement.
	// Matches Sprinter's behaviour as of March 2012.
	if (next_target.option_e_relative)
		next_target.target.e_relative = 1;
	else
		next_target.target.e_relative = 0;

	if (next_target.option_all_relative && !next_target.option_e_relative)
		next_target.target.axis[E] += startpoint.axis[E];

	// implement axis limits
	#ifdef	X_MIN
    if (next_target.target.axis[X] < (int32_t)(X_MIN * 1000.))
      next_target.target.axis[X] = (int32_t)(X_MIN * 1000.);
	#endif
	#ifdef	X_MAX
    if (next_target.target.axis[X] > (int32_t)(X_MAX * 1000.))
      next_target.target.axis[X] = (int32_t)(X_MAX * 1000.);
	#endif
	#ifdef	Y_MIN
    if (next_target.target.axis[Y] < (int32_t)(Y_MIN * 1000.))
      next_target.target.axis[Y] = (int32_t)(Y_MIN * 1000.);
	#endif
	#ifdef	Y_MAX
    if (next_target.target.axis[Y] > (int32_t)(Y_MAX * 1000.))
      next_target.target.axis[Y] = (int32_t)(Y_MAX * 1000.);
	#endif
	#ifdef	Z_MIN
    if (next_target.target.axis[Z] < (int32_t)(Z_MIN * 1000.))
      next_target.target.axis[Z] = (int32_t)(Z_MIN * 1000.);
	#endif
	#ifdef	Z_MAX
    if (next_target.target.axis[Z] > (int32_t)(Z_MAX * 1000.))
      next_target.target.axis[Z] = (int32_t)(Z_MAX * 1000.);
	#endif

	// The GCode documentation was taken from http://reprap.org/wiki/Gcode .

	if (next_target.seen_T) {
	    //? --- T: Select Tool ---
	    //?
	    //? Example: T1
	    //?
	    //? Select extruder number 1 to build with.  Extruder numbering starts at 0.

	    next_tool = next_target.T;
	}

	if (next_target.seen_G) {
		uint8_t axisSelected = 0;

		switch (next_target.G) {
			case 0:
				//? G0: Rapid Linear Motion
				//?
				//? Example: G0 X12
				//?
				//? In this case move rapidly to X = 12 mm.  In fact, the RepRap firmware uses exactly the same code for rapid as it uses for controlled moves (see G1 below), as - for the RepRap machine - this is just as efficient as not doing so.  (The distinction comes from some old machine tools that used to move faster if the axes were not driven in a straight line.  For them G0 allowed any movement in space to get to the destination as fast as possible.)
				//?
				backup_f = next_target.target.F;
				next_target.target.F = MAXIMUM_FEEDRATE_X * 2L;
				enqueue(&next_target.target);
				next_target.target.F = backup_f;
				break;

			case 1:
				//? --- G1: Linear Motion at Feed Rate ---
				//?
				//? Example: G1 X90.6 Y13.8 E22.4
				//?
				//? Go in a straight line from the current (X, Y) point to the point (90.6, 13.8), extruding material as the move happens from the current extruded length to a length of 22.4 mm.
				//?
				enqueue(&next_target.target);
				break;

				//	G2 - Arc Clockwise
				// unimplemented

				//	G3 - Arc Counter-clockwise
				// unimplemented

			case 4:
				//? --- G4: Dwell ---
				//?
				//? Example: G4 P200
				//?
				//? In this case sit still doing nothing for 200 milliseconds.  During delays the state of the machine (for example the temperatures of its extruders) will still be preserved and controlled.
				//?
				queue_wait();
				// delay
				if (next_target.seen_P) {
					for (;next_target.P > 0;next_target.P--) {
						clock();
						delay_ms(1);
					}
				}
				break;

			case 20:
				//? --- G20: Set Units to Inches ---
				//?
				//? Example: G20
				//?
				//? Units from now on are in inches.
				//?
				next_target.option_inches = 1;
				break;

			case 21:
				//? --- G21: Set Units to Millimeters ---
				//?
				//? Example: G21
				//?
				//? Units from now on are in millimeters.  (This is the RepRap default.)
				//?
				next_target.option_inches = 0;
				break;

			case 28:
				//? --- G28: Home ---
				//?
				//? Example: G28
				//?
        		//? This causes the RepRap machine to search for its X, Y and Z
        		//? endstops. It does so at high speed, so as to get there fast. When
        		//? it arrives it backs off slowly until the endstop is released again.
        		//? Backing off slowly ensures more accurate positioning.
        		//? If you add axis characters, then just the axes specified will be
        		//? seached. Thus
        		//?   G28 X Y72.3
        		//? will zero the X and Y axes, but not Z. Coordinate values are
        		//? ignored.

				queue_wait();

				if (next_target.seen_X) {
					#if defined	X_MIN_PIN
						home_x_negative();
					#elif defined X_MAX_PIN
						home_x_positive();
					#endif
					axisSelected = 1;
				}
				if (next_target.seen_Y) {
					#if defined	Y_MIN_PIN
						home_y_negative();
					#elif defined Y_MAX_PIN
						home_y_positive();
					#endif
					axisSelected = 1;
				}
				if (next_target.seen_Z) {
          			#if defined Z_MIN_PIN
          			  home_z_negative();
          			#elif defined Z_MAX_PIN
          			  home_z_positive();
					#endif
					axisSelected = 1;
				}
				// there's no point in moving E, as E has no endstops

				if (!axisSelected) {
					home();
				}
				break;

			case 90:
				//? --- G90: Set to Absolute Positioning ---
				//?
				//? Example: G90
				//?
				//? All coordinates from now on are absolute relative to the origin
				//? of the machine. This is the RepRap default.
				//?
				//? If you ever want to switch back and forth between relative and
				//? absolute movement keep in mind, X, Y and Z follow the machine's
				//? coordinate system while E doesn't change it's position in the
				//? coordinate system on relative movements.
				//?

				// No wait_queue() needed.
				next_target.option_all_relative = 0;
				break;

			case 91:
				//? --- G91: Set to Relative Positioning ---
				//?
				//? Example: G91
				//?
				//? All coordinates from now on are relative to the last position.
				//?

				// No wait_queue() needed.
				next_target.option_all_relative = 1;
				break;

			case 92:
				//? --- G92: Set Position ---
				//?
				//? Example: G92 X10 E90
				//?
				//? Allows programming of absolute zero point, by reseting the current position to the values specified.  This would set the machine's X coordinate to 10, and the extrude coordinate to 90. No physical motion will occur.
				//?

				queue_wait();

				if (next_target.seen_X) {
          			startpoint.axis[X] = next_target.target.axis[X];
					axisSelected = 1;
				}
				if (next_target.seen_Y) {
          			startpoint.axis[Y] = next_target.target.axis[Y];
					axisSelected = 1;
				}
				if (next_target.seen_Z) {
          			startpoint.axis[Z] = next_target.target.axis[Z];
					axisSelected = 1;
				}
				if (next_target.seen_E) {
          			startpoint.axis[E] = next_target.target.axis[E];
					axisSelected = 1;
				}

				if (axisSelected == 0) {
          			startpoint.axis[X] = next_target.target.axis[X] =
          			startpoint.axis[Y] = next_target.target.axis[Y] =
          			startpoint.axis[Z] = next_target.target.axis[Z] =
          			startpoint.axis[E] = next_target.target.axis[E] = 0;
				}

				dda_new_startpoint();
				break;

			case 161:
				//? --- G161: Home negative ---
				//?
				//? Find the minimum limit of the specified axes by searching for the limit switch.
				//?
        		#if defined X_MIN_PIN
        		  if (next_target.seen_X)
        		    home_x_negative();
        		#endif
        		#if defined Y_MIN_PIN
        		  if (next_target.seen_Y)
        		    home_y_negative();
        		#endif
        		#if defined Z_MIN_PIN
        		  if (next_target.seen_Z)
        		    home_z_negative();
        		#endif
				break;

			case 162:
				//? --- G162: Home positive ---
				//?
				//? Find the maximum limit of the specified axes by searching for the limit switch.
				//?
        		#if defined X_MAX_PIN
        		  if (next_target.seen_X)
        		    home_x_positive();
        		#endif
        		#if defined Y_MAX_PIN
        		  if (next_target.seen_Y)
        		    home_y_positive();
        		#endif
        		#if defined Z_MAX_PIN
        		  if (next_target.seen_Z)
        		    home_z_positive();
        		#endif
				break;

			// unknown gcode: spit an error
			default:
				sersendf_P(PSTR("E: Bad G-code %d\n"), next_target.G);
				return;
		}
	}
	else if (next_target.seen_M) {
		switch (next_target.M) {
			case 0:
				//? --- M0: machine stop ---
				//?
				//? Example: M0
				//?
				//? http://linuxcnc.org/handbook/RS274NGC_3/RS274NGC_33a.html#1002379
				//? Unimplemented, especially the restart after the stop. Fall trough to M2.
				//?

			case 2:
			case 84:
				//? --- M2: program end ---
				//?
				//? Example: M2
				//?
				//? http://linuxcnc.org/handbook/RS274NGC_3/RS274NGC_33a.html#1002379
				//?
				queue_wait();
				power_off();
        		serial_writestr_P(PSTR("\nstop\n"));
				break;

			case 82:
				//? --- M82 - Set E codes absolute ---
				//?
				//? This is the default and overrides G90/G91.
				//? M82/M83 is not documented in the RepRap wiki, behaviour
				//? was taken from Sprinter as of March 2012.
				//?
				//? While E does relative movements, it doesn't change its
				//? position in the coordinate system. See also comment on G90.
				//?

				// No wait_queue() needed.
				next_target.option_e_relative = 0;
				break;

			case 83:
				//? --- M83 - Set E codes relative ---
				//?
				//? Counterpart to M82.
				//?

				// No wait_queue() needed.
				next_target.option_e_relative = 1;
				break;

			case 110:
				//? --- M110: Set Current Line Number ---
				//?
				//? Example: N123 M110
				//?
				//? Set the current line number to 123.  Thus the expected next line after this command will be 124.
				//? This is a no-op in Teacup.
				//?
				break;

      		#ifdef DEBUG
				case 111:
					//? --- M111: Set Debug Level ---
					//?
					//? Example: M111 S6
					//?
					//? Set the level of debugging information transmitted back to the host to level 6.  The level is the OR of three bits:
					//?
					//? <Pre>
					//? #define         DEBUG_PID       1
					//? #define         DEBUG_DDA       2
					//? #define         DEBUG_POSITION  4
					//? </pre>
					//?
					//? This command is only available in DEBUG builds of Teacup.
	
					if ( ! next_target.seen_S)
						break;
					
					debug_flags = next_target.S;
					break;
      		#endif /* DEBUG */

      		case 112:
        		//? --- M112: Emergency Stop ---
        		//?
        		//? Example: M112
        		//?
        		//? Any moves in progress are immediately terminated, then the printer
        		//? shuts down. All motors and heaters are turned off. Only way to
        		//? restart is to press the reset button on the master microcontroller.
        		//? See also M0.
        		//?
        		timer_stop();
        		queue_flush();
        		power_off();
        		cli();
        		for (;;)
          		wd_reset();
        		break;

			case 114:
				//? --- M114: Get Current Position ---
				//?
				//? Example: M114
				//?
				//? This causes the RepRap machine to report its current X, Y, Z and E coordinates to the host.
				//?
				//? For example, the machine returns a string such as:
				//?
				//? <tt>ok C: X:0.00 Y:0.00 Z:0.00 E:0.00</tt>
				//?
				#ifdef ENFORCE_ORDER
					// wait for all moves to complete
					queue_wait();
				#endif
				update_current_position();
				sersendf_P(PSTR("X:%lq,Y:%lq,Z:%lq,E:%lq,F:%lu\n"),
                        current_position.axis[X], current_position.axis[Y],
                        current_position.axis[Z], current_position.axis[E],
				        current_position.F);

        		if (mb_tail_dda != NULL) {
          			if (DEBUG_POSITION && (debug_flags & DEBUG_POSITION)) {
            			sersendf_P(PSTR("Endpoint: X:%ld,Y:%ld,Z:%ld,E:%ld,F:%lu,c:%lu}\n"),
                       	mb_tail_dda->endpoint.axis[X],
                       	mb_tail_dda->endpoint.axis[Y],
                       	mb_tail_dda->endpoint.axis[Z],
                       	mb_tail_dda->endpoint.axis[E],
                       	mb_tail_dda->endpoint.F,
                       	#ifdef ACCELERATION_REPRAP
                         	mb_tail_dda->end_c
                       	#else
                        	mb_tail_dda->c
                       	#endif
            			);
          			}
          			print_queue();
        		}

				break;

			case 115:
				//? --- M115: Get Firmware Version and Capabilities ---
				//?
				//? Example: M115
				//?
				//? Request the Firmware Version and Capabilities of the current microcontroller
				//? The details are returned to the host computer as key:value pairs separated by spaces and terminated with a linefeed.
				//?
				//? sample data from firmware:
				//?  FIRMWARE_NAME:Teacup FIRMWARE_URL:http://github.com/traumflug/Teacup_Firmware/ PROTOCOL_VERSION:1.0 MACHINE_TYPE:Mendel EXTRUDER_COUNT:1 TEMP_SENSOR_COUNT:1 HEATER_COUNT:1
				//?

        		sersendf_P(PSTR("FIRMWARE_NAME:SPRINTER-Firmware "
                    "FIRMWARE_URL:http://github.com/Beeclust-MRSL/SPRINTER-Firmware"
                    "PROTOCOL_VERSION:1.0 MACHINE_TYPE: SPRINTER V5.0"));
				break;

      		case 220:
        		//? --- M220: Set speed factor override percentage ---
        		if ( ! next_target.seen_S)
        		  break;
        		// Scale 100% = 256
        		next_target.target.f_multiplier = (next_target.S * 64 + 12) / 25;
        		break;


      		#ifdef DEBUG
				case 240:
					//? --- M240: echo off ---
					//? Disable echo.
					//? This command is only available in DEBUG builds.
					debug_flags &= ~DEBUG_ECHO;
					serial_writestr_P(PSTR("Echo off\n"));
					break;
		
				case 241:
					//? --- M241: echo on ---
					//? Enable echo.
					//? This command is only available in DEBUG builds.
					debug_flags |= DEBUG_ECHO;
					serial_writestr_P(PSTR("Echo on\n"));
					break;
      		#endif /* DEBUG */


			case 240:

				if (next_target.seen_S){
					strip = next_target.S;
					while(last_inkjet + 800 > micros());

					for(uint8_t i = 0; i <= 11; i++){
	
	              		//See if nozzle is set to fire
	              		if(strip & 1<<i){

                			//Write the nozzle number to the pin shield as 4 bits
                			if(i & 1<<0)
                			  	WRITE(INK_PINA, 1);
                			if(i & 1<<1)
                			  	WRITE(INK_PINB, 1);
                			if(i & 1<<2)
                			  	WRITE(INK_PINC, 1);
                			if(i & 1<<3)
                			  	WRITE(INK_PIND, 1);
	
	                		//Fire the Nozzle
	                		WRITE(INK_PULSE, 1);
			
			                delay_us(5);
			
			                //Set everything low
			               	WRITE(INK_PULSE, 0);
			
			                WRITE(INK_PINA, 0);
			                WRITE(INK_PINB, 0);
			                WRITE(INK_PINC, 0);
	                		WRITE(INK_PIND, 0);
              			}

              			last_inkjet = micros();
					}
				}

				break;

			case 250:

				if (next_target.seen_S){
					led = next_target.S;

					if(led == 0)
						WRITE(LED_DEBUG, 0);
					else if(led == 1)
						WRITE(LED_DEBUG, 1);
				}

				break;


			// unknown mcode: spit an error
			default:
				sersendf_P(PSTR("E: Bad M-code %d\n"), next_target.M);
		} // switch (next_target.M)
	} // else if (next_target.seen_M)
} // process_gcode_command()
