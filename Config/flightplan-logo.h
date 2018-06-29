// This file is part of MatrixPilot.
//
//    http://code.google.com/p/gentlenav/
//
// Copyright 2009-2012 MatrixPilot Team
// See the AUTHORS.TXT file for a list of authors of MatrixPilot.
//
// MatrixPilot is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// MatrixPilot is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with MatrixPilot.  If not, see <http://www.gnu.org/licenses/>.


////////////////////////////////////////////////////////////////////////////////
// UDB LOGO Flight Planning definitions
// 
// The UDB Logo flight plan language lets you use a language similar to Logo, aka Turtle graphics, to
// control your plane.  You are commanding an imaginary "turtle" to move to specific locations, and the
// plane will head towards the turtle.
// 
// You can also control the camera targeting code by switching from the plane turtle, to the camera turtle
// by using the SET_TURTLE(CAMERA) command.  Then logo commands will move the location that the camera
// is targeting, instead of the location to aim the plane.
// 
// Each time you enter waypoint mode, the state is reset and your logo program starts from the top.  If
// you enter RTL mode, the state is reset and your RTL logo program is run instead.
// The following state is cleared when entering waypoint mode or RTL mode: (but not when your program
// ends and starts over)
//   - The plane turtle and camera turtle begin at the plane's current position and altitude.
//   - Both turtles begin pointing in the plane's current heading.
//   - The flags are all turned off.
//   - The pen is down, and the PLANE turtle is active.
// 
// To use UDB Logo, set FLIGHT_PLAN_TYPE to FP_LOGO in options.h.

//Personal option
#define LOG_WAYPOINT_PER_SUBROUTINE     1  // Odd numbered Logo subroutines are logged as a waypoint. Recommended for THERMALLING_MISSION

//NOTE: WAYPOINT_PROXIMITY_RADIUS, USE_FIXED_ORIGIN, FIXED_ORIGIN_LOCATION are now defined in options.h
////////////////////////////////////////////////////////////////////////////////
// Waypoint handling

// Move on to the next waypoint when getting within this distance of the current goal (in meters)
// e.g. in options.h you may find the following #define WAYPOINT_PROXIMITY_RADIUS	25

// Origin Location
// When using relative waypoints, the default is to interpret those waypoints as relative to the
// plane's power-up location.  Here you can choose to use any specific, fixed 3D location as the
// origin point for your relative waypoints.
//
// USE_FIXED_ORIGIN should be 0 to use the power-up location as the origin for relative waypoints.
// Set it to 1 to use a fixed location as the origin, no matter where you power up.
// FIXED_ORIGIN_LOCATION is the location to use as the origin for relative waypoints.  It uses the
// format { X, Y, Z } where:
// X is Longitude in degrees * 10^7
// Y is Latitude in degrees * 10^7
// Z is altitude above sea level, in meters, as a floating point value.
// 
// If you are using waypoints for an autonomous landing, it is a good idea to set the altitude value
// to be the altitude of the landing point, and then express the heights of all of the waypoints with
// respect to the landing point.
// If you are using OpenLog, an easy way to determine the altitude of your landing point is to
// examine the telemetry after a flight, take a look in the .csv file, it will be easy to spot the
// altitude, expressed in meters.

// For example in options.h you may find:-
//      #define USE_FIXED_ORIGIN        0
//      #define FIXED_ORIGIN_LOCATION   { 113480854, 472580108, 578 }    // Innsbruck

////////////////////////////////////////////////////////////////////////////////
// Commands
// 
// Use the following commands to create your logo paths:
// 
// HOME                 - Return the turtle to the origin, aiming North.
// 
// FD(x)                - Move the turtle forward x meters, in the turtle's current direction.
// BK(x)                - Move the turtle backwards x meters, in the turtle's current direction.
// USE_CURRENT_POS      - Move the turtle to the plane's current {X,Y} position.  Mostly useful
//                        while being sneaky using PEN_UP.

// RT(x)                - Rotate the turtle to the right by x degrees.
// LT(x)                - Rotate the turtle to the left by x degrees.
// FIXED_BANK_ROTATE              - Perform roll to a fixed bank x deg for 30 deg heading change to the right and fly on for ~2 sec, position/navigation will be ignored (THERMALLING_MISSION)
// BANK_1S              - Perform level flight for 1 second, position/navigation will be ignored. This is used for centering in thermals (THERMALLING_MISSION)
// SET_ANGLE(x)         - Set the turtle to point x degrees clockwise from N.
// USE_CURRENT_ANGLE    - Aim the turtle in the direction that the plane is currently headed.
// USE_ANGLE_TO_GOAL    - Aim the turtle in the direction of the goal from the location of the plane.

// EAST(x)              - Move the turtle x meters East.
// WEST(x)              - Move the turtle x meters West.
// SET_X_POS(x)         - Set the X value of the turtle (meters East of the origin) to x.
// 
// NORTH(y)             - Move the turtle y meters North.
// SOUTH(y)             - Move the turtle y meters South.
// SET_Y_POS(y)         - Set the Y value of the turtle (meters North of the origin) to y.
// 
// SET_POS(x, y)        - Set both x and y at the same time.
// SET_ABS_POS(x, y)    - Set absolute X,Y location (long,lat) in degrees * 10^7

// ALT_UP(z)            - Gain z meters of altitude.
// ALT_DOWN(z)          - Drop z meters of altitude.
// SET_ALT(z)           - Set altitude to z.

// SPEED_INCREASE(x)    - Increases the target speed by x dm/s
// SPEED_DECREASE(x)    - Decreases the target speed by x dm/s
// SET_SPEED(x)         - Sets the target speed to x dm/s

// REPEAT(n)            - Repeat all of the instructions until the matching END, n times
// REPEAT_FOREVER       - Repeat all of the instructions until the matching END, forever
// END                  - End the current REPEAT loop or Subroutine definition

// IF_EQ(val, x)        - Looks up a system value (listed below) and checks if it's equal to x.
//                        If so, runs commands until reaching ELSE or END.  If not, skips to ELSE 
//                        and runs until END, or just skips to END if there's no ELSE.
//                        Available IF commands: IF_EQ(equal), IF_NE(not equal), 
//                        IF_GT(val>x), IF_LT(val<x),IF_GE(val>=x), IF_LE(val<=x).
// ELSE                 - Starts a list of commands that get run if the preceding IF failed.

// PEN_UP               - While the pen is up, logo code execution does not stop to wait for the
//                        plane to move to each new position of the turtle before continuing.
//                        This allows you to use multiple logo instructions to get the turtle to
//                        the next goal location before commanding the plane to fly there by 
//                        putting the pen back down.
// PEN_DOWN             - When the pen is down, the plane moves to each new position of the turtle
//                        before more logo instructions are interpereted.
// PEN_TOGGLE           - Toggle the pen between up and down.

// SET_TURTLE(T)        - Choose to control either the plane's turtle, or the camera turtle.
//                        Use either SET_TURTLE(PLANE) or SET_TURTLE(CAMERA).


// Commands for Modifying Flags
// 
// FLAG_ON(F)           - Turn on flag F.  (See below for a list of flags.)
// FLAG_OFF(F)          - Turn off flag F.
// FLAG_TOGGLE(F)       - Toggle flag F.
// 
// The supported flags are the following:
// 
// F_TAKEOFF            - More quickly gain altitude at takeoff.
// F_INVERTED           - Fly with the plane upside down. (only if STABILIZE_INVERTED_FLIGHT is set to 1 in options.h)
// F_HOVER              - Hover the plane with the nose up. (only if STABILIZE_HOVER is set to 1 in options.h)
//                        NOTE: while hovering, no navigation is performed, and throttle is under manual control.
// F_TRIGGER            - Trigger an action to happen at this point in the flight.  (See the Trigger Action section of the options.h file.)
// F_ALTITUDE_GOAL      - Climb or descend to the given altitude.
// F_CROSS_TRACK        - Navigate using cross-tracking.  Best used for longer flight legs.
// F_LAND               - Fly with the throttle off.


// Commands for Creating and Calling Subroutines
//
// TO(FUNC)             - Begin defining subroutine FUNC (requires #define FUNC N where N is an
//                        integer, unique among your defined subroutines.  End each subroutine
//                        definition with an END.
// DO(FUNC)             - Run subroutine FUNC.  When it finishes, control returns to the line after
//                        the DO() instruction.
// EXEC(FUNC)           - Call FUNC as though it were the beginning of the logo program.  This will never return.
//                        When/if FUNC finishes, logo will start back at the beginning of the program.
// DO(LOGO_MAIN) or
// EXEC(LOGO_MAIN)      - Restart at the top of the LOGO program
// DO_ARG(FUNC, PARAM)  - Run subroutine FUNC, using an integer value as a parameter.
// EXEC_ARG(FUNC, PARAM)- Exec subroutine FUNC, using an integer value as a parameter.
// 
// FD_PARAM             - From within a subroutine, call the FD command using the parameter
//                        passed to this subroutine as the distance.
// RT_PARAM             - From within a subroutine, call the RT command using the parameter
//                        passed to this subroutine as the angle.
// REPEAT_PARAM         - Start a REPEAT block, using the current subroutine's parameter as the
//                        number of times to repeat.
// DO_PARAM(FUNC)       - Call subroutine FUNC with a parameter equal to the current subroutine's
//                        parameter value.
// 
// PARAM_ADD(x)         - Adds x to the current subroutine's current parameter value.  Fun
//                        inside repeats inside subroutines!
// PARAM_SUB(x)         - Subtracts x from the current subroutine's current parameter value.
// PARAM_MUL(x)         - Multiplies the current subroutine's current parameter value by x.
// PARAM_DIV(x)         - Divides the current subroutine's current parameter value by x.
// PARAM_SET(x)         - Sets the current subroutine's current parameter value to x.
// 
// LOAD_TO_PARAM(val)   - Loads a system value (listed below) into the current subroutine's parameter value.
//
// All parameter-related commands: 
//        FD_PARAM, BK_PARAM, RT_PARAM, LT_PARAM, SET_ANGLE_PARAM, 
//        EAST_PARAM, WEST_PARAM, NORTH_PARAM, SOUTH_PARAM, ALT_UP_PARAM, ALT_DOWN_PARAM, 
//        SET_X_POS_PARAM, SET_Y_POS_PARAM, SET_ALT_PARAM, 
//        SPEED_INCREASE_PARAM, SPEED_DECREASE_PARAM, SET_SPEED_PARAM
//        REPEAT_PARAM, DO_PARAM(FUNC), EXEC_PARAM(FUNC)
//        PARAM_SET(x), PARAM_ADD(x), PARAM_SUB(x), PARAM_MUL(x), PARAM_DIV(x)
//        IF_EQ_PARAM(x), IF_NE_PARAM(x), IF_GT_PARAM(x), IF_LT_PARAM(x), IF_GE_PARAM(x), IF_LE_PARAM(x)


// SET_INTERRUPT(f)     - Sets a user-defined logo function to be called at 40Hz.  Be careful not to modify
//                        the turtle location from within your interrupt function unless you really want to!
//                        Usually you'll just want your interrupt function to check some condition, and do
//                        something only if it's true.  (Like fly home only if you get too far away.)
// CLEAR_INTERRUPT      - Clears/disables the interrupt function.  Not usually needed.


// System Values for use with LOAD_TO_PARAM(val) and IF_XX() commands
// 
// DIST_TO_HOME         - in m
// DIST_TO_GOAL         - in m
// ALT                  - in m
// CURRENT_ANGLE        - in degrees. 0-359 (clockwise, 0=North)
// ANGLE_TO_HOME        - in degrees. 0-359 (clockwise, 0=North)
// ANGLE_TO_GOAL        - in degrees. 0-359 (clockwise, 0=North)
// REL_ANGLE_TO_HOME    - in degrees. -180-179 (0=heading directly towards Home. Home to the right of the nose of the plane is positive)
// REL_ANGLE_TO_GOAL    - in degrees. -180-179 (0=heading directly towards Goal. Goal to the right of the nose of the plane is positive)
// REL_ANGLE_TO_OPPOSITE - in degrees. -180-179 (0=heading directly towards opposite point. (THERMALLING_MISSION)
// GROUND_SPEED         - in cm/s
// AIR_SPEED            - in cm/s
// AIR_SPEED_Z          - in cm/s
// WIND_SPEED           - in cm/s
// WIND_SPEED_X         - in cm/s
// WIND_SPEED_Y         - in cm/s
// WIND_SPEED_Z         - in cm/s
// WIND_FROM_ANGLE      - in degrees Wind from 0-359 (clockwise, 0=North)

//THERMALLING_MISSION //custom system values
//  AIR_SPEED_Z_DELTA   - in cm/s   (THERMALLING_MISSION)
// 	AIR_SPEED_Z_VS_START
// 	GEOFENCE_STATUS
// 	GEOFENCE_TURN
// 	MOTOR_OFF_TIMER
// 	READ_DESIRED_SPEED
// 	FORCE_CROSS_FINISH_LINE
// 	READ_FLY_COMMAND_COUNTER
// 	FORCE_FINISH_BAD_NAV
//  BATTERY_VOLTAGE     - in tens of volts  
//  READ_F_LAND         - read the LAND flag, 0 if off, 1 if on 
//  READ_THROTTLE_OUTPUT_CHANNEL - read the LAND flag, 0 if off, 1 if on  
//  FORCE_RESET    reset program from within interrupt programs
//THERMALLING_MISSION //custom system values

// PARAM                - current param value
// XX_INPUT_CHANNEL     - channel value from 2000-4000 (any channel defined in options.h, e.g. THROTTLE_INPUT_CHANNEL)



////////////////////////////////////////////////////////////////////////////////
// Notes:
//  - Altitudes are relative to the starting point, and the initial altitude goal is 100 meters up.
//  - All angles are in degrees.
//  - Repeat commands and subroutines can be nested up to 12-deep.
//  - If the end of the list of instructions is reached, we start over at the top from the current location and angle.
//    This does not take up one of the 12 nested repeat levels.
//  - If you use many small FD() commands to make curves, I suggest enabling cross tracking: FLAG_ON(F_CROSS_TRACK)
//  - All Subroutines have to appear after the end of your main logo program.


////////////////////////////////////////////////////////////////////////////////
// Define the main flight plan as:
// 
// #define FOO 1
// 
// const struct logoInstructionDef instructions[] = {
//		instruction1
//		instruction2
//		etc.
//		END
//
//		TO (FOO)
//			etc.
//		END
//	};
// 
// and the Failsafe RTL course as:
// 
// #define BAR 2
// 
// const struct logoInstructionDef rtlInstructions[] = {
//		instruction1
//		instruction2
//		etc.
//		END
//
//		TO (BAR)
//			etc.
//		END
//	};


#ifndef THERMALLING_MISSION

////////////////////////////////////////////////////////////////////////////////
// Main Flight Plan
//
// Fly a 100m square at an altitude of 100m, beginning above the origin, pointing North

#define SQUARE 1

const struct logoInstructionDef instructions[] = {

	SET_ALT(100)

	// Go Home and point North
	HOME

	REPEAT_FOREVER
		DO_ARG(SQUARE, 100)
	END

	TO (SQUARE)
		REPEAT(4)
			FD_PARAM
			RT(90)
		END
	END
};

////////////////////////////////////////////////////////////////////////////////
// RTL Flight Plan
// 
// On entering RTL mode, turn off the engine, fly home, and circle indefinitely until touching down

const struct logoInstructionDef rtlInstructions[] = {

	// Use cross-tracking for navigation
	FLAG_ON(F_CROSS_TRACK)

	// Turn off engine for RTL
	// Move this line down below the HOME to return home with power before circling unpowered.
	FLAG_ON(F_LAND)

	// Fly home
	HOME

	// Once we arrive home, aim the turtle in the
	// direction that the plane is already moving.
	USE_CURRENT_ANGLE

	REPEAT_FOREVER
		// Fly a circle (36-point regular polygon)
		REPEAT(36)
			RT(10)
			FD(8)
		END
	END
	
};
#endif


////////////////////////////////////////////////////////////////////////////////
// More Examples

/*
// Fly a 200m square starting at the current location and altitude, in the current direction
	REPEAT(4)
		FD(200)
		RT(90)
	END
*/

/*
// Fly a round-cornered square
	FLAG_ON(F_CROSS_TRACK)

	REPEAT(4)
		FD(170)
		REPEAT(6)
			LT(15)
			FD(10)
		END
	END
*/

/*
// Set the camera target to a point 100m North of the origin, then circle that point
	SET_TURTLE(CAMERA)
	HOME
	FD(100)
	SET_TURTLE(PLANE)

	FLAG_ON(F_CROSS_TRACK)

	HOME
	LT(90)

	REPEAT_FOREVER
		// Fly a circle (36-point regular polygon)
		REPEAT(36)
			RT(10)
			FD(20)
		END
	END
*/

/*
// Fly a giant, 2.5km diameter, 10-pointed star with external loops at each point
	FLAG_ON(F_CROSS_TRACK)

	REPEAT(10)
		FD(2000)

		REPEAT(18)
			RT(14) // == RT((180+72)/18)
			FD(50)
		END
	END
*/

/*
// Come in for an automatic landing at the HOME position
// from the current direction of the plane.
// 1. Aim for 32m altitude at 250m from HOME
// 2. Fly to 200m from HOME and turn off power
// 3. Aim for -32m altitude, 200m past home, which should
//    touch down very close to HOME.

	FLAG_ON(F_CROSS_TRACK)

	SET_ALT(32)

	PEN_UP
	HOME
	USE_ANGLE_TO_GOAL
	BK(250)
	PEN_DOWN
	FLAG_ON(F_LAND)
	PEN_UP
	HOME
	USE_ANGLE_TO_GOAL
	BK(200)
	PEN_DOWN
	SET_ALT(-32)
	PEN_UP
	HOME
	USE_ANGLE_TO_GOAL
	FD(200)
	PEN_DOWN
*/

/*
// Example of using some math on PARAM values to make cool spirals
#define SPIRAL_IN                   1
#define SPIRAL_OUT                  2
#define FWD_100_MINUS_PARAM_OVER_2  3

const struct logoInstructionDef instructions[] = {
	
DO_ARG(SPIRAL_IN, 10)
RT(100)
DO_ARG(SPIRAL_OUT,  70)

END



TO (SPIRAL_IN)
	REPEAT(30)
		DO_PARAM(FWD_100_MINUS_PARAM_OVER_2)
		RT_PARAM
		PARAM_ADD(2)
	END
END


TO (SPIRAL_OUT)
	REPEAT(30)
		PARAM_SUB(2)
		RT_PARAM
		DO_PARAM(FWD_100_MINUS_PARAM_OVER_2)
	END
END


TO (FWD_100_MINUS_PARAM_OVER_2)
	PARAM_MUL(-1)
	PARAM_ADD(100)
	PARAM_DIV(2)
	FD_PARAM
END
*/

/*
// Example of using an interrupt handler to stop the plane from getting too far away
// Notice mid-pattern if we get >200m away from home, and if so, fly home.
#define INT_HANDLER                 1

const struct logoInstructionDef instructions[] = {

SET_INTERRUPT(INT_HANDLER)

REPEAT_FOREVER
	FD(20)
	RT(10)
END

END


TO (INT_HANDLER)
	IF_GT(DIST_TO_HOME, 200)
		HOME
	END
END
*/

/*
// Example of using an interrupt handler to toggle between 2 flight plans.
// When starting the flightplan, decide whether to circle left or right, based on which direction
// initially turns towards home.  From then on, the circling direction can be changed by moving the
// rudder input channel to one side or the other.

#define CIRCLE_RIGHT                1
#define CIRCLE_LEFT                 2
#define INT_HANDLER_RIGHT           3
#define INT_HANDLER_LEFT            4

const struct logoInstructionDef instructions[] = {

IF_GT(REL_ANGLE_TO_HOME, 0)
	EXEC(CIRCLE_RIGHT)
ELSE
	EXEC(CIRCLE_LEFT)
END


TO (CIRCLE_RIGHT)
	USE_CURRENT_POS
	SET_INTERRUPT(INT_HANDLER_RIGHT)
	REPEAT_FOREVER
		FD(10)
		RT(10)
	END
END

TO (CIRCLE_LEFT)
	USE_CURRENT_POS
	SET_INTERRUPT(INT_HANDLER_LEFT)
	REPEAT_FOREVER
		FD(10)
		LT(10)
	END
END


TO (INT_HANDLER_RIGHT)
	IF_LT(RUDDER_INPUT_CHANNEL, 2800)
		EXEC(CIRCLE_LEFT)
	END
END

TO (INT_HANDLER_LEFT)
	IF_GT(RUDDER_INPUT_CHANNEL, 3200)
		EXEC(CIRCLE_RIGHT)
	END
END
};
*/

/*

////////////////////////////////////////////////////////////////////////////////
// Landing circuit Flight Plan
//

#if ( MODEL_GRAFAS == 1 )
#define FINAL_ALT                      20  // in meters. Landing circuit: start of Final, used for 3 points in the landing circuit
#else
#define FINAL_ALT                      22  // in meters. Landing circuit: start of Final, used for 3 points in the landing circuit
#endif

//
// Fly a 100m square at an altitude of 100m, beginning above the origin, pointing North

#define SQUARE 1

#define SET_ALT_ALT                          62
#define LOITER_LAND                          57
#define DESCENT_PATTERN                      59
#define DOWNWIND                             41
#define BASE                                 43
#define FINAL                                45


const struct logoInstructionDef instructions[] = {


	//Main  -  main program when motor is off
	SET_ALT(100)

	//takeoff
	REPEAT(15)
		USE_CURRENT_ANGLE
		FD(12)
	END

	DO_ARG(SQUARE, 300)
	DO (LOITER_LAND)

	TO (SQUARE)
		REPEAT(4)
			FD_PARAM
			RT(90)
		END
	END



	TO (SET_ALT_ALT)
	//TO (FS_SET_ALT_ALT)
		//target the current altitude, preventing motor or butterfly

		//this is used for setting up the starting point of a slope
		//note: SET_ALT(ALT) is wrong!!!
		LOAD_TO_PARAM(ALT)
		SET_ALT_PARAM
	END
	END



	TO (DESCENT_PATTERN)
	//TO (FS_DESCENT_PATTERN)

		REPEAT(2)
			//turn
			REPEAT(18)
				IF_GT(ALT,FINAL_ALT*3)
					ALT_DOWN(1)  //keep going down
					FLAG_ON(F_LAND) //brake if you have to
				ELSE
					FLAG_OFF(F_LAND)  // no brakes
				END
				RT(20)
				FD(DESIRED_SPEED_NORMAL_F0/5)
			END
		END
	END
	END



	TO (LOITER_LAND)
	//TO (FS_LOITER_LAND)
		//Fly to circuit entrypoint

		//Motor may be activated (slider up) if < 80m and if the charge level of the battery is sufficient
		//Glide if > 80m
		//When close to the entry point;
		//If high, circle down to 80m
		//if low (<50m) go straight for an "emergency" final, this may be with tailwind (dont know if the charge level of the battery is sufficient)
		CLEAR_INTERRUPT
		FLAG_ON(F_CROSS_TRACK)
		SET_SPEED(DESIRED_SPEED_NORMAL_F0) //dm/s

		FD(DESIRED_SPEED_NORMAL_F0/10)
		SET_ALT((FINAL_ALT*3)+10)
		//Motor may be activated (slider up) if low and if the charge level of the battery is sufficient
		FLAG_OFF(F_LAND)  //Motor on if needed, no brakes
		//to entry point
		PEN_UP

			// goto circuit entry point / hoog aanknopings punt
			HOME

			// Get circuit hand and heading  - configure these headings for your flying field
			// -270 == Lefthand circuit, final heading West 270, 270 == Righthand circuit, final heading West 270
			//    NE	E	|	SE		S   ||   SW        W   |   NW       N|
			//  23-67,68-112,113-157,158-202, 203-247,248-292,293-337,338-22
			//                                 wind FROM
			IF_LT(WIND_FROM_ANGLE,203)                    //  NE     E       SE       S   (N)
				IF_LT(WIND_FROM_ANGLE,113)                //  NE     E   (N)
					IF_LT(WIND_FROM_ANGLE,64)             //  N(E)
						IF_LT(WIND_FROM_ANGLE,22)         //  0..22
							PARAM_SET(-326)      //  N
						ELSE
							PARAM_SET(64)        //  NE
						END
					ELSE
						PARAM_SET(64)            //  E
					END
				ELSE                            //  SE       S
					IF_LT(WIND_FROM_ANGLE,158)
						PARAM_SET(148)           //  SE
					ELSE
						PARAM_SET(148)           //  S
					END
				END
			ELSE
				//>= 203                        //  SW     W      NW       N
				IF_LT(WIND_FROM_ANGLE,293)                   //  SW     W
					IF_LT(WIND_FROM_ANGLE,248)
						PARAM_SET(-243)          //  SW
					ELSE
						PARAM_SET(-243)          //  W
					END
				ELSE                             //  NW       N
					IF_LT(WIND_FROM_ANGLE,338)
						PARAM_SET(-326)          //  NW
					ELSE
						PARAM_SET(-326)          //  N
					END
				END
			END

			IF_GT(PARAM,0)  //param > 0
				//right hand circuit
				SET_ANGLE_PARAM   //virtually upwind of runway
				FD(150)
				RT(90)
			ELSE
				//left hand circuit
				PARAM_MUL(-1)
				SET_ANGLE_PARAM   //virtually upwind of runway
				PARAM_MUL(-1)
				FD(150)
				LT(90)
			END
			FD(150)
		PEN_DOWN    //to entry point

		//point to downwind
		IF_GE(PARAM,0)  //param => 0
			//right hand circuit
			SET_ANGLE_PARAM   //virtually upwind of runway
		ELSE
			//left hand circuit
			PARAM_MUL(-1)
			SET_ANGLE_PARAM   //virtually upwind of runway
			PARAM_MUL(-1)
		END
		//reverse heading, same for left and right
		RT(180)

		FD(DESIRED_SPEED_NORMAL_F0/3) //~3 sec  bring pattern closer to Home

		//set up an up-slope to entry point  in case we are low, else just glide there
		REPEAT_FOREVER
			//We have arrived above the circuit entry point
			//If high, fly circles down to 80m
			//if low (<50m) go straight for an "emergency" final  (dont know if the charge level of the battery is sufficient)

			IF_GT(ALT,(FINAL_ALT*3)+20)
				DO (SET_ALT_ALT)
				//DO (FS_SET_ALT_ALT)
				DO (DESCENT_PATTERN)   //use a subroutine to preserve PARAM with circuit data
				//DO (FS_DESCENT_PATTERN)   //use a subroutine to preserve PARAM with circuit data
			ELSE
				IF_GT(ALT,(FINAL_ALT)+10)  //else go straight for an "emergency" final
					DO (SET_ALT_ALT)
					//DO (FS_SET_ALT_ALT)

					// goto circuit entry point again / hoog aanknopings punt
					PEN_UP

						HOME

						IF_GE(PARAM,0)  //param => 0
							//right hand circuit
							SET_ANGLE_PARAM   //virtually upwind of runway
							FD(150)
							RT(90)
						ELSE
							//left hand circuit
							PARAM_MUL(-1)
							SET_ANGLE_PARAM   //virtually upwind of runway
							PARAM_MUL(-1)
							FD(150)
							LT(90)
						END
						FD(150)
					PEN_DOWN

					//Downwind
					DO_PARAM (DOWNWIND)
					//DO_PARAM (FS_DOWNWIND)

					//Base
					DO_PARAM (BASE)
					//DO_PARAM (FS_BASE)
				END // >50 m

				//Final
				DO_PARAM (FINAL)
				//DO_PARAM (FS_FINAL)

			END // 70m
		END // repeat
	END
	END



	TO (DOWNWIND)
	//TO (FS_DOWNWIND)
		// assume 50m < alt < 80m

		SET_ALT((FINAL_ALT*2))
		FLAG_ON(F_LAND) //brake if you have to
		//turn to and fly the downwind leg
		PEN_UP
			HOME
			IF_GE(PARAM,0)  //param => 0
				//right hand circuit
				SET_ANGLE_PARAM   //virtually upwind of runway
				RT(90)
				FD(150)
				RT(90)
			ELSE
				//left hand circuit
				PARAM_MUL(-1)
				SET_ANGLE_PARAM   //virtually upwind of runway
				PARAM_MUL(-1)
				LT(90)
				FD(150)
				LT(90)
			END
			FD(150)
		PEN_DOWN

		FLAG_OFF(F_LAND) // no braking in turns
		IF_GE(PARAM,0)  //param => 0
			//right hand circuit
			REPEAT(5)
				RT(10)
				FD(DESIRED_SPEED_NORMAL_F0/10)
				IF_GT(ALT,FINAL_ALT*2)
					ALT_DOWN(1)
				END
			END
		ELSE
			//left hand circuit
			REPEAT(5)
				LT(10)
				FD(DESIRED_SPEED_NORMAL_F0/10)
				IF_GT(ALT,FINAL_ALT*2)
					ALT_DOWN(1)
				END
			END
		END
	END
	END



	TO (BASE)
	//TO (FS_BASE)
		//assume alt less than 50m

		FLAG_OFF(F_LAND) // no braking in turns

		//turn to and fly the base leg
		SET_SPEED(DESIRED_SPEED_NORMAL_F0) //dm/s
		//set up the startingpoint of the glideslope
		IF_GE(PARAM,0)  //param => 0
			//right hand circuit
			REPEAT(5)               //5 not 4: compensate offset flighttrack vs waypoint for precise landing
				RT(10)
				FD(DESIRED_SPEED_NORMAL_F0/10)
				IF_GT(ALT,FINAL_ALT*2-5)
					ALT_DOWN(1)
				END
			END
		ELSE
			//left hand circuit
			REPEAT(5)               //5 not 4: compensate offset flighttrack vs waypoint for precise landing
				LT(10)
				FD(DESIRED_SPEED_NORMAL_F0/10)
				IF_GT(ALT,FINAL_ALT*2-5)
					ALT_DOWN(1)
				END
			END
		END
		SET_ALT(FINAL_ALT)
		PEN_UP     //base leg
			//set up the endpoint of the glideslope
			FD(60)
			FLAG_ON(F_LAND)    //use brakes / butterfly
		PEN_DOWN

		DO (SET_ALT_ALT)
		//DO (FS_SET_ALT_ALT)
		FLAG_OFF(F_LAND)    // no brakes in turn
		IF_GE(PARAM,0)  //param => 0
			//right hand circuit
			REPEAT(5)
				RT(10)
				FD(DESIRED_SPEED_NORMAL_F0/10)
				IF_GT(ALT,FINAL_ALT-5)
					ALT_DOWN(2)
				END
			END
		ELSE
			//left hand circuit
			REPEAT(5)
				LT(10)
				FD(DESIRED_SPEED_NORMAL_F0/10)
				IF_GT(ALT,FINAL_ALT-5)
					ALT_DOWN(2)
				END
			END
		END
	END
	END



	TO (FINAL)
	//TO (FS_FINAL)
		//turn to and fly the final

		//assume alt less than 50m  (target 35m)
		FLAG_OFF(F_LAND)    // no brakes in turn
		//set up the startingpoint of the glideslope
		IF_GE(PARAM,0)  //param => 0
			//right hand circuit
			REPEAT(4)
				RT(10)
				FD(DESIRED_SPEED_NORMAL_F0/10)
				IF_GT(ALT,FINAL_ALT)
					ALT_DOWN(2)
				END
			END
		ELSE
			//left hand circuit
			REPEAT(4)
				LT(10)
				FD(DESIRED_SPEED_NORMAL_F0/10)
				IF_GT(ALT,FINAL_ALT)
					ALT_DOWN(2)
				END
			END
		END


		//set up the endpoint of the glideslope   1:7
		FLAG_ON(F_LAND)     //use brakes / butterfly
		SET_ALT(FINAL_ALT*-1) //target altitude for the next waypoint below ground
		PEN_UP
			HOME
			USE_ANGLE_TO_GOAL
			//compensate offset in flighttrack vs waypoint for precise landing
			IF_GE(PARAM,0)  //param => 0
				//right hand circuit
				RT(10)
			ELSE
				//left hand circuit
				LT(10)
			END

			FD(80)
		PEN_DOWN

		// go around if landing did not happen
		EXEC (LOGO_MAIN)

	END
	END

};
*/
#ifdef THERMALLING_MISSION

// ****************************************************************
//    LET e-glider mission - 2016
// ****************************************************************

// This script performs a LET (Local Endurance Thermalling) mission
// It does geofencing, motor climbs and thermalling
// The program navigates the area randomly to search for rising air.
// The program detects and then moves the glider to the center of thermals, even if the thermals drift downwind.
// Two types of geofences are used: normal geofence and wind geofence
// The wind geofence allows for optimisation of the position of the glider, to have enough space to thermal if rising air is found.
// the wind geofence is only enforced if no thermals are found
// the soft geofences are smaller in size and are intended to position the aircraft in the geofence, crossing the bigger (real) geofence should be a rare event
// The motor is controlled to maintain altitude when no thermals are found.
// The adaptive throttle climbrate is optimized for longer flights. The motor is stopped when too much sink or much lift is encountered, to preserve battery power.
// multiple smaller subroutines are used to allow recording as waypoints in telemetry for debugging
// 'Check' routines have an even number, odd numbered subroutines (actions) will be logged as waypoint
// 'Forward' commands are usualy timed to last about 1 sec
// Barometer data is used to measure altitude and climbrate
// Glide speed control is selected by LOGO (slow, normal and fast). Flaps (camber) are controlled simultaneously (F4,F0 or F-4)

// to use this script:
//
//1. add this line to Options.h:
//  #define THERMALLING_MISSION        1    // enable code to calculate angle and distance to UPWIND_POINT
//
//2. #define WAYPOINT_RADIUS           40
//
// select AIRFRAME_TYPE  AIRFRAME_GLIDER   (required)
//		for pure gliders setup THROTTLE_INPUT_CHANNEL  UNUSED; the code for motor control will be disabled
//		configure mixer settings in servomix.c  (after changing mixer settings only this module will need recompilation.)
// For this mission GLIDE_AIRSPEED_CONTROL is recommended
// To enable GLIDE_AIRSPEED_CONTROL setup the following options:
//  - in options.h             SPEED_CONTROL = 1          and configure DESIRED_SPEED
//	- in airspeed_options.h    GLIDE_AIRSPEED_CONTROL = 1  and configure settings for your aircraft
//	- in gain_variables.h      ALTITUDE_GAINS_VARIABLE=1
//
// see https://groups.google.com/forum/#!topic/uavdevboard/yn5PnR6pk7Q
//
// GEOFENCE_STATUS : 0,1,2  0= soft/wind gf, 1=wind gf, 2 geofence (alarm)   
// GEOFENCE_TURN : geofence angle to return



#define GEOFENCE_SIZE                 400  // radius of circle to keep aircaft within line of sight
//#define UPWIND_POINT_FACTOR           0.5  // multiply "windspeed in cm/s" by factor for distance from home i.e. windspeed = 540 cm/s (3 bft) * 0.5 = 270 M from home
//#define UPWIND_POINT_DISTANCE_LIMIT   270  // in meters


#if( HILSIM == 1)
/*
#define MOTOR_ON_TRIGGER_ALT          200  // in meters
#define MOTOR_ON_IN_SINK_ALT          180  // in meters, set low. Altitude where ground objects must be avoided using motor despite sink
#define MOTOR_OFF_TRIGGER_ALT         230  // in meters
#define MAX_THERMALLING_ALT           300  // in meters
*/
#define MOTOR_ON_TRIGGER_ALT           50  // in meters
#define MOTOR_ON_IN_SINK_ALT           50  // in meters, set low. Altitude where ground objects must be avoided using motor despite sink
#define MOTOR_OFF_TRIGGER_ALT          90  // in meters
#define MAX_THERMALLING_ALT           120  // in meters

#define CLIMBR_THERMAL_TRIGGER         40  // cm/sec >= 0.2 m/s climb is the trigger to start thermalling
#define CLIMBR_THERMAL_CLIMB_MIN     -140  // cm/sec > -1.0 maximum sink allowed, else abort thermalling
//#define MOTOR_CLIMB_MIN               -50  // cm/sec minimal climbrate that is expected   else abort the Motor climb
#define MOTOR_CLIMB_MAX               120  // cm/sec maximal climbrate that is expected   else start thermalling

#define FINAL_ALT                      22  // in meters. Landing circuit: start of Final, used for 3 points in the landing circuit
#define SPEED_MIN			          105  // in dm/h     38 km/h	10,56
#define SPEED_MAX				      116  // in dm/h     42 km/h	11,67
#else
//Dutch rules outside of Knvvl fields...
#define MOTOR_ON_TRIGGER_ALT           70  // in meters
#define MOTOR_ON_IN_SINK_ALT           50  // in meters, set low. Altitude where ground objects must be avoided using motor despite sink
#define MOTOR_OFF_TRIGGER_ALT          90  // in meters
#define MAX_THERMALLING_ALT           120  // in meters
#define CLIMBR_THERMAL_TRIGGER         20  // cm/sec >= 0.2 m/s climb is the trigger to start thermalling
#define CLIMBR_THERMAL_CLIMB_MIN     -100  // cm/sec > -1.0 maximum sink allowed, else abort thermalling
//#define MOTOR_CLIMB_MIN                 0  // cm/sec minimal climbrate that is expected   else abort the Motor climb
#define MOTOR_CLIMB_MAX               120  // cm/sec maximal climbrate that is expected   else start thermalling

#if ( MODEL_GRAFAS == 1 )
#define FINAL_ALT                      16  // in meters. Landing circuit: start of Final, used for 3 points in the landing circuit
#define SPEED_MIN			           97  // in dm/h     35 km/h
#define SPEED_MAX				      113  // in dm/h     41 km/h
#elif ( MODEL_LINEA == 1 )
#define FINAL_ALT                      16  // in meters. Landing circuit: start of Final, used for 3 points in the landing circuit
#define SPEED_MIN			           95  // in dm/h      km/h
#define SPEED_MAX				      105  // in dm/h      km/h
#else   //Fantasy
#define FINAL_ALT                      18  // in meters. Landing circuit: start of Final, used for 3 points in the landing circuit
#define SPEED_MIN			          105  // in dm/h     38 km/h	10,56
#define SPEED_MAX				      116  // in dm/h     42 km/h	11,67
#endif
#endif



//Custom change made: Only odd numbers (actions) will be logged as waypoint

//Geofences
#define CRUISE                                3
#define PLAN_RETURN_GEOFENCE                  4
#define RETURN_GEOFENCE                       5
#define PLAN_SOFT_GEOFENCE                    6
#define INT_PILOT_INPUT                       8
#define RETURN_SOFT_GEOFENCE                  9
#define INT_RETURN_GEOFENCE                  10
#define INT_RETURN_SOFT_GEOFENCE             12
#define INT_CRUISE                           20

//Thermals
#define THERMALLING                          14
#define WAIT_DECREASE_CLIMBRATE              13
#define THERMALLING_TURN                     15
#define INT_THERMALLING                      16
#define THERMALLING_SHIFT_CIRCLE             17
#define SINK                                 19
#define INT_SINK                             22

//Motor
#define TAKEOFF                              21
#define RETURN_MC_GEOFENCE                   23
#define MOTOR_CLIMB_FORWARD                  27
#define PILOT_INPUT_IN_MC                    33
#define RETURN_MC_SOFT_GEOFENCE              35

//Misc
#define CHECK_PILOT                          34
#define CHECK_GF                             36
#define CHECK_SOFT_GF                        18
#define CHECK_LAND                           38
#define CHECK_SINK                           40
#define CHECK_THERMAL                        42
#define CHECK_HIGH                           44
#define CHECK_LATE                           46
#define CHECK_OVERSHOOT                      48
#define CHECK_MOTOR                          50
#define CHECK_TAKEOFF                        64
#define CHECK_TEST_MODE                      82
#define PILOT_INPUT                          47
#define TOO_HIGH                             53
#define INT_TOO_HIGH                         56
//#define BETTER_LIFT                          55
#define RESET_NAVIGATION                     58

//Land
#define SET_ALT_ALT                          60
#define LOITER_LAND                          57
#define DESCENT_PATTERN                      59
#define DOWNWIND                             41
#define BASE                                 43
#define FINAL                                45

//Failsafe Land
#define FS_SET_ALT_ALT                       62
#define FS_DOWNWIND                          63
#define FS_BASE                              65
#define FS_FINAL                             67
#define FS_DESCENT_PATTERN                   69
#define FS_LOITER_LAND                       71

//Polar_plot
#define PP_CHECKS                            80
#define PP_POLAR_PLOT                        81
#define PP_PATTERN                           83
#define PP_RETURN_MC_SOFT_GEOFENCE           85
#define PP_CRUISE                            87
#define PP_MOTOR_CLIMB_FORWARD               89

const struct logoInstructionDef instructions[] = {

	//Main  -  main program when motor is off
	//LOGO_MAIN  LET

		CLEAR_INTERRUPT
		//cleanup
		SET_ALT(MAX_THERMALLING_ALT-10)    // the last 20m will be used to gradually apply brakes  - depends on brake gain
		SET_SPEED(DESIRED_SPEED_NORMAL_F0) //dm/s
		PEN_DOWN
		SET_INTERRUPT(INT_CRUISE)
		DO (PLAN_RETURN_GEOFENCE)   //geofence
		DO (PLAN_SOFT_GEOFENCE)   //soft geofence
		IF_EQ( MOTOR_OFF_TIMER,0 )  //motor stopped more than 4 seconds ago
			DO (CRUISE)   // prevent overshoots
		ELSE
			DO (MOTOR_CLIMB_FORWARD)  //prevent overshoots
		END
	END
	END



	TO (CRUISE)
		FD(DESIRED_SPEED_NORMAL_F0/10)
	END
	END



	// Normal geofence routines


	TO (PLAN_RETURN_GEOFENCE)
		IF_EQ(GEOFENCE_STATUS, 2)
			SET_INTERRUPT(INT_RETURN_GEOFENCE)
			REPEAT(5)
				IF_EQ(GEOFENCE_STATUS, 2)
					IF_NE(GEOFENCE_TURN, 0)
						IF_LT(GEOFENCE_TURN, 0)
							LT(10)
						ELSE
							RT(10)
						END
					END
					IF_EQ( MOTOR_OFF_TIMER,0 )
						DO (RETURN_GEOFENCE)
					ELSE
						DO (RETURN_MC_GEOFENCE)
					END
				END
			END
			EXEC (LOGO_MAIN)
		END
	END
	END



	TO (RETURN_GEOFENCE)
		FD(DESIRED_SPEED_NORMAL_F0/10)
	END
	END



	TO (RETURN_SOFT_GEOFENCE)
		FD(DESIRED_SPEED_NORMAL_F0/10)
	END
	END



	TO (PLAN_SOFT_GEOFENCE)
		//assume not strict
		IF_EQ(GEOFENCE_STATUS, 1)
			SET_INTERRUPT(INT_RETURN_SOFT_GEOFENCE)
			REPEAT(5)
				IF_NE(GEOFENCE_TURN, 0)
					IF_LT(GEOFENCE_TURN, 0)
						LT(10)
					ELSE
						RT(10)
					END
				END
				IF_EQ( MOTOR_OFF_TIMER,0 )
					DO (RETURN_SOFT_GEOFENCE)
				ELSE
					DO (RETURN_MC_SOFT_GEOFENCE)
				END
			END
			EXEC (LOGO_MAIN)
		END
	END
	END




	//Thermalling routines


	TO (THERMALLING)
	    //intial turn after rising air has been detected
		//after decrease, it is assumed the core has been crossed
		//then first a tighter half turn, then a wider one
		//consecutive turns use fine shifts if needed

		SET_INTERRUPT(INT_THERMALLING)
		FLAG_ON(F_LAND)    //Motor off
		SET_SPEED(DESIRED_SPEED_SLOW_F4)
		//wait up to 6 sec for the climbrate to decrease
		LOAD_TO_PARAM(AIR_SPEED_Z_DELTA)    //prime the delta; store current vario value
		PARAM_SET(0) //clear;
 		REPEAT(6)    //6 sec max
			IF_GE(PARAM,0)
				DO (WAIT_DECREASE_CLIMBRATE)
				LOAD_TO_PARAM(AIR_SPEED_Z_DELTA)   // cm/s
			END
		END

		//lock turn direction here
		LOAD_TO_PARAM(SET_DIRECTION)

		//do while turn 180 deg, aim for ~4 sec behind the starting point, for a turn around the core. compensate for the widening turn during the time it takes to level of
		REPEAT(5) //6 sec =~ 180 deg = 6 * "30 deg per loop"
			//use motor to compensate sink if turn takes us outside of the thermal
			IF_LT(AIR_SPEED_Z,CLIMBR_THERMAL_TRIGGER)
				FLAG_OFF(F_LAND)    //Motor on
			END
			//and off when no longer needed
 			IF_GT(AIR_SPEED_Z,MOTOR_CLIMB_MAX)
				FLAG_ON(F_LAND)    //Motor off
			END

			DO (THERMALLING_TURN)
		END  //repeat
		FLAG_ON(F_LAND)    //Motor off
		DO (THERMALLING_TURN)

		//Shift the circle for 3 sec
		DO (THERMALLING_SHIFT_CIRCLE)
		DO (THERMALLING_SHIFT_CIRCLE)
		DO (THERMALLING_SHIFT_CIRCLE)

		//now continue around the core
		REPEAT_FOREVER
			IF_LT(AIR_SPEED_Z,CLIMBR_THERMAL_CLIMB_MIN)
				EXEC (LOGO_MAIN)
			END
			IF_GT( MOTOR_OFF_TIMER,0 )   //only with motor off
				EXEC (LOGO_MAIN)
			END
			IF_LT( ALT,MOTOR_ON_IN_SINK_ALT+15)
				EXEC (LOGO_MAIN)
			END

			//read delta
			LOAD_TO_PARAM(AIR_SPEED_Z_DELTA)   // cm/s
			//if climb improved 0.1 m/s irt last value, shortly reduce bank, shifting the cicle towards better climb
			IF_GE(PARAM,8)
				DO (THERMALLING_SHIFT_CIRCLE)  // small cicle shift
			ELSE
				DO (THERMALLING_TURN)
			END
		END
	END
	END



	TO (WAIT_DECREASE_CLIMBRATE)
		//Level off/Shift the circle for 1 sec, log the action as a "waypoint"
		BANK_1S(0)
	END
	END



	TO (THERMALLING_TURN)
		//Custom solution using new command FIXED_BANK_ROTATE()
		FIXED_BANK_ROTATE(30)   // perform roll to a fixed bank x deg for 30 deg heading change to the right and fly on for ~2 sec, position/navigation will be ignored
	END
	END



	TO (THERMALLING_SHIFT_CIRCLE)
		//Level off/Shift the circle for 1 sec, log the action as a "waypoint"
		BANK_1S(0)
	END
	END


/*
	TO (BETTER_LIFT)
		//indicates lift is better then it was at the beginnning of the thermalling turn
		//try to core the thermal in small steps

		BANK_1S(0)     // reduce bank for one sec, shifting the circle slightly
	END
	END
*/

	TO (SINK)
		SET_INTERRUPT(INT_SINK)
		SET_SPEED(DESIRED_SPEED_FAST_FMIN4) //dm/s
		// method: avoid flying in sink, assume sink area in front of aircraft
		// turn at least 120 deg while in sink, away from start heading
		// choose l/r using gf code  (no matter if close to home, rare), because that (beyond home) heading leads to the biggest area where non-sinking air might be
		// complete gf turns while sink lasts, towards home direction
		// keep checking GF while responding to sink
		// in mc: sink = exit to main

		//perform a precalculated turn and a level stretch to opposite side of area
		LOAD_TO_PARAM(REL_ANGLE_TO_OPPOSITE)   // gf -180..179)
		IF_LT(REL_ANGLE_TO_OPPOSITE, -10)          //
			//make the turn to Home a smooth one
			PARAM_MUL(-1)
			PARAM_DIV(10)
			//special : force at least 12 = 120 deg turn to evade sink
			REPEAT_PARAM
				//exit as soon as sink has gone
				IF_GT(AIR_SPEED_Z,CLIMBR_THERMAL_CLIMB_MIN) //> -1 m/s,	if so, exit
					SET_SPEED(DESIRED_SPEED_NORMAL_F0) //dm/s
					EXEC (LOGO_MAIN)
				END

				LT(10)
				FD(DESIRED_SPEED_FAST_FMIN4/10)	//"SINK"
			END
		END
		IF_GT(REL_ANGLE_TO_OPPOSITE, 10)          //
			PARAM_DIV(10)
			REPEAT_PARAM
				//exit as soon as sink has gone
				IF_GT(AIR_SPEED_Z,CLIMBR_THERMAL_CLIMB_MIN) //> -1 m/s,	if so, exit sink routine
					SET_SPEED(DESIRED_SPEED_NORMAL_F0) //dm/s
					EXEC (LOGO_MAIN)
				END

				RT(10)
				FD(DESIRED_SPEED_FAST_FMIN4/10)	//"SINK"
			END
		END

		// end with straight (not likely) with checks
		PARAM_SET(5)

		REPEAT_PARAM
			//exit as soon as sink has gone
			IF_GT(AIR_SPEED_Z,CLIMBR_THERMAL_CLIMB_MIN) //limit sink to -1 m/s,	if so, exit sink routine
				SET_SPEED(DESIRED_SPEED_NORMAL_F0) //dm/s
				EXEC (LOGO_MAIN)
			END

			FD(DESIRED_SPEED_FAST_FMIN4/10)	//"SINK"
		END

		SET_SPEED(DESIRED_SPEED_NORMAL_F0) //dm/s
		EXEC(LOGO_MAIN)
	END
	END



// Motor Climb  routines

	TO (TAKEOFF)    //use motor if too low, switch off if too high , check geofence
		SET_INTERRUPT(INT_RETURN_SOFT_GEOFENCE)
		//SET_INTERRUPT(INT_RETURN_GEOFENCE)
		//allow for level takeoff in current direection when in autonmous mode
		IF_LT(ALT, 10)  //below: auto takeoff / hand launch with motor on in Autonomous mode
			//if relative angle is much different from turtle,correct
			REPEAT(60)
				//SET_INTERRUPT(INT_RETURN_GEOFENCE)
				IF_LT(ALT, 10)  //below: auto takeoff / hand launch with motor on in Autonomous mode
					//BANK_1S  //allow heading to stabilize on takeoff
					BANK_1S(0)
				ELSE
					EXEC (LOGO_MAIN)
				END
			END
		END
		EXEC (LOGO_MAIN)
	END
	END



	TO (RETURN_MC_GEOFENCE)
		FD(DESIRED_SPEED_NORMAL_F0/10)
	END
	END



	TO (RETURN_MC_SOFT_GEOFENCE)
		FD(DESIRED_SPEED_NORMAL_F0/10)
	END
	END


//Misc functions


	TO (MOTOR_CLIMB_FORWARD)
		//start/continue a slow climb with motor

		//if in an area with some lift, circle more
		IF_GT(AIR_SPEED_Z,MOTOR_CLIMB_MAX - 30)    // > ~0.7 m/s climb is expected, circle if 0.9, exit if 1.2
			RT(10)
		END

		FD(DESIRED_SPEED_NORMAL_F0/10)
	END
	END



	TO (TOO_HIGH)
		//indicates lift while too high

		SET_INTERRUPT(INT_TOO_HIGH)

		REPEAT_FOREVER
			//try to head upwind
			LOAD_TO_PARAM(REL_ANGLE_TO_WIND)    // wgf !!!non-standard LOGO command!!! -	-180..179)

			IF_LT(REL_ANGLE_TO_WIND, 0) // wgf	angle < -30		(-31..-180) =   1-150 L
				//LT(10)
				BANK_1S(-20)
			ELSE
				IF_GE(REL_ANGLE_TO_WIND, -30) // wgf	angle >= -30  (-30..149) = 0..179 R
					//RT(10)
					BANK_1S(20)
				END
			END

			IF_LT(ALT,MAX_THERMALLING_ALT)
				EXEC (LOGO_MAIN)
			END
		END
		EXEC (LOGO_MAIN)
	END
	END



	TO (RESET_NAVIGATION)
		//the pilot has changed the angle of the aircraft, now use that as the new heading
		PEN_UP
			USE_CURRENT_ANGLE
			USE_CURRENT_POS		//centre on waypoint 'here', removing the drift error
			FD(WAYPOINT_PROXIMITY_RADIUS)	//back to edge of radius,	target is now directly in front again (on arrival point)
		PEN_DOWN
	END
	END



	TO (PILOT_INPUT)
		SET_INTERRUPT(INT_PILOT_INPUT)
		REPEAT(10)			//keep pilot control as long stick is off-centre,max 10 loops
			IF_LT(AILERON_INPUT_CHANNEL ,2850)
				IF_GT(AILERON_INPUT_CHANNEL ,1700)
					BANK_1S(-20)
				END
			END
			IF_GT(AILERON_INPUT_CHANNEL,3150)
				BANK_1S(20)
			END
		END
		EXEC (LOGO_MAIN)
	END
	END



	TO (CHECK_PILOT)
		IF_LT(AILERON_INPUT_CHANNEL ,2850)
			IF_GT(AILERON_INPUT_CHANNEL ,1700)
				EXEC (PILOT_INPUT)
			END
		END
		IF_GT(AILERON_INPUT_CHANNEL ,3150)
			EXEC (PILOT_INPUT)
		END
	END
	END



	TO (CHECK_GF)
		IF_EQ(GEOFENCE_STATUS,2 )              //outside geofence
			EXEC (PLAN_RETURN_GEOFENCE)
		END
	END
	END



	TO (CHECK_SOFT_GF)
		IF_EQ(GEOFENCE_STATUS,1 )              //inside geofence
			EXEC (PLAN_SOFT_GEOFENCE)
		END
	END
	END



	TO (CHECK_LAND)
		IF_LT(BRAKE_THR_SEL_INPUT_CHANNEL ,2700)     // automode only
			IF_GT(BRAKE_THR_SEL_INPUT_CHANNEL ,1700) // abstract flightplan workaround: only real low, ignore 0
				EXEC (LOITER_LAND)
			END
		END
		IF_LT(BATTERY_VOLTAGE, VOLTAGE_SENSOR_ALARM)     // land automatically when battery is low, usefull when no telemetry is available
			EXEC (LOITER_LAND)
		END
	END
	END



	TO (CHECK_SINK)
		IF_LT(AIR_SPEED_Z,CLIMBR_THERMAL_CLIMB_MIN) //if sink, exit the sink
			EXEC (SINK)
		END
	END
	END



	TO (CHECK_THERMAL)
		//check for thermals
		IF_GE( ALT,MOTOR_ON_IN_SINK_ALT+15)
			IF_EQ( MOTOR_OFF_TIMER,0 )  //motor has stopped more than 4 secons ago
				//glided into a thermal
				IF_GE(AIR_SPEED_Z,CLIMBR_THERMAL_TRIGGER)  //>= 0.2 m/s climb is the trigger, also check GEOFENCE
					//lift found
					//keep flying straight until decreasing lift
					//wait for decrease of lift
					EXEC (THERMALLING)     //wait up to 6 sec for the climbrate decrease, keep the best climbrate
					//current is less
					//now beyond the best climbrate..
					//turn up to 270 deg + 3sec straight if not better
					//abort the turn if better climbrate is found
					//every check: if positive, take action, then restart program
				END // 0.4 m/s trigger
			ELSE
				//termalling could still be justified if detected just after or during motorclimb
				IF_GE(AIR_SPEED_Z,MOTOR_CLIMB_MAX)  //>= 1.2 m/s climb is the trigger, also check GEOFENCE
					//lift found
					//keep flying straight until decreasing lift
					//wait for decrease of lift
					EXEC (THERMALLING)     //wait up to 6 sec for the climbrate decrease, keep the best climbrate
					//current is less
					//now beyond the best climbrate..
					//turn up to 270 deg + 3sec straight if not better
					//abort the turn if better climbrate is found
					//every check: if positive, take action, then restart program
				END // 1.2 m/s trigger
			END
		END
	END
	END



	TO (CHECK_HIGH)
		IF_GT(ALT,MAX_THERMALLING_ALT )     // not too high
			EXEC (TOO_HIGH)
		END
	END
	END



	TO (CHECK_LATE)
		IF_GT(READ_FLY_COMMAND_COUNTER,80 )             //fly command takes too long
			IF_GT(FORCE_RESET,0) //test is not relevant, this sets the value
			END
		END
	END
	END



	TO (CHECK_OVERSHOOT)
		IF_LT(REL_ANGLE_TO_GOAL,-60)
			IF_GT(FORCE_RESET,0) //test is not relevant, this sets the value
			END
		END
		IF_GT(REL_ANGLE_TO_GOAL,60)
			IF_GT(FORCE_RESET,0) //test is not relevant, this sets the value
			END
		END
	END
	END



	TO (CHECK_MOTOR)
#if ( THROTTLE_INPUT_CHANNEL != CHANNEL_UNUSED )    //no motor support in case of gliders
		//motor handling
		IF_LT(ALT, MOTOR_ON_TRIGGER_ALT)            // not low,  check every cycle
			IF_GT(THROTTLE_INPUT_CHANNEL, 3400)     // matches level at wich ESC would start motor, which is close to full throttle
				FLAG_OFF(F_LAND)                    //Motor on
			END
		END
		IF_GT(ALT, MOTOR_ON_TRIGGER_ALT+3)            // delay from 70m motor start
			IF_EQ(READ_F_LAND,0)                      // motor is requested
			    IF_EQ( MOTOR_OFF_TIMER,0 )            // but is not running
					FLAG_ON(F_LAND)                   // give up 
				END	
			END
		END
		IF_GT(ALT,MOTOR_OFF_TRIGGER_ALT)             //settle into gliding
			FLAG_ON(F_LAND)	//Motor off
		END
#endif
	END
	END



	TO (CHECK_TEST_MODE)
		IF_GT(TEST_MODE_INPUT_CHANNEL ,3900)     // automode only
			EXEC (PP_POLAR_PLOT)
		END
	END
	END



	TO (CHECK_TAKEOFF)
		IF_LT(ALT, 10)  //below: auto takeoff / hand launch with motor on in Autonomous mode
			EXEC (TAKEOFF)
		END
	END
	END



	TO (INT_PILOT_INPUT)  //interrupt routine
		//DO (CHECK_PILOT)
		//DO (CHECK_GF)
		//DO (CHECK_LAND)
		//DO (CHECK_SINK)
		//DO (CHECK_THERMAL)
		//DO (CHECK_HIGH)
		//DO (CHECK_LATE)
//		DO (CHECK_OVERSHOOT)
		DO (CHECK_MOTOR)
		//DO (CHECK_SOFT_GF)
		//DO (CHECK_TAKEOFF)
		//DO (CHECK_TEST_MODE)
	END
	END



	TO (INT_RETURN_GEOFENCE)  //interrupt routine
		DO (CHECK_PILOT)
		//DO (CHECK_GF)
		//DO (CHECK_LAND)
		//DO (CHECK_SINK)
		//DO (CHECK_THERMAL)
		//DO (CHECK_HIGH)
/*
		DO (CHECK_LATE)
*/
//		DO (CHECK_OVERSHOOT)
		DO (CHECK_MOTOR)
		//DO (CHECK_SOFT_GF)
		//DO (CHECK_TAKEOFF)
		//DO (CHECK_TEST_MODE)
	END
	END



	TO (INT_RETURN_SOFT_GEOFENCE)  //interrupt routine
		//check pilot, gf, slider, batt, sink, thermal, high, late, overshoot, motor

		DO (CHECK_PILOT)
		DO (CHECK_GF)


		DO (CHECK_LAND)
		DO (CHECK_SINK)
		DO (CHECK_THERMAL)
		DO (CHECK_HIGH)
/*
		DO (CHECK_LATE)
*/
//		DO (CHECK_OVERSHOOT)
		DO (CHECK_MOTOR)

		//DO (CHECK_SOFT_GF)
		//DO (CHECK_TAKEOFF)   !
		DO (CHECK_TEST_MODE)
	END
	END



	TO (INT_CRUISE)  //interrupt routine
		//check pilot, gf, slider, batt, sink, thermal, high, late, overshoot, motor
		REPEAT(3)
			DO (CHECK_PILOT)
			DO (CHECK_GF)

			DO (CHECK_LAND)
			DO (CHECK_SINK)
			DO (CHECK_THERMAL)
			DO (CHECK_HIGH)
	/*
			DO (CHECK_LATE)
	*/
	//		DO (CHECK_OVERSHOOT)
			DO (CHECK_MOTOR)
	//		DO (CHECK_SOFT_GF)
			DO (CHECK_TAKEOFF)
			DO (CHECK_TEST_MODE)
		END
	END
	END



	TO (INT_THERMALLING)  //interrupt routine
		//check pilot, gf, slider, batt, sink,		, high, late, overshoot, motor

		DO (CHECK_PILOT)
		DO (CHECK_GF)
		DO (CHECK_LAND)
/*
		DO (CHECK_SINK)
		//DO (CHECK_THERMAL)
*/
		DO (CHECK_HIGH)
		//DO (CHECK_LATE)
//		DO (CHECK_OVERSHOOT)
		DO (CHECK_MOTOR)
		//DO (CHECK_SOFT_GF)
		//DO (CHECK_TAKEOFF)
//		DO (CHECK_TEST_MODE)
	END
	END



	TO (INT_SINK)  //interrupt routine
		//check pilot, gf, slider, batt,	 ,		  , 	, late, overshoot, motor

		DO (CHECK_PILOT)
		DO (CHECK_GF)
		DO (CHECK_LAND)
		//DO (CHECK_SINK)
		//DO (CHECK_THERMAL)
/*
		DO (CHECK_HIGH)
		DO (CHECK_LATE)
*/
//		DO (CHECK_OVERSHOOT)
		DO (CHECK_MOTOR)
		//DO (CHECK_SOFT_GF)
		//DO (CHECK_TAKEOFF)
		DO (CHECK_TEST_MODE)
	END
	END



	TO (INT_TOO_HIGH)  //interrupt routine
		//check pilot, gf, slider, batt,	 ,		  , 	, late, overshoot, motor

		DO (CHECK_PILOT)
		DO (CHECK_GF)
		DO (CHECK_LAND)
		//DO (CHECK_SINK)
		//DO (CHECK_THERMAL)
		//DO (CHECK_HIGH)
/*
		DO (CHECK_LATE)
*/		//DO (CHECK_SOFT_GF)
//		DO (CHECK_OVERSHOOT)
		DO (CHECK_MOTOR)
		//DO (CHECK_TAKEOFF)
		DO (CHECK_TEST_MODE)
	END
	END




//Polar_Plot
	TO (PP_POLAR_PLOT)

		CLEAR_INTERRUPT  //don't change navigation
		SET_ALT(MOTOR_OFF_TRIGGER_ALT)

		//to starting point
		FLAG_OFF(F_LAND) //Motor on
		PEN_UP
			HOME      // Go Home and point North
			RT(118)   // this is also the correct angle
			FD(145)
		PEN_DOWN

		//PARAM_SET(SPEED_MIN)  //start with 1
		LOAD_TO_PARAM(READ_DESIRED_SPEED)      //make script restartable, continue with last desiredSpeed

		REPEAT_FOREVER

			IF_LT(PARAM,SPEED_MAX+0)
				PARAM_ADD(1+0)          // Increases the target speed by x m/s   custom: 1 dm/s !
			ELSE
				PARAM_SET(SPEED_MIN)  //start over
			END
			SET_SPEED_PARAM
			DO(PP_PATTERN)	              // turn, settle, glide, settle

		END
	END
	END


	TO (PP_PATTERN)
		//basic shape: one turn, one glide
		//turn 180 deg, [cicle 360 if too low], glide , motor on if alt <50m, climb

		//climb, turn
		FLAG_OFF(F_LAND) //Motor on
		REPEAT(18)
			DO (PP_RETURN_MC_SOFT_GEOFENCE)
			RT(10)
		END

		//add a circle if still too low
		REPEAT(5)
			IF_LT(ALT, MOTOR_ON_IN_SINK_ALT+40)  //need at least 40m for a glide, else circle
				REPEAT(36)
					DO (PP_RETURN_MC_SOFT_GEOFENCE)
					RT(10)
				END
			END
		END

		//settle into glide	 if high enough
		IF_GT(ALT, MOTOR_ON_IN_SINK_ALT+40)
			FLAG_ON(F_LAND)    //Motor off
			REPEAT(6)
				DO (PP_MOTOR_CLIMB_FORWARD)     // log as motor
			END

			//measure sinkrate with this speed
			REPEAT(35)                               //100- 47/2 = 78m , = 47*11 = 500
				IF_EQ( MOTOR_OFF_TIMER,0 )  //motor has stopped more than 4 secons ago
					//check minimum alt
					IF_LT(ALT, MOTOR_ON_IN_SINK_ALT)
						DO (PP_MOTOR_CLIMB_FORWARD)     // log as motor first
						FLAG_OFF(F_LAND)    //Motor on
					ELSE
						DO (PP_CRUISE)   	// glide, measure sinkrate
					END
				ELSE
					DO (PP_MOTOR_CLIMB_FORWARD)
				END
			END
		ELSE
			//should not happen
			REPEAT(41)
				DO (PP_MOTOR_CLIMB_FORWARD)
			END
		END

		//prepare motor on, continue glide, but log it as motor to prevent motor influence
		DO (PP_MOTOR_CLIMB_FORWARD)            // log as motor
		FLAG_OFF(F_LAND)    //Motor on
	END
	END



	TO (PP_CRUISE)
		DO (PP_CHECKS)
		FD(11)
	END
	END


	TO (PP_MOTOR_CLIMB_FORWARD)
		DO (PP_CHECKS)
		FD(11)
	END
	END


	TO (PP_RETURN_MC_SOFT_GEOFENCE)   //use this for (wider) turns
		DO (PP_CHECKS)
		FD(23)
	END
	END


	TO (PP_CHECKS)        // is motor needed, landing requested, is pilot in control?
		//see if calling subroutine needs to end

		IF_LT(BRAKE_THR_SEL_INPUT_CHANNEL ,2700)
			IF_GT(BRAKE_THR_SEL_INPUT_CHANNEL ,1700) // abstract flightplan workaround: only real low, ignore 0
				EXEC (LOITER_LAND)
			END
		END
		IF_LT(BATTERY_VOLTAGE, VOLTAGE_SENSOR_ALARM)     // land automatically when battery is low, usefull when no telemetry is available
			EXEC (LOITER_LAND)
		END
		IF_LT(TEST_MODE_INPUT_CHANNEL ,2200)
			EXEC (LOGO_MAIN)
		END
	END
	END






//Landing


	TO (SET_ALT_ALT)
	//TO (FS_SET_ALT_ALT)

		//target the current altitude, preventing motor or butterfly
		//this is used for setting up the starting point of a slope
		//note: SET_ALT(ALT) is wrong!!!
		LOAD_TO_PARAM(ALT)
		SET_ALT_PARAM
	END
	END



	TO (DESCENT_PATTERN)
	//TO (FS_DESCENT_PATTERN)
		//turn
		REPEAT(12)
			IF_GT(ALT,FINAL_ALT*3+5)
				DO (SET_ALT_ALT)    //keep aware of current altitude
				ALT_DOWN(8)  //keep going down  , this and FD controls brakes
				FLAG_ON(F_LAND) //brake if you have to
			ELSE
				FLAG_OFF(F_LAND)  // no brakes
			END
			RT(30)
			FD(DESIRED_SPEED_NORMAL_F0/3)
		END
	END
	END



	TO (LOITER_LAND)
	//TO (FS_LOITER_LAND)
		//Fly to circuit entrypoint

		//Motor may be activated (slider up) if < 80m and if the charge level of the battery is sufficient
		//Glide if > 80m
		//When close to the entry point;
		//If high, circle down to 80m
		//if low (<50m) go straight for an "emergency" final, this may be with tailwind (dont know if the charge level of the battery is sufficient)
		CLEAR_INTERRUPT
		FLAG_ON(F_CROSS_TRACK)
		SET_SPEED(DESIRED_SPEED_NORMAL_F0) //dm/s

		DO (SET_ALT_ALT)           //level to upwind point
		IF_LT(ALT,FINAL_ALT*3)
			IF_GT(ALT,FINAL_ALT)   // only if not too low
				SET_ALT((FINAL_ALT*3)+0)
			END
		END
		FD(DESIRED_SPEED_NORMAL_F0/10)
		ALT_DOWN(10)  //start going down
		//SET_ALT((FINAL_ALT*3)+0)

		//Motor may be activated if low (But not too low) and if the charge level of the battery is sufficient
		//decide if brakes or motor are needed most here
		//high: brakes if needed, <final: brakes, else motor + glide
		//downside: heavy thermals are not adressed, will be only after arrival above entry point
		FLAG_ON(F_LAND)  //brakes
		IF_LT(ALT,FINAL_ALT*3+40) //won't make it gliding (this does not change arrival point altitude)
			IF_GT(ALT,FINAL_ALT)
				FLAG_OFF(F_LAND)  //Motor on if needed, no brakes
			END
		END

		//to entry point
		PEN_UP

			// goto circuit entry point / hoog aanknopings punt
			HOME
			//SET_ALT((FINAL_ALT*3)+0)

			// Get circuit hand and heading  - configure these headings for your flying field
			// -270 == Lefthand circuit, final heading West 270, 270 == Righthand circuit, final heading West 270
			//     NE	E	|	SE		S   ||   SW        W   |   NW       N|
			//  23-67,68-112,113-157,158-202, 203-247,248-292,293-337,338-22
			//                                 wind FROM
			IF_LT(WIND_FROM_ANGLE,203)                    //  NE     E       SE       S   (N)
				IF_LT(WIND_FROM_ANGLE,113)                //  NE     E   (N)
					IF_LT(WIND_FROM_ANGLE,64)             //  N(E)
						IF_LT(WIND_FROM_ANGLE,22)         //  0..22
							PARAM_SET(-326)    //  N
						ELSE
							PARAM_SET(64)      //  NE
						END
					ELSE
						PARAM_SET(64)          //  E
					END
				ELSE                            //  SE       S
					IF_LT(WIND_FROM_ANGLE,158)
						PARAM_SET(148)         //  SE
					ELSE
						PARAM_SET(148)         //  S
					END
				END
			ELSE
				//>= 203                        //  SW     W      NW       N
				IF_LT(WIND_FROM_ANGLE,293)                 //  SW     W
					IF_LT(WIND_FROM_ANGLE,248)
						PARAM_SET(-243)        //  SW
					ELSE
						PARAM_SET(-243)        //  W
					END
				ELSE                            //  NW       N
					IF_LT(WIND_FROM_ANGLE,338)
						PARAM_SET(-326)        //  NW
					ELSE
						PARAM_SET(-326)        //  N
					END
				END
			END

			IF_GT(PARAM,0)  //param > 0
				//right hand circuit
				SET_ANGLE_PARAM   //virtually upwind of runway
				FD(165)
				RT(90)
			ELSE
				//left hand circuit
				PARAM_MUL(-1)
				SET_ANGLE_PARAM   //virtually upwind of runway
				PARAM_MUL(-1)
				FD(165)
				LT(90)
			END
			FD(150)  // total length of BASE leg
		PEN_DOWN    //to entry point

		//point to downwind
		IF_GE(PARAM,0)  //param => 0
			//right hand circuit
			SET_ANGLE_PARAM   //virtually upwind of runway
		ELSE
			//left hand circuit
			PARAM_MUL(-1)
			SET_ANGLE_PARAM   //virtually upwind of runway
			PARAM_MUL(-1)
		END
		//reverse heading, same for left and right
		RT(180)

		FD(DESIRED_SPEED_NORMAL_F0/3) //~3 sec  bring pattern closer to Home

		//set up an up-slope to entry point  in case we are low, else just glide there
		REPEAT_FOREVER
			//We have arrived above the circuit entry point
			//If high, fly circles down to 80m
			//if low (<50m) go straight for an "emergency" final  (dont know if the charge level of the battery is sufficient)

			IF_GT(ALT,(FINAL_ALT*4)+0)  // < ~16m can be removed on Downwind, else spiral down
				DO (SET_ALT_ALT)
				//DO (FS_SET_ALT_ALT)
				DO (DESCENT_PATTERN)   //use a subroutine to preserve PARAM with circuit data
				//DO (FS_DESCENT_PATTERN)   //use a subroutine to preserve PARAM with circuit data
			ELSE
				IF_GT(ALT,(FINAL_ALT)+10)  //else go straight for an "emergency" final
					DO (SET_ALT_ALT)
					//DO (FS_SET_ALT_ALT)

					// goto circuit entry point again / hoog aanknopings punt
					PEN_UP

						HOME

						IF_GE(PARAM,0)  //param => 0
							//right hand circuit
							SET_ANGLE_PARAM   //virtually upwind of runway
							FD(165)
							RT(90)
						ELSE
							//left hand circuit
							PARAM_MUL(-1)
							SET_ANGLE_PARAM   //virtually upwind of runway
							PARAM_MUL(-1)
							FD(165)
							LT(90)
						END
						FD(150)
					PEN_DOWN

					//Downwind
					DO_PARAM (DOWNWIND)
					//DO_PARAM (FS_DOWNWIND)

					//Base
					DO_PARAM (BASE)
					//DO_PARAM (FS_BASE)
				END // >50 m

				//Final
				DO_PARAM (FINAL)
				//DO_PARAM (FS_FINAL)

			END // 70m
		END // repeat
	END
	END



	TO (DOWNWIND)
	//TO (FS_DOWNWIND)
		// assume 50m < alt < 80m

		SET_ALT((FINAL_ALT*2+5))
		FLAG_ON(F_LAND) //brake if you have to
		//turn to and fly the downwind leg
		PEN_UP
			HOME
			IF_GE(PARAM,0)  //param => 0
				//right hand circuit
				SET_ANGLE_PARAM   //virtually upwind of runway
				RT(90)
				FD(150)
				RT(90)
			ELSE
				//left hand circuit
				PARAM_MUL(-1)
				SET_ANGLE_PARAM   //virtually upwind of runway
				PARAM_MUL(-1)
				LT(90)
				FD(150)
				LT(90)
			END
			FD(135)
		PEN_DOWN

		FLAG_OFF(F_LAND) // no braking in turns
		IF_GE(PARAM,0)  //param => 0
			//right hand circuit
			REPEAT(5)
				RT(10)
				FD(DESIRED_SPEED_NORMAL_F0/10)
				IF_GT(ALT,FINAL_ALT*2)
					ALT_DOWN(1)
				END
			END
		ELSE
			//left hand circuit
			REPEAT(5)
				LT(10)
				FD(DESIRED_SPEED_NORMAL_F0/10)
				IF_GT(ALT,FINAL_ALT*2)
					ALT_DOWN(1)
				END
			END
		END
	END
	END



	TO (BASE)
	//TO (FS_BASE)
		//assume alt less than 50m

		FLAG_OFF(F_LAND) // no braking in turns

		//turn to and fly the base leg
		SET_SPEED(DESIRED_SPEED_NORMAL_F0) //dm/s
		//set up the startingpoint of the glideslope
		IF_GE(PARAM,0)  //param => 0
			//right hand circuit
			REPEAT(5)               //5 not 4: compensate offset flighttrack vs waypoint for precise landing
				RT(10)
				FD(DESIRED_SPEED_NORMAL_F0/10)
				IF_GT(ALT,FINAL_ALT*2-5)
					ALT_DOWN(1)
				END
			END
			RT(5)
		ELSE
			//left hand circuit
			REPEAT(5)               //5 not 4: compensate offset flighttrack vs waypoint for precise landing
				LT(10)
				FD(DESIRED_SPEED_NORMAL_F0/10)
				IF_GT(ALT,FINAL_ALT*2-5)
					ALT_DOWN(1)
				END
			END
			LT(5)
		END
		SET_ALT(FINAL_ALT+5)
		PEN_UP     //base leg
			//set up the endpoint of the glideslope
			FD(70)
			FLAG_ON(F_LAND)    //use brakes / butterfly
		PEN_DOWN

		DO (SET_ALT_ALT)
		//DO (FS_SET_ALT_ALT)
		FLAG_OFF(F_LAND)    // no brakes in turn
		IF_GE(PARAM,0)  //param => 0
			//right hand circuit
			REPEAT(5)
				RT(10)
				FD(DESIRED_SPEED_NORMAL_F0/10)
				IF_GT(ALT,FINAL_ALT)
					ALT_DOWN(1)
				END
			END
			RT(5)
		ELSE
			//left hand circuit
			REPEAT(5)
				LT(10)
				FD(DESIRED_SPEED_NORMAL_F0/10)
				IF_GT(ALT,FINAL_ALT)
					ALT_DOWN(1)
				END
			END
			LT(5)
		END
	END
	END



	TO (FINAL)
	//TO (FS_FINAL)
		//turn to and fly the final

		//assume alt less than 50m  (target 35m)
		FLAG_OFF(F_LAND)    // no brakes in turn
		//set up the startingpoint of the glideslope
		IF_GE(PARAM,0)  //param => 0
			//right hand circuit
			REPEAT(4)
				RT(10)
				FD(DESIRED_SPEED_NORMAL_F0/10)
				IF_GT(ALT,FINAL_ALT)
					ALT_DOWN(1)
				END
			END
		ELSE
			//left hand circuit
			REPEAT(4)
				LT(10)
				FD(DESIRED_SPEED_NORMAL_F0/10)
				IF_GT(ALT,FINAL_ALT)
					ALT_DOWN(1)
				END
			END
		END


		//set up the endpoint of the glideslope   1:7
		FLAG_ON(F_LAND)     //use brakes / butterfly
		SET_ALT(0) //target altitude for touchdown
		PEN_UP
			HOME
			USE_ANGLE_TO_GOAL
			//compensate offset in flighttrack vs waypoint for precise landing
			IF_GE(PARAM,0)  //param => 0
				//right hand circuit
				RT(12)
			ELSE
				//left hand circuit
				LT(12)
			END
		PEN_DOWN

		SET_ALT(-40) //target altitude for the next waypoint below ground
		FD(160)

		// go around if landing did not happen
		EXEC (LOGO_MAIN)

	END
	END


};

////////////////////////////////////////////////////////////////////////////////
// RTL Flight Plan
//
// On entering RTL mode, turn off the engine, fly home, and circle indefinitely until touching down

const struct logoInstructionDef rtlInstructions[] = {


	EXEC (FS_LOITER_LAND)
	END
	END



	//TO (SET_ALT_ALT)
	TO (FS_SET_ALT_ALT)
		//target the current altitude, preventing motor or butterfly

		//this is used for setting up the starting point of a slope
		//note: SET_ALT(ALT) is wrong!!!
		LOAD_TO_PARAM(ALT)
		SET_ALT_PARAM
	END
	END



	//TO (DESCENT_PATTERN)
	TO (FS_DESCENT_PATTERN)
		//turn
		REPEAT(18)
			IF_GT(ALT,FINAL_ALT*3+5)
				DO (SET_ALT_ALT)    //keep aware of current altitude
				ALT_DOWN(8)  //keep going down  , this and FD controls brakes
				FLAG_ON(F_LAND) //brake if you have to
			ELSE
				FLAG_OFF(F_LAND)  // no brakes
			END
			RT(20)
			FD(DESIRED_SPEED_NORMAL_F0/5)
		END
	END
	END



	//TO (LOITER_LAND)
	TO (FS_LOITER_LAND)
		//Fly to circuit entrypoint

		//Motor may be activated (slider up) if < 80m and if the charge level of the battery is sufficient
		//Glide if > 80m
		//When close to the entry point;
		//If high, circle down to 80m
		//if low (<50m) go straight for an "emergency" final, this may be with tailwind (dont know if the charge level of the battery is sufficient)
		CLEAR_INTERRUPT
		FLAG_ON(F_CROSS_TRACK)
		SET_SPEED(DESIRED_SPEED_NORMAL_F0) //dm/s

		//DO (SET_ALT_ALT)           //level to upwind point
		DO (FS_SET_ALT_ALT)           //level to upwind point
		IF_LT(ALT,FINAL_ALT*3)
			IF_GT(ALT,FINAL_ALT)   // only if not too low
				SET_ALT((FINAL_ALT*3)+0)
			END
		END
		FD(DESIRED_SPEED_NORMAL_F0/10)
		ALT_DOWN(10)  //start going down
		//SET_ALT((FINAL_ALT*3)+0)

		//Motor may be activated if low (But not too low) and if the charge level of the battery is sufficient
		//decide if brakes or motor are needed most here
		//high: brakes if needed, <final: brakes, else motor + glide
		//downside: heavy thermals are not adressed, will be only after arrival above entry point
		FLAG_ON(F_LAND)  //brakes
		IF_LT(ALT,FINAL_ALT*3+40) //won't make it gliding (this does not change arrival point altitude)
			IF_GT(ALT,FINAL_ALT)
				FLAG_OFF(F_LAND)  //Motor on if needed, no brakes
			END
		END

		//to entry point
		PEN_UP

			// goto circuit entry point / hoog aanknopings punt
			HOME
			//SET_ALT((FINAL_ALT*3)+0)

			// Get circuit hand and heading  - configure these headings for your flying field
			// -270 == Lefthand circuit, final heading West 270, 270 == Righthand circuit, final heading West 270
			//     NE	E	|	SE		S   ||   SW        W   |   NW       N|
			//  23-67,68-112,113-157,158-202, 203-247,248-292,293-337,338-22
			//                                 wind FROM
			IF_LT(WIND_FROM_ANGLE,203)                    //  NE     E       SE       S   (N)
				IF_LT(WIND_FROM_ANGLE,113)                //  NE     E   (N)
					IF_LT(WIND_FROM_ANGLE,64)             //  N(E)
						IF_LT(WIND_FROM_ANGLE,22)         //  0..22
							PARAM_SET(-326)    //  N
						ELSE
							PARAM_SET(64)      //  NE
						END
					ELSE
						PARAM_SET(64)          //  E
					END
				ELSE                            //  SE       S
					IF_LT(WIND_FROM_ANGLE,158)
						PARAM_SET(148)         //  SE
					ELSE
						PARAM_SET(148)         //  S
					END
				END
			ELSE
				//>= 203                        //  SW     W      NW       N
				IF_LT(WIND_FROM_ANGLE,293)                 //  SW     W
					IF_LT(WIND_FROM_ANGLE,248)
						PARAM_SET(-243)        //  SW
					ELSE
						PARAM_SET(-243)        //  W
					END
				ELSE                            //  NW       N
					IF_LT(WIND_FROM_ANGLE,338)
						PARAM_SET(-326)        //  NW
					ELSE
						PARAM_SET(-326)        //  N
					END
				END
			END

			IF_GT(PARAM,0)  //param > 0
				//right hand circuit
				SET_ANGLE_PARAM   //virtually upwind of runway
				FD(165)
				RT(90)
			ELSE
				//left hand circuit
				PARAM_MUL(-1)
				SET_ANGLE_PARAM   //virtually upwind of runway
				PARAM_MUL(-1)
				FD(165)
				LT(90)
			END
			FD(150)  // total length of BASE leg
		PEN_DOWN    //to entry point

		//point to downwind
		IF_GE(PARAM,0)  //param => 0
			//right hand circuit
			SET_ANGLE_PARAM   //virtually upwind of runway
		ELSE
			//left hand circuit
			PARAM_MUL(-1)
			SET_ANGLE_PARAM   //virtually upwind of runway
			PARAM_MUL(-1)
		END
		//reverse heading, same for left and right
		RT(180)

		FD(DESIRED_SPEED_NORMAL_F0/3) //~3 sec  bring pattern closer to Home

		//set up an up-slope to entry point  in case we are low, else just glide there
		REPEAT_FOREVER
			//We have arrived above the circuit entry point
			//If high, fly circles down to 80m
			//if low (<50m) go straight for an "emergency" final  (dont know if the charge level of the battery is sufficient)

			IF_GT(ALT,(FINAL_ALT*4)+0)  // < ~16m can be removed on Downwind, else spiral down
				//DO (SET_ALT_ALT)
				DO (FS_SET_ALT_ALT)
				//DO (DESCENT_PATTERN)   //use a subroutine to preserve PARAM with circuit data
				DO (FS_DESCENT_PATTERN)   //use a subroutine to preserve PARAM with circuit data
			ELSE
				IF_GT(ALT,(FINAL_ALT)+10)  //else go straight for an "emergency" final
					//DO (SET_ALT_ALT)
					DO (FS_SET_ALT_ALT)

					// goto circuit entry point again / hoog aanknopings punt
					PEN_UP

						HOME

						IF_GE(PARAM,0)  //param => 0
							//right hand circuit
							SET_ANGLE_PARAM   //virtually upwind of runway
							FD(165)
							RT(90)
						ELSE
							//left hand circuit
							PARAM_MUL(-1)
							SET_ANGLE_PARAM   //virtually upwind of runway
							PARAM_MUL(-1)
							FD(165)
							LT(90)
						END
						FD(150)
					PEN_DOWN

					//Downwind
					//DO_PARAM (DOWNWIND)
					DO_PARAM (FS_DOWNWIND)

					//Base
					//DO_PARAM (BASE)
					DO_PARAM (FS_BASE)
				END // >50 m

				//Final
				//DO_PARAM (FINAL)
				DO_PARAM (FS_FINAL)

			END // 70m
		END // repeat
	END
	END



	//TO (DOWNWIND)
	TO (FS_DOWNWIND)
		// assume 50m < alt < 80m

		SET_ALT((FINAL_ALT*2))
		FLAG_ON(F_LAND) //brake if you have to
		//turn to and fly the downwind leg
		PEN_UP
			HOME
			IF_GE(PARAM,0)  //param => 0
				//right hand circuit
				SET_ANGLE_PARAM   //virtually upwind of runway
				RT(90)
				FD(150)
				RT(90)
			ELSE
				//left hand circuit
				PARAM_MUL(-1)
				SET_ANGLE_PARAM   //virtually upwind of runway
				PARAM_MUL(-1)
				LT(90)
				FD(150)
				LT(90)
			END
			FD(135)
		PEN_DOWN

		FLAG_OFF(F_LAND) // no braking in turns
		IF_GE(PARAM,0)  //param => 0
			//right hand circuit
			REPEAT(5)
				RT(10)
				FD(DESIRED_SPEED_NORMAL_F0/10)
				IF_GT(ALT,FINAL_ALT*2)
					ALT_DOWN(1)
				END
			END
		ELSE
			//left hand circuit
			REPEAT(5)
				LT(10)
				FD(DESIRED_SPEED_NORMAL_F0/10)
				IF_GT(ALT,FINAL_ALT*2)
					ALT_DOWN(1)
				END
			END
		END
	END
	END



	//TO (BASE)
	TO (FS_BASE)
		//assume alt less than 50m

		FLAG_OFF(F_LAND) // no braking in turns

		//turn to and fly the base leg
		SET_SPEED(DESIRED_SPEED_NORMAL_F0) //dm/s
		//set up the startingpoint of the glideslope
		IF_GE(PARAM,0)  //param => 0
			//right hand circuit
			REPEAT(5)               //5 not 4: compensate offset flighttrack vs waypoint for precise landing
				RT(10)
				FD(DESIRED_SPEED_NORMAL_F0/10)
				IF_GT(ALT,FINAL_ALT*2-5)
					ALT_DOWN(1)
				END
			END
			RT(5)
		ELSE
			//left hand circuit
			REPEAT(5)               //5 not 4: compensate offset flighttrack vs waypoint for precise landing
				LT(10)
				FD(DESIRED_SPEED_NORMAL_F0/10)
				IF_GT(ALT,FINAL_ALT*2-5)
					ALT_DOWN(1)
				END
			END
			LT(5)
		END
		SET_ALT(FINAL_ALT)
		PEN_UP     //base leg
			//set up the endpoint of the glideslope
			FD(70)
			FLAG_ON(F_LAND)    //use brakes / butterfly
		PEN_DOWN

		//DO (SET_ALT_ALT)
		DO (FS_SET_ALT_ALT)
		FLAG_OFF(F_LAND)    // no brakes in turn
		IF_GE(PARAM,0)  //param => 0
			//right hand circuit
			REPEAT(5)
				RT(10)
				FD(DESIRED_SPEED_NORMAL_F0/10)
				IF_GT(ALT,FINAL_ALT-5)
					ALT_DOWN(2)
				END
			END
			RT(5)
		ELSE
			//left hand circuit
			REPEAT(5)
				LT(10)
				FD(DESIRED_SPEED_NORMAL_F0/10)
				IF_GT(ALT,FINAL_ALT-5)
					ALT_DOWN(2)
				END
			END
			LT(5)
		END
	END
	END



	//TO (FINAL)
	TO (FS_FINAL)
		//turn to and fly the final

		//assume alt less than 50m  (target 35m)
		FLAG_OFF(F_LAND)    // no brakes in turn
		//set up the startingpoint of the glideslope
		IF_GE(PARAM,0)  //param => 0
			//right hand circuit
			REPEAT(4)
				RT(10)
				FD(DESIRED_SPEED_NORMAL_F0/10)
				IF_GT(ALT,FINAL_ALT)
					ALT_DOWN(2)
				END
			END
		ELSE
			//left hand circuit
			REPEAT(4)
				LT(10)
				FD(DESIRED_SPEED_NORMAL_F0/10)
				IF_GT(ALT,FINAL_ALT)
					ALT_DOWN(2)
				END
			END
		END


		//set up the endpoint of the glideslope   1:7
		FLAG_ON(F_LAND)     //use brakes / butterfly
		SET_ALT(0) //target altitude for touchdown
		PEN_UP
			HOME
			USE_ANGLE_TO_GOAL
			//compensate offset in flighttrack vs waypoint for precise landing
			IF_GE(PARAM,0)  //param => 0
				//right hand circuit
				RT(12)
			ELSE
				//left hand circuit
				LT(12)
			END

		PEN_DOWN
		SET_ALT(-40) //target altitude for the next waypoint below ground
		FD(160)

		// go around if landing did not happen
		EXEC (LOGO_MAIN)

	END
	END


};

#endif