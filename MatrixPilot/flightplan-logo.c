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


#include "defines.h"
#include "config.h"
#include "navigate.h"
#include "behaviour.h"
#include "flightplan.h"
#include "cameraCntrl.h"
#include "altitudeCntrl.h"
#include "flightplan_logo.h"
#include "../libDCM/rmat.h"
#include "../libDCM/estWind.h"
#include "../libDCM/mathlibNAV.h"
#include "../libDCM/deadReckoning.h"
#include "../libDCM/gpsParseCommon.h"
#include <stdlib.h>

//#if (FLIGHT_PLAN_TYPE == FP_LOGO)


struct logoInstructionDef {
	uint16_t cmd        :  6;
	uint16_t do_fly     :  1;
	uint16_t use_param  :  1;
	uint16_t subcmd     :  8;
	int16_t arg         : 16;
};

#define PLANE           0
#define CAMERA          1

// Note that any instruction with an odd subcmd is a FLY command.
// Interpretation stops on a FLY command until the plane arrives at that
// location, similar to a waypoint.  This includes PEN_DOWN.
// When the pen is up, FLY commands do not stop the interpreter.  So when
// the pen goes back down, we FLY to wherever the turtle has moved to
// while the pen was up.  We also skip flying when the CAMERA turtle is
// the active turtle.

// Define the conditional VAL values for IF commands
enum {
	LOGO_VAL_ZERO = 0,
	// XX_INPUT_CHANNEL // leave room for input channels: 1 - NUM_INPUTS (up to 15)
	DIST_TO_HOME = 16,
	DIST_TO_GOAL,
	ALT,
	CURRENT_ANGLE,
	ANGLE_TO_HOME,
	ANGLE_TO_GOAL,
	REL_ANGLE_TO_HOME,
	REL_ANGLE_TO_GOAL,
#if ( THERMALLING_MISSION == 1 )  //custom commands
	REL_ANGLE_TO_OPPOSITE, 
	REL_ANGLE_TO_WIND,
#endif
	GROUND_SPEED,
	AIR_SPEED,
	AIR_SPEED_Z,   // if THERMALLING_MISSION custom command
	WIND_SPEED,
	WIND_SPEED_X,
	WIND_SPEED_Y,
	WIND_SPEED_Z,
	WIND_FROM_ANGLE,
#if ( THERMALLING_MISSION == 1 )  //custom system values
	BATTERY_VOLTAGE,
	AIR_SPEED_Z_DELTA,
	AIR_SPEED_Z_VS_START,
	CLEAR_Z_BEST,
	READ_F_LAND,
 	GEOFENCE_STATUS,
	GEOFENCE_TURN,
	MOTOR_OFF_TIMER,
	READ_DESIRED_SPEED,
	READ_THROTTLE_OUTPUT_CHANNEL,
	FORCE_CROSS_FINISH_LINE,
	READ_FLY_COMMAND_COUNTER,
	FORCE_FINISH_BAD_NAV,
	FORCE_RESET,
	SET_DIRECTION,
#endif
	PARAM
};

// Define the Low-level Commands
//                             cmd, fly,param,sub,   x
#define _REPEAT(n, pr)          {1,   0,   pr,  0,   n},
#define _END                    {1,   0,   0,   1,   0},
#define _ELSE                   {1,   0,   0,   3,   0},
#define _TO(fn)                 {1,   0,   0,   2,   fn},

#define _DO(fn, x, pr)          {2,   0,   pr,  fn,  x},
#define _EXEC(fn, x, pr)        {10,  0,   pr,  fn,  x},

#define _FD(x, fl, pr)          {3,   fl,  pr,  0,   x},
#if ( THERMALLING_MISSION == 1 )
#define _FIXED_BANK_ROTATE(x, fl, pr)     {3,   fl,  pr,  1,   x},  //custom command
#define _BANK_1S(x, fl, pr)    	{3,   fl,  pr,  2,   x},  //custom command
#endif

#define _RT(x, pr)              {4,   0,   pr,  0,   x},
#define _SET_ANGLE(x, pr)       {4,   0,   pr,  1,   x},
#define _USE_CURRENT_ANGLE      {4,   0,   0,   2,   0},
#define _USE_ANGLE_TO_GOAL      {4,   0,   0,   3,   0},

#define _MV_X(x, fl, pr)        {5,   fl,  pr,  0,   x},
#define _SET_X(x, fl, pr)       {5,   fl,  pr,  1,   x},
#define _MV_Y(y, fl, pr)        {5,   fl,  pr,  2,   y},
#define _SET_Y(y, fl, pr)       {5,   fl,  pr,  3,   y},
#define _MV_Z(z, fl, pr)        {5,   fl,  pr,  4,   z},
#define _SET_Z(z, fl, pr)       {5,   fl,  pr,  5,   z},
#define _USE_CURRENT_POS(fl)    {5,   fl,  0,   6,   0},
#define _HOME(fl)               {5,   fl,  0,   7,   0},

#define _SET_ABS_VAL_HIGH(x)    {5,   0,   0,   8,   x}, // Set the high and then low words for X and
#define _SET_ABS_X_LOW(x)       {5,   0,   0,   9,   x}, // then Y, as 4 consecutive instructions.
#define _SET_ABS_Y_LOW(y, fl)   {5,   fl,  0,   10,  y}, // (as VAL_HIGH, X_LOW, VAL_HIGH, Y_LOW)

#define _FLAG_ON(f)             {6,   0,   0,   0,   f},
#define _FLAG_OFF(f)            {6,   0,   0,   1,   f},
#define _FLAG_TOGGLE(f)         {6,   0,   0,   2,   f},

#define _PEN_UP                 {7,   0,   0,   0,   0},
#define _PEN_DOWN               {7,   1,   0,   1,   0},
#define _PEN_TOGGLE             {7,   0,   0,   2,   0},

#define _SET_TURTLE(x)          {8,   0,   0,   0,   x},

#define _PARAM_SET(x)           {9,   0,   0,   0,   x},
#define _PARAM_ADD(x)           {9,   0,   0,   1,   x},
#define _PARAM_MUL(x)           {9,   0,   0,   2,   x},
#define _PARAM_DIV(x)           {9,   0,   0,   3,   x},

#define _SPEED_INCREASE(s, pr)  {11,  0,   pr,  0,   s},
#define _SET_SPEED(s, pr)       {11,  0,   pr,  1,   s},

#define _SET_INTERRUPT(fn)      {12,  0,   0,   1,   fn},
#define _CLEAR_INTERRUPT        {12,  0,   0,   0,   0},

#define _LOAD_TO_PARAM(val)     {13,  0,   0,   val, 0},

#define _IF_EQ(val, x, pr)      {14,  0,   pr,  val, x},
#define _IF_NE(val, x, pr)      {15,  0,   pr,  val, x},
#define _IF_GT(val, x, pr)      {16,  0,   pr,  val, x},
#define _IF_LT(val, x, pr)      {17,  0,   pr,  val, x},
#define _IF_GE(val, x, pr)      {18,  0,   pr,  val, x},
#define _IF_LE(val, x, pr)      {19,  0,   pr,  val, x},

// Define the High-level Commands
#define FD(x)                   _FD(x, 1, 0)
#define BK(x)                   _FD(-x, 1, 0)
#define FD_PARAM                _FD(1, 1, 1)
#define BK_PARAM                _FD(-1, 1, 1)

#if ( THERMALLING_MISSION == 1 )
#define FIXED_BANK_ROTATE(x)              _FIXED_BANK_ROTATE(x, 1, 0)           //custom command
#define BANK_1S(x)             	_BANK_1S(x, 1, 0)        //custom command
#endif

#define RT(x)                   _RT(x, 0)
#define LT(x)                   _RT(-x, 0)
#define SET_ANGLE(x)            _SET_ANGLE(x, 0)
#define RT_PARAM                _RT(1, 1)
#define LT_PARAM                _RT(-1, 1)
#define SET_ANGLE_PARAM         _SET_ANGLE(0, 1)
#define USE_CURRENT_ANGLE       _USE_CURRENT_ANGLE
#define USE_ANGLE_TO_GOAL       _USE_ANGLE_TO_GOAL

#define EAST(x)                 _MV_X(x, 1, 0)
#define WEST(x)                 _MV_X(-x, 1, 0)
#define SET_X_POS(x)            _SET_X(x, 1, 0)
#define EAST_PARAM              _MV_X(1, 1, 1)
#define WEST_PARAM              _MV_X(-1, 1, 1)
#define SET_X_POS_PARAM         _SET_X(1, 1, 1)
#define USE_CURRENT_POS         _USE_CURRENT_POS(1)

#define NORTH(y)                _MV_Y(y, 1, 0)
#define SOUTH(y)                _MV_Y(-y, 1, 0)
#define SET_Y_POS(y)            _SET_Y(y, 1, 0)
#define NORTH_PARAM             _MV_Y(1, 1, 1)
#define SOUTH_PARAM             _MV_Y(-1, 1, 1)
#define SET_Y_POS_PARAM         _SET_Y(1, 1, 1)

#define ALT_UP(z)               _MV_Z(z, 0, 0)
#define ALT_DOWN(z)             _MV_Z(-z, 0, 0)
#define SET_ALT(z)              _SET_Z(z, 0, 0)
#define ALT_UP_PARAM            _MV_Z(1, 0, 1)
#define ALT_DOWN_PARAM          _MV_Z(-1, 0, 1)
#define SET_ALT_PARAM           _SET_Z(1, 0, 1)

#define SPEED_INCREASE(x)       _SPEED_INCREASE(x, 0)
#define SPEED_DECREASE(x)       _SPEED_INCREASE(-x, 0)
#define SET_SPEED(x)            _SET_SPEED(x, 0)
#define SPEED_INCREASE_PARAM    _SPEED_INCREASE(1, 1)
#define SPEED_DECREASE_PARAM    _SPEED_INCREASE(-1, 1)
#define SET_SPEED_PARAM         _SET_SPEED(0, 1)

#define FLAG_ON(f)              _FLAG_ON(f)
#define FLAG_OFF(f)             _FLAG_OFF(f)
#define FLAG_TOGGLE(f)          _FLAG_TOGGLE(f)

#define PEN_UP                  _PEN_UP
#define PEN_DOWN                _PEN_DOWN
#define PEN_TOGGLE              _PEN_TOGGLE

#define SET_TURTLE(x)           _SET_TURTLE(x)

#define REPEAT(n)               _REPEAT(n, 0)
#define REPEAT_PARAM            _REPEAT(1, 1)
#define REPEAT_FOREVER          _REPEAT(-1, 0)
#define END                     _END
#define ELSE                    _ELSE

#define TO(func)                _TO(func)

#define DO(func)                _DO(func, 0, 0)
#define DO_ARG(func, arg)       _DO(func, arg, 0)
#define DO_PARAM(func)          _DO(func, 1, 1)

#define EXEC(func)              _EXEC(func, 0, 0)
#define EXEC_ARG(func, arg)     _EXEC(func, arg, 0)
#define EXEC_PARAM(func)        _EXEC(func, 1, 1)

#define PARAM_SET(x)            _PARAM_SET(x)
#define PARAM_ADD(x)            _PARAM_ADD(x)
#define PARAM_SUB(x)            _PARAM_ADD(-x)
#define PARAM_MUL(x)            _PARAM_MUL(x)
#define PARAM_DIV(x)            _PARAM_DIV(x)

#define SET_INTERRUPT(fn)       _SET_INTERRUPT(fn)
#define CLEAR_INTERRUPT         _CLEAR_INTERRUPT

#define LOAD_TO_PARAM(val)      _LOAD_TO_PARAM(val)

#define IF_EQ(val, x)           _IF_EQ(val, x, 0)
#define IF_NE(val, x)           _IF_NE(val, x, 0)
#define IF_GT(val, x)           _IF_GT(val, x, 0)
#define IF_LT(val, x)           _IF_LT(val, x, 0)
#define IF_GE(val, x)           _IF_GE(val, x, 0)
#define IF_LE(val, x)           _IF_LE(val, x, 0)
#define IF_EQ_PARAM(val)        _IF_EQ(val, 1, 1)
#define IF_NE_PARAM(val)        _IF_NE(val, 1, 1)
#define IF_GT_PARAM(val)        _IF_GT(val, 1, 1)
#define IF_LT_PARAM(val)        _IF_LT(val, 1, 1)
#define IF_GE_PARAM(val)        _IF_GE(val, 1, 1)
#define IF_LE_PARAM(val)        _IF_LE(val, 1, 1)

#define SET_POS(x, y)           _SET_X(x, 0, 0) _SET_Y(y, 1, 0)
#define SET_ABS_POS(x, y)       _SET_ABS_VAL_HIGH((((uint32_t)(x))>>16)&0xFFFF) _SET_ABS_X_LOW(((uint32_t)(x))&0xFFFF) \
                                _SET_ABS_VAL_HIGH((((uint32_t)(y))>>16)&0xFFFF) _SET_ABS_Y_LOW(((uint32_t)(y))&0xFFFF, 1)
#define HOME                    _HOME(1)

#define LOGO_MAIN   0   // Allows for DO(LOGO_MAIN) or EXEC(LOGO_MAIN) to start at the top


#include "flightplan-logo.h"


#define NUM_INSTRUCTIONS ((sizeof instructions) / sizeof (struct logoInstructionDef))
#define NUM_RTL_INSTRUCTIONS ((sizeof rtlInstructions) / sizeof (struct logoInstructionDef))
static int16_t instructionIndex = 0;
//int16_t waypointIndex = 0; // used for telemetry
static int16_t absoluteHighWord = 0;
static union longww absoluteXLong;

static struct logoInstructionDef *currentInstructionSet = (struct logoInstructionDef*)instructions;
static int16_t numInstructionsInCurrentSet = NUM_INSTRUCTIONS;

#if ( THERMALLING_MISSION == 1 )
int16_t vario = 0;// in cm/s ,used for Logo  - defined in flightplan_logo.c and set in altitudeCntrlVariable.c - running average (3s)
static int16_t vario_old = 0;// in cm/s ,used for AIR_SPEED_Z_DELTA
static int16_t airSpeedZAverage;
int16_t fixedBankTargetAngle = 0; // heading,  used for Logo  - for FIXED_BANK_ROTATE command
int16_t fixedBankActiveCounter; //  used for Logo  - for FIXED_BANK_ROTATE and BANK_1S commands
boolean fixedBankActive = false; // used for Logo  - for FIXED_BANK_ROTATE and BANK_1S commands
boolean angleTargetActive = false; // used for Logo  - for FIXED_BANK_ROTATE command
int16_t fixedBankDeg;  // deg bank, used for Logo  - for FIXED_BANK_ROTATE and BANK_1S commands
static int16_t oldAngle;      // for SET_DIRECTION and FIXED_BANK_ROTATE   set by BANK_1S
static boolean rotateClockwise;  //topview   for SET_DIRECTION and FIXED_BANK_ROTATE

#ifndef THERMALLING_TURN     // in case not specified
#define THERMALLING_TURN  0  // not implemented
#endif
static int16_t get_current_angle(void);
static int16_t motorOffTimer = 0;
static int16_t airSpeedZStart = 0;   //climbrate at the start of a thermal turn
static float avgBatteryVoltage = 110;  //kickstart average filter with nominal value; it only starts when LOGO starts      
static int16_t flyCommandCounter = 0;  //count up 40 times per sec when in a fly command
static int16_t airSpeedZBest = 0;
static int16_t airSpeedZBestCount = 0;  //used in AIR_SPEED_Z_VS_START
#if ( MY_PERSONAL_OPTIONS == 1 )
boolean regularFlyingField; // declared and used by flightplan-logo.c and set by telemetry.c 
boolean forceCrossFinishLine;   //used by interrupt routine to sigmal an event that needs immediate action
static boolean forceFinishReset = false;   //used by interrupt routine to sigmal an event that needs immediate action
#endif


//Geofence

// define a struct with geofence shapes
typedef struct tag_geofenceShape {
	float ellips_diameter_x;
	float ellips_diameter_y;
	float fuzzy_factor;
	float linear_x_slope;
	float linear_y_slope;
	float offset_x_plus_y;
	int16_t sign;  
} geofenceShape;

//define an array of geofence shapes - define on what side of the lines the aicraft should be
//shapes can be any combination of lines, circles and ellipses
//Calculating the parameters:
//LOGO defines HOME as {x, y} = {0,0}. This is the centre of the geofence
//For a straight line, or the longitudinal axis of an ellipse:
//How many meters is the line from Home to North? "Fill it in at offset_x_plus_y.
//If x 'x + 1, what happens to y. Devide by 1000. use this in linear_y_slope  .
//For an ellipse
//What is the biggest East-West distance from your field? "The root of it? => 1 / ellips_diameter_x.  Devide by 1000
//What is the greatest distance north-south from your field? "The root of it? => 1 / ellips_diameter_y.   Devide by 1000
//                        (Square to put tilted ellipse)
//For a circle:
//Above you have already completed the same for ellips_diameter_x and ellips_diameter_y. No further news.
//And guess the fuzzy_factor. Start at 0, i have no idea what effect it has.

#define NUMB_OF_GEO_SHAPES      2  //per set
geofenceShape geofenceShapes[NUMB_OF_GEO_SHAPES * 2]=
{
	{0.0000013,0.0000013,0,0,0,0.78,1},   //circle, Lageweg, 400m radius
	{0,0,0,0.0015,0.00285,0.96,1},        //line, Lageweg, through Home, powerlines,  angle > 296 || angle < 116 
	{0.0000013,0.0000013,0,0,0,0.854,1},  //circle, Lageweg, ~350m radius
	{0,0,0,0.0015,0.00285,1.159,1}        //line, Lageweg, below Home, powerlines,  angle > 296 || angle < 116 
};

//define a set for outcomes of the geofence check, for left, ahead and right
typedef struct tag_geoScores {
	float geoScoreLeft;
	float geoScoreAhead;
	float geoScoreRight;
} geoScores;

geoScores geofenceScore;
geoScores geofenceScore_old;

int16_t geoTurn;       //turn -40, 0, or 40 deg in 4 sec    scope: flightplan_logo.c
int16_t geoStatus;     //0,1,2  0= soft/wind gf, 1=wind gf, 2 geofence (alarm)   scope: flightplan_logo.c  
static int16_t letHeartbeat;  //to get 1Hz timebase
int16_t bestFarScore = 0;          //of four directions, find the angle to the farthest point possible
int16_t bestFarScoreAngle = -1;
int16_t relAngleToOpposite = 0;  

#define AHEAD_PREFERENCE 1.0001    //used to give geofence turns less preference over straight ahead
//definitions, should be in header file?
float geoPreference(float x, float y, int16_t shapeNumb);   //use linear programmming to test position against a single geofence shape, return a score 0..1
void areaGeoScore(int16_t angle, int16_t numbOfDirections, int16_t metersAhead, int16_t windSeconds);  //return three scores 40m ahead, one for left 40 deg, one ahead , and one for right 40 deg windSeconds; translate x and y downwind, equivalent of x sec drift
void geoSetStatus(); // run once a second. windSeconds; translate x and y downwind, equivalent of x sec drift
void geoSetTurn();   // Call from LOGO. convert a set of score outcomes to a number of degrees for a turn (-40, 0, 40) from (set before) global variable   geofenceScore 

#endif //THERMALLING_MISSION



// If we've processed this many instructions without commanding the plane to fly,
// then stop and continue on the next run through

#if ( THERMALLING_MISSION == 1 )
#define MAX_INSTRUCTIONS_PER_CYCLE  10
#else
#define MAX_INSTRUCTIONS_PER_CYCLE  32
#endif

static int16_t instructionsProcessed = 0;

// Storage for command injection
static struct logoInstructionDef logo_inject_instr;
static uint8_t logo_inject_pos = 0;

#if ( MY_PERSONAL_OPTIONS == 1 )
boolean regularFlyingField; //  declared and used by flightplan_logo.c and set by telemetry.c
#endif

#define LOGO_INJECT_READY 255

// Storage for interrupt handling
static int16_t interruptIndex = 0;     // instruction index of the beginning of the interrupt function
static int8_t interruptStackBase = 0;  // stack depth when entering interrupt (clear interrupt when dropping below this depth)

// How many layers deep can Ifs, Repeats and Subroutines be nested
#define LOGO_STACK_DEPTH            12

struct logoStackFrame {
	uint16_t frameType              :  2;
	int16_t returnInstructionIndex  : 14;   // instructionIndex before the first instruction of the subroutine (a TO or REPEAT line, or -1 for MAIN)
	int16_t arg                     : 16;
};
struct logoStackFrame logoStack[LOGO_STACK_DEPTH];
static int16_t logoStackIndex = 0;

#define LOGO_FRAME_TYPE_IF          1
#define LOGO_FRAME_TYPE_REPEAT      2
#define LOGO_FRAME_TYPE_SUBROUTINE  3

// These values are relative to the origin, and North
// x and y are in 16.16 fixed point
struct logoLocation { 
	union longww x;
	union longww y;
	int16_t z;
};
static struct logoLocation turtleLocations[2];
static struct relative3D lastGoal = {0, 0, 0};

// Angles are stored as 0-359
static int16_t turtleAngles[2] = {0, 0};

static uint8_t currentTurtle;
static int16_t penState;

static boolean process_one_instruction(struct logoInstructionDef instr);
static void update_goal_from(struct relative3D old_waypoint);
static void process_instructions(void);

int16_t flightplan_logo_index_get(void)
{
	return waypointIndex;
}



void flightplan_logo_init(void)
{
	DPRINT("flightplan_logo_init()\r\n");
}

// In the future, we could include more than 2 flight plans...
// flightplanNum is 0 for the main lgo instructions, and 1 for RTL instructions
//void init_flightplan(int16_t flightplanNum)
void flightplan_logo_begin(int16_t flightplanNum)
{
	struct relative2D curHeading;
	struct relative3D IMUloc;
	int8_t earth_yaw;
	int16_t angle;

	if (flightplanNum == 1) // RTL instructions set
	{
		currentInstructionSet = (struct logoInstructionDef*)rtlInstructions;
		numInstructionsInCurrentSet = NUM_RTL_INSTRUCTIONS;
	}
	else if (flightplanNum == 0) // Main instructions set
	{
		currentInstructionSet = (struct logoInstructionDef*)instructions;
		numInstructionsInCurrentSet = NUM_INSTRUCTIONS;
	}

	instructionIndex = 0;

	logoStackIndex = 0;
	logoStack[logoStackIndex].frameType = LOGO_FRAME_TYPE_SUBROUTINE;
	logoStack[logoStackIndex].arg = 0;
	logoStack[logoStackIndex].returnInstructionIndex = -1;  // When starting over, begin on instruction 0

	currentTurtle = PLANE;
	penState = 0; // 0 means down.  more than 0 means up

	turtleLocations[PLANE].x._.W1 = IMUlocationx._.W1;
	turtleLocations[PLANE].y._.W1 = IMUlocationy._.W1;
	turtleLocations[PLANE].z = IMUlocationz._.W1;

	turtleLocations[CAMERA].x._.W1 = IMUlocationx._.W1;
	turtleLocations[CAMERA].y._.W1 = IMUlocationy._.W1;
	turtleLocations[CAMERA].z = IMUlocationz._.W1;

	// Calculate heading from Direction Cosine Matrix (rather than GPS), 
	// So that this code works when the plane is static. e.g. at takeoff
	curHeading.x = -rmat[1];
	curHeading.y = rmat[4];
	earth_yaw = rect_to_polar(&curHeading);  //  (0=East,  ccw)
	angle = (earth_yaw * 180 + 64) >> 7;    //  (ccw, 0=East)
	angle = -angle + 90;                            //  (clockwise, 0=North)
	turtleAngles[PLANE] = turtleAngles[CAMERA] = angle;

	setBehavior(0);

	IMUloc.x = IMUlocationx._.W1;
	IMUloc.y = IMUlocationy._.W1;
	IMUloc.z = IMUlocationz._.W1;
	update_goal_from(IMUloc);

	interruptIndex = 0;
	interruptStackBase = 0;

	process_instructions();
}

static boolean logo_goal_has_moved(void)
{
	return (lastGoal.x != turtleLocations[PLANE].x._.W1 ||
	        lastGoal.y != turtleLocations[PLANE].y._.W1 ||
	        lastGoal.z != turtleLocations[PLANE].z);
}

static void update_goal_from(struct relative3D old_goal)
{
	struct relative3D new_goal;
#ifdef USE_EXTENDED_NAV
	struct relative3D_32 old_goal_32, new_goal_32;
#endif

	lastGoal.x = new_goal.x = (turtleLocations[PLANE].x._.W1);
	lastGoal.y = new_goal.y = (turtleLocations[PLANE].y._.W1);
	lastGoal.z = new_goal.z = turtleLocations[PLANE].z;
	
	if (old_goal.x == new_goal.x && old_goal.y == new_goal.y)
	{
		old_goal.x = IMUlocationx._.W1;
		old_goal.y = IMUlocationy._.W1;
		old_goal.z = IMUlocationz._.W1;
	}

#ifdef USE_EXTENDED_NAV
	// TODO: RobD - review this change implemented to restore build, but not runtime tested
	old_goal_32.x = old_goal.x;
	old_goal_32.y = old_goal.y;
	old_goal_32.z = old_goal.z;
	new_goal_32.x = new_goal.x;
	new_goal_32.y = new_goal.y;
	new_goal_32.z = new_goal.z;
	navigate_set_goal(old_goal_32, new_goal_32);
#else
	navigate_set_goal(old_goal, new_goal);
#endif // USE_EXTENDED_NAV

	new_goal.x = (turtleLocations[CAMERA].x._.W1);
	new_goal.y = (turtleLocations[CAMERA].y._.W1);
	new_goal.z = turtleLocations[CAMERA].z;
	set_camera_view(new_goal);
}

void flightplan_logo_update(void)
{
	// first run any injected instruction from the serial port
	if (logo_inject_pos == LOGO_INJECT_READY)
	{
		process_one_instruction(logo_inject_instr);
		if (logo_inject_instr.cmd == 2 || logo_inject_instr.cmd == 10) // DO / EXEC
		{
			instructionIndex++;
			process_instructions();
		}
		else
		{
			if (logo_goal_has_moved())
			{
				update_goal_from(lastGoal);
				navigate_compute_bearing_to_goal();
			}
		}
		logo_inject_pos = 0;
		return;
	}

	// otherwise run the interrupt handler, if configured.
	if (interruptIndex)
	{
		if (tofinish_line >= WAYPOINT_PROXIMITY_RADIUS) // not crossed the finish line
		{
			if (!interruptStackBase)   //if not in-progress
			{
				if (logoStackIndex < LOGO_STACK_DEPTH-1)
				{
					logoStackIndex++;
					logoStack[logoStackIndex].frameType = LOGO_FRAME_TYPE_SUBROUTINE;
					logoStack[logoStackIndex].arg = 0;
					logoStack[logoStackIndex].returnInstructionIndex = instructionIndex-1;
					instructionIndex = interruptIndex+1;
					interruptStackBase = logoStackIndex;
				}
			}
			process_instructions();
			navigate_set_goal_height(turtleLocations[PLANE].z);
			lastGoal.z = turtleLocations[PLANE].z;
		}
		else
		{
			if (interruptStackBase)   //if in-progress
			{
				//cleanup uncompleted interrupt
				instructionIndex = logoStack[interruptStackBase].returnInstructionIndex+1;  //support both end by main and end by END SUBROUTINE
				logoStackIndex =  interruptStackBase - 1;
				interruptStackBase = 0;
			}
		}
	}

	// waypoint arrival is detected computing distance to the "finish line".
	// note: locations are measured in meters
	// locations have a range of +-32000 meters (20 miles) from origin

	if (desired_behavior._.altitude)
	{
		if (abs(IMUheight - navigate_get_goal(NULL)) < ((int16_t)altit.HeightMargin)) // reached altitude goal
		{
			desired_behavior._.altitude = 0;
			process_instructions();
		}
	}
	else
	{
#if ( THERMALLING_MISSION != 1 )
		if (tofinish_line < WAYPOINT_PROXIMITY_RADIUS) // crossed the finish line
		{
			process_instructions();
		}
#else
		// inhibit navigation for x loops, to allow drifting downwind
		if ( fixedBankActive )
		{
			if (!angleTargetActive) // BANK_1S()
			{
				if (fixedBankActiveCounter <= 0)
				{
					fixedBankActive = false;

					//a second has passed, so force this fly command to end

					// Use current position (for x and y)
					turtleLocations[currentTurtle].x._.W0 = 0;
					turtleLocations[currentTurtle].x._.W1 = IMUlocationx._.W1;
					turtleLocations[currentTurtle].y._.W0 = 0;
					turtleLocations[currentTurtle].y._.W1 = IMUlocationy._.W1;

					// move turtle to simulate arrival, to allow the program flow to continue right away
					int16_t cangle = turtleAngles[currentTurtle];   // 0-359 (clockwise, 0=North)
					int8_t b_angle = (cangle * 182 + 128) >> 8;     // 0-255 (clockwise, 0=North)
					b_angle = -b_angle - 64;                        // 0-255 (ccw, 0=East)

					turtleLocations[currentTurtle].x.WW += (__builtin_mulss(-cosine(b_angle), (int16_t)WAYPOINT_PROXIMITY_RADIUS) << 2);  //
					turtleLocations[currentTurtle].y.WW += (__builtin_mulss(-sine(b_angle), (int16_t)WAYPOINT_PROXIMITY_RADIUS) << 2);

					if (interruptStackBase)   //if in-progress
					{
						//cleanup uncompleted interrupt
						instructionIndex = logoStack[interruptStackBase].returnInstructionIndex+1;  //support both end by main and end by END SUBROUTINE
						logoStackIndex =  interruptStackBase - 1;
						interruptStackBase = 0;
					}
					process_instructions();  //as if arrived
				}
			}
			else    // FIXED_BANK_ROTATE
			{
				//check if target of 15 deg right has been reached  or timer times out
				if ( (fixedBankActiveCounter <= 0) |
					 ( rotateClockwise && ( ( ( get_current_angle() - fixedBankTargetAngle + 360) % 360 ) < 180 ) |
					 ( !rotateClockwise && ( ( ( get_current_angle() - fixedBankTargetAngle + 360) % 360 ) > 180 ) ) )
					 )// closest direction is right of target

				//use Gps heading
				//if ( ( ( cog_gpsBB - fixedBankTargetAngle + 360) % 360 ) < 180 ) // closest direction is right of target
				{
					fixedBankActive = false;

					//force this fly command to end

					// Use current position (for x and y)
					turtleLocations[currentTurtle].x._.W0 = 0;
					turtleLocations[currentTurtle].x._.W1 = IMUlocationx._.W1;
					turtleLocations[currentTurtle].y._.W0 = 0;
					turtleLocations[currentTurtle].y._.W1 = IMUlocationy._.W1;

					// move turtle to simulate arrival, to allow the program flow to continue right away
					int16_t cangle = turtleAngles[currentTurtle];   // 0-359 (clockwise, 0=North)
					int8_t b_angle = (cangle * 182 + 128) >> 8;     // 0-255 (clockwise, 0=North)
					b_angle = -b_angle - 64;                        // 0-255 (ccw, 0=East)

					turtleLocations[currentTurtle].x.WW += (__builtin_mulss(-cosine(b_angle), (int16_t)WAYPOINT_PROXIMITY_RADIUS) << 2);  //
					turtleLocations[currentTurtle].y.WW += (__builtin_mulss(-sine(b_angle), (int16_t)WAYPOINT_PROXIMITY_RADIUS) << 2);

					if (interruptStackBase)   //if not in-progress
					{
						//cleanup uncompleted interrupt
						instructionIndex = logoStack[interruptStackBase].returnInstructionIndex+1;  //support both end by main and end by END SUBROUTINE
						logoStackIndex =  interruptStackBase - 1;
						interruptStackBase = 0;
					}
					process_instructions();  //as if arrived
				}
			}
		}
		else
		{
			//
			//org code:
			if ( (tofinish_line < WAYPOINT_PROXIMITY_RADIUS) || forceCrossFinishLine || forceFinishReset ) // crossed the finish line  or interrupt routine sigmalled an event that needs immediate action
			{
				if (forceCrossFinishLine) // crossed the finish line  or interrupt routine sigmalled an event that needs immediate action
				{
					forceCrossFinishLine= false;  
					if ( forceFinishReset )
					{
						forceFinishReset = false;
						//reset Logo
						interruptIndex = 0;       // clear interrupt; instruction index of the beginning of the interrupt function
						instructionsProcessed = 0;
						interruptStackBase = 0;  // stack depth when entering interrupt (clear interrupt when dropping below this depth)
						logoStackIndex = 0;
						currentTurtle = 0;
						logoStack[logoStackIndex].returnInstructionIndex = -1;  // When starting over, begin on instruction 0
						instructionIndex = -1;
					}
				}
				//handle main instructione
				if (interruptStackBase)   //if not in-progress
				{
					//cleanup uncompleted interrupt
					instructionIndex = logoStack[interruptStackBase].returnInstructionIndex+1;  //support both end by main and end by END SUBROUTINE
					logoStackIndex =  interruptStackBase - 1;
					interruptStackBase = 0;
				}
				process_instructions();
			}
		}
#endif  //THERMALLING_MISSION
	}
#if ( THERMALLING_MISSION == 1 )
	//if ( ( flyCommandCounter > 0 ) && (!forceCrossFinishLine) && (!forceFinishReset) )
	if ( flyCommandCounter > 0 )
	{
		flyCommandCounter++;   //count up @ 40Hz
	}
	if ( fixedBankActive )
	{
		if ( fixedBankActiveCounter > 0 )
		{
			fixedBankActiveCounter--;   //count down @ 40Hz
		}
	}
	letHeartbeat++;
	if ( letHeartbeat % 40 == 0 )   //1Hz
	{
		
		//avgBatteryVoltage = (int16_t)( battery_voltage._.W1 );   //heavy filter for voltage
		avgBatteryVoltage = (avgBatteryVoltage * 14.0 + (float)battery_voltage._.W1 )/15.0;   //heavy filter for voltage
		airSpeedZAverage = ( (airSpeedZAverage * 8) + vario) / 9;
		
		geoSetStatus();         //read geofencee status and update status system value

		if (motorOffTimer > 0)   //monitor motor run
		{
			motorOffTimer--;
		}
		//if ((desired_behavior.W & F_LAND) == 0) // set to 4 as long as motor runs, to know time after stopping motor
		if (udb_pwOut[THROTTLE_OUTPUT_CHANNEL] > 2300 ) // set to 4 as long as motor runs, to know time after stopping motor
		{
			motorOffTimer = 4;  // start timer, wait 4 sec before detecting thermals, used by system value MOTOR_OFF_TIMER
		}
	}
	//calculate heading to where there is room to fly 400m, for REL_ANGLE_TO_OPPOSITE... 
	if ( letHeartbeat % 40 == 10 )   //1Hz
	{
		bestFarScore = 2;   //start bad
		bestFarScoreAngle = -1;
		areaGeoScore(-150,1,500,0);  //(int16_t angle, int16_t numbOfDirections, int16_t metersAhead, int16_t windSeconds)
		// score 
		if ( (geofenceScore.geoScoreAhead < bestFarScore) && (geofenceScore.geoScoreAhead == 1) )
		{
			bestFarScore = geofenceScore.geoScoreAhead;
			bestFarScoreAngle = -150;
		}
	}
	if ( letHeartbeat % 40 == 20 )   //1Hz
	{
		areaGeoScore(150,1,500,0);  //(int16_t angle, int16_t numbOfDirections, int16_t metersAhead, int16_t windSeconds)
		if ( (geofenceScore.geoScoreAhead < bestFarScore) && (geofenceScore.geoScoreAhead == 1) )
		{
			bestFarScore = geofenceScore.geoScoreAhead;
			bestFarScoreAngle = 150;
		}
	}
	if ( letHeartbeat % 40 == 25 )   //1Hz
	{
		areaGeoScore(-90,1,500,0);  //(int16_t angle, int16_t numbOfDirections, int16_t metersAhead, int16_t windSeconds)
		if ( (geofenceScore.geoScoreAhead < bestFarScore) && (geofenceScore.geoScoreAhead == 1) )
		{
			bestFarScore = geofenceScore.geoScoreAhead;
			bestFarScoreAngle = -90;
		}
	}
	if ( letHeartbeat % 40 == 30 )   //1Hz
	{
		areaGeoScore(90,1,500,0);  //(int16_t angle, int16_t numbOfDirections, int16_t metersAhead, int16_t windSeconds)
		if ( (geofenceScore.geoScoreAhead < bestFarScore) && (geofenceScore.geoScoreAhead == 1) )
		{
			bestFarScore = geofenceScore.geoScoreAhead;
			bestFarScoreAngle = 90;
		}
				
		if ( bestFarScore > 1 )      //is bestFarScore inside gf?
		{
			bestFarScoreAngle = 0;   //no, just fly ahead
		} 
		else
		{
			relAngleToOpposite = bestFarScoreAngle; //yes
		} 
		//used by REL_ANGLE_TO_OPPOSITE;    
	}
#endif
}

// For DO and EXEC, find the location of the given subroutine
static int16_t find_start_of_subroutine(uint8_t subcmd)
{
	int16_t i;

	if (subcmd == 0) return -1; // subcmd 0 is reserved to always mean the start of the logo program

	for (i = 0; i < numInstructionsInCurrentSet; i++)
	{
		if (currentInstructionSet[i].cmd == 1 && currentInstructionSet[i].subcmd == 2 && currentInstructionSet[i].arg == subcmd)
		{
			return i;
		}
	}
	return -1;
}

// When an IF condition was false, use this to skip to ELSE or END
// When an IF condition was true, and we ran the block, and reach an ELSE, skips to the END
static uint16_t find_end_of_current_if_block(void)
{
	int16_t i;
	int16_t nestedDepth = 0;

	for (i = instructionIndex+1; i < numInstructionsInCurrentSet; i++)
	{
		if (currentInstructionSet[i].cmd == 1 && currentInstructionSet[i].subcmd == 0) nestedDepth++; // into a REPEAT
		else if (currentInstructionSet[i].cmd >= 14 && currentInstructionSet[i].cmd <= 19) nestedDepth++; // into an IF
		else if (nestedDepth > 0 && currentInstructionSet[i].cmd == 1 && currentInstructionSet[i].subcmd == 1) nestedDepth--; // nested END
		else if (nestedDepth == 0 && currentInstructionSet[i].cmd == 1 && (currentInstructionSet[i].subcmd == 1 || currentInstructionSet[i].subcmd == 3))
		{
			// This is the target ELSE or END
			return i;
		}
	}
	return 0;
}

// Referencing PARAM in a LOGO program uses the PARAM from the current subroutine frame, even if
// we're also nested deeper inside of IF or REPEAT frames.  This finds the current subroutine's frame.
static int16_t get_current_stack_parameter_frame_index(void)
{
	int16_t i;

	for (i = logoStackIndex; i >= 0; i--)
	{
		if (logoStack[i].frameType == LOGO_FRAME_TYPE_SUBROUTINE)
		{
			return i;
		}
	}
	return 0;
}

static int16_t get_current_angle(void)
{
	// Calculate heading from Direction Cosine Matrix (rather than GPS), 
	// So that this code works when the plane is static. e.g. at takeoff
	struct relative2D curHeading;
	int8_t earth_yaw;
	int16_t angle;

	curHeading.x = -rmat[1];
	curHeading.y = rmat[4];
	earth_yaw = rect_to_polar(&curHeading); // (0=East,  ccw)
	angle = (earth_yaw * 180 + 64) >> 7;    // (ccw, 0=East)
	angle = -angle + 90;                    // (clockwise, 0=North)
	if (angle < 0) angle += 360;
	return angle;
}

static int16_t get_angle_to_point(int16_t x, int16_t y)
{
	struct relative2D vectorToGoal;
	int8_t dir_to_goal;
	int16_t angle;

	vectorToGoal.x = turtleLocations[currentTurtle].x._.W1 - x;
	vectorToGoal.y = turtleLocations[currentTurtle].y._.W1 - y;
	dir_to_goal = rect_to_polar (&vectorToGoal);

	// dir_to_goal                          // 0-255 (ccw, 0=East)
	angle = (dir_to_goal * 180 + 64) >> 7;  // 0-359 (ccw, 0=East)
	angle = -angle + 90;                    // 0-359 (clockwise, 0=North)
	if (angle < 0) angle += 360;
#if ( MY_PERSONAL_OPTIONS == 1 )
	if (angle >= 360) angle -= 360;
#else
	if (angle > 360) angle -= 360;
#endif

	return angle;
}

static int16_t logo_value_for_identifier(uint8_t ident)
{
	if (ident > 0 && ident <= NUM_INPUTS)
	{
		return udb_pwIn[(int16_t)ident];    // 2000 - 4000
	}

	switch (ident) {
		case DIST_TO_HOME: // in m
			return sqrt_long(IMUlocationx._.W1 * (int32_t)IMUlocationx._.W1 + IMUlocationy._.W1 * (int32_t)IMUlocationy._.W1);

		case DIST_TO_GOAL: // in m
			return tofinish_line;

		case ALT: // in m
			return IMUlocationz._.W1;

		case CURRENT_ANGLE: // in degrees. 0-359 (clockwise, 0=North)
			return get_current_angle();

		case ANGLE_TO_HOME: // in degrees. 0-359 (clockwise, 0=North)
		{
			int16_t angle = get_angle_to_point(0,0);
			angle += 180;
			if (angle < 0) angle += 360;
#if ( MY_PERSONAL_OPTIONS == 1 )
			if (angle >= 360) angle -= 360;
#else
			if (angle > 360) angle -= 360;
#endif

			return angle;
		}
		case ANGLE_TO_GOAL: // in degrees. 0-359 (clockwise, 0=North)
			return get_angle_to_point(IMUlocationx._.W1, IMUlocationy._.W1);

		case REL_ANGLE_TO_HOME: // in degrees. -180-179 (0=heading directly towards Home. Home to the right of the nose of the plane is positive)
		{
			int16_t angle = get_angle_to_point(0,0);
			angle = get_current_angle() - angle;
			angle += 180;
			if (angle < -180) angle += 360;
			if (angle >= 180) angle -= 360;
			return -angle;
		}
		case REL_ANGLE_TO_GOAL: // in degrees. -180-179 (0=heading directly towards Goal. Goal to the right of the nose of the plane is positive)
		{
			int16_t angle = get_current_angle() - get_angle_to_point(IMUlocationx._.W1, IMUlocationy._.W1);
			if (angle < -180) angle += 360;
			if (angle >= 180) angle -= 360;
			return -angle;
		}

#if ( THERMALLING_MISSION == 1 )
		case REL_ANGLE_TO_OPPOSITE: // in degrees. -180..179 (0=heading directly towards goal. clockwise offset is positive)
		{
			return relAngleToOpposite;
		}
		case REL_ANGLE_TO_WIND: // in degrees. -180-179 (0=heading directly towards home. clockwise offset is positive)
		{
			int16_t angle = get_current_angle() - get_angle_to_point(estimatedWind[0], estimatedWind[1]);
			angle += 180;
			if (angle < -180) angle += 360;
			if (angle >= 180) angle -= 360;

			return -angle;
		}
#endif

		case GROUND_SPEED: // in cm/s
			return ground_velocity_magnitudeXY;

		case AIR_SPEED: // in cm/s
			return air_speed_magnitudeXY;

		case AIR_SPEED_Z: // in cm/s
#if ( THERMALLING_MISSION != 1 )
			return IMUvelocityz._.W1 - estimatedWind[2];
#else
			return vario;     // in cm/s
#endif

		case WIND_SPEED: // in cm/s
			return sqrt_long(estimatedWind[0] * (int32_t)estimatedWind[0] + estimatedWind[1] * (int32_t)estimatedWind[1]);

		case WIND_SPEED_X: // in cm/s
			return estimatedWind[0];

		case WIND_SPEED_Y: // in cm/s
			return estimatedWind[1];

		case WIND_SPEED_Z: // in cm/s
			return estimatedWind[2];

		case WIND_FROM_ANGLE: // wind from in degrees 0-359, 0 = North
		{
			int16_t angle = get_angle_to_point(estimatedWind[0], estimatedWind[1]);
			if (angle < 0) angle += 360;
			if (angle > 359) angle -= 360;

			return angle;
		}

#if ( THERMALLING_MISSION == 1 )
		case BATTERY_VOLTAGE: //
		{
			//return battery_voltage._.W1;
			return (int16_t)avgBatteryVoltage;
		}

		case AIR_SPEED_Z_DELTA: //  used for waiting for a decrease in climbrate in a thermal
		{
			static int16_t airSpeedZDelta;
			airSpeedZDelta = vario - vario_old;
			vario_old = vario;
			airSpeedZStart = vario;  //setup for better lift detection
			return airSpeedZDelta;
		}

		case AIR_SPEED_Z_VS_START: 
		{
			//returns 1 if best climbrate exists since last ~270 deg
			//level only if really needed to center best lift
			//by comparing highest vario value against average over the last 9 seconds
			//only act if significantly better and still best after ~270 deg
			static int16_t airSpeedZBestHeading;

			if ( airSpeedZBestCount > 0 )   //waiting for the shift
			{
				airSpeedZBestCount ++;
			}
			//calculated elsewhere: airSpeedZAverage = ( (airSpeedZAverage * 8) + vario) / 9;  @ 1 Hz
			if ( ( vario > ( airSpeedZAverage + 10 )) && ( vario > airSpeedZBest ) )
			{
				airSpeedZBest = vario;
				airSpeedZBestCount = 1;   //start
				airSpeedZBestHeading = get_current_angle();			
			}
			// have we rotated 270 deg right or left since best? use +/- 25 deg margin
			if ( airSpeedZBestCount >= 6 &&
			     (  rotateClockwise && 
			   		( ( ( get_current_angle() - airSpeedZBestHeading + 360 ) % 360 ) > 245 ) && 
					( ( ( get_current_angle() - airSpeedZBestHeading + 360 ) % 360 ) < 295 ) ) |
			     ( !rotateClockwise && 
				 	( ( ( airSpeedZBestHeading - get_current_angle() + 360 ) % 360 ) > 245 ) &&
					( ( ( airSpeedZBestHeading - get_current_angle() + 360 ) % 360 ) < 295 ) )  ) 	
			{
				airSpeedZBestCount = 0;
				airSpeedZBest = 0;
				return 1;     //trigger the shift circle
			}
			else
			{
				return 0;
			}
		}

		case CLEAR_Z_BEST: // clear best climbrate
		{
			airSpeedZBestCount = 0;
			airSpeedZBest = 0;
			return (0);
		}

		case READ_F_LAND: // used for motor climbs
		{
			return ((desired_behavior.W & F_LAND) > 0);
		}

		case READ_DESIRED_SPEED: // used for polar plot 
		{
			return (desiredSpeed);
		}

		case READ_THROTTLE_OUTPUT_CHANNEL: // used for detecting motor hold low state 
		{
		 	return udb_pwOut[THROTTLE_OUTPUT_CHANNEL];    // 2000 - 4000
		 	//return udb_pwOut[7];    // 2000 - 4000
		}

		case GEOFENCE_STATUS: //  used for Geofence
		{
			return geoStatus;
		}
		
		case GEOFENCE_TURN: //  used for Geofence
		{
			return geoTurn;
		}

		case MOTOR_OFF_TIMER: // 4..0 sec. used for waiting after motor off before detecting a thermal
		{
			return motorOffTimer;
		}

		case FORCE_CROSS_FINISH_LINE: // used by interrupt routine to sigmal an event that needs immediate action
		{
			forceCrossFinishLine = true;
			return 0;
		}
		case READ_FLY_COMMAND_COUNTER: // used by interrupt routines to sigmal fly commands that take too long
		{
			return flyCommandCounter;
		}
		case FORCE_FINISH_BAD_NAV: // used by interrupt routine to sigmal an event that needs immediate action
		{
			turtleLocations[currentTurtle].x._.W0 = 0;
			turtleLocations[currentTurtle].x._.W1 = IMUlocationx._.W1;
			turtleLocations[currentTurtle].y._.W0 = 0;
			turtleLocations[currentTurtle].y._.W1 = IMUlocationy._.W1;
			flyCommandCounter = 0;
			forceCrossFinishLine = true;
			return 0;
		}
		case FORCE_RESET: // used by interrupt routine to sigmal an event that needs immediate action
		{
			turtleLocations[0].x._.W0 = 0;
			turtleLocations[0].x._.W1 = IMUlocationx._.W1;
			turtleLocations[0].y._.W0 = 0;
			turtleLocations[0].y._.W1 = IMUlocationy._.W1;
			flyCommandCounter = 0;
			forceCrossFinishLine = true;
			forceFinishReset = true;
			/*
			interruptIndex = 0;       // clear interrupt; instruction index of the beginning of the interrupt function
			instructionsProcessed = 0;
			interruptStackBase = 0;  // stack depth when entering interrupt (clear interrupt when dropping below this depth)
			logoStackIndex = 0;
			currentTurtle = 0;
			//flightplan_logo_begin(0); //clear interrupt and restart main
			*/
			return 0;
		}
		case SET_DIRECTION: // used by FIXED_BANK_ROTATE routine to sigmal left or right rotation
		{
			// if angle decreased: right turn
			rotateClockwise = ( ( ( get_current_angle() - oldAngle + 360) % 360 ) > 180 );
			return 0;
		}

#endif  //THERMALLING_MISSION

		case PARAM:
		{
			int16_t ind = get_current_stack_parameter_frame_index();
			return logoStack[ind].arg;
		}
	}
	return 0;
}

static boolean process_one_instruction(struct logoInstructionDef instr)
{
	if (instr.use_param)
	{
		// Use the subroutine's parameter instead of the instruction's arg value
		int16_t ind = get_current_stack_parameter_frame_index();
		instr.arg = logoStack[ind].arg;
	}

	switch (instr.cmd)
	{
		case 1: // Repeat
			switch (instr.subcmd)
			{
				case 0: // Repeat N times (or forever if N == -1)
					if (logoStackIndex < LOGO_STACK_DEPTH-1)
					{
						logoStackIndex++;
						logoStack[logoStackIndex].frameType = LOGO_FRAME_TYPE_REPEAT;
						logoStack[logoStackIndex].arg = instr.arg;
						logoStack[logoStackIndex].returnInstructionIndex = instructionIndex;
					}
					break;
				case 1: // End
					if (logoStackIndex > 0)
					{
						if (logoStack[logoStackIndex].frameType == LOGO_FRAME_TYPE_REPEAT)
						{
							// END REPEAT
							if (logoStack[logoStackIndex].arg > 1 || logoStack[logoStackIndex].arg == -1)
							{
								if (logoStack[logoStackIndex].arg != -1)
								{
									logoStack[logoStackIndex].arg--;
								}
								instructionIndex = logoStack[logoStackIndex].returnInstructionIndex;
							}
							else
							{
								logoStackIndex--;
							}
						}
						else if (logoStack[logoStackIndex].frameType == LOGO_FRAME_TYPE_SUBROUTINE)
						{
							// END SUBROUTINE
							instructionIndex = logoStack[logoStackIndex].returnInstructionIndex;
							logoStackIndex--;
							if (logoStackIndex < interruptStackBase)
							{
								interruptStackBase = 0;
								instructionsProcessed = MAX_INSTRUCTIONS_PER_CYCLE; // stop processing instructions after finishing interrupt
							}
						}
						else if (logoStack[logoStackIndex].frameType == LOGO_FRAME_TYPE_IF)
						{
							// Do nothing at the end of an IF block
							logoStackIndex--;
						}
					}
					else
					{
						// Extra, unmatched END goes back to the start of the program
						instructionIndex = logoStack[0].returnInstructionIndex;
						logoStackIndex = 0;
						interruptStackBase = 0;
					}
					break;
				
				case 3: // Else
					if (logoStack[logoStackIndex].frameType == LOGO_FRAME_TYPE_IF)
					{
						instructionIndex = find_end_of_current_if_block();
						logoStackIndex--;
					}
					break;
				
				case 2: // To (define a function)
				{
					// Shouldn't ever run these lines.
					// If we do get here, restart from the top of the logo program.
					instructionIndex = logoStack[0].returnInstructionIndex;
					logoStackIndex = 0;
					interruptStackBase = 0;
				}
				break;
			}
			break;

		case 10: // Exec (reset the stack and then call a subroutine)
			instructionIndex = find_start_of_subroutine(instr.subcmd);

#if ( LOG_WAYPOINT_PER_SUBROUTINE == 1 )
			//log Subroutine number instead
			//log only odd subroutine numbers
			if ( (instr.subcmd%2) == 1)	waypointIndex = instr.subcmd ;
#endif
			logoStack[0].returnInstructionIndex = instructionIndex;
			logoStackIndex = 0;
			interruptStackBase = 0;
			break;

		case 2: // Do (call a subroutine)
			if (logoStackIndex < LOGO_STACK_DEPTH-1)
			{
				logoStackIndex++;
				logoStack[logoStackIndex].frameType = LOGO_FRAME_TYPE_SUBROUTINE;
				logoStack[logoStackIndex].arg = instr.arg;
				logoStack[logoStackIndex].returnInstructionIndex = instructionIndex;
			}
			instructionIndex = find_start_of_subroutine(instr.subcmd);
#if ( LOG_WAYPOINT_PER_SUBROUTINE == 1 )
			//log Subroutine number instead
			//log only odd subroutine numbers
			if ( (instr.subcmd%2) == 1) waypointIndex = instr.subcmd ;
#endif
			break;

		case 3: // Forward/Back
			switch (instr.subcmd)
			{
				case 0: // Forward
				{
					int16_t cangle = turtleAngles[currentTurtle];   // 0-359 (clockwise, 0=North)
					int8_t b_angle = (cangle * 182 + 128) >> 8;     // 0-255 (clockwise, 0=North)
					b_angle = -b_angle - 64;                        // 0-255 (ccw, 0=East)
					
					turtleLocations[currentTurtle].x.WW += (__builtin_mulss(-cosine(b_angle), instr.arg) << 2);
					turtleLocations[currentTurtle].y.WW += (__builtin_mulss(-sine(b_angle), instr.arg) << 2);
#if ( THERMALLING_MISSION == 1 )
					flyCommandCounter = 1;    //start counter, count up at 40Hz
#endif  //THERMALLING_MISSION
				}
				break;

#if ( THERMALLING_MISSION == 1 )
				case 1: // FIXED_BANK_ROTATE
				{
					//rotate 30 deg right with a fixed bank or timeout after 2 sec

					//USE_CURRENT_ANGLE
					turtleAngles[currentTurtle] = get_current_angle();

					//rotate turtle too, like RT(). Set the rotation target 30 deg to the right or left
					if ( rotateClockwise )  //topview
					{
						fixedBankTargetAngle = turtleAngles[currentTurtle] + 30; // ~0.5 - 1 sec == 30 deg headingchange
						fixedBankDeg = instr.arg;  //controls roll and yaw,
					}
					else
					{
						fixedBankTargetAngle = turtleAngles[currentTurtle] - 30; // ~0.5 - 1 sec == 30 deg headingchange
						fixedBankDeg = -instr.arg;  //controls roll and yaw,
					}
					//fixedBankTargetAngle = get_current_angle() + 30; // ~0.5 - 1 sec == 30 deg headingchange
					while (fixedBankTargetAngle < 0) fixedBankTargetAngle += 360;
					fixedBankTargetAngle = fixedBankTargetAngle % 360;

					//this is a fly command, do the same as FD()
					int16_t cangle = turtleAngles[currentTurtle];   // 0-359 (clockwise, 0=North)
					int8_t b_angle = (cangle * 182 + 128) >> 8;     // 0-255 (clockwise, 0=North)
					b_angle = -b_angle - 64;                        // 0-255 (ccw, 0=East)

					// 35: should be fd(groundspeed) in m from m/s (ideally), with added margin to keep turtle ahead
					// selected a fixed number I used before, combined with servo calculation
					turtleLocations[currentTurtle].x.WW += (__builtin_mulss(-cosine(b_angle), 35) << 2);
					turtleLocations[currentTurtle].y.WW += (__builtin_mulss(-sine(b_angle), 35) << 2);

					fixedBankActiveCounter = 120; //40Hz = 3 sec
					fixedBankActive = true;     //controls roll and yaw, will be reset when rotation is reached
					angleTargetActive = true;
					break;
				}
				case 2: //BANK_1S
				{
					//maintain a fixed bank or level for one sec

					//USE_CURRENT_ANGLE
					turtleAngles[currentTurtle] = get_current_angle();
					//this is a fly command, do the same as FD()
					int16_t cangle = turtleAngles[currentTurtle];   // 0-359 (clockwise, 0=North)
					int8_t b_angle = (cangle * 182 + 128) >> 8;     // 0-255 (clockwise, 0=North)
					b_angle = -b_angle - 64;                        // 0-255 (ccw, 0=East)

					// 25: should be fd(groundspeed) in m from m/s (ideally), with added margen to keep turtle ahead
					// selected a fixed number I used before, combined with servo calculation
					turtleLocations[currentTurtle].x.WW += (__builtin_mulss(-cosine(b_angle), 25) << 2);
					turtleLocations[currentTurtle].y.WW += (__builtin_mulss(-sine(b_angle), 25) << 2);

                    oldAngle = get_current_angle();  //for SET_DIRECTION
					fixedBankDeg = instr.arg;  //controls roll
					fixedBankActiveCounter = 40; //40Hz = 1 sec
					fixedBankActive = true; 
					angleTargetActive = false;   
					break;
				}
#endif  //THERMALLING_MISSION

			}
			break;

		case 4: // Rotate
			switch (instr.subcmd)
			{
				case 0: // Right
				{
					int16_t angle = turtleAngles[currentTurtle] + instr.arg;
					while (angle < 0) angle += 360;
					angle = angle % 360;
					turtleAngles[currentTurtle] = angle;
					break;
				}
				case 1: // Set Angle
					turtleAngles[currentTurtle] = instr.arg;
					break;
				case 2: // Use current angle
				{
					turtleAngles[currentTurtle] = get_current_angle();
					break;
				}
				case 3: // Use angle to goal
				{
					turtleAngles[currentTurtle] = get_angle_to_point(IMUlocationx._.W1, IMUlocationy._.W1);
					break;
				}
			}
			break;

		case 5: // MV/SET location - X, Y, and Z
			switch (instr.subcmd)
			{
				case 0: // Move X
					turtleLocations[currentTurtle].x._.W1 += instr.arg;
					break;
				case 1: // Set X location
					turtleLocations[currentTurtle].x._.W0 = 0;
					turtleLocations[currentTurtle].x._.W1 = instr.arg;
					break;
				case 2: // Move Y
					turtleLocations[currentTurtle].y._.W1 += instr.arg;
					break;
				case 3: // Set Y location
					turtleLocations[currentTurtle].y._.W0 = 0;
					turtleLocations[currentTurtle].y._.W1 = instr.arg;
					break;
				case 4: // Move Z
					turtleLocations[currentTurtle].z += instr.arg;
					break;
				case 5: // Set Z location
					turtleLocations[currentTurtle].z = instr.arg;
					break;
				case 6: // Use current position (for x and y)
					turtleLocations[currentTurtle].x._.W0 = 0;
					turtleLocations[currentTurtle].x._.W1 = IMUlocationx._.W1;
					turtleLocations[currentTurtle].y._.W0 = 0;
					turtleLocations[currentTurtle].y._.W1 = IMUlocationy._.W1;
					break;
				case 7: // HOME
				{
					turtleAngles[currentTurtle] = 0;
					turtleLocations[currentTurtle].x.WW = 0;
					turtleLocations[currentTurtle].y.WW = 0;
#if ( MY_PERSONAL_OPTIONS == 1 )
					//if we are flying from my regular flying field move home to the landing point (angle 130, 60m) for convenience
					//on other field; initilize the autopilot on the landing spot
					if ( regularFlyingField )
					//if ( lat_origin.WW > 518260000 & lat_origin.WW < 518270000 & lon_origin.WW > 42980000 & lon_origin.WW < 42990000 )
					{
/*
#if ( HILSIM == 1 )
						turtleLocations[currentTurtle].x._.W0 = 0;
						turtleLocations[currentTurtle].x._.W1 = -425;
						turtleLocations[currentTurtle].y._.W0 = 0;
						turtleLocations[currentTurtle].y._.W1 = -300;
#else
*/

						//land_1
						/**/
						turtleLocations[currentTurtle].x._.W0 = 0;
						turtleLocations[currentTurtle].x._.W1 = 45;
						turtleLocations[currentTurtle].y._.W0 = 0;
						turtleLocations[currentTurtle].y._.W1 = -39;
						/**/
						//land_2
						/*
						turtleLocations[currentTurtle].x._.W0 = 0;
						turtleLocations[currentTurtle].x._.W1 = -123;
						turtleLocations[currentTurtle].y._.W0 = 0;
						turtleLocations[currentTurtle].y._.W1 = +25;
						*/
						//land_3
						/*
						turtleLocations[currentTurtle].x._.W0 = 0;
						turtleLocations[currentTurtle].x._.W1 = -92;
						turtleLocations[currentTurtle].y._.W0 = 0;
						turtleLocations[currentTurtle].y._.W1 = -91;
						*/
//#endif
					}
#endif
					break;
				}
				case 8: // Absolute set high value
					absoluteHighWord = instr.arg;
					break;
				case 9: // Absolute set low X value
				{
					absoluteXLong._.W1 = absoluteHighWord;
					absoluteXLong._.W0 = instr.arg;
					break;
				}
				case 10: // Absolute set low Y value
				{
					struct waypoint3D wp;
					struct relative3D rel;
					union longww absoluteYLong;

					absoluteYLong._.W1 = absoluteHighWord;
					absoluteYLong._.W0 = instr.arg;

					wp.x = absoluteXLong.WW;
					wp.y = absoluteYLong.WW;
					wp.z = 0;
					rel = dcm_absolute_to_relative(wp);
					turtleLocations[currentTurtle].x._.W0 = 0;
					turtleLocations[currentTurtle].x._.W1 = rel.x;
					turtleLocations[currentTurtle].y._.W0 = 0;
					turtleLocations[currentTurtle].y._.W1 = rel.y;
					break;
				}
			}
			break;

		case 6: // Flags
			switch (instr.subcmd)
			{
				case 0: // Flag On
					setBehavior(desired_behavior.W | instr.arg);
					break;
				case 1: // Flag Off
					setBehavior(desired_behavior.W & ~instr.arg);
					break;
				case 2: // Flag Toggle
					setBehavior(desired_behavior.W ^ instr.arg);
					break;
			}
			break;

		case 7: // Pen Up/Down
			switch (instr.subcmd)
			{
				case 0: // Pen Up
					penState++;
					break;
				case 1: // Pen Down
					if (penState > 0)
						penState--;
					break;
				case 2: // Pen Toggle
					penState = (penState == 0);
					if (penState == 0) instr.do_fly = 1; // Set the Fly Flag
					break;
			}
			break;

		case 8: // Set Turtle (choose plane or camera target)
			currentTurtle = (instr.arg == CAMERA) ? CAMERA : PLANE;
			break;

		case 9: // Modify PARAM
			switch (instr.subcmd)
			{
				case 0: // Set param
				{
					int16_t ind = get_current_stack_parameter_frame_index();
					logoStack[ind].arg = instr.arg;
					break;
				}
				case 1: // Add to param
				{
					int16_t ind = get_current_stack_parameter_frame_index();
					logoStack[ind].arg += instr.arg;
					break;
				}
				case 2: // Multiply param
				{
					int16_t ind = get_current_stack_parameter_frame_index();
					logoStack[ind].arg *= instr.arg;
					break;
				}
				case 3: // Divide param
				{
					int16_t ind = get_current_stack_parameter_frame_index();
					if (instr.arg != 0) // Avoid divide by 0!
					{
						logoStack[ind].arg /= instr.arg;
					}
					break;
				}
			}
			break;

		case 11: // Speed
#if (SPEED_CONTROL == 1)
			switch (instr.subcmd)
			{
				case 0: // Increase Speed
					desiredSpeed += instr.arg * 1;
					break;
				case 1: // Set Speed

#if (THERMALLING_MISSION != 1)
					desiredSpeed = instr.arg * 1;
#else
				{
					//command changed to dm/s
					if ( ( instr.arg == DESIRED_SPEED_NORMAL_F0 ) | ( instr.arg == DESIRED_SPEED_SLOW_F4 ) | ( instr.arg == DESIRED_SPEED_FAST_FMIN4 ) )
					{
						//calculate and set desiredSpeed
						//normal speed = 0, slow = -1000, high speed = 1000
						if ( instr.arg == DESIRED_SPEED_NORMAL_F0 )
						{
							desiredSpeed = instr.arg; //dm/s
						}
						else
						{
							if ( instr.arg == DESIRED_SPEED_SLOW_F4 )
							{
								desiredSpeed = instr.arg; //dm/s
							}
							else
							{
								if ( instr.arg == DESIRED_SPEED_FAST_FMIN4 )
								{
									desiredSpeed = instr.arg; //dm/s
								} //if
							} //else
						} //else
					} //if
					else
					{
						//    org code, needs 2 indents;
						desiredSpeed = instr.arg * 1;
						//
					} //else
					//limits
					if (desiredSpeed < DESIRED_SPEED_SLOW_F4)
					{
						desiredSpeed = DESIRED_SPEED_SLOW_F4;
					}
					if (desiredSpeed > DESIRED_SPEED_FAST_FMIN4)
					{
						desiredSpeed = DESIRED_SPEED_FAST_FMIN4;
					}
					break;
				}//case1
#endif
				//
			}
			if (desiredSpeed < 0) desiredSpeed = 0;
#endif
			break;

		case 12: // Interrupts
			switch (instr.subcmd) {
				case 1: // Set
					interruptIndex = find_start_of_subroutine(instr.arg);
					break;
				case 0: // Clear
					interruptIndex = 0;
					break;
			}
			break;

		case 13: // Load to PARAM
		{
			int16_t ind = get_current_stack_parameter_frame_index();
			logoStack[ind].arg = logo_value_for_identifier(instr.subcmd);
			break;
		}

		case 14: // IF commands
		case 15:
		case 16:
		case 17:
		case 18:
		case 19:
		{
			int16_t val = logo_value_for_identifier(instr.subcmd);
			boolean condTrue = false;

			if (instr.cmd == 14 && val == instr.arg) condTrue = true;       // IF_EQ
			else if (instr.cmd == 15 && val != instr.arg) condTrue = true;  // IF_NE
			else if (instr.cmd == 16 && val > instr.arg) condTrue = true;   // IF_GT
			else if (instr.cmd == 17 && val < instr.arg) condTrue = true;   // IF_LT
			else if (instr.cmd == 18 && val >= instr.arg) condTrue = true;  // IF_GE
			else if (instr.cmd == 19 && val <= instr.arg) condTrue = true;  // IF_LE

			if (condTrue)
			{
				if (logoStackIndex < LOGO_STACK_DEPTH-1)
				{
					logoStackIndex++;
					logoStack[logoStackIndex].frameType = LOGO_FRAME_TYPE_IF;
				}
			}
			else
			{
				// jump to the matching END or ELSE
				instructionIndex = find_end_of_current_if_block();
				if (currentInstructionSet[instructionIndex].subcmd == 3) // is entering an ELSE block
				{
					if (logoStackIndex < LOGO_STACK_DEPTH-1)
					{
						logoStackIndex++;
						logoStack[logoStackIndex].frameType = LOGO_FRAME_TYPE_IF;
					}
				}
			}
			break;
		}
	}
	return instr.do_fly;
}

static void process_instructions(void)
{
	instructionsProcessed = 0;

	while (1)
	{
		boolean do_fly = process_one_instruction(currentInstructionSet[instructionIndex]);

		instructionsProcessed++;
		instructionIndex++;
		if (instructionIndex >= numInstructionsInCurrentSet) instructionIndex = 0;

		if (do_fly && penState == 0 && currentTurtle == PLANE)
			break;

		if (instructionsProcessed >= MAX_INSTRUCTIONS_PER_CYCLE)
			return;  // don't update goal if we didn't hit a FLY command
	}

#if ( LOG_WAYPOINT_PER_SUBROUTINE != 1 )
	waypointIndex = instructionIndex - 1;
#endif

	if (logo_goal_has_moved())
	{
		update_goal_from(lastGoal);
		navigate_compute_bearing_to_goal();
	}
}

void flightplan_logo_live_begin(void)
{
	if (logo_inject_pos == LOGO_INJECT_READY)
		return;
	logo_inject_pos = 0;
}

void flightplan_logo_live_received_byte(uint8_t inbyte)
{
	if (logo_inject_pos == LOGO_INJECT_READY)
		return;

	switch (logo_inject_pos)
	{
		case 0:
			logo_inject_instr.cmd = inbyte;
			break;

		case 1:
			logo_inject_instr.subcmd = inbyte;
			break;

		case 2:
			logo_inject_instr.do_fly = ((inbyte >> 8) & 0x0F); // TODO: WARNING, right shift by too large amount, data loss
			logo_inject_instr.use_param = (inbyte & 0x0F);
			break;

		case 3:
			logo_inject_instr.arg = inbyte * 256;
			break;

		case 4:
			logo_inject_instr.arg |= inbyte;
			break;

		case 5:
			// too many bytes for this command!
			// increment logo_instr_pos below, which invalidates this command
			break;

		default:
			// don't increment while waiting for previous command to complete
			return;
	}
	logo_inject_pos++;
}

void flightplan_logo_live_commit(void)
{
	// The cmd=1 commands (REPEAT, END, TO) are not allowed
	// to be injected.
	if (logo_inject_pos == 5 && logo_inject_instr.cmd != 1)
	{
		logo_inject_pos = LOGO_INJECT_READY;
	}
	else
	{
		logo_inject_pos = 0;
	}
}
#if ( THERMALLING_MISSION == 1 )

//use linear programmming to test position against a set of geofence shapes
//float geoPreference(float x, float y, geofenceShape geoshape)
float geoPreference(float x, float y, int16_t shape)
{
	static float result;
	
	result = geofenceShapes[shape].ellips_diameter_x*x*x+ geofenceShapes[shape].ellips_diameter_y*y*y+ geofenceShapes[shape].fuzzy_factor*x*y+ geofenceShapes[shape].linear_x_slope*x+ geofenceShapes[shape].linear_y_slope*y+ geofenceShapes[shape].offset_x_plus_y;      // hoever zijn we van de lijn af
	result *= (float)geofenceShapes[shape].sign;              // other side of the line
	if (result < 1)
	{
		result = 1;        // limit when well in desired area
	}
	return result;
}


// this function returns one to three scores x m ahead, for left x deg, one ahead, and one for right x deg (if 2 or 3), using wind drift (if not 0),
// depending on options:
//    - only check geofence (30,0,0,0)               (strictGeofence)
//    - check ahead, left, right with wind  angle is relatice to curent heading
//    - check rel angle,0,90,180 or 270, at 400m distance (to find ANGLE_TO_FARTHEST_POINT) (strictGeofence)

//calls geoPreference() per shape
void areaGeoScore(int16_t angle, int16_t numbOfDirections, int16_t metersAhead, int16_t windSeconds)   // windSeconds; translate x and y downwind, equivalent of x sec drift
{
	static int16_t shapeIndex;
	static float result=1;
	static float x;  //pos in meters
	static float y;  //pos in meters
	static boolean strictGeofence = false;
	static int16_t cangle = 0;
	static int8_t b_angle = 0;
	//static geoScores geoScore;

	if ( windSeconds == 0 )
	{
		strictGeofence = true;
	}
	else
	{
		strictGeofence = false;
	}

	if (numbOfDirections == 1 || numbOfDirections == 3);
	{
		geofenceScore.geoScoreAhead = 1;
		if (metersAhead == 0)
		{
			x = (float)turtleLocations[PLANE].x._.W1 ;       // in m  windSeconds; translate x and y downwind, equivalent of x sec drift
			y = (float)turtleLocations[PLANE].y._.W1 ;       // in m
		}
		else
		{
			//ahead
			x = (float)turtleLocations[PLANE].x._.W1 + (float)windSeconds * (float)(estimatedWind[0])/100.0;       // in m  windSeconds; translate x and y downwind, equivalent of x sec drift
			y = (float)turtleLocations[PLANE].y._.W1 + (float)windSeconds * (float)(estimatedWind[1])/100.0;       // in m

			//code for LT() or RT()
			cangle = turtleAngles[currentTurtle]+angle;   //ahead   // 0-359 (clockwise, 0=North)
			//}
			while (cangle < 0) cangle += 360;
			while (cangle >= 360) cangle -= 360;

			//code from FD()
			b_angle = (cangle * 182 + 128) >> 8;     // 0-255 (clockwise, 0=North)
			b_angle = -b_angle - 64;                        // 0-255 (ccw, 0=East)
			//select point forward "metersAhead"
			//do the check for a point xm in front of plane
			x += (float)((__builtin_mulss(-cosine(b_angle), metersAhead) << 2)>>16);  //from 16.16 to float
			y += (float)((__builtin_mulss(-sine(b_angle), metersAhead) << 2)>>16);
		}
		if ( strictGeofence )
		{
			//for ( i=0; i<numbOfGeoshapes; i++)
			result = 1;
			for ( shapeIndex=0; shapeIndex<NUMB_OF_GEO_SHAPES; shapeIndex++)
			{
				result *= geoPreference(x,y,shapeIndex);
			}
			geofenceScore.geoScoreAhead = result;
		}
		else
		{
			//for ( i=0; i<numbOfGeoshapes; i++)
			result = 1;
			for ( shapeIndex=0; shapeIndex<NUMB_OF_GEO_SHAPES; shapeIndex++)
			{
				result *= geoPreference(x,y,shapeIndex+2);  //use the smaller shapes
			}
			geofenceScore.geoScoreAhead = result;
		}
	}

	if (numbOfDirections >= 2);
	{
		geofenceScore.geoScoreLeft = AHEAD_PREFERENCE;
		geofenceScore.geoScoreRight = AHEAD_PREFERENCE;
		//Left score
		x = (float)turtleLocations[PLANE].x._.W1 + (float)windSeconds * (float)(estimatedWind[0])/100.0;       // in m  windSeconds; translate x and y downwind, equivalent of x sec drift
		y = (float)turtleLocations[PLANE].y._.W1 + (float)windSeconds * (float)(estimatedWind[1])/100.0;       // in m

		//code for LT() or RT()
		cangle = turtleAngles[currentTurtle]-angle;   //left   // 0-359 (clockwise, 0=North)
		while (cangle < 0) cangle += 360;
		while (cangle >= 360) cangle -= 360;

		//code from FD()
		b_angle = (cangle * 182 + 128) >> 8;     // 0-255 (clockwise, 0=North)
		b_angle = -b_angle - 64;                        // 0-255 (ccw, 0=East)
		//select point forward "metersAhead"
		//do the check for a point xm in front of plane
		x += (float)((__builtin_mulss(-cosine(b_angle), metersAhead) << 2)>>16);  //from 16.16 to float
		y += (float)((__builtin_mulss(-sine(b_angle), metersAhead) << 2)>>16);

		if ( strictGeofence )
		{
			//for ( i=0; i<numbOfGeoshapes; i++)
			result = 1;
			for ( shapeIndex=0; shapeIndex<NUMB_OF_GEO_SHAPES; shapeIndex++)
			{
				result *= geoPreference(x,y,shapeIndex);
			}
			geofenceScore.geoScoreLeft = result * 1.000;
		}
		else
		{
			//for ( i=0; i<numbOfGeoshapes; i++)
			result = 1;
			for ( shapeIndex=0; shapeIndex<NUMB_OF_GEO_SHAPES; shapeIndex++)
			{
				result *= geoPreference(x,y,shapeIndex+2);  //use the smaller shapes
			}
			geofenceScore.geoScoreLeft = result * AHEAD_PREFERENCE;
		}

		//Right score
		x = (float)turtleLocations[PLANE].x._.W1 + (float)windSeconds * (float)(estimatedWind[0])/100.0;       // in m  windSeconds; translate x and y downwind, equivalent of x sec drift
		y = (float)turtleLocations[PLANE].y._.W1 + (float)windSeconds * (float)(estimatedWind[1])/100.0;       // in m

		//code for LT() or RT()
		cangle = turtleAngles[currentTurtle]+angle;   //right   // 0-359 (clockwise, 0=North)
		while (cangle < 0) cangle += 360;
		while (cangle >= 360) cangle -= 360;

		//code from FD()
		b_angle = (cangle * 182 + 128) >> 8;     // 0-255 (clockwise, 0=North)
		b_angle = -b_angle - 64;                        // 0-255 (ccw, 0=East)
		//select point forward "metersAhead"
		//do the check for a point xm in front of plane
		x += (float)((__builtin_mulss(-cosine(b_angle), metersAhead) << 2)>>16);  //from 16.16 to float
		y += (float)((__builtin_mulss(-sine(b_angle), metersAhead) << 2)>>16);
	}
	if ( strictGeofence )
	{
		//for ( i=0; i<numbOfGeoshapes; i++)
		result = 1;
		for ( shapeIndex=0; shapeIndex<NUMB_OF_GEO_SHAPES; shapeIndex++)
		{
			result *= geoPreference(x,y,shapeIndex);
		}
		geofenceScore.geoScoreRight = result * 1.000;
	}
	else
	{
		//for ( i=0; i<numbOfGeoshapes; i++)
		result = 1;
		for ( shapeIndex=0; shapeIndex<NUMB_OF_GEO_SHAPES; shapeIndex++)
		{
			result *= geoPreference(x,y,shapeIndex+2);  //use the smaller shapes
		}
		geofenceScore.geoScoreRight = result * AHEAD_PREFERENCE;  //small preference for ahead
	}
}

// LOGO will use geoStatus first (GET_GF_STATUS)
// LOGO then will use geoTurn to plan the turn
// test geofence shapes from strictest to least strict, does more tests if needed
// calls areaGeoScore() several times with angle, numbOfDirections, metersAhead  and  windSeconds
// status               advice mandatory
//    0, soft/wind gf     0             if new thermal, thermal, else do turn (soft)
//    1, wind gf          0             ignore if in thermal, else turn
//    2, geofence         1             alarm, do geofence only
void geoSetStatus() // set geoStatus. WindSeconds; translate x and y downwind, equivalent of x sec drift
{
	//float turn;     //0..1
	//int16_t geoStatus;     //0,1,2
	//int16_t geoTurn;       //turn -40, 0, or 40 deg in 4 sec    scope: flightplan_logo.c
	//geofenceScore =
	areaGeoScore(30,1,0,0);  //angle, numbOfDirections, metersAhead, windSeconds)   strict
	if ( geofenceScore.geoScoreAhead > 1 )  //out
	{
		geofenceScore.geoScoreAhead = 2;  //ahead must be avoided, will in some cases cause flying far, brake the balance
		geoStatus = 2;                             // geofence crossed, not good...
		//now check what to do, act on soft -not strict- geofence only
		areaGeoScore(30,1,40,0);  // - look ahead -soft-, what action?
		//if no problem ahead (allmost back) then do nothing

		if ( geofenceScore.geoScoreAhead > 1 )    // action needed
		{                                         // what action?
			areaGeoScore(30,2,40,0);               // l/r
		}
		geoSetTurn();
	}
	else                                          //inside, now check what to do
	{
		//now check ">=1 escape radius" -  maintain room for a 120+ turn on at least one side
		areaGeoScore(60,2,150,20);
		if ( (geofenceScore.geoScoreAhead > AHEAD_PREFERENCE) && (geofenceScore.geoScoreAhead > AHEAD_PREFERENCE) )
		{
			geofenceScore.geoScoreAhead = 2;          //not ahead, take best turn to maintain room for turning
		}
		else   //ok
		{
			//keep old score
			//geofenceScore_old = geofenceScore;

			areaGeoScore(30,1,50,0);  //  - look ahead, what action?
			if ( geofenceScore.geoScoreAhead > 1 )    // no room for a new thermal turn
			{
				geoStatus = 1;                        // assume no room for a new thermal turn
				areaGeoScore(30,2,50,0);              // now act (upwind side), regardless wind
			}
			else
			{
				// upwind side ok, now check downwind side
				areaGeoScore(30,1,50,20);  //- look ahead, what action?
				if ( geofenceScore.geoScoreAhead == 1 )  //no problem ahead + wind
				{
					geoStatus = 0;                        // no action, there is room for a new thermal turn
					areaGeoScore(30,2,50,20);             //- look ahead, what action?
				}
				else
				{
					geoStatus = 1;                        // no room for a new thermal turn , act
					areaGeoScore(30,2,50,20);             //- look ahead, what action?
				}
			}
		}
		geoSetTurn();
	}
}


void geoSetTurn()   // set geoTurn;   //-40, 0 or 40 deg
{
	// geoScoreAhead lowest value 1 allways wins
	// geoScoreLeft, geoScoreRight lowest values are AHEAD_PREFERENCE
	if ( (geofenceScore.geoScoreAhead < geofenceScore.geoScoreLeft) && (geofenceScore.geoScoreAhead < geofenceScore.geoScoreRight) )
	{
		geoTurn = 0;
	}
	else if (geofenceScore.geoScoreLeft < geofenceScore.geoScoreRight)
	{
		geoTurn = -40;
	}
	else
	{
		geoTurn = 40;
	}
}

#endif //THERMALLING_MISSION

//#endif // (FLIGHT_PLAN_TYPE == FP_LOGO)
