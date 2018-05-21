// This file is part of MatrixPilot.
//
//    http://code.google.com/p/gentlenav/
//
// Copyright 2009-2011 MatrixPilot Team
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

// To use this library, you must set ALTITUDE_GAINS_VARIABLE == 1 in options.h


#include "defines.h"
#include "navigate.h"
#include "behaviour.h"
#include "servoPrepare.h"
#include "config.h"
#include "states.h"
#include "airspeedCntrl.h"
#include "altitudeCntrl.h"
#include "sonarCntrl.h"
#include "../libDCM/deadReckoning.h"
#if ( THERMALLING_MISSION == 1 )
#include "../libDCM/estAltitude.h"
#endif  //THERMALLING_MISSION
#include "../libUDB/servoOut.h"

#if (ALTITUDE_GAINS_VARIABLE == 1)


#define THROTTLEFILTSHIFT 12
#define DEADBAND 150
#define MAXTHROTTLE         (2.0*SERVORANGE*altit.AltHoldThrottleMax)
#define FIXED_WP_THROTTLE   (2.0*SERVORANGE*RACING_MODE_WP_THROTTLE)
#define THROTTLEHEIGHTGAIN (((altit.AltHoldThrottleMax - altit.AltHoldThrottleMin)*2.0*SERVORANGE)/(altit.HeightMargin*2.0))
#define PITCHATMAX (altit.AltHoldPitchMax*(RMAX/57.3))
#define PITCHATMIN (altit.AltHoldPitchMin*(RMAX/57.3))
#define PITCHATZERO (altit.AltHoldPitchHigh*(RMAX/57.3))
#define PITCHHEIGHTGAIN ((PITCHATMAX - PITCHATMIN) / (altit.HeightMargin*2.0))
#define HEIGHTTHROTTLEGAIN ((1.5*(altit.HeightTargetMax-altit.HeightTargetMin)* 1024.0) / (SERVORANGE*SERVOSAT))

#if (AIRFRAME_TYPE == AIRFRAME_GLIDER)
static int16_t autopilotBrake = 0; // braking by autopilot,   0 brake = 0, full brake == 1700
inline int16_t get_autopilotBrake(void) {return autopilotBrake; };
#endif //AIRFRAME_GLIDER

union longww throttleFiltered = { 0 };
int16_t pitchAltitudeAdjust = 0;
boolean filterManual = false;
int16_t desiredHeight;

static void normalAltitudeCntrl(void);
static void manualThrottle(int16_t throttleIn);
static void hoverAltitudeCntrl(void);

// External variables
int16_t height_target_min;
int16_t height_target_max;
int16_t height_margin;
fractional alt_hold_throttle_min;
fractional alt_hold_throttle_max;
int16_t alt_hold_pitch_min;
int16_t alt_hold_pitch_max;
int16_t alt_hold_pitch_high;
int16_t rtl_pitch_down;

#if ( THERMALLING_MISSION == 1 )
#if ( HILSIM == 1 )
static int32_t speed_height_old = 0;
#endif
static int16_t varioCounter = 0;            // 0..3 to create 1Hz from 4Hz
extern int16_t vario;                       // in cm/s   used for Logo by  - defined in flightplan_logo.c and set in altitudeCntrlVariable.c
static boolean motorClimbRunStarted;        // boolean climbing with motor so far this run
static int16_t avgMotorRunClimbrate;        // average climbrate in cm/s in this run
static int16_t sinkMotorOffTimer;           // wait period to postpone climbing with motor due to sink  (30 sec)
static int16_t motorClimbRunCount;          // counter (40Hz steps) for motor run
static float avgMotorClimbrate;             // average climbrate after settle all runs in m/s
static float steadyClimbPowerFactor = 0.8;  // reduce throttle for a 0.7 m/s climb, start with 0.7
static boolean motorClimbSinkStarted;       // boolean climbing with motor in sink between 50 and 50 m
#endif  //THERMALLING_MISSION

// Internal computed variables.
int16_t max_throttle;
int16_t throttle_height_gain;
int16_t pitch_at_max;
int16_t pitch_at_min;
int16_t pitch_at_zero;
int16_t pitch_height_gain;
int16_t height_throttle_gain;

int16_t desiredSpeed;
boolean speed_control;

void init_altitudeCntrlVariable(void)
{

// External variables
	height_target_min     = altit.HeightTargetMin;
	height_target_max     = altit.HeightTargetMax;
	height_margin         = altit.HeightMargin;
	alt_hold_throttle_min = altit.AltHoldThrottleMin * RMAX;
	alt_hold_throttle_max = altit.AltHoldThrottleMax * RMAX;
	alt_hold_pitch_min    = altit.AltHoldPitchMin;
	alt_hold_pitch_max    = altit.AltHoldPitchMax;
	alt_hold_pitch_high   = altit.AltHoldPitchHigh;
	rtl_pitch_down        = gains.RtlPitchDown;

// Internal computed variables.  Values defined above.
	max_throttle          = MAXTHROTTLE;
	throttle_height_gain  = THROTTLEHEIGHTGAIN;
	pitch_at_max          = PITCHATMAX;
	pitch_at_min          = PITCHATMIN;
	pitch_at_zero         = PITCHATZERO;
	pitch_height_gain     = PITCHHEIGHTGAIN;
	height_throttle_gain  = HEIGHTTHROTTLEGAIN;

// Initialize to the value from options.h.  Allow updating this value from LOGO/MavLink/etc.
// Stored in 10ths of meters per second
	desiredSpeed = (altit.DesiredSpeed*10);
	speed_control = SPEED_CONTROL;
}

void save_altitudeCntrlVariable(void)
{
	altit.HeightTargetMax = height_target_max;
	altit.HeightTargetMin = height_target_min;
//	height_margin;
	altit.AltHoldThrottleMin = alt_hold_throttle_min / RMAX;
	altit.AltHoldThrottleMax = alt_hold_throttle_max / RMAX;
	altit.AltHoldPitchMin = alt_hold_pitch_min;
	altit.AltHoldPitchMax = alt_hold_pitch_max;
	altit.AltHoldPitchHigh = alt_hold_pitch_high;
//	rtl_pitch_down;
//	desiredSpeed / 10;
//	speed_control;
}

static int32_t excess_energy_height(int16_t targetAspd, int16_t acutalAirspeed) // computes (1/2gravity)*(actual_speed^2 - desired_speed^2)
{
	union longww accum;

	// targetAspd * 6 / 10
	// 1/10 to scale from cm/s to dm/s
	// 6 is ~1/(2*g) with adjustments?
	accum.WW = __builtin_mulsu(targetAspd, 39321);
	int16_t speedAccum = accum._.W1;
	int32_t equivalent_energy_air_speed = -(__builtin_mulss(speedAccum, speedAccum));

	// adjust airspeed value for 1/(2*g^2)
	accum.WW = __builtin_mulsu(acutalAirspeed, 37877);
	accum.WW = __builtin_mulss(accum._.W1 , accum._.W1);
	equivalent_energy_air_speed += accum.WW;

	return equivalent_energy_air_speed;
}

void altitudeCntrl(void)
{
#if (USE_SONAR_INPUT != 0)
	calculate_sonar_height_above_ground();
#endif
	if (canStabilizeHover() && current_orientation == F_HOVER)
	{
		hoverAltitudeCntrl();
	}
	else
	{
		normalAltitudeCntrl();
	}
}

static void set_throttle_control(int16_t throttle)
{
	int16_t throttleIn;

	if (state_flags._.altitude_hold_throttle || state_flags._.altitude_hold_pitch || filterManual)
	{
		if (udb_flags._.radio_on == 1)
		{
			throttleIn = udb_pwIn[THROTTLE_INPUT_CHANNEL];
		}
		else
		{
			throttleIn = udb_pwTrim[THROTTLE_INPUT_CHANNEL];
		}

		int16_t temp = throttleIn + REVERSE_IF_NEEDED(THROTTLE_CHANNEL_REVERSED, throttle);

		if (THROTTLE_CHANNEL_REVERSED)
		{
			if (temp > udb_pwTrim[THROTTLE_INPUT_CHANNEL]) throttle = throttleIn - udb_pwTrim[THROTTLE_INPUT_CHANNEL];
		}
		else
		{
			if (temp < udb_pwTrim[THROTTLE_INPUT_CHANNEL]) throttle = udb_pwTrim[THROTTLE_INPUT_CHANNEL] - throttleIn;
		}

		throttle_control = throttle;
	}
	else
	{
		throttle_control = 0;
	}
}

void setTargetAltitude(int16_t targetAlt)
{
	desiredHeight = targetAlt;
}

static void normalAltitudeCntrl(void)
{
	union longww throttleAccum;
#if ( THERMALLING_MISSION == 1 )
//	makes no sense if speed_control is on (may be useful for RACING_MODE)
/*
	union longww pitchAccum;
*/
#endif  //THERMALLING_MISSION

	int16_t throttleIn;
	int16_t throttleInOffset;
	union longww heightError = { 0 };
	int32_t speed_height;

	union longww temp;

	temp.WW = __builtin_mulss(alt_hold_throttle_max , 2.0 * SERVORANGE);
	temp.WW <<= 2;
	if(temp._.W0 & 0x8000) temp._.W1 ++;
	max_throttle =	temp._.W1;

	temp.WW = __builtin_mulss((alt_hold_throttle_max - alt_hold_throttle_min) , 2.0 * SERVORANGE);
	temp.WW <<= 2;
	if(temp._.W0 & 0x8000) temp._.W1++;
	temp._.W0 = temp._.W1;
	temp._.W1 = 0;
	throttle_height_gain =	__builtin_divsd(temp.WW, (height_margin << 1));
	throttle_height_gain <<= 1;

	temp.WW =  __builtin_mulss(alt_hold_pitch_max, (int16_t) ((RMAX * 64.0) / 57.3));
	temp.WW <<= 10;
	if(temp._.W0 & 0x8000) temp._.W1++;
	pitch_at_max = temp._.W1;

	temp.WW =  __builtin_mulss(alt_hold_pitch_min, (int16_t) ((RMAX * 64.0) / 57.3));
	temp.WW <<= 10;
	if(temp._.W0 & 0x8000) temp._.W1++;
	pitch_at_min = temp._.W1;

	temp.WW =  __builtin_mulss(alt_hold_pitch_high, (int16_t) ((RMAX * 64.0) / 57.3));
	temp.WW <<= 10;
	if(temp._.W0 & 0x8000) temp._.W1++;
	pitch_at_zero = temp._.W1;

	temp.WW = 0;
	temp._.W0 = pitch_at_max - pitch_at_min;
	pitch_height_gain =	__builtin_divsd(temp.WW , (height_margin << 1));

	temp.WW = __builtin_mulss((height_target_max-height_target_min), 1.5 * 1024.0);
	temp.WW <<= 2;
	height_throttle_gain =	__builtin_divsd(temp.WW , (SERVORANGE*SERVOSAT));
	height_throttle_gain >>= 2;

	int16_t height_marginx8 = height_margin << 3;

	speed_height = excess_energy_height(target_airspeed, airspeed); // equivalent height of the airspeed

#if ( THERMALLING_MISSION == 1 )
	varioCounter++;
	if ( (varioCounter % 10) == 0)  //1 out of 10 times == 4Hz
	{
		// vario in altitudeCntrlVariable.c because : if in logo c, no vario possible in manual and stab modes
		//attempt to compensate energy by mixing speedheigth
/*
#if (USE_BAROMETER_ALTITUDE == 0)
		vario = ( ( vario * 23 ) + (int16_t)(IMUvelocityz._.W1) + () ) / 24;    //update @ 4Hz, in cm/sec, used in flightplan_logo.c
		speed_height_old = speed_height;
#else
		vario = ( ( vario * 11 ) + (int16_t)(get_barometer_vert_velocity()/10) )/ 12;    //from estAltitude.c   in cm/sec
#endif
*/
		vario = ( ( vario * 11 ) + (int16_t)(IMUvelocityz._.W1) )/ 12;    //update @ 4Hz, 3 sec filter, based on GPS, in cm/sec, used in flightplan_logo.c

	}
#endif  //THERMALLING_MISSION

	if (udb_flags._.radio_on == 1)
	{
		throttleIn = udb_pwIn[THROTTLE_INPUT_CHANNEL];
		// keep the In and Trim throttle values within 2000-4000 to account for
		// Spektrum receivers using failsafe values below 2000.
		throttleInOffset = udb_servo_pulsesat(udb_pwIn[THROTTLE_INPUT_CHANNEL]) - udb_servo_pulsesat(udb_pwTrim[THROTTLE_INPUT_CHANNEL]);
	}
	else
	{
		throttleIn = udb_pwTrim[THROTTLE_INPUT_CHANNEL];
		throttleInOffset = 0;
	}

	if (state_flags._.altitude_hold_throttle || state_flags._.altitude_hold_pitch)
	{
		if (THROTTLE_CHANNEL_REVERSED) throttleInOffset = - throttleInOffset;

		if (state_flags._.GPS_steering)
		{
			desiredHeight = navigate_desired_height();
//			if (desired_behavior._.takeoff || desired_behavior._.altitude)
//			{
//				desiredHeight = goal.height;
//			}
//			else
//			{
//				desiredHeight = goal.fromHeight + (((goal.height - goal.fromHeight) * (int32_t)progress_to_goal)>>12);
//			}
		}
		else
		{
			if (settings._.AltitudeholdStabilized == AH_PITCH_ONLY)
			{
				// In stabilized mode using pitch-only altitude hold, use desiredHeight as
				// set from the state machine upon entering stabilized mode in ent_stabilizedS()
			}
			//else if ((settings._.AltitudeholdStabilized == AH_FULL) ||/// (settings._.AltitudeholdStabilized == AH_THROTTLE_ONLY))
			else if (settings._.AltitudeholdStabilized == AH_FULL)
			{
				// In stabilized mode using full altitude hold, use the throttle stick value to determine desiredHeight,
				desiredHeight = ((__builtin_mulss(height_throttle_gain, throttleInOffset - ((int16_t)(DEADBAND)))) >> 11)
				                + height_target_min;
			}
			if (desiredHeight < (int16_t)(height_target_min)) desiredHeight = (int16_t)(height_target_min);
			if (desiredHeight > (int16_t)(height_target_max)) desiredHeight = (int16_t)(height_target_max);
		}

		if (throttleInOffset < (int16_t)(DEADBAND) && udb_flags._.radio_on)
		{
			pitchAltitudeAdjust = 0;
			throttleAccum.WW  = 0;
		}
		else
		{
			heightError._.W1 = - desiredHeight;
			heightError.WW = (heightError.WW + IMUlocationz.WW + speed_height) >> 13;
			if (heightError._.W0 < -height_marginx8)
			{
				throttleAccum.WW = (int16_t)(max_throttle);
			}
			else if (heightError._.W0 > height_marginx8)
			{
				throttleAccum.WW = 0;
			}
			else
			{
				throttleAccum.WW = (int16_t)(max_throttle) + (__builtin_mulss(throttle_height_gain, (-heightError._.W0 - height_marginx8))>>3);
				if (throttleAccum.WW > (int16_t)(max_throttle)) throttleAccum.WW = (int16_t)(max_throttle);
			}

/* //THERMALLING_MISSION  this makes no sense if speed_control is on (may be usefull for RACING_MODE)

			heightError._.W1 = - desiredHeight;
			heightError.WW = (heightError.WW + IMUlocationz.WW - speed_height) >> 13;
			if (heightError._.W0 < -height_marginx8)
			{
				pitchAltitudeAdjust = (int16_t)(pitch_at_max);
			}
			else if (heightError._.W0 > height_marginx8)
			{
				pitchAltitudeAdjust = (int16_t)(pitch_at_zero);
			}
			else
			{
				pitchAccum.WW = __builtin_mulss((int16_t)(pitch_height_gain) , - heightError._.W0 - height_marginx8)>>3;
				pitchAltitudeAdjust = (int16_t)(pitch_at_max) + pitchAccum._.W0;
			}
*/
//#endif  //THERMALLING_MISSION



#if ( THERMALLING_MISSION == 1 )
			//motorClimbRunStarted - boolean climbing with motor so far this run
			//avgMotorRunClimbrate - average climbrate of seconds in this run
			//sinkMotorOffTimer - wait period to postpone climbing with motor due to sink
			//should save battery power by not climbing in sink, but dropping to and maintaining minimal altitude
			//monitor mc, stop if avg since start is not minimal climb
			//glide to minimal and/or wait 30 sec
			//land flag off, alt set to 50m
			//normal mc: use avg power for climbrate 0.7
			if (state_flags._.GPS_steering && ( desiredHeight > 70 ) )   //LOGO only, and not in landing phase.
			{
				if ( ( IMUlocationz._.W1 > 50 ) && ( IMUlocationz._.W1 < 90 ) ) //only keep track above 60m , don't train above 90m
				{
					if ( throttleAccum.WW == (int16_t)(max_throttle) && (!desired_behavior._.land) && (udb_pwIn[TEST_MODE_INPUT_CHANNEL]<3500) )   // motor is running, only if max, not in polar plot program
					{
						if ( sinkMotorOffTimer == 0 ) //only do this if the sinkMotorOffTimer is not running
						{
							if ( motorClimbRunStarted == 0 )
							{
								avgMotorRunClimbrate = 70; //startvalue. climbing has not begun yet
								motorClimbRunStarted = 1;
								motorClimbRunCount = 0;
							}
							else
							{
								motorClimbRunCount++;
							}
							if ( motorClimbRunCount > (4 * 40) )    //wait for motor climb to settle
							{
								avgMotorRunClimbrate = ((avgMotorRunClimbrate * 24.0) + vario)/25.0; // cm/s  10sec running avg, 40Hz	(from 70 to <0 after ~10 sec value -1)
								if ( avgMotorRunClimbrate < 10 ) //average should be above 0.1 m/s, else suppress motor for 30 sec.
								{
									//glide to minimal and/or wait 30 sec
									sinkMotorOffTimer = 1;  //start timer, motor only for maintaining 50m
									motorClimbRunStarted = 0; //stop tuning
								}
								else
								{
									avgMotorClimbrate = (((avgMotorClimbrate * 299.0 )+ (float)vario)/300.0); // cm/s  20sec running avg, 40Hz (after ~20 sec v)
									//assume normal power is enough for 2 m/s climb, reduce throttle for a 0.7 m/s climb
									//short fluctuations allow for thermal detection
									//increase / decrease power as needed for a 0.7 m/s climb
									// i.e. 0.7m/s => 0.7,  0.6m/s => 0.8 .. 0.8m/s => 0.6
									if (avgMotorClimbrate < 70)
									{
										steadyClimbPowerFactor += 0.0001;
									}
									if (avgMotorClimbrate > 70)
									{
										steadyClimbPowerFactor -= 0.0001;
									}
									if (steadyClimbPowerFactor < 0.70)
									{
										steadyClimbPowerFactor = 0.70;
									}
									if (steadyClimbPowerFactor > 1.0)
									{
										steadyClimbPowerFactor = 1.0;
									}
								}
							}
						}
					}
					else  //!throttle  | land
					{
						motorClimbRunStarted = 0;	//motor climbrate tuning must start over next time
						//if LOGO aborts climb, timer must keep on running
						//sinkMotorOffTimer = 0;      //end suppress motor
					}
				} //>50m
			}
			else //!Gps
			{
				motorClimbRunStarted = 0;	  //motor climrate tuning must start over next time
				sinkMotorOffTimer = 0;        //end suppress motor
				steadyClimbPowerFactor =0.80; //reset for stabilized mode ~30pct
			}
#endif // ( THERMALLING_MISSION == 1 )



#if ( THERMALLING_MISSION == 1 )
			/*
			if ( IMUlocationz._.W1 > 60 ) //only above 60m, when below, we may need more power to maintain minimal safe altitude
			{
				if ( sinkMotorOffTimer > 0 )
				{
					// no throttle above 60m with timout counter running
					throttleAccum.WW = 0;
				}
				else
				{
					// reduce throttle for a 0.7 m/s climb
					throttleAccum.WW = (int16_t)((float)(throttleAccum.WW) * steadyClimbPowerFactor);
				}
			}
			else   //below 60m
			*/
			{
				if ( sinkMotorOffTimer > 0 )
				{

					/*
					//use throttle as needed to remain at safe minimal altitude of 50m
					heightError._.W1 = -50;  //leave LOGO in control of desiredHeight
					heightError.WW = (heightError.WW + IMUlocationz.WW + speed_height) >> 13;
					if (heightError._.W0 < -height_marginx8)
					{
						throttleAccum.WW = (int16_t)(max_throttle);
					}
					else if (heightError._.W0 > height_marginx8)
					{
						throttleAccum.WW = 0;
					}
					else
					{
						throttleAccum.WW = (int16_t)(max_throttle) + (__builtin_mulss(throttle_height_gain, (-heightError._.W0))>>3);
						if (throttleAccum.WW > (int16_t)(max_throttle)) throttleAccum.WW = (int16_t)(max_throttle);
						if(throttleAccum.WW < (int16_t)((float)(max_throttle) * 0.5))
						{
							 throttleAccum.WW = (int16_t)((float)(max_throttle) * 0.5);  //use at least minimal power below 60m
						}
					}
					*/
					    //sink was detected. To save power, use burst of full throttle (enough to mostly counter/climb in sink) and glides.
					if ( (IMUlocationz._.W1) < 60 )
					{
						if ( (IMUlocationz._.W1) < 50 )
						{
							motorClimbSinkStarted = true;  // boolean climbing with motor in sink between 50 and 50 m
						}
						if ( motorClimbSinkStarted )
						{
							if ( vario > 70 )
							{
								// reduce throttle for a 0.7 m/s climb
								throttleAccum.WW = (int16_t)((float)(throttleAccum.WW) * steadyClimbPowerFactor);
							}
							else
							{
								throttleAccum.WW = (int16_t)(max_throttle);
							}
						}
						else
						{
							throttleAccum.WW = 0;
						}
					}
					else
					{
						motorClimbSinkStarted = false;
						throttleAccum.WW = 0;
					}
				}
				else
				{
					if ( desiredHeight > 60 ) // if not landing (in that case, don't modify throttleAccum.WW)
					{
						if ( (IMUlocationz._.W1) < 50 )
						{
								throttleAccum.WW = (int16_t)(max_throttle);
						}
						else
						{
							// reduce throttle for a 0.7 m/s climb
							throttleAccum.WW = (int16_t)((float)(throttleAccum.WW) * steadyClimbPowerFactor);
						}
					}
				}
			}
			//control autopilotBrake : brakes when too high or above glideslope
			//don't use speed_height
			//for Gliders we need a constant brake output
			heightError._.W1 = - desiredHeight; //
			heightError.WW = (heightError.WW + IMUlocationz.WW ) >> 13;
			if (desired_behavior._.land)    //if F_LAND is set in Logo
			{
				if (heightError._.W0 > 0)
				{
					//autopilotBrake; assume 0 brake = 0, full brake == 1700
					autopilotBrake = (__builtin_mulss(80, (heightError._.W0))>>3);
				}
				else
				{
					autopilotBrake = 0;
				}
			}
			else
			{
				autopilotBrake = 0;
			}

#endif  //AIRFRAME_GLIDER

#if (RACING_MODE == 1)
			if (state_flags._.GPS_steering)
			{
				throttleAccum.WW = (int32_t)(FIXED_WP_THROTTLE);
			}
#endif
		}


#if ( THERMALLING_MISSION == 1 )
		if ( sinkMotorOffTimer > 0 )
		{
			sinkMotorOffTimer++;
			// this will suppress motor , climb was insufficient due to sink
			if ( sinkMotorOffTimer > (40 * 30) )  //30 sec
			{
				motorClimbRunStarted = 0;	//motor climrate tuning must start over next time
				sinkMotorOffTimer = 0;      //end suppress motor
			}
		}
#endif // ( THERMALLING_MISSION == 1 )



		if (!state_flags._.altitude_hold_throttle)
		{
			manualThrottle(throttleIn);
		}
#if ( THERMALLING_MISSION != 1 )
		else if (state_flags._.GPS_steering && desired_behavior._.land)
#else
		else if (state_flags._.GPS_steering && desired_behavior._.land && (IMUlocationz._.W1 < 5 || IMUlocationz._.W1 > 50) ) //use both motor and brakes for glideslopes in landing circuit
#endif // ( THERMALLING_MISSION == 1 )
		{
			// place a ceiling, in other words, go down, but not up.
			if (pitchAltitudeAdjust > 0)
			{
				pitchAltitudeAdjust = 0;
			}

			throttleFiltered.WW += (((int32_t)(udb_pwTrim[THROTTLE_INPUT_CHANNEL] - throttleFiltered._.W1))<<THROTTLEFILTSHIFT);
			set_throttle_control(throttleFiltered._.W1 - throttleIn);
			filterManual = true;
		}
		else
		{
			// Servo reversing is handled in servoMix.c
			int16_t throttleOut = udb_servo_pulsesat(udb_pwTrim[THROTTLE_INPUT_CHANNEL] + throttleAccum.WW);
			throttleFiltered.WW += (((int32_t)(throttleOut - throttleFiltered._.W1)) << THROTTLEFILTSHIFT);
			set_throttle_control(throttleFiltered._.W1 - throttleIn);
			filterManual = true;
		}

		if (!state_flags._.altitude_hold_pitch)
		{
			pitchAltitudeAdjust = 0;
		}
	}
	else
	{
		pitchAltitudeAdjust = 0;
		manualThrottle(throttleIn);
	}
}

static void manualThrottle(int16_t throttleIn)
{
	int16_t throttle_control_pre;

	throttleFiltered.WW += (((int32_t)(throttleIn - throttleFiltered._.W1)) << THROTTLEFILTSHIFT);

	if (filterManual)
	{
		// Continue to filter the throttle control value in manual mode to avoid large, instant
		// changes to throttle value, which can burn out a brushed motor.  But after fading over
		// to the new throttle value, stop applying the filter to the throttle out to allow
		// faster control.
		throttle_control_pre = throttleFiltered._.W1 - throttleIn;
		if (throttle_control_pre < 10) filterManual = false;
	}
	else
	{
		throttle_control_pre = 0;
	}
	set_throttle_control(throttle_control_pre);
}

// For now, hovering does not attempt to control the throttle, and instead
// gives manual throttle control back to the pilot.
static void hoverAltitudeCntrl(void)
{
	int16_t throttle_control_pre;
	int16_t throttleIn = (udb_flags._.radio_on == 1) ? udb_pwIn[THROTTLE_INPUT_CHANNEL] : udb_pwTrim[THROTTLE_INPUT_CHANNEL];

	throttleFiltered.WW += (((int32_t)(throttleIn - throttleFiltered._.W1)) << THROTTLEFILTSHIFT);

	if (filterManual)
	{
		// Continue to filter the throttle control value in manual mode to avoid large, instant
		// changes to throttle value, which can burn out a brushed motor.  But after fading over
		// to the new throttle value, stop applying the filter to the throttle out to allow
		// faster control.
		throttle_control_pre = throttleFiltered._.W1 - throttleIn;
		if (throttle_control_pre < 10) filterManual = false;
	}
	else
	{
		throttle_control_pre = 0;
	}

	set_throttle_control(throttle_control_pre);
}

#else

void init_altitudeCntrlVariable(void)
{
}

#endif //(ALTITUDE_GAINS_VARIABLE == 1)
