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


#ifndef AIRSPEED_OPTIONS_H
#define AIRSPEED_OPTIONS_H



#if ( MODEL_FANTASY == 1 && HILSIM == 0 )
//Fantasy

// Airspeeds in m/s
#define MINIMUM_GROUNDSPEED         -3.0
//will be redefined?
#define MINIMUM_AIRSPEED             8.5    // 30 km/h
#define MAXIMUM_AIRSPEED            16.39   // 59 km/h
#define CRUISE_AIRSPEED             13.50   // 49 km/h, Gliding airspeed when aircraft is level

// Cruise airspeed may be below minimum airspeed for high drag aircraft.

// Gliding airspeed control
// Must use ALTITUDE_GAINS_VARIABLE =1 with this option.
#define GLIDE_AIRSPEED_CONTROL       1

// Pitch feedforward for gliding airspeed
// linearly interpolated from cruise airspeed to min and max airspeed
#define AIRSPEED_PITCH_MIN_ASPD     12.0     // Default off, start with 5.0
#define AIRSPEED_PITCH_MAX_ASPD     -0.5     // Default off, start with -10.0

// Maximum rate of pitch demand change in deg/s.  Used to make control smoother.
// Default 10.0, Higher for small aircraft. Too low may cause instability.
// Maximum value is 720deg/s.  
#define AIRSPEED_PITCH_ADJ_RATE     12.0

// Airspeed error integrator
#define AIRSPEED_PITCH_KI            0.010   // Integration rate.  low = unstable, high = slow response.
#define AIRSPEED_PITCH_KI_MAX        8.0     // Limit of integration control in degrees.  Start with 5.0.

#endif //Fantasy



#if ( MODEL_FANTASY == 1 && HILSIM == 1 )
//Fantasy hilsim

// Gliding airspeed control
// Must use ALTITUDE_GAINS_VARIABLE=1 with this option.
#define GLIDE_AIRSPEED_CONTROL       1

// Airspeeds in m/s
#define MINIMUM_GROUNDSPEED         -3.0
#define CRUISE_AIRSPEED              8.6    // 31 km/h, Gliding airspeed when aircraft is level
#define MINIMUM_AIRSPEED             9.0    // 35 km/h    //was 9.7
#define MAXIMUM_AIRSPEED            13.0    // 47 km/h

// Cruise airspeed may be below minimum airspeed for high drag aircraft.

// Pitch feedforward for gliding airspeed
// linearly interpolated from cruise airspeed to min and max airspeed
#define AIRSPEED_PITCH_MIN_ASPD     -4.2    // Default off, start with 5.0
#define AIRSPEED_PITCH_MAX_ASPD    -11.0    // Default off, start with -10.0
 
// Maximum rate of pitch demand change in deg/s.  Used to make control smoother.
// Default 10.0, Higher for small aircraft. Too low may cause instability.
// Maximum value is 720deg/s.  
#define AIRSPEED_PITCH_ADJ_RATE     20.0

// Airspeed error integrator
#define AIRSPEED_PITCH_KI           0.011   // Integration rate.  High = unstable, low = slow response.
#define AIRSPEED_PITCH_KI_MAX       7.0     // Limit of integration control in degrees.  Start with 5.0.

#endif //Fantasy hilsim



#if ( MODEL_GRAFAS == 1 )
//Grafas

// Airspeeds in m/s
#define MINIMUM_GROUNDSPEED         -3.0
//will be redefined?
#define MINIMUM_AIRSPEED             5.7   // 3? km/h
#define CRUISE_AIRSPEED             11.8   // 4? km/h, Gliding airspeed when aircraft is level
#define MAXIMUM_AIRSPEED            17.1   // 5? km/h

// Cruise airspeed may be below minimum airspeed for high drag aircraft.

// Gliding airspeed control
// Must use ALTITUDE_GAINS_VARIABLE=1 with this option.
#define GLIDE_AIRSPEED_CONTROL       1

// Pitch feedforward for gliding airspeed
// linearly interpolated from cruise airspeed to min and max airspeed
#define AIRSPEED_PITCH_MIN_ASPD      8.0     // Default off, start with 5.0
#define AIRSPEED_PITCH_MAX_ASPD     -8.0     // Default off, start with -10.0

// Maximum rate of pitch demand change in deg/s.  Used to make control smoother.
// Default 10.0, Higher for small aircraft. Too low may cause instability.
// Maximum value is 720deg/s.  
	
#define AIRSPEED_PITCH_ADJ_RATE     16.0
	
// Airspeed error integrator
#define AIRSPEED_PITCH_KI            0.010   // Integration rate.  High? = unstable, low? = slow response.
#define AIRSPEED_PITCH_KI_MAX       10.0     // Limit of integration control in degrees.  Start with 5.0.

#endif //Grafas



#if ( MODEL_LINEA == 1 )
//Linea

// Airspeeds in m/s
#define MINIMUM_GROUNDSPEED         -3.0
//will be redefined?
#define MINIMUM_AIRSPEED             9.0		// 32 km/h
#define MAXIMUM_AIRSPEED            15.0    // 54 km/h
#define CRUISE_AIRSPEED             11.6    // 42 km/h Gliding airspeed when aircraft is level

// Cruise airspeed may be below minimum airspeed for high drag aircraft.

// Gliding airspeed control
// Must use ALTITUDE_GAINS_VARIABLE=1 with this option.
#define GLIDE_AIRSPEED_CONTROL       1

// Pitch feedforward for gliding airspeed
// linearly interpolated from cruise airspeed to min and max airspeed
#define AIRSPEED_PITCH_MIN_ASPD      6.0     // Default off, start with 5.0
#define AIRSPEED_PITCH_MAX_ASPD     -7.0     // Default off, start with -10.0

// Maximum rate of pitch demand change in deg/s.  Used to make control smoother.
// Default 10.0, Higher for small aircraft. Too low may cause instability.
// Maximum value is 720deg/s.
#define AIRSPEED_PITCH_ADJ_RATE     12.0

// Airspeed error integrator
#define AIRSPEED_PITCH_KI            0.2    // Integrataion rate.  High = unstable, low = slow response.
#define AIRSPEED_PITCH_KI_MAX        4.0    // Limit of integration control in degrees.  Start with 5.0.

#endif //Linea





#endif // AIRSPEED_OPTIONS_H
