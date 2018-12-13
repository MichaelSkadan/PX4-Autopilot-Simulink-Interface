/*******************************************************************************
* File:
* PX4Interface.h
*
* Description:
* Simulink C code S-Function PX4 Interface header file.
* Declares public PX4 Interface functions utilized by the Simulink S-Function.
* These functions provide support for configuring and communicating with a PX4.
*
* Note: 
* Current implementation communicates with PX4 via MAVLink Version 2 over UDP 
* on an IP network and requires PX4 to have IP adress "192.168.46.2".
*
* Author:
* Michael Skadan
*
********************************************************************************
* Notices:
* Copyright 2018, United States Government as represented by the Administrator
* of the National Aeronautics and Space Administration. All Rights Reserved.
*
* MAVLINK__________________________________________
* Portions of this software were generated using Mavlink
* (https://github.com/mavlink) and are subject to the following MIT License:
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of the generated software (the "Generated Software"), to deal in the Generated
* Software without restriction, including without limitation the rights to use,
* copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
* the Generated Software, and to permit persons to whom the Generated Software
* is furnished to do so, subject to the following conditions:
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Generated Software.
*
* THE GENERATED SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO
* EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE GENERATED SOFTWARE OR THE USE
* OR OTHER DEALINGS IN THE GENERATED SOFTWARE.
*
* NASA Disclaimers
* No Warranty:
* THE SUBJECT SOFTWARE IS PROVIDED "AS IS" WITHOUT ANY WARRANTY OF ANY KIND,
* EITHER EXPRESSED, IMPLIED, OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, ANY
* WARRANTY THAT THE SUBJECT SOFTWARE WILL CONFORM TO SPECIFICATIONS, ANY IMPLIED
* WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, OR FREEDOM
* FROM INFRINGEMENT, ANY WARRANTY THAT THE SUBJECT SOFTWARE WILL BE ERROR FREE,
* OR ANY WARRANTY THAT DOCUMENTATION, IF PROVIDED, WILL CONFORM TO THE SUBJECT
* SOFTWARE. THIS AGREEMENT DOES NOT, IN ANY MANNER, CONSTITUTE AN ENDORSEMENT BY
* GOVERNMENT AGENCY OR ANY PRIOR RECIPIENT OF ANY RESULTS, RESULTING DESIGNS,
* HARDWARE, SOFTWARE PRODUCTS OR ANY OTHER APPLICATIONS RESULTING FROM USE OF
* THE SUBJECT SOFTWARE.  FURTHER, GOVERNMENT AGENCY DISCLAIMS ALL WARRANTIES AND
* LIABILITIES REGARDING THIRD-PARTY SOFTWARE, IF PRESENT IN THE ORIGINAL
* SOFTWARE, AND DISTRIBUTES IT "AS IS."
*
* Waiver and Indemnity:
* RECIPIENT AGREES TO WAIVE ANY AND ALL CLAIMS AGAINST THE UNITED STATES
* GOVERNMENT, ITS CONTRACTORS AND SUBCONTRACTORS, AS WELL AS ANY PRIOR
* RECIPIENT.  IF RECIPIENT'S USE OF THE SUBJECT SOFTWARE RESULTS IN ANY
* LIABILITIES, DEMANDS, DAMAGES, EXPENSES OR LOSSES ARISING FROM SUCH USE,
* INCLUDING ANY DAMAGES FROM PRODUCTS BASED ON, OR RESULTING FROM, RECIPIENT'S
* USE OF THE SUBJECT SOFTWARE, RECIPIENT SHALL INDEMNIFY AND HOLD HARMLESS THE
* UNITED STATES GOVERNMENT, ITS CONTRACTORS AND SUBCONTRACTORS, AS WELL AS ANY
* PRIOR RECIPIENT, TO THE EXTENT PERMITTED BY LAW.  RECIPIENT'S SOLE REMEDY FOR
* ANY SUCH MATTER SHALL BE THE IMMEDIATE, UNILATERAL TERMINATION OF THIS
* AGREEMENT.
*
*******************************************************************************/

#ifndef PX4_INTERFACE_H
#define PX4_INTERFACE_H

#include <stdint.h>

/*******************************************************************************
* This function is called when the S-function begins execution.
* Configures resources for communcation with PX4 via UDP on IP network.
* Note: Current implementation requires PX4 to have IP adress "192.168.46.2".
*******************************************************************************/
void start();

/*******************************************************************************
* This function is called each time step of the S-functions execution.
* Provides sensor input to PX4 and obtains actuator output from PX4.
* These input and ouput values correspond directly to the following MAVLink
* messages: HIL_ACTUATOR_CONTROLS, HIL_SENSOR, HIL_GPS, and RC_CHANNELS.
* parameters:
*  delta - input time since last step calculation (microsecond)
*  imu - input contains current values from inertial measurement unit sensor
*    imu[0]: X Acceleration (m/s^2)
*    imu[1]: Y Acceleration (m/s^2)
*    imu[2]: Z Acceleration (m/s^2)
*    imu[3]: X Angular Speed (rad/s)
*    imu[4]: Y Angular Speed (rad/s)
*    imu[5]: Z Angular Speed (rad/s)
*    imu[6]: X Magnetic Field (Gauss)
*    imu[7]: Y Magnetic Field (Gauss)
*    imu[8]: Z Magnetic Field (Gauss)
*    imu[9]: Absolute Pressure (millibar)
*    imu[10]: Differential Pressure (millibar)
*    imu[11]: Altitude Pressure Calabrated (m)
*    imu[12]: Temperature (millibar)
*  gps - input contains current values from global position system sensor
*    gps[0]: Latitude (degrees * 1E7)
*    gps[1]: Longitude (degrees * 1E7)
*    gps[2]: Altitude (AMSL meters * 1000) +Up
*    gps[3]: GPS HDOP (cm)
*    gps[4]: GPS VDOP (cm)
*    gps[5]: GPS Ground Speed (cm/s)
*    gps[6]: GPS North Velocity (cm/s)
*    gps[7]: GPS East Velocity (cm/s)
*    gps[8]: GPS Down Velocity (cm/s) +Down
*    gps[9]: Course Over Ground (degrees * 100)
*    gps[10]: Fix Type (0 = None, 1 = 2D, 2 = 3D)
*    gps[11]: Satellites Visible
*  rc - input contains current values from remote control sensor
*    rc[0 - 17] are Channels 1 through 18 (microseconds)
*    rc[18]: Used Channel Count
*    rc[19]: Received Signal Strength Indicator (0 = 0% - 100 = 100%)
*  actuators - output current actuator commands received from PX4
*    actuators[0 - 15] are Channels 1 through 16 (0.0 = 0% - 1.0 = 100%)
*******************************************************************************/
void step(uint32_t delta[1], double imu[13], double gps[12], uint16_t rc[20], double actuators[16]);

#endif