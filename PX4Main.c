/*******************************************************************************
* File:
* PX4Main.c
*
* Description:
* Simulink C code S-Function PX4 Autopilot Interface implementation source file.
* This functionality provides support for configuring and communicating with a PX4.
*
* Note: 
* Current implementation communicates with PX4 via MAVLink Version 2 over 
* UDP on an IP network and requires PX4 to have IP adress "192.168.46.2".
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

// Public Function Declarations
#include "PX4Interface.h"

// Define Math Constants
#define _USE_MATH_DEFINES

#include <math.h>

#include <stdbool.h>
#include <string.h>

// Disable winsock deprecated API warnings
#define _WINSOCK_DEPRECATED_NO_WARNINGS

#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32")

// Support text output on Matlab console
#include "mex.h"

//------------------------------------------------------------------------------
// Private Communications functionality forward declarations
//------------------------------------------------------------------------------
void initializeCommunications();
void autopilot_input(uint32_t delta[1], double imu[13], double gps[12], uint16_t rc[20]);
void autopilot_output(double actuators[17]);


//------------------------------------------------------------------------------
// Public functionality utilized by Simulink S-Function
//------------------------------------------------------------------------------

/******************************************************************************
* This function is called when the S-function begins execution.
* Configures resources for communcation with PX4 via UDP on IP network.
* Note: Current implementation requires PX4 to have IP adress "192.168.46.2".
*******************************************************************************/
void start()
{
  initializeCommunications();
}

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
void step(uint32_t delta[1], double imu[13], double gps[12], uint16_t rc[20], double actuators[16])
{
  // Send the current sensor measurement values to PX4
  autopilot_input(delta, imu, gps, rc);
  // Receive the current actuator commands from the PX4
  autopilot_output(actuators);
}

//------------------------------------------------------------------------------
// Private functionality utilized by public interface functions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// MAVLink functionality
// MAVLink code snippets copied directly from https://github.com/mavlink/c_library_v2
// Snippets copied from the following files: mavlink.h, mavlink_types.h, 
// mavlink_sha256.h, mavlink_msg_heartbeat.h, mavlink_msg_hil_sensor.h, 
// mavlink_msg_hil_gps.h, mavlink_msg_rc_channels.h, common.h, and checksum.h
//------------------------------------------------------------------------------

// Copied from mavlink_types.h -------------------------------------------------
#define _MAV_PAYLOAD(msg) ((const char *)(&((msg)->payload64[0])))
#define _MAV_PAYLOAD_NON_CONST(msg) ((char *)(&((msg)->payload64[0])))

// checksum is immediately after the payload bytes
#define mavlink_ck_a(msg) *((msg)->len + (uint8_t *)_MAV_PAYLOAD_NON_CONST(msg))
#define mavlink_ck_b(msg) *(((msg)->len+(uint16_t)1) + (uint8_t *)_MAV_PAYLOAD_NON_CONST(msg))

typedef enum {
  MAVLINK_COMM_0,
  MAVLINK_COMM_1,
  MAVLINK_COMM_2,
  MAVLINK_COMM_3
} mavlink_channel_t;

/*
 * applications can set MAVLINK_COMM_NUM_BUFFERS to the maximum number
 * of buffers they will use. If more are used, then the result will be
 * a stack overrun
 */
#define MAVLINK_COMM_NUM_BUFFERS           16

typedef enum {
  MAVLINK_PARSE_STATE_UNINIT = 0,
  MAVLINK_PARSE_STATE_IDLE,
  MAVLINK_PARSE_STATE_GOT_STX,
  MAVLINK_PARSE_STATE_GOT_LENGTH,
  MAVLINK_PARSE_STATE_GOT_INCOMPAT_FLAGS,
  MAVLINK_PARSE_STATE_GOT_COMPAT_FLAGS,
  MAVLINK_PARSE_STATE_GOT_SEQ,
  MAVLINK_PARSE_STATE_GOT_SYSID,
  MAVLINK_PARSE_STATE_GOT_COMPID,
  MAVLINK_PARSE_STATE_GOT_MSGID1,
  MAVLINK_PARSE_STATE_GOT_MSGID2,
  MAVLINK_PARSE_STATE_GOT_MSGID3,
  MAVLINK_PARSE_STATE_GOT_PAYLOAD,
  MAVLINK_PARSE_STATE_GOT_CRC1,
  MAVLINK_PARSE_STATE_GOT_BAD_CRC1,
  MAVLINK_PARSE_STATE_SIGNATURE_WAIT
} mavlink_parse_state_t; // The state machine for the comm parser

typedef enum {
  MAVLINK_FRAMING_INCOMPLETE = 0,
  MAVLINK_FRAMING_OK = 1,
  MAVLINK_FRAMING_BAD_CRC = 2,
  MAVLINK_FRAMING_BAD_SIGNATURE = 3
} mavlink_framing_t;

#define MAVLINK_STATUS_FLAG_OUT_MAVLINK1 2 // generate MAVLink1 by default

// Macro to define packed structures
#define MAVPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )

#define MAVLINK_MAX_PAYLOAD_LEN 255 ///< Maximum payload length
#define MAVLINK_CORE_HEADER_LEN 9 ///< Length of core header (of the comm. layer)
#define MAVLINK_CORE_HEADER_MAVLINK1_LEN 5 ///< Length of MAVLink1 core header (of the comm. layer)
#define MAVLINK_NUM_HEADER_BYTES (MAVLINK_CORE_HEADER_LEN + 1) ///< Length of all header bytes, including core and stx
#define MAVLINK_NUM_CHECKSUM_BYTES 2
#define MAVLINK_NUM_NON_PAYLOAD_BYTES (MAVLINK_NUM_HEADER_BYTES + MAVLINK_NUM_CHECKSUM_BYTES)

#define MAVLINK_SIGNATURE_BLOCK_LEN        13

#define MAVLINK_MAX_PACKET_LEN (MAVLINK_MAX_PAYLOAD_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES)

MAVPACKED(
typedef struct __mavlink_message {
    uint16_t checksum;      ///< sent at end of packet
    uint8_t magic;          ///< protocol magic marker
    uint8_t len;            ///< Length of payload
    uint8_t incompat_flags; ///< flags that must be understood
    uint8_t compat_flags;   ///< flags that can be ignored if not understood
    uint8_t seq;            ///< Sequence of packet
    uint8_t sysid;          ///< ID of message sender system/aircraft
    uint8_t compid;         ///< ID of the message sender component
    uint32_t msgid:24;      ///< ID of message in payload
    uint64_t payload64[(MAVLINK_MAX_PAYLOAD_LEN+MAVLINK_NUM_CHECKSUM_BYTES+7)/8];
    uint8_t ck[2];          ///< incoming checksum bytes
    uint8_t signature[MAVLINK_SIGNATURE_BLOCK_LEN];
}) mavlink_message_t;

#define MAVLINK_STATUS_FLAG_IN_MAVLINK1  1 // last incoming packet was MAVLink1

#define MAVLINK_STX_MAVLINK1               0xFE

typedef struct __mavlink_status {
    uint8_t msg_received;               ///< Number of received messages
    uint8_t buffer_overrun;             ///< Number of buffer overruns
    uint8_t parse_error;                ///< Number of parse errors
    mavlink_parse_state_t parse_state;  ///< Parsing state machine
    uint8_t packet_idx;                 ///< Index in current packet
    uint8_t current_rx_seq;             ///< Sequence number of last packet received
    uint8_t current_tx_seq;             ///< Sequence number of last packet sent
    uint16_t packet_rx_success_count;   ///< Received packets
    uint16_t packet_rx_drop_count;      ///< Number of packet drops
    uint8_t flags;                      ///< MAVLINK_STATUS_FLAG_*
    uint8_t signature_wait;             ///< number of signature bytes left to receive
    struct __mavlink_signing *signing;  ///< optional signing state
    struct __mavlink_signing_streams *signing_streams; ///< global record of stream timestamps
} mavlink_status_t;

/*
  a callback function to allow for accepting unsigned packets
 */
typedef bool (*mavlink_accept_unsigned_t)(const mavlink_status_t *status, uint32_t msgid);

/*
  flags controlling signing
 */
#define MAVLINK_SIGNING_FLAG_SIGN_OUTGOING 1

/*
  state of MAVLink signing for this channel
 */
typedef struct __mavlink_signing {
    uint8_t flags;                     ///< MAVLINK_SIGNING_FLAG_*
    uint8_t link_id;
    uint64_t timestamp;
    uint8_t secret_key[32];
    mavlink_accept_unsigned_t accept_unsigned_callback;
} mavlink_signing_t;

/*
  timestamp state of each logical signing stream. This needs to be the same structure for all
  connections in order to be secure
 */
#define MAVLINK_MAX_SIGNING_STREAMS        16

typedef struct __mavlink_signing_streams {
  uint16_t num_signing_streams;
  struct __mavlink_signing_stream {
    uint8_t link_id;
    uint8_t sysid;
    uint8_t compid;
    uint8_t timestamp_bytes[6];
  } stream[MAVLINK_MAX_SIGNING_STREAMS];
} mavlink_signing_streams_t;

/*
  entry in table of information about each message type
 */
typedef struct __mavlink_msg_entry {
    uint32_t msgid;
    uint8_t crc_extra;
    uint8_t msg_len;
    uint8_t flags;             // MAV_MSG_ENTRY_FLAG_*
    uint8_t target_system_ofs; // payload offset to target_system, or 0
    uint8_t target_component_ofs; // payload offset to target_component, or 0
} mavlink_msg_entry_t;

/*
  incompat_flags bits
 */
#define MAVLINK_IFLAG_SIGNED  0x01
#define MAVLINK_IFLAG_MASK    0x01 // mask of all understood bits

// Copied from mavlink.h -------------------------------------------------------
#define MAVLINK_STX 253

// Copied from checksum.h ------------------------------------------------------
#define X25_INIT_CRC                       0xffff

// Copied from mavlink_sha256.h ------------------------------------------------
typedef struct {
  unsigned int sz[2];
  uint32_t counter[8];
  union {
    unsigned char save_bytes[64];
    uint32_t save_u32[16];
  } u;
} mavlink_sha256_ctx;

#define Ch(x,y,z) (((x) & (y)) ^ ((~(x)) & (z)))
#define Maj(x,y,z) (((x) & (y)) ^ ((x) & (z)) ^ ((y) & (z)))

#define ROTR(x,n)   (((x)>>(n)) | ((x) << (32 - (n))))

#define Sigma0(x) (ROTR(x,2)  ^ ROTR(x,13) ^ ROTR(x,22))
#define Sigma1(x) (ROTR(x,6)  ^ ROTR(x,11) ^ ROTR(x,25))
#define sigma0(x) (ROTR(x,7)  ^ ROTR(x,18) ^ ((x)>>3))
#define sigma1(x) (ROTR(x,17) ^ ROTR(x,19) ^ ((x)>>10))

#define A m->counter[0]
#define B m->counter[1]
#define C m->counter[2]
#define D m->counter[3]
#define E m->counter[4]
#define F m->counter[5]
#define G m->counter[6]
#define H m->counter[7]

static const uint32_t mavlink_sha256_constant_256[64] = 
{
  0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5,
  0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
  0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3,
  0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
  0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc,
  0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
  0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
  0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
  0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13,
  0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
  0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3,
  0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
  0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5,
  0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
  0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
  0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2
};

// Copied from common.h ------------------------------------------------------
#define MAVLINK_MESSAGE_LENGTHS {}

// Copied from mavlink_msg_heartbeat.h -----------------------------------------
#define MAVLINK_MSG_ID_HEARTBEAT 0

MAVPACKED(
typedef struct __mavlink_heartbeat_t {
 uint32_t custom_mode; /*< A bitfield for use for autopilot-specific flags.*/
 uint8_t type; /*< Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)*/
 uint8_t autopilot; /*< Autopilot type / class. defined in MAV_AUTOPILOT ENUM*/
 uint8_t base_mode; /*< System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h*/
 uint8_t system_status; /*< System status flag, see MAV_STATE ENUM*/
 uint8_t mavlink_version; /*< MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_version*/
}) mavlink_heartbeat_t;

#define MAVLINK_MSG_ID_HEARTBEAT_LEN 9

// Copied from mavlink_msg_hil_sensor.h ----------------------------------------
#define MAVLINK_MSG_ID_HIL_SENSOR 107

MAVPACKED(
typedef struct __mavlink_hil_sensor_t {
 uint64_t time_usec; /*< Timestamp (microseconds, synced to UNIX time or since system boot)*/
 float xacc; /*< X acceleration (m/s^2)*/
 float yacc; /*< Y acceleration (m/s^2)*/
 float zacc; /*< Z acceleration (m/s^2)*/
 float xgyro; /*< Angular speed around X axis in body frame (rad / sec)*/
 float ygyro; /*< Angular speed around Y axis in body frame (rad / sec)*/
 float zgyro; /*< Angular speed around Z axis in body frame (rad / sec)*/
 float xmag; /*< X Magnetic field (Gauss)*/
 float ymag; /*< Y Magnetic field (Gauss)*/
 float zmag; /*< Z Magnetic field (Gauss)*/
 float abs_pressure; /*< Absolute pressure in millibar*/
 float diff_pressure; /*< Differential pressure (airspeed) in millibar*/
 float pressure_alt; /*< Altitude calculated from pressure*/
 float temperature; /*< Temperature in degrees celsius*/
 uint32_t fields_updated; /*< Bitmask for fields that have updated since last message, bit 0 = xacc, bit 12: temperature, bit 31: full reset of attitude/position/velocities/etc was performed in sim.*/
}) mavlink_hil_sensor_t;

#define MAVLINK_MSG_ID_HIL_SENSOR_LEN 64
#define MAVLINK_MSG_ID_HIL_SENSOR_MIN_LEN 64

#define MAVLINK_MSG_ID_HIL_SENSOR_CRC 108

// Copied from mavlink_msg_hil_gps.h -------------------------------------------
#define MAVLINK_MSG_ID_HIL_GPS 113

MAVPACKED(
typedef struct __mavlink_hil_gps_t {
 uint64_t time_usec; /*< Timestamp (microseconds since UNIX epoch or microseconds since system boot)*/
 int32_t lat; /*< Latitude (WGS84), in degrees * 1E7*/
 int32_t lon; /*< Longitude (WGS84), in degrees * 1E7*/
 int32_t alt; /*< Altitude (AMSL, not WGS84), in meters * 1000 (positive for up)*/
 uint16_t eph; /*< GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535*/
 uint16_t epv; /*< GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: 65535*/
 uint16_t vel; /*< GPS ground speed in cm/s. If unknown, set to: 65535*/
 int16_t vn; /*< GPS velocity in cm/s in NORTH direction in earth-fixed NED frame*/
 int16_t ve; /*< GPS velocity in cm/s in EAST direction in earth-fixed NED frame*/
 int16_t vd; /*< GPS velocity in cm/s in DOWN direction in earth-fixed NED frame*/
 uint16_t cog; /*< Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535*/
 uint8_t fix_type; /*< 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.*/
 uint8_t satellites_visible; /*< Number of satellites visible. If unknown, set to 255*/
}) mavlink_hil_gps_t;

#define MAVLINK_MSG_ID_HIL_GPS_LEN 36
#define MAVLINK_MSG_ID_HIL_GPS_MIN_LEN 36

#define MAVLINK_MSG_ID_HIL_GPS_CRC 124

// Copied from mavlink_msg_rc_channels.h ---------------------------------------
#define MAVLINK_MSG_ID_RC_CHANNELS 65

MAVPACKED(
typedef struct __mavlink_rc_channels_t {
 uint32_t time_boot_ms; /*< Timestamp (milliseconds since system boot)*/
 uint16_t chan1_raw; /*< RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused.*/
 uint16_t chan2_raw; /*< RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused.*/
 uint16_t chan3_raw; /*< RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused.*/
 uint16_t chan4_raw; /*< RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused.*/
 uint16_t chan5_raw; /*< RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused.*/
 uint16_t chan6_raw; /*< RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused.*/
 uint16_t chan7_raw; /*< RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused.*/
 uint16_t chan8_raw; /*< RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused.*/
 uint16_t chan9_raw; /*< RC channel 9 value, in microseconds. A value of UINT16_MAX implies the channel is unused.*/
 uint16_t chan10_raw; /*< RC channel 10 value, in microseconds. A value of UINT16_MAX implies the channel is unused.*/
 uint16_t chan11_raw; /*< RC channel 11 value, in microseconds. A value of UINT16_MAX implies the channel is unused.*/
 uint16_t chan12_raw; /*< RC channel 12 value, in microseconds. A value of UINT16_MAX implies the channel is unused.*/
 uint16_t chan13_raw; /*< RC channel 13 value, in microseconds. A value of UINT16_MAX implies the channel is unused.*/
 uint16_t chan14_raw; /*< RC channel 14 value, in microseconds. A value of UINT16_MAX implies the channel is unused.*/
 uint16_t chan15_raw; /*< RC channel 15 value, in microseconds. A value of UINT16_MAX implies the channel is unused.*/
 uint16_t chan16_raw; /*< RC channel 16 value, in microseconds. A value of UINT16_MAX implies the channel is unused.*/
 uint16_t chan17_raw; /*< RC channel 17 value, in microseconds. A value of UINT16_MAX implies the channel is unused.*/
 uint16_t chan18_raw; /*< RC channel 18 value, in microseconds. A value of UINT16_MAX implies the channel is unused.*/
 uint8_t chancount; /*< Total number of RC channels being received. This can be larger than 18, indicating that more channels are available but not given in this message. This value should be 0 when no RC channels are available.*/
 uint8_t rssi; /*< Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.*/
}) mavlink_rc_channels_t;

#define MAVLINK_MSG_ID_RC_CHANNELS_LEN 42
#define MAVLINK_MSG_ID_RC_CHANNELS_MIN_LEN 42

#define MAVLINK_MSG_ID_RC_CHANNELS_CRC 118

// Copied from mavlink_msg_rc_channels.h ---------------------------------------
#define MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS 93

MAVPACKED(
typedef struct __mavlink_hil_actuator_controls_t {
 uint64_t time_usec; /*< Timestamp (microseconds since UNIX epoch or microseconds since system boot)*/
 uint64_t flags; /*< Flags as bitfield, reserved for future use.*/
 float controls[16]; /*< Control outputs -1 .. 1. Channel assignment depends on the simulated hardware.*/
 uint8_t mode; /*< System mode (MAV_MODE), includes arming state.*/
}) mavlink_hil_actuator_controls_t;

#define MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_LEN 81


// MAVLink functionality forward declarations
uint16_t mavlink_msg_hil_sensor_construct_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, uint64_t time_usec, double imu[13], uint32_t fields_updated);
uint16_t mavlink_msg_hil_gps_construct_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, uint64_t time_usec, double gps[12]);
uint16_t mavlink_msg_rc_channels_construct_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, uint32_t time_boot_ms, double rc[20]);
mavlink_status_t* mavlink_get_channel_status(uint8_t chan);
mavlink_message_t* mavlink_get_channel_buffer(uint8_t chan);
uint8_t mavlink_sign_packet(mavlink_signing_t *signing, uint8_t signature[MAVLINK_SIGNATURE_BLOCK_LEN], const uint8_t *header, uint8_t header_len, const uint8_t *packet, uint8_t packet_len, const uint8_t crc[2]);
uint8_t _mav_trim_payload(const char *payload, uint8_t length);
bool mavlink_signature_check(mavlink_signing_t *signing, mavlink_signing_streams_t *signing_streams, const mavlink_message_t *msg);
uint16_t mavlink_finalize_message_buffer(mavlink_message_t* msg, uint8_t system_id, uint8_t component_id, mavlink_status_t* status, uint8_t min_length, uint8_t length, uint8_t crc_extra);
void _mav_parse_error(mavlink_status_t *status);
uint16_t mavlink_finalize_message_chan(mavlink_message_t* msg, uint8_t system_id, uint8_t component_id, uint8_t chan, uint8_t min_length, uint8_t length, uint8_t crc_extra);
uint16_t mavlink_msg_to_send_buffer(uint8_t *buf, const mavlink_message_t *msg);
void mavlink_start_checksum(mavlink_message_t* msg);
void mavlink_update_checksum(mavlink_message_t* msg, uint8_t c);
const mavlink_msg_entry_t* mavlink_get_msg_entry(uint32_t msgid);
uint8_t mavlink_frame_char_buffer(mavlink_message_t* rxmsg, mavlink_status_t* status, uint8_t c, mavlink_message_t* r_message, mavlink_status_t* r_mavlink_status);
uint8_t mavlink_frame_char(uint8_t chan, uint8_t c, mavlink_message_t* r_message, mavlink_status_t* r_mavlink_status);
uint8_t mavlink_parse_char(uint8_t chan, uint8_t c, mavlink_message_t* r_message, mavlink_status_t* r_mavlink_status);
void mavlink_sha256_init(mavlink_sha256_ctx *m);
void mavlink_sha256_calc(mavlink_sha256_ctx *m, uint32_t *in);
void mavlink_sha256_update(mavlink_sha256_ctx *m, const void *v, uint32_t len);
void mavlink_sha256_final_48(mavlink_sha256_ctx *m, uint8_t result[6]);
void crc_accumulate(uint8_t data, uint16_t *crcAccum);
void crc_init(uint16_t* crcAccum);
uint16_t crc_calculate(const uint8_t* pBuffer, uint16_t length);
void crc_accumulate_buffer(uint16_t *crcAccum, const char *pBuffer, uint16_t length);
void mavlink_msg_heartbeat_decode(const mavlink_message_t* msg, mavlink_heartbeat_t* heartbeat);
void mavlink_msg_hil_actuator_controls_decode(const mavlink_message_t* msg, mavlink_hil_actuator_controls_t* hil_actuator_controls);

/******************************************************************************
* Construct IMU data MAVlink message. 
* Combined functionality of mavlink_msg_hil_sensor_encode_chan and 
* mavlink_msg_hil_sensor_pack_chan into a single function.
* Based on code in mavlink_msg_hil_sensor.h
*
* Pack a hil_sensor message on a channel
* parameters:
*  system_id ID of this system
*  component_id ID of this component (e.g. 200 for IMU)
*  chan The MAVLink channel this message will be sent over
*  msg The MAVLink message to compress the data into
*  time_usec Timestamp (microseconds, synced to UNIX time or since system boot)
*  imu current inertia measurment unit values
*  fields_updated Bitmask for fields that have updated since last message, bit 0 = xacc, bit 12: temperature, bit 31: full reset of attitude/position/velocities/etc was performed in sim.
* return:
*  length of the message in bytes (excluding serial stream start sign)
*******************************************************************************/
uint16_t mavlink_msg_hil_sensor_construct_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, uint64_t time_usec, double imu[13], uint32_t fields_updated)
{
  // copy the values into the packet
  mavlink_hil_sensor_t packet;
  packet.time_usec      = time_usec;
  packet.xacc           = (float)imu[0];
  packet.yacc           = (float)imu[1];
  packet.zacc           = (float)imu[2];
  packet.xgyro          = (float)imu[3];
  packet.ygyro          = (float)imu[4];
  packet.zgyro          = (float)imu[5];
  packet.xmag           = (float)imu[6];
  packet.ymag           = (float)imu[7];
  packet.zmag           = (float)imu[8];
  packet.abs_pressure   = (float)imu[9];
  packet.diff_pressure  = (float)imu[10];
  packet.pressure_alt   = (float)imu[11];
  packet.temperature    = (float)imu[12];
  packet.fields_updated = fields_updated;

  // copy the packet into the payload of the MAVLink message
  memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HIL_SENSOR_LEN);
  // Specify the type of MAVlink message
  msg->msgid = MAVLINK_MSG_ID_HIL_SENSOR;

  // Calculate checksum and set length and aircraft id.
  return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_HIL_SENSOR_MIN_LEN, MAVLINK_MSG_ID_HIL_SENSOR_LEN, MAVLINK_MSG_ID_HIL_SENSOR_CRC);
}

/******************************************************************************
* Construct GPS data MAVlink message.
* Combined functionality of mavlink_msg_hil_gps_encode_chan and 
* mavlink_msg_hil_gps_pack_chan into a single function.
* Based on code in mavlink_msg_hil_gps.h
*
* Pack a hil_gps message on a channel
* parameters:
*  system_id ID of this system
*  component_id ID of this component (e.g. 200 for IMU)
*  chan The MAVLink channel this message will be sent over
*  msg The MAVLink message to compress the data into
*  time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
*  gps current Global Position System values
* return:
*  length of the message in bytes (excluding serial stream start sign)
*******************************************************************************/
uint16_t mavlink_msg_hil_gps_construct_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, uint64_t time_usec, double gps[12])
{
  // copy the values into the packet
  mavlink_hil_gps_t packet;
  packet.time_usec          = (uint64_t)time_usec;
  packet.lat                = (int32_t)gps[0];
  packet.lon                = (int32_t)gps[1];
  packet.alt                = (int32_t)gps[2];
  packet.eph                = (uint16_t)gps[3];
  packet.epv                = (uint16_t)gps[4];
  packet.vel                = (uint16_t)gps[5];
  packet.vn                 = (int16_t)gps[6];
  packet.ve                 = (int16_t)gps[7];
  packet.vd                 = (int16_t)gps[8];
  packet.cog                = (uint16_t)gps[9];
  packet.fix_type           = (uint8_t)gps[10];
  packet.satellites_visible = (uint8_t)gps[11];

  // copy the packet into the payload of the MAVLink message
  memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HIL_GPS_LEN);
  // Specify the type of MAVlink message
  msg->msgid = MAVLINK_MSG_ID_HIL_GPS;

  // Calculate checksum and set length and aircraft id.
  return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_HIL_GPS_MIN_LEN, MAVLINK_MSG_ID_HIL_GPS_LEN, MAVLINK_MSG_ID_HIL_GPS_CRC);
}

/******************************************************************************
* Construct RC data MAVlink message.
* Combined functionality of mavlink_msg_rc_channels_encode_chan and 
* mavlink_msg_rc_channels_pack_chan into a single function.
* Based on code in mavlink_msg_rc_channels.h
*
* Pack a rc_channels message
* parameters:
*  system_id ID of this system
*  component_id ID of this component (e.g. 200 for IMU)
*  chan The MAVLink channel this message will be sent over
*  msg The MAVLink message to compress the data into
*  time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
*  rc current Remote Control values
* return:
*  length of the message in bytes (excluding serial stream start sign)
*******************************************************************************/
uint16_t mavlink_msg_rc_channels_construct_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, uint32_t time_boot_ms, double rc[20])
{
  // copy the values into the packet
  mavlink_rc_channels_t packet;
  packet.time_boot_ms = (uint32_t)time_boot_ms;
  packet.chan1_raw    = (uint16_t)rc[0];
  packet.chan2_raw    = (uint16_t)rc[1];
  packet.chan3_raw    = (uint16_t)rc[2];
  packet.chan4_raw    = (uint16_t)rc[3];
  packet.chan5_raw    = (uint16_t)rc[4];
  packet.chan6_raw    = (uint16_t)rc[5];
  packet.chan7_raw    = (uint16_t)rc[6];
  packet.chan8_raw    = (uint16_t)rc[7];
  packet.chan9_raw    = (uint16_t)rc[8];
  packet.chan10_raw   = (uint16_t)rc[9];
  packet.chan11_raw   = (uint16_t)rc[10];
  packet.chan12_raw   = (uint16_t)rc[11];
  packet.chan13_raw   = (uint16_t)rc[12];
  packet.chan14_raw   = (uint16_t)rc[13];
  packet.chan15_raw   = (uint16_t)rc[14];
  packet.chan16_raw   = (uint16_t)rc[15];
  packet.chan17_raw   = (uint16_t)rc[16];
  packet.chan18_raw   = (uint16_t)rc[17];
  packet.chancount    = (uint8_t)rc[18];
  packet.rssi         = (uint8_t)rc[19];

  // copy the packet into the payload of the MAVLink message
  memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RC_CHANNELS_LEN);
  // Specify the type of MAVlink message
  msg->msgid = MAVLINK_MSG_ID_RC_CHANNELS;

  // Calculate checksum and set length and aircraft id.
  return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RC_CHANNELS_MIN_LEN, MAVLINK_MSG_ID_RC_CHANNELS_LEN, MAVLINK_MSG_ID_RC_CHANNELS_CRC);
}

/******************************************************************************
* Copied from mavlink_helpers.h
* Internal function to give access to the channel status for each channel
******************************************************************************/
mavlink_status_t* mavlink_get_channel_status(uint8_t chan)
{
#ifdef MAVLINK_EXTERNAL_RX_STATUS
  // No m_mavlink_status array defined in function,
  // has to be defined externally
#else
  static mavlink_status_t m_mavlink_status[MAVLINK_COMM_NUM_BUFFERS];
#endif
  return &m_mavlink_status[chan];
}

/******************************************************************************
* Copied from mavlink_helpers.h
* Internal function to give access to the channel buffer for each channel
******************************************************************************/
mavlink_message_t* mavlink_get_channel_buffer(uint8_t chan)
{
#ifdef MAVLINK_EXTERNAL_RX_BUFFER
  // No m_mavlink_buffer array defined in function,
  // has to be defined externally
#else
  static mavlink_message_t m_mavlink_buffer[MAVLINK_COMM_NUM_BUFFERS];
#endif
  return &m_mavlink_buffer[chan];
}

/******************************************************************************
* Copied from mavlink_helpers.h
* create a signature block for a packet
******************************************************************************/
uint8_t mavlink_sign_packet(mavlink_signing_t *signing, uint8_t signature[MAVLINK_SIGNATURE_BLOCK_LEN], const uint8_t *header, uint8_t header_len, const uint8_t *packet, uint8_t packet_len, const uint8_t crc[2])
{
  mavlink_sha256_ctx ctx;
  union {
    uint64_t t64;
    uint8_t t8[8];
  } tstamp;
  if (signing == NULL || !(signing->flags & MAVLINK_SIGNING_FLAG_SIGN_OUTGOING)) {
    return 0;
  }
  signature[0] = signing->link_id;
  tstamp.t64 = signing->timestamp;
  memcpy(&signature[1], tstamp.t8, 6);
  signing->timestamp++;

  mavlink_sha256_init(&ctx);
  mavlink_sha256_update(&ctx, signing->secret_key, sizeof(signing->secret_key));
  mavlink_sha256_update(&ctx, header, header_len);
  mavlink_sha256_update(&ctx, packet, packet_len);
  mavlink_sha256_update(&ctx, crc, 2);
  mavlink_sha256_update(&ctx, signature, 7);
  mavlink_sha256_final_48(&ctx, &signature[7]);

  return MAVLINK_SIGNATURE_BLOCK_LEN;
}

/******************************************************************************
* Copied from mavlink_helpers.h
* return new packet length for trimming payload of any trailing zero
* bytes. Used in MAVLink2 to give simple support for variable length
* arrays.
******************************************************************************/
uint8_t _mav_trim_payload(const char *payload, uint8_t length)
{
  while (length > 1 && payload[length - 1] == 0) {
    length--;
  }
  return length;
}

/******************************************************************************
* Copied from mavlink_helpers.h
* check a signature block for a packet
******************************************************************************/
bool mavlink_signature_check(mavlink_signing_t *signing, mavlink_signing_streams_t *signing_streams, const mavlink_message_t *msg)
{
  if (signing == NULL) {
    return true;
  }
  const uint8_t *p = (const uint8_t *)&msg->magic;
  const uint8_t *psig = msg->signature;
  const uint8_t *incoming_signature = psig + 7;
  mavlink_sha256_ctx ctx;
  uint8_t signature[6];
  uint16_t i;

  mavlink_sha256_init(&ctx);
  mavlink_sha256_update(&ctx, signing->secret_key, sizeof(signing->secret_key));
  mavlink_sha256_update(&ctx, p, MAVLINK_CORE_HEADER_LEN + 1 + msg->len);
  mavlink_sha256_update(&ctx, msg->ck, 2);
  mavlink_sha256_update(&ctx, psig, 1 + 6);
  mavlink_sha256_final_48(&ctx, signature);
  if (memcmp(signature, incoming_signature, 6) != 0) {
    return false;
  }

  // now check timestamp
  union tstamp {
    uint64_t t64;
    uint8_t t8[8];
  } tstamp;
  uint8_t link_id = psig[0];
  tstamp.t64 = 0;
  memcpy(tstamp.t8, psig + 1, 6);

  if (signing_streams == NULL) {
    return false;
  }

  // find stream
  for (i = 0; i<signing_streams->num_signing_streams; i++) {
    if (msg->sysid == signing_streams->stream[i].sysid &&
      msg->compid == signing_streams->stream[i].compid &&
      link_id == signing_streams->stream[i].link_id) {
      break;
    }
  }
  if (i == signing_streams->num_signing_streams) {
    if (signing_streams->num_signing_streams >= MAVLINK_MAX_SIGNING_STREAMS) {
      // over max number of streams
      return false;
    }
    // new stream. Only accept if timestamp is not more than 1 minute old
    if (tstamp.t64 + 6000 * 1000UL < signing->timestamp) {
      return false;
    }
    // add new stream
    signing_streams->stream[i].sysid = msg->sysid;
    signing_streams->stream[i].compid = msg->compid;
    signing_streams->stream[i].link_id = link_id;
    signing_streams->num_signing_streams++;
  }
  else {
    union tstamp last_tstamp;
    last_tstamp.t64 = 0;
    memcpy(last_tstamp.t8, signing_streams->stream[i].timestamp_bytes, 6);
    if (tstamp.t64 <= last_tstamp.t64) {
      // repeating old timestamp
      return false;
    }
  }

  // remember last timestamp
  memcpy(signing_streams->stream[i].timestamp_bytes, psig + 1, 6);

  // our next timestamp must be at least this timestamp
  if (tstamp.t64 > signing->timestamp) {
    signing->timestamp = tstamp.t64;
  }
  return true;
}

/******************************************************************************
* Copied from mavlink_helpers.h
* Finalize a MAVLink message with channel assignment
*
* This function calculates the checksum and sets length and aircraft id correctly.
* It assumes that the message id and the payload are already correctly set. This function
* can also be used if the message header has already been written before (as in mavlink_msg_xxx_pack
* instead of mavlink_msg_xxx_pack_headerless), it just introduces little extra overhead.
*
* msg Message to finalize
* system_id Id of the sending (this) system, 1-127
* length Message length
******************************************************************************/
uint16_t mavlink_finalize_message_buffer(mavlink_message_t* msg, uint8_t system_id, uint8_t component_id,
  mavlink_status_t* status, uint8_t min_length, uint8_t length, uint8_t crc_extra)
{
  bool mavlink1 = (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) != 0;
  bool signing = (!mavlink1) && status->signing && (status->signing->flags & MAVLINK_SIGNING_FLAG_SIGN_OUTGOING);
  uint8_t signature_len = signing ? MAVLINK_SIGNATURE_BLOCK_LEN : 0;
  uint8_t header_len = MAVLINK_CORE_HEADER_LEN + 1;
  uint8_t buf[MAVLINK_CORE_HEADER_LEN + 1];
  if (mavlink1) {
    msg->magic = MAVLINK_STX_MAVLINK1;
    header_len = MAVLINK_CORE_HEADER_MAVLINK1_LEN + 1;
  }
  else {
    msg->magic = MAVLINK_STX;
  }
  msg->len = mavlink1 ? min_length : _mav_trim_payload(_MAV_PAYLOAD(msg), length);
  msg->sysid = system_id;
  msg->compid = component_id;
  msg->incompat_flags = 0;
  if (signing) {
    msg->incompat_flags |= MAVLINK_IFLAG_SIGNED;
  }
  msg->compat_flags = 0;
  msg->seq = status->current_tx_seq;
  status->current_tx_seq = status->current_tx_seq + 1;

  // form the header as a byte array for the crc
  buf[0] = msg->magic;
  buf[1] = msg->len;
  if (mavlink1) {
    buf[2] = msg->seq;
    buf[3] = msg->sysid;
    buf[4] = msg->compid;
    buf[5] = msg->msgid & 0xFF;
  }
  else {
    buf[2] = msg->incompat_flags;
    buf[3] = msg->compat_flags;
    buf[4] = msg->seq;
    buf[5] = msg->sysid;
    buf[6] = msg->compid;
    buf[7] = msg->msgid & 0xFF;
    buf[8] = (msg->msgid >> 8) & 0xFF;
    buf[9] = (msg->msgid >> 16) & 0xFF;
  }

  msg->checksum = crc_calculate(&buf[1], header_len - 1);
  crc_accumulate_buffer(&msg->checksum, _MAV_PAYLOAD(msg), msg->len);
  crc_accumulate(crc_extra, &msg->checksum);
  mavlink_ck_a(msg) = (uint8_t)(msg->checksum & 0xFF);
  mavlink_ck_b(msg) = (uint8_t)(msg->checksum >> 8);

  if (signing) {
    mavlink_sign_packet(status->signing,
      msg->signature,
      (const uint8_t *)buf, header_len,
      (const uint8_t *)_MAV_PAYLOAD(msg), msg->len,
      (const uint8_t *)_MAV_PAYLOAD(msg) + (uint16_t)msg->len);
  }

  return msg->len + header_len + 2 + signature_len;
}

/******************************************************************************
* Copied from mavlink_helpers.h
******************************************************************************/
void _mav_parse_error(mavlink_status_t *status)
{
  status->parse_error++;
}

/******************************************************************************
* Copied from mavlink_helpers.h
* Finalize a MAVLink message with MAVLINK_COMM_0 as default channel
******************************************************************************/
uint16_t mavlink_finalize_message_chan(mavlink_message_t* msg, uint8_t system_id, uint8_t component_id,
  uint8_t chan, uint8_t min_length, uint8_t length, uint8_t crc_extra)
{
  mavlink_status_t *status = mavlink_get_channel_status(chan);
  return mavlink_finalize_message_buffer(msg, system_id, component_id, status, min_length, length, crc_extra);
}

/******************************************************************************
* Copied from mavlink_helpers.h
* Pack a message to send it over a serial byte stream
******************************************************************************/
uint16_t mavlink_msg_to_send_buffer(uint8_t *buf, const mavlink_message_t *msg)
{
  uint8_t signature_len, header_len;
  uint8_t *ck;
  uint8_t length = msg->len;

  if (msg->magic == MAVLINK_STX_MAVLINK1) {
    signature_len = 0;
    header_len = MAVLINK_CORE_HEADER_MAVLINK1_LEN;
    buf[0] = msg->magic;
    buf[1] = length;
    buf[2] = msg->seq;
    buf[3] = msg->sysid;
    buf[4] = msg->compid;
    buf[5] = msg->msgid & 0xFF;
    memcpy(&buf[6], _MAV_PAYLOAD(msg), msg->len);
    ck = buf + header_len + 1 + (uint16_t)msg->len;
  }
  else {
    length = _mav_trim_payload(_MAV_PAYLOAD(msg), length);
    header_len = MAVLINK_CORE_HEADER_LEN;
    buf[0] = msg->magic;
    buf[1] = length;
    buf[2] = msg->incompat_flags;
    buf[3] = msg->compat_flags;
    buf[4] = msg->seq;
    buf[5] = msg->sysid;
    buf[6] = msg->compid;
    buf[7] = msg->msgid & 0xFF;
    buf[8] = (msg->msgid >> 8) & 0xFF;
    buf[9] = (msg->msgid >> 16) & 0xFF;
    memcpy(&buf[10], _MAV_PAYLOAD(msg), length);
    ck = buf + header_len + 1 + (uint16_t)length;
    signature_len = (msg->incompat_flags & MAVLINK_IFLAG_SIGNED) ? MAVLINK_SIGNATURE_BLOCK_LEN : 0;
  }
  ck[0] = (uint8_t)(msg->checksum & 0xFF);
  ck[1] = (uint8_t)(msg->checksum >> 8);
  if (signature_len > 0) {
    memcpy(&ck[2], msg->signature, signature_len);
  }

  return header_len + 1 + 2 + (uint16_t)length + (uint16_t)signature_len;
}

/******************************************************************************
* Copied from mavlink_helpers.h
******************************************************************************/
void mavlink_start_checksum(mavlink_message_t* msg)
{
  crc_init(&msg->checksum);
}

/******************************************************************************
* Copied from mavlink_helpers.h
******************************************************************************/
void mavlink_update_checksum(mavlink_message_t* msg, uint8_t c)
{
  crc_accumulate(c, &msg->checksum);
}

/******************************************************************************
* Copied from mavlink_helpers.h
* return the crc_entry value for a msgid
******************************************************************************/
const mavlink_msg_entry_t* mavlink_get_msg_entry(uint32_t msgid)
{
  static const mavlink_msg_entry_t mavlink_message_crcs[] = { { 0, 50, 9, 0, 0, 0 },{ 1, 124, 31, 0, 0, 0 },{ 2, 137, 12, 0, 0, 0 },{ 4, 237, 14, 3, 12, 13 },{ 5, 217, 28, 1, 0, 0 },{ 6, 104, 3, 0, 0, 0 },{ 7, 119, 32, 0, 0, 0 },{ 11, 89, 6, 1, 4, 0 },{ 20, 214, 20, 3, 2, 3 },{ 21, 159, 2, 3, 0, 1 },{ 22, 220, 25, 0, 0, 0 },{ 23, 168, 23, 3, 4, 5 },{ 24, 24, 30, 0, 0, 0 },{ 25, 23, 101, 0, 0, 0 },{ 26, 170, 22, 0, 0, 0 },{ 27, 144, 26, 0, 0, 0 },{ 28, 67, 16, 0, 0, 0 },{ 29, 115, 14, 0, 0, 0 },{ 30, 39, 28, 0, 0, 0 },{ 31, 246, 32, 0, 0, 0 },{ 32, 185, 28, 0, 0, 0 },{ 33, 104, 28, 0, 0, 0 },{ 34, 237, 22, 0, 0, 0 },{ 35, 244, 22, 0, 0, 0 },{ 36, 222, 21, 0, 0, 0 },{ 37, 212, 6, 3, 4, 5 },{ 38, 9, 6, 3, 4, 5 },{ 39, 254, 37, 3, 32, 33 },{ 40, 230, 4, 3, 2, 3 },{ 41, 28, 4, 3, 2, 3 },{ 42, 28, 2, 0, 0, 0 },{ 43, 132, 2, 3, 0, 1 },{ 44, 221, 4, 3, 2, 3 },{ 45, 232, 2, 3, 0, 1 },{ 46, 11, 2, 0, 0, 0 },{ 47, 153, 3, 3, 0, 1 },{ 48, 41, 13, 1, 12, 0 },{ 49, 39, 12, 0, 0, 0 },{ 50, 78, 37, 3, 18, 19 },{ 51, 196, 4, 3, 2, 3 },{ 54, 15, 27, 3, 24, 25 },{ 55, 3, 25, 0, 0, 0 },{ 61, 167, 72, 0, 0, 0 },{ 62, 183, 26, 0, 0, 0 },{ 63, 119, 181, 0, 0, 0 },{ 64, 191, 225, 0, 0, 0 },{ 65, 118, 42, 0, 0, 0 },{ 66, 148, 6, 3, 2, 3 },{ 67, 21, 4, 0, 0, 0 },{ 69, 243, 11, 0, 0, 0 },{ 70, 124, 18, 3, 16, 17 },{ 73, 38, 37, 3, 32, 33 },{ 74, 20, 20, 0, 0, 0 },{ 75, 158, 35, 3, 30, 31 },{ 76, 152, 33, 3, 30, 31 },{ 77, 143, 3, 0, 0, 0 },{ 81, 106, 22, 0, 0, 0 },{ 82, 49, 39, 3, 36, 37 },{ 83, 22, 37, 0, 0, 0 },{ 84, 143, 53, 3, 50, 51 },{ 85, 140, 51, 0, 0, 0 },{ 86, 5, 53, 3, 50, 51 },{ 87, 150, 51, 0, 0, 0 },{ 89, 231, 28, 0, 0, 0 },{ 90, 183, 56, 0, 0, 0 },{ 91, 63, 42, 0, 0, 0 },{ 92, 54, 33, 0, 0, 0 },{ 93, 47, 81, 0, 0, 0 },{ 100, 175, 26, 0, 0, 0 },{ 101, 102, 32, 0, 0, 0 },{ 102, 158, 32, 0, 0, 0 },{ 103, 208, 20, 0, 0, 0 },{ 104, 56, 32, 0, 0, 0 },{ 105, 93, 62, 0, 0, 0 },{ 106, 138, 44, 0, 0, 0 },{ 107, 108, 64, 0, 0, 0 },{ 108, 32, 84, 0, 0, 0 },{ 109, 185, 9, 0, 0, 0 },{ 110, 84, 254, 3, 1, 2 },{ 111, 34, 16, 0, 0, 0 },{ 112, 174, 12, 0, 0, 0 },{ 113, 124, 36, 0, 0, 0 },{ 114, 237, 44, 0, 0, 0 },{ 115, 4, 64, 0, 0, 0 },{ 116, 76, 22, 0, 0, 0 },{ 117, 128, 6, 3, 4, 5 },{ 118, 56, 14, 0, 0, 0 },{ 119, 116, 12, 3, 10, 11 },{ 120, 134, 97, 0, 0, 0 },{ 121, 237, 2, 3, 0, 1 },{ 122, 203, 2, 3, 0, 1 },{ 123, 250, 113, 3, 0, 1 },{ 124, 87, 35, 0, 0, 0 },{ 125, 203, 6, 0, 0, 0 },{ 126, 220, 79, 0, 0, 0 },{ 127, 25, 35, 0, 0, 0 },{ 128, 226, 35, 0, 0, 0 },{ 129, 46, 22, 0, 0, 0 },{ 130, 29, 13, 0, 0, 0 },{ 131, 223, 255, 0, 0, 0 },{ 132, 85, 14, 0, 0, 0 },{ 133, 6, 18, 0, 0, 0 },{ 134, 229, 43, 0, 0, 0 },{ 135, 203, 8, 0, 0, 0 },{ 136, 1, 22, 0, 0, 0 },{ 137, 195, 14, 0, 0, 0 },{ 138, 109, 36, 0, 0, 0 },{ 139, 168, 43, 3, 41, 42 },{ 140, 181, 41, 0, 0, 0 },{ 141, 47, 32, 0, 0, 0 },{ 142, 72, 243, 0, 0, 0 },{ 143, 131, 14, 0, 0, 0 },{ 144, 127, 93, 0, 0, 0 },{ 146, 103, 100, 0, 0, 0 },{ 147, 154, 36, 0, 0, 0 },{ 148, 178, 60, 0, 0, 0 },{ 149, 200, 30, 0, 0, 0 },{ 230, 163, 42, 0, 0, 0 },{ 231, 105, 40, 0, 0, 0 },{ 232, 151, 63, 0, 0, 0 },{ 233, 35, 182, 0, 0, 0 },{ 234, 150, 40, 0, 0, 0 },{ 241, 90, 32, 0, 0, 0 },{ 242, 104, 52, 0, 0, 0 },{ 243, 85, 53, 1, 52, 0 },{ 244, 95, 6, 0, 0, 0 },{ 245, 130, 2, 0, 0, 0 },{ 246, 184, 38, 0, 0, 0 },{ 247, 81, 19, 0, 0, 0 },{ 248, 8, 254, 3, 3, 4 },{ 249, 204, 36, 0, 0, 0 },{ 250, 49, 30, 0, 0, 0 },{ 251, 170, 18, 0, 0, 0 },{ 252, 44, 18, 0, 0, 0 },{ 253, 83, 51, 0, 0, 0 },{ 254, 46, 9, 0, 0, 0 },{ 256, 71, 42, 3, 8, 9 },{ 257, 131, 9, 0, 0, 0 },{ 258, 187, 32, 3, 0, 1 },{ 259, 92, 235, 0, 0, 0 },{ 260, 146, 5, 0, 0, 0 },{ 261, 179, 27, 0, 0, 0 },{ 262, 12, 18, 0, 0, 0 },{ 263, 133, 255, 0, 0, 0 },{ 264, 49, 28, 0, 0, 0 },{ 265, 26, 16, 0, 0, 0 },{ 266, 193, 255, 3, 2, 3 },{ 267, 35, 255, 3, 2, 3 },{ 268, 14, 4, 3, 2, 3 },{ 269, 58, 246, 0, 0, 0 },{ 270, 232, 247, 3, 14, 15 },{ 299, 19, 96, 0, 0, 0 },{ 300, 217, 22, 0, 0, 0 },{ 310, 28, 17, 0, 0, 0 },{ 311, 95, 116, 0, 0, 0 },{ 320, 243, 20, 3, 2, 3 },{ 321, 88, 2, 3, 0, 1 },{ 322, 243, 149, 0, 0, 0 },{ 323, 78, 147, 3, 0, 1 },{ 324, 132, 146, 0, 0, 0 } };
  /*
  use a bisection search to find the right entry. A perfect hash may be better
  Note that this assumes the table is sorted by msgid
  */
  uint32_t low = 0, high = sizeof(mavlink_message_crcs) / sizeof(mavlink_message_crcs[0]);
  while (low < high) {
    uint32_t mid = (low + 1 + high) / 2;
    if (msgid < mavlink_message_crcs[mid].msgid) {
      high = mid - 1;
      continue;
    }
    if (msgid > mavlink_message_crcs[mid].msgid) {
      low = mid;
      continue;
    }
    low = mid;
    break;
  }
  if (mavlink_message_crcs[low].msgid != msgid) {
    // msgid is not in the table
    return NULL;
  }
  return &mavlink_message_crcs[low];
}

/******************************************************************************
* Copied from mavlink_helpers.h
* This is a varient of mavlink_frame_char() but with caller supplied
* parsing buffers. It is useful when you want to create a MAVLink
* parser in a library that doesn't use any global variables
*
* rxmsg    parsing message buffer
* status   parsing starus buffer
* c        The char to parse
*
* returnMsg NULL if no message could be decoded, the message data else
* returnStats if a message was decoded, this is filled with the channel's stats
* return: 0 if no message could be decoded, 1 on good message and CRC, 2 on bad CRC
******************************************************************************/
uint8_t mavlink_frame_char_buffer(mavlink_message_t* rxmsg, mavlink_status_t* status, uint8_t c, mavlink_message_t* r_message, mavlink_status_t* r_mavlink_status)
{
  /* Enable this option to check the length of each message.
  This allows invalid messages to be caught much sooner. Use if the transmission
  medium is prone to missing (or extra) characters (e.g. a radio that fades in
  and out). Only use if the channel will only contain messages types listed in
  the headers.
  */
#ifdef MAVLINK_CHECK_MESSAGE_LENGTH
#ifndef MAVLINK_MESSAGE_LENGTH
  static const uint8_t mavlink_message_lengths[256] = MAVLINK_MESSAGE_LENGTHS;
#define MAVLINK_MESSAGE_LENGTH(msgid) mavlink_message_lengths[msgid]
#endif
#endif

  int bufferIndex = 0;

  status->msg_received = MAVLINK_FRAMING_INCOMPLETE;

  switch (status->parse_state)
  {
  case MAVLINK_PARSE_STATE_UNINIT:
  case MAVLINK_PARSE_STATE_IDLE:
    if (c == MAVLINK_STX)
    {
      status->parse_state = MAVLINK_PARSE_STATE_GOT_STX;
      rxmsg->len = 0;
      rxmsg->magic = c;
      status->flags &= ~MAVLINK_STATUS_FLAG_IN_MAVLINK1;
      mavlink_start_checksum(rxmsg);
    }
    else if (c == MAVLINK_STX_MAVLINK1)
    {
      status->parse_state = MAVLINK_PARSE_STATE_GOT_STX;
      rxmsg->len = 0;
      rxmsg->magic = c;
      status->flags |= MAVLINK_STATUS_FLAG_IN_MAVLINK1;
      mavlink_start_checksum(rxmsg);
    }
    break;

  case MAVLINK_PARSE_STATE_GOT_STX:
    if (status->msg_received
      /* Support shorter buffers than the
      default maximum packet size */
#if (MAVLINK_MAX_PAYLOAD_LEN < 255)
      || c > MAVLINK_MAX_PAYLOAD_LEN
#endif
      )
    {
      status->buffer_overrun++;
      _mav_parse_error(status);
      status->msg_received = 0;
      status->parse_state = MAVLINK_PARSE_STATE_IDLE;
    }
    else
    {
      // NOT counting STX, LENGTH, SEQ, SYSID, COMPID, MSGID, CRC1 and CRC2
      rxmsg->len = c;
      status->packet_idx = 0;
      mavlink_update_checksum(rxmsg, c);
      if (status->flags & MAVLINK_STATUS_FLAG_IN_MAVLINK1) {
        rxmsg->incompat_flags = 0;
        rxmsg->compat_flags = 0;
        status->parse_state = MAVLINK_PARSE_STATE_GOT_COMPAT_FLAGS;
      }
      else {
        status->parse_state = MAVLINK_PARSE_STATE_GOT_LENGTH;
      }
    }
    break;

  case MAVLINK_PARSE_STATE_GOT_LENGTH:
    rxmsg->incompat_flags = c;
    if ((rxmsg->incompat_flags & ~MAVLINK_IFLAG_MASK) != 0) {
      // message includes an incompatible feature flag
      _mav_parse_error(status);
      status->msg_received = 0;
      status->parse_state = MAVLINK_PARSE_STATE_IDLE;
      break;
    }
    mavlink_update_checksum(rxmsg, c);
    status->parse_state = MAVLINK_PARSE_STATE_GOT_INCOMPAT_FLAGS;
    break;

  case MAVLINK_PARSE_STATE_GOT_INCOMPAT_FLAGS:
    rxmsg->compat_flags = c;
    mavlink_update_checksum(rxmsg, c);
    status->parse_state = MAVLINK_PARSE_STATE_GOT_COMPAT_FLAGS;
    break;

  case MAVLINK_PARSE_STATE_GOT_COMPAT_FLAGS:
    rxmsg->seq = c;
    mavlink_update_checksum(rxmsg, c);
    status->parse_state = MAVLINK_PARSE_STATE_GOT_SEQ;
    break;

  case MAVLINK_PARSE_STATE_GOT_SEQ:
    rxmsg->sysid = c;
    mavlink_update_checksum(rxmsg, c);
    status->parse_state = MAVLINK_PARSE_STATE_GOT_SYSID;
    break;

  case MAVLINK_PARSE_STATE_GOT_SYSID:
    rxmsg->compid = c;
    mavlink_update_checksum(rxmsg, c);
    status->parse_state = MAVLINK_PARSE_STATE_GOT_COMPID;
    break;

  case MAVLINK_PARSE_STATE_GOT_COMPID:
    rxmsg->msgid = c;
    mavlink_update_checksum(rxmsg, c);
    if (status->flags & MAVLINK_STATUS_FLAG_IN_MAVLINK1) {
      status->parse_state = MAVLINK_PARSE_STATE_GOT_MSGID3;
#ifdef MAVLINK_CHECK_MESSAGE_LENGTH
      if (rxmsg->len != MAVLINK_MESSAGE_LENGTH(rxmsg->msgid))
      {
        _mav_parse_error(status);
        status->parse_state = MAVLINK_PARSE_STATE_IDLE;
        break;
      }
#endif
    }
    else {
      status->parse_state = MAVLINK_PARSE_STATE_GOT_MSGID1;
    }
    break;

  case MAVLINK_PARSE_STATE_GOT_MSGID1:
    rxmsg->msgid |= c << 8;
    mavlink_update_checksum(rxmsg, c);
    status->parse_state = MAVLINK_PARSE_STATE_GOT_MSGID2;
    break;

  case MAVLINK_PARSE_STATE_GOT_MSGID2:
    rxmsg->msgid |= c << 16;
    mavlink_update_checksum(rxmsg, c);
    status->parse_state = MAVLINK_PARSE_STATE_GOT_MSGID3;
#ifdef MAVLINK_CHECK_MESSAGE_LENGTH
    if (rxmsg->len != MAVLINK_MESSAGE_LENGTH(rxmsg->msgid))
    {
      _mav_parse_error(status);
      status->parse_state = MAVLINK_PARSE_STATE_IDLE;
      break;
    }
#endif
    break;

  case MAVLINK_PARSE_STATE_GOT_MSGID3:
    _MAV_PAYLOAD_NON_CONST(rxmsg)[status->packet_idx++] = (char)c;
    mavlink_update_checksum(rxmsg, c);
    if (status->packet_idx == rxmsg->len)
    {
      status->parse_state = MAVLINK_PARSE_STATE_GOT_PAYLOAD;
    }
    break;

  case MAVLINK_PARSE_STATE_GOT_PAYLOAD: {
    const mavlink_msg_entry_t *e = mavlink_get_msg_entry(rxmsg->msgid);
    uint8_t crc_extra = e ? e->crc_extra : 0;
    mavlink_update_checksum(rxmsg, crc_extra);
    if (c != (rxmsg->checksum & 0xFF)) {
      status->parse_state = MAVLINK_PARSE_STATE_GOT_BAD_CRC1;
    }
    else {
      status->parse_state = MAVLINK_PARSE_STATE_GOT_CRC1;
    }
    rxmsg->ck[0] = c;

    // zero-fill the packet to cope with short incoming packets
    if (e && status->packet_idx < e->msg_len) {
      memset(&_MAV_PAYLOAD_NON_CONST(rxmsg)[status->packet_idx], 0, e->msg_len - status->packet_idx);
    }
    break;
  }

  case MAVLINK_PARSE_STATE_GOT_CRC1:
  case MAVLINK_PARSE_STATE_GOT_BAD_CRC1:
    if (status->parse_state == MAVLINK_PARSE_STATE_GOT_BAD_CRC1 || c != (rxmsg->checksum >> 8)) {
      // got a bad CRC message
      status->msg_received = MAVLINK_FRAMING_BAD_CRC;
    }
    else {
      // Successfully got message
      status->msg_received = MAVLINK_FRAMING_OK;
    }
    rxmsg->ck[1] = c;

    if (rxmsg->incompat_flags & MAVLINK_IFLAG_SIGNED) {
      status->parse_state = MAVLINK_PARSE_STATE_SIGNATURE_WAIT;
      status->signature_wait = MAVLINK_SIGNATURE_BLOCK_LEN;

      // If the CRC is already wrong, don't overwrite msg_received,
      // otherwise we can end up with garbage flagged as valid.
      if (status->msg_received != MAVLINK_FRAMING_BAD_CRC) {
        status->msg_received = MAVLINK_FRAMING_INCOMPLETE;
      }
    }
    else {
      if (status->signing &&
        (status->signing->accept_unsigned_callback == NULL ||
          !status->signing->accept_unsigned_callback(status, rxmsg->msgid))) {

        // If the CRC is already wrong, don't overwrite msg_received.
        if (status->msg_received != MAVLINK_FRAMING_BAD_CRC) {
          status->msg_received = MAVLINK_FRAMING_BAD_SIGNATURE;
        }
      }
      status->parse_state = MAVLINK_PARSE_STATE_IDLE;
      memcpy(r_message, rxmsg, sizeof(mavlink_message_t));
    }
    break;
  case MAVLINK_PARSE_STATE_SIGNATURE_WAIT:
    rxmsg->signature[MAVLINK_SIGNATURE_BLOCK_LEN - status->signature_wait] = c;
    status->signature_wait--;
    if (status->signature_wait == 0) {
      // we have the whole signature, check it is OK
      bool sig_ok = mavlink_signature_check(status->signing, status->signing_streams, rxmsg);
      if (!sig_ok &&
        (status->signing->accept_unsigned_callback &&
          status->signing->accept_unsigned_callback(status, rxmsg->msgid))) {
        // accepted via application level override
        sig_ok = true;
      }
      if (sig_ok) {
        status->msg_received = MAVLINK_FRAMING_OK;
      }
      else {
        status->msg_received = MAVLINK_FRAMING_BAD_SIGNATURE;
      }
      status->parse_state = MAVLINK_PARSE_STATE_IDLE;
      memcpy(r_message, rxmsg, sizeof(mavlink_message_t));
    }
    break;
  }

  bufferIndex++;
  // If a message has been sucessfully decoded, check index
  if (status->msg_received == MAVLINK_FRAMING_OK)
  {
    //while(status->current_seq != rxmsg->seq)
    //{
    //  status->packet_rx_drop_count++;
    //               status->current_seq++;
    //}
    status->current_rx_seq = rxmsg->seq;
    // Initial condition: If no packet has been received so far, drop count is undefined
    if (status->packet_rx_success_count == 0) status->packet_rx_drop_count = 0;
    // Count this packet as received
    status->packet_rx_success_count++;
  }

  r_message->len = rxmsg->len; // Provide visibility on how far we are into current msg
  r_mavlink_status->parse_state = status->parse_state;
  r_mavlink_status->packet_idx = status->packet_idx;
  r_mavlink_status->current_rx_seq = status->current_rx_seq + 1;
  r_mavlink_status->packet_rx_success_count = status->packet_rx_success_count;
  r_mavlink_status->packet_rx_drop_count = status->parse_error;
  r_mavlink_status->flags = status->flags;
  status->parse_error = 0;

  if (status->msg_received == MAVLINK_FRAMING_BAD_CRC) {
    /*
    the CRC came out wrong. We now need to overwrite the
    msg CRC with the one on the wire so that if the
    caller decides to forward the message anyway that
    mavlink_msg_to_send_buffer() won't overwrite the
    checksum
    */
    r_message->checksum = rxmsg->ck[0] | (rxmsg->ck[1] << 8);
  }

  return status->msg_received;
}

/******************************************************************************
* Copied from mavlink_helpers.h
* This is a convenience function which handles the complete MAVLink parsing.
* the function will parse one byte at a time and return the complete packet once
* it could be successfully decoded. This function will return 0, 1 or
* 2 (MAVLINK_FRAMING_INCOMPLETE, MAVLINK_FRAMING_OK or MAVLINK_FRAMING_BAD_CRC)
*
* Messages are parsed into an internal buffer (one for each channel). When a complete
* message is received it is copies into *returnMsg and the channel's status is
* copied into *returnStats.
*
* chan     ID of the current channel. This allows to parse different channels with this function.
*          a channel is not a physical message channel like a serial port, but a logic partition of
*          the communication streams in this case. COMM_NB is the limit for the number of channels
*          on MCU (e.g. ARM7), while COMM_NB_HIGH is the limit for the number of channels in Linux/Windows
* c        The char to parse
*
* returnMsg NULL if no message could be decoded, the message data else
* returnStats if a message was decoded, this is filled with the channel's stats
* return: 0 if no message could be decoded, 1 on good message and CRC, 2 on bad CRC
*
******************************************************************************/
uint8_t mavlink_frame_char(uint8_t chan, uint8_t c, mavlink_message_t* r_message, mavlink_status_t* r_mavlink_status)
{
  return mavlink_frame_char_buffer(mavlink_get_channel_buffer(chan),
    mavlink_get_channel_status(chan),
    c,
    r_message,
    r_mavlink_status);
}

/******************************************************************************
* Copied from mavlink_helpers.h
* This is a convenience function which handles the complete MAVLink parsing.
* the function will parse one byte at a time and return the complete packet once
* it could be successfully decoded. This function will return 0 or 1.
*
* Messages are parsed into an internal buffer (one for each channel). When a complete
* message is received it is copies into *returnMsg and the channel's status is
* copied into *returnStats.
*
* chan     ID of the current channel. This allows to parse different channels with this function.
*          a channel is not a physical message channel like a serial port, but a logic partition of
*          the communication streams in this case. COMM_NB is the limit for the number of channels
*          on MCU (e.g. ARM7), while COMM_NB_HIGH is the limit for the number of channels in Linux/Windows
* c        The char to parse
*
* returnMsg NULL if no message could be decoded, the message data else
* returnStats if a message was decoded, this is filled with the channel's stats
* return: 0 if no message could be decoded or bad CRC, 1 on good message and CRC
******************************************************************************/
uint8_t mavlink_parse_char(uint8_t chan, uint8_t c, mavlink_message_t* r_message, mavlink_status_t* r_mavlink_status)
{
  uint8_t msg_received = mavlink_frame_char(chan, c, r_message, r_mavlink_status);
  if (msg_received == MAVLINK_FRAMING_BAD_CRC ||
    msg_received == MAVLINK_FRAMING_BAD_SIGNATURE) {
    // we got a bad CRC. Treat as a parse failure
    mavlink_message_t* rxmsg = mavlink_get_channel_buffer(chan);
    mavlink_status_t* status = mavlink_get_channel_status(chan);
    _mav_parse_error(status);
    status->msg_received = MAVLINK_FRAMING_INCOMPLETE;
    status->parse_state = MAVLINK_PARSE_STATE_IDLE;
    if (c == MAVLINK_STX)
    {
      status->parse_state = MAVLINK_PARSE_STATE_GOT_STX;
      rxmsg->len = 0;
      mavlink_start_checksum(rxmsg);
    }
    return 0;
  }
  return msg_received;
}

/******************************************************************************
* Copied from mavlink_sha256.h
******************************************************************************/
void mavlink_sha256_init(mavlink_sha256_ctx *m)
{
  m->sz[0] = 0;
  m->sz[1] = 0;
  A = 0x6a09e667;
  B = 0xbb67ae85;
  C = 0x3c6ef372;
  D = 0xa54ff53a;
  E = 0x510e527f;
  F = 0x9b05688c;
  G = 0x1f83d9ab;
  H = 0x5be0cd19;
}

/******************************************************************************
* Copied from mavlink_sha256.h
******************************************************************************/
void mavlink_sha256_calc(mavlink_sha256_ctx *m, uint32_t *in)
{
  uint32_t AA, BB, CC, DD, EE, FF, GG, HH;
  uint32_t data[64];
  int i;

  AA = A;
  BB = B;
  CC = C;
  DD = D;
  EE = E;
  FF = F;
  GG = G;
  HH = H;

  for (i = 0; i < 16; ++i)
    data[i] = in[i];
  for (i = 16; i < 64; ++i)
    data[i] = sigma1(data[i - 2]) + data[i - 7] +
    sigma0(data[i - 15]) + data[i - 16];

  for (i = 0; i < 64; i++) {
    uint32_t T1, T2;

    T1 = HH + Sigma1(EE) + Ch(EE, FF, GG) + mavlink_sha256_constant_256[i] + data[i];
    T2 = Sigma0(AA) + Maj(AA, BB, CC);

    HH = GG;
    GG = FF;
    FF = EE;
    EE = DD + T1;
    DD = CC;
    CC = BB;
    BB = AA;
    AA = T1 + T2;
  }

  A += AA;
  B += BB;
  C += CC;
  D += DD;
  E += EE;
  F += FF;
  G += GG;
  H += HH;
}

/******************************************************************************
* Copied from mavlink_sha256.h
******************************************************************************/
void mavlink_sha256_update(mavlink_sha256_ctx *m, const void *v, uint32_t len)
{
    const unsigned char *p = (const unsigned char *)v;
    uint32_t old_sz = m->sz[0];
    uint32_t offset;

    m->sz[0] += len * 8;
    if (m->sz[0] < old_sz)
    ++m->sz[1];
    offset = (old_sz / 8) % 64;
    while(len > 0){
    uint32_t l = 64 - offset;
        if (len < l) {
            l = len;
        }
    memcpy(m->u.save_bytes + offset, p, l);
    offset += l;
    p += l;
    len -= l;
    if(offset == 64){
        int i;
        uint32_t current[16];
        const uint32_t *u = m->u.save_u32;
        for (i = 0; i < 16; i++){
                const uint8_t *p1 = (const uint8_t *)&u[i];
                uint8_t *p2 = (uint8_t *)&current[i];
                p2[0] = p1[3];
                p2[1] = p1[2];
                p2[2] = p1[1];
                p2[3] = p1[0];
        }
        mavlink_sha256_calc(m, current);
        offset = 0;
    }
    }
}

/******************************************************************************
* Copied from mavlink_sha256.h
******************************************************************************/
void mavlink_sha256_final_48(mavlink_sha256_ctx *m, uint8_t result[6])
{
  unsigned char zeros[72];
  unsigned offset = (m->sz[0] / 8) % 64;
  unsigned int dstart = (120 - offset - 1) % 64 + 1;
  uint8_t *p = (uint8_t *)&m->counter[0];

  *zeros = 0x80;
  memset(zeros + 1, 0, sizeof(zeros) - 1);
  zeros[dstart + 7] = (m->sz[0] >> 0) & 0xff;
  zeros[dstart + 6] = (m->sz[0] >> 8) & 0xff;
  zeros[dstart + 5] = (m->sz[0] >> 16) & 0xff;
  zeros[dstart + 4] = (m->sz[0] >> 24) & 0xff;
  zeros[dstart + 3] = (m->sz[1] >> 0) & 0xff;
  zeros[dstart + 2] = (m->sz[1] >> 8) & 0xff;
  zeros[dstart + 1] = (m->sz[1] >> 16) & 0xff;
  zeros[dstart + 0] = (m->sz[1] >> 24) & 0xff;

  mavlink_sha256_update(m, zeros, dstart + 8);

  // this ordering makes the result consistent with taking the first
  // 6 bytes of more conventional sha256 functions. It assumes
  // little-endian ordering of m->counter
  result[0] = p[3];
  result[1] = p[2];
  result[2] = p[1];
  result[3] = p[0];
  result[4] = p[7];
  result[5] = p[6];
}

/******************************************************************************
* Copied from checksum.h
* Accumulate the X.25 CRC by adding one char at a time.
*
* The checksum function adds the hash of one char at a time to the
* 16 bit checksum (uint16_t).
*
* data new char to hash
* crcAccum the already accumulated checksum
******************************************************************************/
void crc_accumulate(uint8_t data, uint16_t *crcAccum)
{
  /*Accumulate one byte of data into the CRC*/
  uint8_t tmp;

  tmp = data ^ (uint8_t)(*crcAccum & 0xff);
  tmp ^= (tmp << 4);
  *crcAccum = (*crcAccum >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
}

/******************************************************************************
* Copied from checksum.h
* Initiliaze the buffer for the X.25 CRC
*
* crcAccum the 16 bit X.25 CRC
******************************************************************************/
void crc_init(uint16_t* crcAccum)
{
  *crcAccum = X25_INIT_CRC;
}

/******************************************************************************
* Copied from checksum.h
* Calculates the X.25 checksum on a byte buffer
*
* pBuffer buffer containing the byte array to hash
* length  length of the byte array
* the checksum over the buffer bytes
******************************************************************************/
uint16_t crc_calculate(const uint8_t* pBuffer, uint16_t length)
{
  uint16_t crcTmp;
  crc_init(&crcTmp);
  while (length--) {
    crc_accumulate(*pBuffer++, &crcTmp);
  }
  return crcTmp;
}

/******************************************************************************
* Copied from checksum.h
* Accumulate the X.25 CRC by adding an array of bytes
*
* The checksum function adds the hash of one char at a time to the
* 16 bit checksum (uint16_t).
*
* data new bytes to hash
* crcAccum the already accumulated checksum
******************************************************************************/
void crc_accumulate_buffer(uint16_t *crcAccum, const char *pBuffer, uint16_t length)
{
  const uint8_t *p = (const uint8_t *)pBuffer;
  while (length--) {
    crc_accumulate(*p++, crcAccum);
  }
}

/******************************************************************************
* Copied from mavlink_msg_heartbeat.h
* Decode a heartbeat message into a struct
*
* msg The message to decode
* heartbeat C-struct to decode the message contents into
*******************************************************************************/
void mavlink_msg_heartbeat_decode(const mavlink_message_t* msg, mavlink_heartbeat_t* heartbeat)
{
  uint8_t len = msg->len < MAVLINK_MSG_ID_HEARTBEAT_LEN ? msg->len : MAVLINK_MSG_ID_HEARTBEAT_LEN;
  memset(heartbeat, 0, MAVLINK_MSG_ID_HEARTBEAT_LEN);
  memcpy(heartbeat, _MAV_PAYLOAD(msg), len);
}

/******************************************************************************
* Copied from mavlink_msg_hil_actuator_controls.h
* Decode a hil_actuator_controls message into a struct
*
* msg The message to decode
* hil_actuator_controls C-struct to decode the message contents into
*******************************************************************************/
void mavlink_msg_hil_actuator_controls_decode(const mavlink_message_t* msg, mavlink_hil_actuator_controls_t* hil_actuator_controls)
{
  uint8_t len = msg->len < MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_LEN ? msg->len : MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_LEN;
  memset(hil_actuator_controls, 0, MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_LEN);
  memcpy(hil_actuator_controls, _MAV_PAYLOAD(msg), len);
}

//------------------------------------------------------------------------------
// Communications functions
//------------------------------------------------------------------------------

// Communications function forward declarations
void handle_message(mavlink_message_t *msg, double actuators[17]);
void sendIMU(uint64_t time_usec, double imu[13], uint32_t fields_updated);
void sendGPS(uint64_t time_usec, double gps[12]);
void sendRC(uint32_t time_boot_ms, double rc[20]);
void send_mavlink_message(const mavlink_message_t *message, const int destination_port);

// Communications Information
SOCKET _socket = INVALID_SOCKET;      // Communications Interface
struct sockaddr_in _server;           // Communications address, port...
fd_set read_set;                      // To detect new output received from PX4

/******************************************************************************
* Initialize Windows Socket Library and configure UDP communications to PX4
* Creates a communications socket at IP address 192.168.46.2 on port 14560.
* This function must be called before communicating with the PX4.
*******************************************************************************/
void initializeCommunications()
{
  // Windows Sockets implementation Information
  WSADATA wsa;

  // Initialise winsock library
  if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
  {
    mexPrintf("Failed. Error Code: %d\n", WSAGetLastError());
    return;
  }

  // try to setup UDP socket for communication
  memset((char *)&_server, 0, sizeof(_server));
  _server.sin_family = AF_INET;
  _server.sin_addr.S_un.S_addr = inet_addr("192.168.46.2");
  _server.sin_port = htons(14560);

  if ((_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == SOCKET_ERROR)
  {
    mexPrintf("socket() failed with error code: %d\n", WSAGetLastError());
  }

  // Initializes file descriptor
  FD_ZERO(&read_set);
}

/******************************************************************************
* Send current sensor data to PX4.
* Must be called every cycle of the simulation.
* New IMU data is sent to PX4 every 4000 microseconds (250Hz)
* New GPS data is sent to PX4 every 500000 microseconds (2Hz)
* New Remote Control data is sent to PX4 every cycle
* parameters:
*  delta - input time since last step calculation (microsecond)
*  imu - input contains current values from inertial measurement unit sensor
*  gps - input contains current values from global position system sensor
*  rc - input contains current values from remote control sensor
*******************************************************************************/
void autopilot_input(uint32_t delta[1], double imu[13], double gps[12], uint16_t rc[20])
{
  // Simulation time in microseconds
  // Incremented every cycle by Simulink Time Step Delta
  static uint64_t time = 0;                    

  // Keeping track of time.
  time += delta[0];

  // IMU data sent to the PX4 at 250Hz
  // Keep track of last time new IMU data was sent to the PX4
  static uint64_t last_imu_time = 0;
  // Determine if at least 4000 microseconds has passed since new data was sent
  if( time > delta[0] && last_imu_time - time >= 4000 )
  {
    // Send new IMU data to PX4
    sendIMU(time, imu, 4095); // 4095 indicates new values for all data members
    // Keep track of when the latest IMU data was sent
    last_imu_time = time;
  }

  // GPS data sent to the PX4 at 2Hz
  // Keep track of last time new GPS data was sent to the PX4
  static uint64_t last_gps_time = 0;
  // Determine if at least 500000 microseconds has passed since new data was sent
  if( time > delta[0] && last_gps_time - time >= 500000 )
  {
    // Send new GPS data to PX4
    sendGPS(time, gps);
    // Keep track of when the latest GPS data was sent
    last_gps_time = time;
  }

  // Remote Control data sent every cycle of the simulation
  sendRC((uint32_t)(time / 1000), rc);
}

/******************************************************************************
* Receive data from PX4.
* Must be called every cycle of the simulation.
* New actuator data is received every cycle
* parameters:
*  actuators - output contains the current actuators commands received from PX4
*******************************************************************************/
void autopilot_output(double actuators[17])
{
  // Define how long to wait for new data to be received
  static const struct timeval timeout = { 0,1000 }; // 1000 microseconds
  // Allocate space for received data to be copied in to
  static unsigned char _buf[1024];
  // Calculate the space required for the sockaddr_in
  static socklen_t _addrlen = sizeof(_server);

  // Testing for new data to receive
  FD_SET(_socket, &read_set);
  // wait up to 1000uS for new data to be ready to be received
  if (select(0, &read_set, NULL, NULL, &timeout) > 0)
  {
    // Determine if new data is ready to be received
    if (FD_ISSET(_socket, &read_set)) 
    {
      // Receive new data into allocated memory buffer
      int len = recvfrom(_socket, (char*)_buf, sizeof(_buf), 0, (struct sockaddr *)&_server, &_addrlen);

      // Verify that data was receive
      if (len > 0) 
      {
        // Create a blank MAVLink message to fill with received data
        mavlink_message_t msg;
        // Track the status
        mavlink_status_t udp_status = {0};

        // Process each received byte until we have a message
        for (int i = 0; i < len; i++) {
          if (mavlink_parse_char(0, _buf[i], &msg, &udp_status)) {
            // have a message, handle it
            handle_message(&msg, actuators);
          }
        }
      }
    }
  }
}

/******************************************************************************
* Process received MAVLink messages.
* Receive actuator commands from PX4.
* Actuator values are copied to Simulink S-Function output.
* parameters:
*  msg - input the received MAVLink message to handle
*  actuators - output contains the current actuators commands received from PX4
*******************************************************************************/
void handle_message(mavlink_message_t *msg, double actuators[17])
{
  // Determine what message
  switch (msg->msgid) 
  {
  case MAVLINK_MSG_ID_HEARTBEAT:
    mavlink_heartbeat_t hb;
    mavlink_msg_heartbeat_decode(msg, &hb);
    //mexPrintf(".");
    break;

  case MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS:
    mavlink_hil_actuator_controls_t ac;
    mavlink_msg_hil_actuator_controls_decode(msg, &ac);
    actuators[0]  = ac.controls[0];
    actuators[1]  = ac.controls[1];
    actuators[2]  = ac.controls[2];
    actuators[3]  = ac.controls[3];
    actuators[4]  = ac.controls[4];
    actuators[5]  = ac.controls[5];
    actuators[6]  = ac.controls[6];
    actuators[7]  = ac.controls[7];
    actuators[8]  = ac.controls[8];
    actuators[9]  = ac.controls[9];
    actuators[10] = ac.controls[10];
    actuators[11] = ac.controls[11];
    actuators[12] = ac.controls[12];
    actuators[13] = ac.controls[13];
    actuators[14] = ac.controls[14];
    actuators[15] = ac.controls[15];
    break;

  default:
    mexPrintf("Message ID: %d received but NOT processed.\n", msg->msgid);
    break;
  }
}

/******************************************************************************
* Send IMU data in MAVLink message to PX4.
* parameters:
*  time_usec - input current simulation time of sensor measurments in microseconds
*  imu - input contains current values from inertial measurement unit sensor
*  fields_updated - input bit field indicating which IMU values are new
*******************************************************************************/
void sendIMU(uint64_t time_usec, double imu[13], uint32_t fields_updated)
{
  // Create blank MAVLink message.
  mavlink_message_t msg;
  // Construct message with IMU data.
  mavlink_msg_hil_sensor_construct_chan(1, 200, MAVLINK_COMM_0, &msg, time_usec, imu, fields_updated);
  // Send MAVLink message.
  send_mavlink_message(&msg, 0);
}

/******************************************************************************
* Send GPS data in MAVLink message to PX4.
* parameters:
*  time_usec - input current simulation time of sensor measurments in microseconds
*  gps - input contains current values from global position system sensor
*  fields_updated - input bit field indicating which IMU values are new
*******************************************************************************/
void sendGPS(uint64_t time_usec, double gps[12])
{
  // Create blank MAVLink message.
  mavlink_message_t msg;
  // Construct message with IMU data.
  mavlink_msg_hil_gps_construct_chan(1, 200, MAVLINK_COMM_0, &msg, time_usec, gps);
  // Send MAVLink message.
  send_mavlink_message(&msg, 0);
}

/******************************************************************************
* Send Remote Control data in MAVLink message to PX4.
* parameters:
*  time_usec - input current simulation time of sensor measurments in microseconds
*  rc - input contains current values from remote control sensor
*  fields_updated - input bit field indicating which IMU values are new
*******************************************************************************/
void sendRC(uint32_t time_boot_ms, double rc[20])
{
  // Create blank MAVLink message.
  mavlink_message_t msg;
  // Construct message with RC data.
  mavlink_msg_rc_channels_construct_chan(1, 200, MAVLINK_COMM_0, &msg, time_boot_ms, rc);
  // Send MAVLink message.
  send_mavlink_message(&msg, 0);
}

/******************************************************************************
* Send MAVLink message to PX4.
* parameters:
*  message - MAVLink message to send
*  destination_port - port to send MAVLink message on
*******************************************************************************/
void send_mavlink_message(const mavlink_message_t *message, const int destination_port)
{
  // Allocate a memory buffer to send
  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  // Pack the buffer with the MAVLink message
  int packetlen = mavlink_msg_to_send_buffer(buffer, message);
  // Send the buffered MAVLink message
  int len = sendto(_socket, (char*)buffer, packetlen, 0, (struct sockaddr *)&_server, sizeof(_server));
  // Alert if there was an issue sending the data
  if (len <= 0)
  {
    mexPrintf("Failed sending mavlink message\n");
  }
}