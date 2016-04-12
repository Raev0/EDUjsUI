/* -*- c++ -*- */

/*
    Reprap firmware based on Sprinter and grbl.
 Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 This firmware is a mashup between Sprinter and grbl.
  (https://github.com/kliment/Sprinter)
  (https://github.com/simen/grbl/tree)

 It has preliminary support for Matthew Roberts advance algorithm
    http://reprap.org/pipermail/reprap-dev/2011-May/003323.html
 */

#include "Marlin.h"

#ifdef ENABLE_AUTO_BED_LEVELING
#include "vector_3.h"
  #ifdef AUTO_BED_LEVELING_GRID
    #include "qr_solve.h"
  #endif
#endif // ENABLE_AUTO_BED_LEVELING
#include "POWER_DOWN_RESUME.h"
//#include "ultralcd.h"
#include "planner.h"
#include "stepper.h"
#include "temperature.h"
#include "motion_control.h"
#include "cardreader.h"
#include "watchdog.h"
#include "ConfigurationStore.h"
#include "language.h"
#include "pins_arduino.h"
#include "math.h"
#include "buzzer_sound.h"
#include "EEPROM.h"
#include <avr/io.h>
#include <avr/boot.h>
#include <avr/eeprom.h>
#include <stdint.h>
#include "absacc.h"
#ifdef BLINKM
  #include "BlinkM.h"
  #include "Wire.h"
#endif

#if NUM_SERVOS > 0
  #include "Servo.h"
#endif

#if defined(DIGIPOTSS_PIN) && DIGIPOTSS_PIN > -1
  #include <SPI.h>
#endif
#define EEPROM_SECTION  __attribute__ ((section (".eeprom")))

#include "tftprotocol.h"

// look here for descriptions of G-codes: http://linuxcnc.org/handbook/gcode/g-code.html
// http://objects.reprap.org/wiki/Mendel_User_Manual:_RepRapGCodes

//Implemented Codes
//-------------------
// G0  -> G1
// G1  - Coordinated Movement X Y Z E
// G2  - CW ARC
// G3  - CCW ARC
// G4  - Dwell S<seconds> or P<milliseconds>
// G10 - retract filament according to settings of M207
// G11 - retract recover filament according to settings of M208
// G28 - Home all Axis
// G29 - Detailed Z-Probe, probes the bed at 3 or more points.  Will fail if you haven't homed yet.
// G30 - Single Z Probe, probes bed at current XY location.
// G31 - Dock sled (Z_PROBE_SLED only)
// G32 - Undock sled (Z_PROBE_SLED only)
// G90 - Use Absolute Coordinates
// G91 - Use Relative Coordinates
// G92 - Set current position to coordinates given

// M Codes
// M0   - Unconditional stop - Wait for user to press a button on the LCD (Only if ULTRA_LCD is enabled)
// M1   - Same as M0
// M17  - Enable/Power all stepper motors
// M18  - Disable all stepper motors; same as M84
// M20  - List SD card
// M21  - Init SD card
// M22  - Release SD card
// M23  - Select SD file (M23 filename.g)
// M24  - Start/resume SD print
// M25  - Pause SD print
// M26  - Set SD position in bytes (M26 S12345)
// M27  - Report SD print status
// M28  - Start SD write (M28 filename.g)
// M29  - Stop SD write
// M30  - Delete file from SD (M30 filename.g)
// M31  - Output time since last M109 or SD card start to serial
// M32  - Select file and start SD print (Can be used _while_ printing from SD card files):
//        syntax "M32 /path/filename#", or "M32 S<startpos bytes> !filename#"
//        Call gcode file : "M32 P !filename#" and return to caller file after finishing (similar to #include).
//        The '#' is necessary when calling from within sd files, as it stops buffer prereading
// M42  - Change pin status via gcode Use M42 Px Sy to set pin x to value y, when omitting Px the onboard led will be used.
// M80  - Turn on Power Supply
// M81  - Turn off Power Supply
// M82  - Set E codes absolute (default)
// M83  - Set E codes relative while in Absolute Coordinates (G90) mode
// M84  - Disable steppers until next move,
//        or use S<seconds> to specify an inactivity timeout, after which the steppers will be disabled.  S0 to disable the timeout.
// M85  - Set inactivity shutdown timer with parameter S<seconds>. To disable set zero (default)
// M92  - Set axis_steps_per_unit - same syntax as G92
// M104 - Set extruder target temp
// M105 - Read current temp
// M106 - Fan on
// M107 - Fan off
// M109 - Sxxx Wait for extruder current temp to reach target temp. Waits only when heating
//        Rxxx Wait for extruder current temp to reach target temp. Waits when heating and cooling
//        IF AUTOTEMP is enabled, S<mintemp> B<maxtemp> F<factor>. Exit autotemp by any M109 without F
// M112 - Emergency stop
// M114 - Output current position to serial port
// M115 - Capabilities string
// M117 - display message
// M119 - Output Endstop status to serial port
// M126 - Solenoid Air Valve Open (BariCUDA support by jmil)
// M127 - Solenoid Air Valve Closed (BariCUDA vent to atmospheric pressure by jmil)
// M128 - EtoP Open (BariCUDA EtoP = electricity to air pressure transducer by jmil)
// M129 - EtoP Closed (BariCUDA EtoP = electricity to air pressure transducer by jmil)
// M140 - Set bed target temp
// M150 - Set BlinkM Color Output R: Red<0-255> U(!): Green<0-255> B: Blue<0-255> over i2c, G for green does not work.
// M190 - Sxxx Wait for bed current temp to reach target temp. Waits only when heating
//        Rxxx Wait for bed current temp to reach target temp. Waits when heating and cooling
// M200 D<millimeters>- set filament diameter and set E axis units to cubic millimeters (use S0 to set back to millimeters).
// M201 - Set max acceleration in units/s^2 for print moves (M201 X1000 Y1000)
// M202 - Set max acceleration in units/s^2 for travel moves (M202 X1000 Y1000) Unused in Marlin!!
// M203 - Set maximum feedrate that your machine can sustain (M203 X200 Y200 Z300 E10000) in mm/sec
// M204 - Set default acceleration: S normal moves T filament only moves (M204 S3000 T7000) in mm/sec^2  also sets minimum segment time in ms (B20000) to prevent buffer under-runs and M20 minimum feedrate
// M205 -  advanced settings:  minimum travel speed S=while printing T=travel only,  B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk, E=maximum E jerk
// M206 - Set additional homing offset
// M207 - Set retract length S[positive mm] F[feedrate mm/min] Z[additional zlift/hop], stays in mm regardless of M200 setting
// M208 - Set recover=unretract length S[positive mm surplus to the M207 S*] F[feedrate mm/sec]
// M209 - S<1=true/0=false> enable automatic retract detect if the slicer did not support G10/11: every normal extrude-only move will be classified as retract depending on the direction.
// M218 - Set hotend offset (in mm): T<extruder_number> X<offset_on_X> Y<offset_on_Y>
// M220 S<factor in percent>- set speed factor override percentage
// M221 S<factor in percent>- set extrude factor override percentage
// M226 P<pin number> S<pin state>- Wait until the specified pin reaches the state required
// M240 - Trigger a camera to take a photograph
// M250 - Set LCD contrast C<contrast value> (value 0..63)
// M280 - Set servo position absolute. P: servo index, S: angle or microseconds
// M300 - Play beep sound S<frequency Hz> P<duration ms>
// M301 - Set PID parameters P I and D
// M302 - Allow cold extrudes, or set the minimum extrude S<temperature>.
// M303 - PID relay autotune S<temperature> sets the target temperature. (default target temperature = 150C)
// M304 - Set bed PID parameters P I and D
// M400 - Finish all moves
// M401 - Lower z-probe if present
// M402 - Raise z-probe if present
// M404 - N<dia in mm> Enter the nominal filament width (3mm, 1.75mm ) or will display nominal filament width without parameters
// M405 - Turn on Filament Sensor extrusion control.  Optional D<delay in cm> to set delay in centimeters between sensor and extruder
// M406 - Turn off Filament Sensor extrusion control
// M407 - Displays measured filament diameter
// M500 - Store parameters in EEPROM
// M501 - Read parameters from EEPROM (if you need reset them after you changed them temporarily).
// M502 - Revert to the default "factory settings".  You still need to store them in EEPROM afterwards if you want to.
// M503 - Print the current settings (from memory not from EEPROM). Use S0 to leave off headings.
// M540 - Use S[0|1] to enable or disable the stop SD card print on endstop hit (requires ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)
// M600 - Pause for filament change X[pos] Y[pos] Z[relative lift] E[initial retract] L[later retract distance for removal]
// M665 - Set delta configurations
// M666 - Set delta endstop adjustment
// M605 - Set dual x-carriage movement mode: S<mode> [ X<duplication x-offset> R<duplication temp offset> ]
// M907 - Set digital trimpot motor current using axis codes.
// M908 - Control digital trimpot directly.
// M350 - Set microstepping mode.
// M351 - Toggle MS1 MS2 pins directly.

// ************ SCARA Specific - This can change to suit future G-code regulations
// M360 - SCARA calibration: Move to cal-position ThetaA (0 deg calibration)
// M361 - SCARA calibration: Move to cal-position ThetaB (90 deg calibration - steps per degree)
// M362 - SCARA calibration: Move to cal-position PsiA (0 deg calibration)
// M363 - SCARA calibration: Move to cal-position PsiB (90 deg calibration - steps per degree)
// M364 - SCARA calibration: Move to cal-position PSIC (90 deg to Theta calibration position)
// M365 - SCARA calibration: Scaling factor, X, Y, Z axis
//************* SCARA End ***************

// M928 - Start SD logging (M928 filename.g) - ended by M29
// M999 - Restart after being stopped by error

#ifdef SDSUPPORT
  CardReader card;
#endif
float homing_feedrate[] = HOMING_FEEDRATE;
bool axis_relative_modes[] = AXIS_RELATIVE_MODES;
int feedmultiply = 100; //100->1 200->2
int saved_feedmultiply;
int extrudemultiply = 100; //100->1 200->2
int w_h_4_2_0;
int w_h_4_2_1;
int w_h_4_2_2;
int w_h_4_2_3;
int w_h_4_2_4;
int w_h_4_2_5;
int w_h_4_2_6;
int w_h_4_2_7;
int w_d_x_5_12;
float w_d_x_5_16;
float z_value_offset=0;
union z_value back_z_value;
union std_error p2p_3mm_std_error;
union set_angle servo_set_angle;
float negative_z_offset = 2.54;//2.7-0.16 extruder error is 3.0mm+-0.3mm 0.16 is the heat expansion

char * w_d_x_temp_1="M104 T0 S200";
char * w_d_x_temp_2="M109 T0 S200";
uint8_t step_resume=0;
bool w_h_4_check=0;
static bool w_d_x_8_18_check=0;
static bool w_d_x_8_18_checke=0;
extern unsigned char permit_1;
uint32_t w_h_4_1;//zct
uint32_t w_h_4_2;//zct
uint32_t w_h_4_1_e;
uint32_t w_h_4_2_e;
int w_h_EE_check;
uint16_t feedrate_test=2000;
 uint32_t now_time;
 uint32_t old_time;
unsigned char eeprom_var[10] EEPROM_SECTION;
BOOTLOADER_SECTION
unsigned char sn[10];
//char * w_h_power_off="M33";
//extern char * w_h_str_clear;
int extruder_multiply[EXTRUDERS] = { 100
  #if EXTRUDERS > 1
    , 100
    #if EXTRUDERS > 2
      , 100
	    #if EXTRUDERS > 3
      	, 100
	    #endif
    #endif
  #endif
};
bool volumetric_enabled = false;
float filament_size[EXTRUDERS] = { DEFAULT_NOMINAL_FILAMENT_DIA
  #if EXTRUDERS > 1
      , DEFAULT_NOMINAL_FILAMENT_DIA
    #if EXTRUDERS > 2
       , DEFAULT_NOMINAL_FILAMENT_DIA
      #if EXTRUDERS > 3
        , DEFAULT_NOMINAL_FILAMENT_DIA
      #endif
    #endif
  #endif
};
float volumetric_multiplier[EXTRUDERS] = {1.0
  #if EXTRUDERS > 1
    , 1.0
    #if EXTRUDERS > 2
      , 1.0
      #if EXTRUDERS > 3
        , 1.0
      #endif
    #endif
  #endif
};
float current_position[NUM_AXIS] = { 0.0, 0.0, 0.0, 0.0 };
float add_homing[3] = { 0, 0, 0 };
#ifdef DELTA
  float endstop_adj[3] = { 0, 0, 0 };
#endif

float min_pos[3] = { X_MIN_POS, Y_MIN_POS, Z_MIN_POS };
float max_pos[3] = { X_MAX_POS, Y_MAX_POS, Z_MAX_POS };
bool axis_known_position[3] = { false, false, false };
float zprobe_zoffset=2.54;

// Extruder offset
#if EXTRUDERS > 1
#ifndef DUAL_X_CARRIAGE
  #define NUM_EXTRUDER_OFFSETS 2 // only in XY plane
#else
  #define NUM_EXTRUDER_OFFSETS 3 // supports offsets in XYZ plane
#endif
float extruder_offset[NUM_EXTRUDER_OFFSETS][EXTRUDERS] = {
  #if defined(EXTRUDER_OFFSET_X)
    EXTRUDER_OFFSET_X
  #else
    0
  #endif
  ,
  #if defined(EXTRUDER_OFFSET_Y)
    EXTRUDER_OFFSET_Y
  #else
    0
  #endif
};
#endif

uint8_t active_extruder = 0;
int fanSpeed = 0;

#ifdef SERVO_ENDSTOPS
  int servo_endstops[] = SERVO_ENDSTOPS;
  int servo_endstop_angles[] = SERVO_ENDSTOP_ANGLES;
#endif

#ifdef BARICUDA
  int ValvePressure = 0;
  int EtoPPressure = 0;
#endif

#ifdef FWRETRACT

  bool autoretract_enabled = false;
  bool retracted[EXTRUDERS] = { false
    #if EXTRUDERS > 1
      , false
      #if EXTRUDERS > 2
        , false
        #if EXTRUDERS > 3
       	  , false
      	#endif
      #endif
    #endif
  };
  bool retracted_swap[EXTRUDERS] = { false
    #if EXTRUDERS > 1
      , false
      #if EXTRUDERS > 2
        , false
        #if EXTRUDERS > 3
       	  , false
      	#endif
      #endif
    #endif
  };

  float retract_length = RETRACT_LENGTH;
  float retract_length_swap = RETRACT_LENGTH_SWAP;
  float retract_feedrate = RETRACT_FEEDRATE;
  float retract_zlift = RETRACT_ZLIFT;
  float retract_recover_length = RETRACT_RECOVER_LENGTH;
  float retract_recover_length_swap = RETRACT_RECOVER_LENGTH_SWAP;
  float retract_recover_feedrate = RETRACT_RECOVER_FEEDRATE;

#endif // FWRETRACT

#ifdef ULTIPANEL
  bool powersupply =
    #ifdef PS_DEFAULT_OFF
      false
    #else
  	  true
    #endif
  ;
#endif

#ifdef DELTA
  float delta[3] = { 0, 0, 0 };
  #define SIN_60 0.8660254037844386
  #define COS_60 0.5
  // these are the default values, can be overriden with M665
  float delta_radius = DELTA_RADIUS;
  float delta_tower1_x = -SIN_60 * delta_radius; // front left tower
  float delta_tower1_y = -COS_60 * delta_radius;
  float delta_tower2_x =  SIN_60 * delta_radius; // front right tower
  float delta_tower2_y = -COS_60 * delta_radius;
  float delta_tower3_x = 0;                      // back middle tower
  float delta_tower3_y = delta_radius;
  float delta_diagonal_rod = DELTA_DIAGONAL_ROD;
  float delta_diagonal_rod_2 = sq(delta_diagonal_rod);
  float delta_segments_per_second = DELTA_SEGMENTS_PER_SECOND;
#endif

#ifdef SCARA
  float axis_scaling[3] = { 1, 1, 1 };    // Build size scaling, default to 1
#endif

bool cancel_heatup = false;

#ifdef FILAMENT_SENSOR
  //Variables for Filament Sensor input
  float filament_width_nominal=DEFAULT_NOMINAL_FILAMENT_DIA;  //Set nominal filament width, can be changed with M404
  bool filament_sensor=false;  //M405 turns on filament_sensor control, M406 turns it off
  float filament_width_meas=DEFAULT_MEASURED_FILAMENT_DIA; //Stores the measured filament diameter
  signed char measurement_delay[MAX_MEASUREMENT_DELAY+1];  //ring buffer to delay measurement  store extruder factor after subtracting 100
  int delay_index1=0;  //index into ring buffer
  int delay_index2=-1;  //index into ring buffer - set to -1 on startup to indicate ring buffer needs to be initialized
  float delay_dist=0; //delay distance counter
  int meas_delay_cm = MEASUREMENT_DELAY_CM;  //distance delay setting
#endif
//CBYTE[0x0002]=0x57;
const char errormagic[] PROGMEM = "Error:";
const char echomagic[] PROGMEM = "echo:";

const char axis_codes[NUM_AXIS] = {'X', 'Y', 'Z', 'E'};
static float destination[NUM_AXIS] = { 0, 0, 0, 0 };

#ifndef DELTA
  static float delta[3] = { 0, 0, 0 };
#endif

static float offset[3] = { 0, 0, 0 };
static bool home_all_axis = true;
static float feedrate = 1500.0, next_feedrate, saved_feedrate;
static long gcode_N, gcode_LastN, Stopped_gcode_LastN = 0;

static bool relative_mode = false;  //Determines Absolute or Relative Coordinates
static char w_h_3_19=0;
 char cmdbuffer[BUFSIZE][MAX_CMD_SIZE];
static bool fromsd[BUFSIZE];
 int bufindr = 0;
static int bufindw = 0;
static int buflen = 0;

static char serial_char;
static int serial_count = 0;
static boolean comment_mode = false;
static char *strchr_pointer; ///< A pointer to find chars in the command string (X, Y, Z, E, etc.)
static int strchr_position;
const int sensitive_pins[] = SENSITIVE_PINS; ///< Sensitive pin list for M42

// Inactivity shutdown
static unsigned long previous_millis_cmd = 0;
static unsigned long max_inactive_time = 0;
static unsigned long stepper_inactive_time = DEFAULT_STEPPER_DEACTIVE_TIME*1000l;

unsigned long starttime = 0; ///< Print job start time
unsigned long stoptime = 0;  ///< Print job stop time
//SdBaseFile file_w_h;
static uint8_t tmp_extruder;


bool Stopped = false;

#if NUM_SERVOS > 0
  Servo servos[NUM_SERVOS];
#endif

bool CooldownNoWait = true;
bool target_direction;

#ifdef CHDK
  unsigned long chdkHigh = 0;
  boolean chdkActive = false;
#endif

bool uart2ready=false;
String fileselected;
bool oldcardstatus;
bool monitormode=false;
uint16_t startmonitormillis;
uint16_t mnttimer=MNT_INTERNAL;
uint8_t querytype;
String stopmsg;
int movingcount[NUM_AXIS]={0,0,0,0};
//int axis2move=Z_AXIS;
//float move_axis_scale=0.1;
#ifdef ULTIPANEL
  static float manual_feedrate[] = MANUAL_FEEDRATE;
#endif // ULTIPANEL
int noteDurations_a[] = {6,6,6,6,6,2,4};
 #define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978
int melody_a[] = {NOTE_D8, NOTE_D8,NOTE_D4,NOTE_D4, NOTE_D8,NOTE_D8, 0};
  extern int plaPreheatHotendTemp;
  extern int plaPreheatHPBTemp;
  extern int plaPreheatFanSpeed;

  extern int absPreheatHotendTemp;
  extern int absPreheatHPBTemp;
  extern int absPreheatFanSpeed;
  bool heat4fila=false;
  float axisstep=steplength1;
  float movedirect;
  uint8_t filamove=nomove;
  uint8_t emovecount=EINTERVAL;
  bool emovable=false;
  bool fanop=false;
  bool invalid4resume=false;
  bool B_fileselected=false;
  double total_time=0;
  bool old_z_detected=false;
  bool old_y_detected=false;
  bool old_x_detected=false;
  int buflen_stack;
  int bufindr_stack;
  int bufindw_stack;
  //unsigned char block_buffer_tail_stack;
  //unsigned char block_buffer_head_stack;
  static float x_pos_stack;
  static float y_pos_stack;
  static float feedrate_stack;
  //extern block_t block_buffer[BLOCK_BUFFER_SIZE];            // A ring buffer for motion instfructions
  //block_t block_buffer_mirror[BLOCK_BUFFER_SIZE];            // A ring buffer for motion instfructions
  //unsigned char block_buffer_head_stack;
bool whatisendstop();
uint32_t reposition_buffer[BLOCK_BUFFER_SIZE+1];
uint32_t rePosition;
uint32_t power_off_store[8];
char machine_series[16];
String rx_msg;
  //bool block_buffer_pause=false;
 // extern float position[AXIS_NUM];



  #define qtx_max  172
  #define qtx_min   36
  #define qty_max   200
  #define qty_min    70
//===========================================================================
//=============================Routines======================================
//===========================================================================
void lcd_move_z();
void get_arc_coordinates();
bool setTargetedHotend(int code);
//zct: extern void standby_and_poweroff();
uint8_t tft_rx();
void tft_status_monitor_update();
void tft_status_temperature_update();
void tft_stopbutton_click();

void tft_update();
void pwroff_re_proc();

void serial_echopair_P(const char *s_P, float v)
    { serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char *s_P, double v)
    { serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char *s_P, unsigned long v)
    { serialprintPGM(s_P); SERIAL_ECHO(v); }

#ifdef SDSUPPORT
  #include "SdFatUtil.h"
  int freeMemory() { return SdFatUtil::FreeRam(); }
#else
  extern "C" {
    extern unsigned int __bss_end;
    extern unsigned int __heap_start;
    extern void *__brkval;

    int freeMemory() {
      int free_memory;

      if ((int)__brkval == 0)
        free_memory = ((int)&free_memory) - ((int)&__bss_end);
      else
        free_memory = ((int)&free_memory) - ((int)__brkval);

      return free_memory;
    }
  }
#endif //!SDSUPPORT

//adds an command to the main command buffer
//thats really done in a non-safe way.
//needs overworking someday
void enquecommand(const char *cmd)
{
  if(buflen < BUFSIZE)
  {
    //this is dangerous if a mixing of serial and this happens
    strcpy(&(cmdbuffer[bufindw][0]),cmd);
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM(MSG_Enqueing);
    SERIAL_ECHO(cmdbuffer[bufindw]);
    SERIAL_ECHOLNPGM("\"");
    bufindw= (bufindw + 1)%BUFSIZE;
    buflen += 1;
  }
}

void enquecommand_P(const char *cmd)
{
  if(buflen < BUFSIZE)
  {
    //this is dangerous if a mixing of serial and this happens
    strcpy_P(&(cmdbuffer[bufindw][0]),cmd);
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM(MSG_Enqueing);
    SERIAL_ECHO(cmdbuffer[bufindw]);
    SERIAL_ECHOLNPGM("\"");
    bufindw= (bufindw + 1)%BUFSIZE;
    buflen += 1;
  }
}

void setup_killpin()
{
  #if defined(KILL_PIN) && KILL_PIN > -1
    SET_INPUT(KILL_PIN);
    WRITE(KILL_PIN,HIGH);
  #endif
}

// Set home pin
void setup_homepin(void)
{
#if defined(HOME_PIN) && HOME_PIN > -1
   SET_INPUT(HOME_PIN);
   WRITE(HOME_PIN,HIGH);
#endif
}

void setup_photpin()
{
  #if defined(PHOTOGRAPH_PIN) && PHOTOGRAPH_PIN > -1
    SET_OUTPUT(PHOTOGRAPH_PIN);
    WRITE(PHOTOGRAPH_PIN, LOW);
  #endif
}

void setup_powerhold()
{
  #if defined(SUICIDE_PIN) && SUICIDE_PIN > -1
    SET_OUTPUT(SUICIDE_PIN);
    WRITE(SUICIDE_PIN, HIGH);
  #endif
  #if defined(PS_ON_PIN) && PS_ON_PIN > -1
    SET_OUTPUT(PS_ON_PIN);
	#if defined(PS_DEFAULT_OFF)
	  WRITE(PS_ON_PIN, PS_ON_ASLEEP);
    #else
	  WRITE(PS_ON_PIN, PS_ON_AWAKE);
	#endif
  #endif
}

void suicide()
{
  #if defined(SUICIDE_PIN) && SUICIDE_PIN > -1
    SET_OUTPUT(SUICIDE_PIN);
    WRITE(SUICIDE_PIN, LOW);
  #endif
}

void servo_init()
{
  #if (NUM_SERVOS >= 1) && defined(SERVO0_PIN) && (SERVO0_PIN > -1)
    servos[0].attach(SERVO0_PIN);
  #endif
  #if (NUM_SERVOS >= 2) && defined(SERVO1_PIN) && (SERVO1_PIN > -1)
    servos[1].attach(SERVO1_PIN);
  #endif
  #if (NUM_SERVOS >= 3) && defined(SERVO2_PIN) && (SERVO2_PIN > -1)
    servos[2].attach(SERVO2_PIN);
  #endif
  #if (NUM_SERVOS >= 4) && defined(SERVO3_PIN) && (SERVO3_PIN > -1)
    servos[3].attach(SERVO3_PIN);
  #endif
  #if (NUM_SERVOS >= 5)
    #error "TODO: enter initalisation code for more servos"
  #endif

  // Set position of Servo Endstops that are defined
  #ifdef SERVO_ENDSTOPS
  for(int8_t i = 0; i < 3; i++)
  {
    if(servo_endstops[i] > -1) {
      servos[servo_endstops[i]].write(servo_endstop_angles[i * 2 + 1]);
    }
  }
  #endif

  #if defined (ENABLE_AUTO_BED_LEVELING) && (PROBE_SERVO_DEACTIVATION_DELAY > 0)
  delay(PROBE_SERVO_DEACTIVATION_DELAY);
  servos[servo_endstops[Z_AXIS]].detach();
  #endif
}

void setup()
{
   unsigned char i;
    w_d_x_5_12=0;
        for(i=0;i<9;i++)
        {
        sn[i]=boot_signature_byte_get((0x00)+i);
        }
        EEPROM.write(30,sn[0]);
        EEPROM.write(31,sn[1]);
        EEPROM.write(32,sn[2]);
        EEPROM.write(33,sn[3]);
        EEPROM.write(34,sn[4]);
        EEPROM.write(35,sn[5]);
        EEPROM.write(36,sn[6]);
        EEPROM.write(37,sn[7]);
        EEPROM.write(38,sn[8]);
        EEPROM.write(39,sn[9]);
        uint32_t w_h_flash_clear_test;
 //   if(EEPROM.read(31)!=144){boot_page_erase(0x0200);for(unsigned char i=0;i<=1024;i++){ w_h_flash_clear_test=0x0200+0x0100*i;boot_page_erase(w_h_flash_clear_test);}} w_d_x_5_12=1;
//       delay(10);
   setup_killpin(); ///zct: stop pin
  setup_powerhold();
  MYSERIAL.begin(BAUDRATE);
  SERIAL_PROTOCOLLNPGM("start");
  SERIAL_ECHO_START;
 w_h_4_1=0;
 w_h_4_2=0;
 w_h_4_1_e=756;
 w_h_4_2_e=899;
  byte mcu = MCUSR;
  if(mcu & 1) SERIAL_ECHOLNPGM(MSG_POWERUP);
  if(mcu & 2) SERIAL_ECHOLNPGM(MSG_EXTERNAL_RESET);
  if(mcu & 4) SERIAL_ECHOLNPGM(MSG_BROWNOUT_RESET);
  if(mcu & 8) SERIAL_ECHOLNPGM(MSG_WATCHDOG_RESET);
  if(mcu & 32) SERIAL_ECHOLNPGM(MSG_SOFTWARE_RESET);
  MCUSR=0;
  SERIAL_ECHOPGM(MSG_MARLIN);
  SERIAL_ECHOLNPGM(STRING_VERSION);
  #ifdef STRING_VERSION_CONFIG_H
    #ifdef STRING_CONFIG_H_AUTHOR
      SERIAL_ECHO_START;
      SERIAL_ECHOPGM(MSG_CONFIGURATION_VER);
      SERIAL_ECHOPGM(STRING_VERSION_CONFIG_H);
      SERIAL_ECHOPGM(MSG_AUTHOR);
      SERIAL_ECHOLNPGM(STRING_CONFIG_H_AUTHOR);
      SERIAL_ECHOPGM("Compiled: ");
      SERIAL_ECHOLNPGM(__DATE__);
    #endif // STRING_CONFIG_H_AUTHOR
  #endif // STRING_VERSION_CONFIG_H
  SERIAL_ECHO_START;
  SERIAL_ECHOPGM(MSG_FREE_MEMORY);
  SERIAL_ECHO(freeMemory());
  SERIAL_ECHOPGM(MSG_PLANNER_BUFFER_BYTES);
  SERIAL_ECHOLN((int)sizeof(block_t)*BLOCK_BUFFER_SIZE);
 // ini sd
  for(int8_t i = 0; i < BUFSIZE; i++)
  {
    fromsd[i] = false;
  }

  // loads data from EEPROM if available else uses defaults (and resets step acceleration rate)
  Config_RetrieveSettings();
  START_SOUND();
  tp_init();    // Initialize temperature loop
  plan_init();  // Initialize planner;
  watchdog_init();
  st_init();    // Initialize stepper, this enables interrupts!
  setup_photpin();
  servo_init();
  stepper_permit();
  step_init();
  step_check();
 pinMode(POWER_OFF,OUTPUT);
 digitalWrite(POWER_OFF,LOW);
 pinMode(POWER_OFF_CHECK,INPUT);
 digitalWrite(POWER_OFF_CHECK,HIGH);
// initiate the SD card detected. valid the pull up resistance. ZCT 2015/9/14
pinMode(SDCARDDETECT,INPUT);
    WRITE(SDCARDDETECT, HIGH);
oldcardstatus=false;

 POWER_DOWN_RESUME_INIT();
//the compensation distance between the pin tips and the extruder.
for (int i=0;i<4;i++){
			p2p_3mm_std_error.echar[i]=EEPROM.read(i+200);
}
zprobe_zoffset=abs(p2p_3mm_std_error.v-3)>0.3?zprobe_zoffset:p2p_3mm_std_error.v;
// the basic distance is 3mm, if larger than 3.3 mm or less than 2.7mm. the offset will be limited to 3mm
for (int i=0;i<2;i++){
			servo_set_angle.schar[i]=EEPROM.read(i+204);
}
servo_endstop_angles[4]=servo_set_angle.v>0?servo_set_angle.v:servo_endstop_angles[4];

  #if defined(CONTROLLERFAN_PIN) && CONTROLLERFAN_PIN > -1
    SET_OUTPUT(CONTROLLERFAN_PIN); //Set pin used for driver cooling fan
  #endif

  #ifdef DIGIPOT_I2C
    digipot_i2c_init();
  #endif
#ifdef Z_PROBE_SLED
  pinMode(SERVO0_PIN, OUTPUT);
  digitalWrite(SERVO0_PIN, LOW); // turn it off
#endif // Z_PROBE_SLED
  setup_homepin();


GSERIAL.begin(BAUDRATE_2);
GSERIAL.setTimeout(rxfinger);
ECHO();
ECHO("waiting for the iniaition of the touch screen XD");

card.initsd();
//qkstart4debug;
}



void loop()
{
	 if(buflen < (BUFSIZE-1)){
			get_command();
		}
	  	 // zct: excute the command as below
	if(buflen){ //zct: buflen is occupied
    #ifdef SDSUPPORT
      if(card.saving) //zct:  when writing to SD card
      {
        if(strstr_P(cmdbuffer[bufindr], PSTR("M29")) == NULL) //zct: SD is not writing hazard M29 - Stop SD write
        {
          card.write_command(cmdbuffer[bufindr]); //zct: ?????????????????????/

          if(card.logging)   //zct  sd card file is readily opened to ?read?
          {
            process_commands(); //zct: executing
          }
          else
          {
            SERIAL_PROTOCOLLNPGM(MSG_OK);
          }
        }
        else
        {
          card.closefile();
          SERIAL_PROTOCOLLNPGM(MSG_FILE_SAVED);
        }
      }
      else // card is not saving
      {
        process_commands();
      }
    #else
      process_commands(); ///zct: invalid for EDB
    #endif //SDSUPPORT
	//Monitormode probabaly should be added here ZCT

	//Monitormode probabaly should be added here ZCT

	if (buflen>0)
	{
		buflen = (buflen-1);  //zct:  next buff bit
		bufindr = (bufindr + 1)%BUFSIZE; //zct:  next buff
	}

  }//endof if(buflen)
  manage_heater();
  manage_inactivity();
  checkHitEndstops();
//if(w_h_3_19==0)START_SOUND();
w_h_3_19=1;
/*
if(!digitalRead(POWER_OFF_CHECK))
{
//  OVER_PRINT_SOUND();
 if(!digitalRead(POWER_OFF_CHECK))
 digitalWrite(POWER_OFF,HIGH);
 }
 */
 if(step_resume==0) {step_resume=1;}
// zct : the uart2 tft touch screen is controller here
 	  tft_update();

}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//								name :void get_command()
//								funtion闂備浇娉曢崰鎰板几婵犳艾绠柨鐕傛嫹 read gcode from uart0 for : 1: ready signal 闂備浇娉曢崰鎰板几婵犳艾绠柨鐕傛嫹 2闂備浇娉曢崰鎰板几婵犳艾绠紒灞绢劕inting status , 3: printing code
//								author: Mr. whocares
//								comments: zct 2015/07/04
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void get_command()
{
  while( MYSERIAL.available() > 0  && buflen < BUFSIZE) {
    serial_char = MYSERIAL.read();  // zct: read debugging info from uart0
    if(serial_char == '\n' ||
       serial_char == '\r' ||
       (serial_char == ':' && comment_mode == false) ||
       serial_count >= (MAX_CMD_SIZE - 1) )  // zct:  too long code
    {
      if(!serial_count) { //// zct:  empty line (too short)
        comment_mode = false; //for new command
        return;
      }

	  //zct:  read from the code buffer below and send the error message from the serial line

	  cmdbuffer[bufindw][serial_count] = 0; //zct: initiate the code buffer
      if(!comment_mode){
        comment_mode = false; //for new command
        fromsd[bufindw] = false;
        if(strchr(cmdbuffer[bufindw], 'N') != NULL) //zct:  check the first pos of the string 'N' in the cmdbuffer
        {
          strchr_pointer = strchr(cmdbuffer[bufindw], 'N');
          gcode_N = (strtol(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL, 10)); //zct: get the gcode in register gcode_N

		if(gcode_N != gcode_LastN+1 && (strstr_P(cmdbuffer[bufindw], PSTR("M110")) == NULL) ) {   //zct: M110: is not mention in the code list? gcode_N != gcode_LastN+1 means not contiuallyo
            SERIAL_ERROR_START;
            SERIAL_ERRORPGM(MSG_ERR_LINE_NO);
            SERIAL_ERRORLN(gcode_LastN);
            //Serial.println(gcode_N);
            FlushSerialRequestResend();
            serial_count = 0;
            return;
          }// end of 780

          if(strchr(cmdbuffer[bufindw], '*') != NULL)
          {
            byte checksum = 0;
            byte count = 0;
            while(cmdbuffer[bufindw][count] != '*') checksum = checksum^cmdbuffer[bufindw][count++];
            strchr_pointer = strchr(cmdbuffer[bufindw], '*');

            if( (int)(strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL)) != checksum) {
              SERIAL_ERROR_START;
              SERIAL_ERRORPGM(MSG_ERR_CHECKSUM_MISMATCH);
              SERIAL_ERRORLN(gcode_LastN);
              FlushSerialRequestResend();
              serial_count = 0;
              return;
            }// end of 797
            //if no errors, continue parsing
          }// end of 790
          else
          {
            SERIAL_ERROR_START;
            SERIAL_ERRORPGM(MSG_ERR_NO_CHECKSUM);
            SERIAL_ERRORLN(gcode_LastN);
            FlushSerialRequestResend();
            serial_count = 0;
            return;
          }
          gcode_LastN = gcode_N;
          //if no errors, continue parsing
        }// end of 776
        else  // if we don't receive 'N' but still see '*'
        {
          if((strchr(cmdbuffer[bufindw], '*') != NULL))
          {
            SERIAL_ERROR_START;
            SERIAL_ERRORPGM(MSG_ERR_NO_LINENUMBER_WITH_CHECKSUM);
            SERIAL_ERRORLN(gcode_LastN);
            serial_count = 0;
            return;
          }
        }
		/////////////////////////////////////////////////////////////////////////////////////////
		//////////////////////////////////////////////////zct: if G is recieved
        if((strchr(cmdbuffer[bufindw], 'G') != NULL)){
          strchr_pointer = strchr(cmdbuffer[bufindw], 'G');
          switch((int)((strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL)))){
          case 0:
          case 1:
          case 2:
          case 3:
            if (Stopped == true) {
              SERIAL_ERRORLNPGM(MSG_ERR_STOPPED);
              //zct: LCD_MESSAGEPGM(MSG_STOPPED);
            }
            break;
          default:
            break;
          }

        } // end of 832
		/////////////////////////////////////////////////////////////////////////////////////////
        //If command was e-stop process now: M112 emergency stop
        if(strcmp(cmdbuffer[bufindw], "M112") == 0)
          kill();
        /////////////////////////////////////////////////////////////////////////////////////////
        //If command was e-stop process now: M112 emergency stop
        bufindw = (bufindw + 1)%BUFSIZE;
        buflen += 1;
      } // end of 773 NOT comment_mode
	// end of reading code has 'N'
      serial_count = 0; //new line
    }//end of if(serial_char == '\n' || serial_char == '\r' ||  (serial_char == ':' && comment_mode == false) || (serial_char == ':' && comment_mode == false) ||serial_count >= (MAX_CMD_SIZE - 1) )
    else
    {
      if(serial_char == ';') comment_mode = true;
      if(!comment_mode) cmdbuffer[bufindw][serial_count++] = serial_char;
    }
  } // end of while


  #ifdef SDSUPPORT
    if(!card.sdprinting||serial_count!=0){
    return;}

  //'#' stops reading from SD to the buffer prematurely, so procedural macro calls are possible
  // if it occurs, stop_buffering is triggered and the buffer is ran dry.
  // this character _can_ occur in serial com, due to checksums. however, no checksums are used in SD printing
  // zct: reading from the sd card below
  static bool stop_buffering=false;
  if(buflen==0) stop_buffering=false;

  while( !card.eof()  && buflen < BUFSIZE && !stop_buffering) {
    int16_t n=card.get();
    serial_char = (char)n;
    if(serial_char == '\n' ||
       serial_char == '\r' ||
       (serial_char == '#' && comment_mode == false) ||
       (serial_char == ':' && comment_mode == false) ||
       serial_count >= (MAX_CMD_SIZE - 1)||n==-1)
    {


      if(card.eof()){
        SERIAL_PROTOCOLLNPGM(MSG_FILE_PRINTED);
        stoptime=millis();
        char time[30];
        unsigned long t=(stoptime-starttime)/1000;
        int hours, minutes;
        minutes=(t/60)%60;
        hours=t/60/60;
        sprintf_P(time, PSTR("%i hours %i minutes"),hours, minutes);
        SERIAL_ECHO_START;
        SERIAL_ECHOLN(time);
        //ZCT: lcd_setstatus(time);
        card.printingHasFinished();
        card.checkautostart(true);

      }
      if(serial_char=='#')
        stop_buffering=true;

      if(!serial_count)
      {
        comment_mode = false; //for new command
        return; //if empty line
      }
      cmdbuffer[bufindw][serial_count] = 0; //terminate string
//      if(!comment_mode){
        fromsd[bufindw] = true;

		rePosition=reposition_buffer[0];
		for (uint8_t i=0;i<BLOCK_BUFFER_SIZE+1;i++){
					reposition_buffer[i]=reposition_buffer[i+1];
		}
		reposition_buffer[BLOCK_BUFFER_SIZE+1]=card.curPosition();
		//tft_console_set(card.curCluster(),reposition_buffer[1],rePosition);
		for(uint8_t i=0;i<4;i++){
					power_off_store[i]=card.curCluster()>>i*8;
				    power_off_store[i+4]=rePosition>>i*8;
		}
		old_time=now_time;
		now_time=millis();
		ECHO.println(now_time-old_time);

		buflen += 1;
        bufindw = (bufindw + 1)%BUFSIZE;
        //ECHO("resume position is HAHAHAHAHAHAHA");
		//ECHO(rePosition);

		//ECHO("position is:");
	    //ECHO(card.curPosition());
//      }
      comment_mode = false; //for new command
      serial_count = 0; //clear buffer
    }
    else
    {
      if(serial_char == ';') comment_mode = true;
      if(!comment_mode) cmdbuffer[bufindw][serial_count++] = serial_char;
    }
  }

  #endif //SDSUPPORT

} // end of get_command


float code_value()
{
  return (strtod(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL));
}

long code_value_long()
{
  return (strtol(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL, 10));
}

bool code_seen(char code)
{
  strchr_pointer = strchr(cmdbuffer[bufindr], code);
  return (strchr_pointer != NULL);  //Return True if a character was found
}

#define DEFINE_PGM_READ_ANY(type, reader)       \
    static inline type pgm_read_any(const type *p)  \
    { return pgm_read_##reader##_near(p); }

DEFINE_PGM_READ_ANY(float,       float);
DEFINE_PGM_READ_ANY(signed char, byte);

#define XYZ_CONSTS_FROM_CONFIG(type, array, CONFIG) \
static const PROGMEM type array##_P[3] =        \
    { X_##CONFIG, Y_##CONFIG, Z_##CONFIG };     \
static inline type array(int axis)          \
    { return pgm_read_any(&array##_P[axis]); }

XYZ_CONSTS_FROM_CONFIG(float, base_min_pos,    MIN_POS);
XYZ_CONSTS_FROM_CONFIG(float, base_max_pos,    MAX_POS);
XYZ_CONSTS_FROM_CONFIG(float, base_home_pos,   HOME_POS);
XYZ_CONSTS_FROM_CONFIG(float, max_length,      MAX_LENGTH);
XYZ_CONSTS_FROM_CONFIG(float, home_retract_mm, HOME_RETRACT_MM);
XYZ_CONSTS_FROM_CONFIG(signed char, home_dir,  HOME_DIR);

#ifdef DUAL_X_CARRIAGE
  #if EXTRUDERS == 1 || defined(COREXY) \
      || !defined(X2_ENABLE_PIN) || !defined(X2_STEP_PIN) || !defined(X2_DIR_PIN) \
      || !defined(X2_HOME_POS) || !defined(X2_MIN_POS) || !defined(X2_MAX_POS) \
      || !defined(X_MAX_PIN) || X_MAX_PIN < 0
    #error "Missing or invalid definitions for DUAL_X_CARRIAGE mode."
  #endif
  #if X_HOME_DIR != -1 || X2_HOME_DIR != 1
    #error "Please use canonical x-carriage assignment" // the x-carriages are defined by their homing directions
  #endif

#define DXC_FULL_CONTROL_MODE 0
#define DXC_AUTO_PARK_MODE    1
#define DXC_DUPLICATION_MODE  2
static int dual_x_carriage_mode = DEFAULT_DUAL_X_CARRIAGE_MODE;

static float x_home_pos(int extruder) {
  if (extruder == 0)
    return base_home_pos(X_AXIS) + add_homing[X_AXIS];
  else
    // In dual carriage mode the extruder offset provides an override of the
    // second X-carriage offset when homed - otherwise X2_HOME_POS is used.
    // This allow soft recalibration of the second extruder offset position without firmware reflash
    // (through the M218 command).
    return (extruder_offset[X_AXIS][1] > 0) ? extruder_offset[X_AXIS][1] : X2_HOME_POS;
}

static int x_home_dir(int extruder) {
  return (extruder == 0) ? X_HOME_DIR : X2_HOME_DIR;
}

static float inactive_extruder_x_pos = X2_MAX_POS; // used in mode 0 & 1
static bool active_extruder_parked = false; // used in mode 1 & 2
static float raised_parked_position[NUM_AXIS]; // used in mode 1
static unsigned long delayed_move_time = 0; // used in mode 1
static float duplicate_extruder_x_offset = DEFAULT_DUPLICATION_X_OFFSET; // used in mode 2
static float duplicate_extruder_temp_offset = 0; // used in mode 2
bool extruder_duplication_enabled = false; // used in mode 2
#endif //DUAL_X_CARRIAGE

static void axis_is_at_home(int axis) {
#ifdef DUAL_X_CARRIAGE
  if (axis == X_AXIS) {
    if (active_extruder != 0) {
      current_position[X_AXIS] = x_home_pos(active_extruder);
      min_pos[X_AXIS] =          X2_MIN_POS;
      max_pos[X_AXIS] =          max(extruder_offset[X_AXIS][1], X2_MAX_POS);
      return;
    }
    else if (dual_x_carriage_mode == DXC_DUPLICATION_MODE && active_extruder == 0) {
      current_position[X_AXIS] = base_home_pos(X_AXIS) + add_homing[X_AXIS];
      min_pos[X_AXIS] =          base_min_pos(X_AXIS) + add_homing[X_AXIS];
      max_pos[X_AXIS] =          min(base_max_pos(X_AXIS) + add_homing[X_AXIS],
                                  max(extruder_offset[X_AXIS][1], X2_MAX_POS) - duplicate_extruder_x_offset);
      return;
    }
  }
#endif
#ifdef SCARA
   float homeposition[3];
   char i;

   if (axis < 2)
   {

     for (i=0; i<3; i++)
     {
        homeposition[i] = base_home_pos(i);
     }
	// SERIAL_ECHOPGM("homeposition[x]= "); SERIAL_ECHO(homeposition[0]);
   //  SERIAL_ECHOPGM("homeposition[y]= "); SERIAL_ECHOLN(homeposition[1]);
   // Works out real Homeposition angles using inverse kinematics,
   // and calculates homing offset using forward kinematics
     calculate_delta(homeposition);

    // SERIAL_ECHOPGM("base Theta= "); SERIAL_ECHO(delta[X_AXIS]);
    // SERIAL_ECHOPGM(" base Psi+Theta="); SERIAL_ECHOLN(delta[Y_AXIS]);

     for (i=0; i<2; i++)
     {
        delta[i] -= add_homing[i];
     }

    // SERIAL_ECHOPGM("addhome X="); SERIAL_ECHO(add_homing[X_AXIS]);
	// SERIAL_ECHOPGM(" addhome Y="); SERIAL_ECHO(add_homing[Y_AXIS]);
    // SERIAL_ECHOPGM(" addhome Theta="); SERIAL_ECHO(delta[X_AXIS]);
    // SERIAL_ECHOPGM(" addhome Psi+Theta="); SERIAL_ECHOLN(delta[Y_AXIS]);

     calculate_SCARA_forward_Transform(delta);

    // SERIAL_ECHOPGM("Delta X="); SERIAL_ECHO(delta[X_AXIS]);
    // SERIAL_ECHOPGM(" Delta Y="); SERIAL_ECHOLN(delta[Y_AXIS]);

    current_position[axis] = delta[axis];

    // SCARA home positions are based on configuration since the actual limits are determined by the
    // inverse kinematic transform.
    min_pos[axis] =          base_min_pos(axis); // + (delta[axis] - base_home_pos(axis));
    max_pos[axis] =          base_max_pos(axis); // + (delta[axis] - base_home_pos(axis));
   }
   else
   {
      current_position[axis] = base_home_pos(axis) + add_homing[axis];
      min_pos[axis] =          base_min_pos(axis) + add_homing[axis];
      max_pos[axis] =          base_max_pos(axis) + add_homing[axis];
   }
#else
  current_position[axis] = base_home_pos(axis) + add_homing[axis];
  min_pos[axis] =          base_min_pos(axis) + add_homing[axis];
  max_pos[axis] =          base_max_pos(axis) + add_homing[axis];
#endif
}

#ifdef ENABLE_AUTO_BED_LEVELING
#ifdef AUTO_BED_LEVELING_GRID
static void set_bed_level_equation_lsq(double *plane_equation_coefficients)
{
    vector_3 planeNormal = vector_3(-plane_equation_coefficients[0], -plane_equation_coefficients[1], 1);
    planeNormal.debug("planeNormal");
    plan_bed_level_matrix = matrix_3x3::create_look_at(planeNormal);
    //bedLevel.debug("bedLevel");

    //plan_bed_level_matrix.debug("bed level before");
    //vector_3 uncorrected_position = plan_get_position_mm();
    //uncorrected_position.debug("position before");

    vector_3 corrected_position = plan_get_position();
//    corrected_position.debug("position after");
    current_position[X_AXIS] = corrected_position.x;
    current_position[Y_AXIS] = corrected_position.y;
    current_position[Z_AXIS] = corrected_position.z;

    // put the bed at 0 so we don't go below it.
    current_position[Z_AXIS] = zprobe_zoffset; // in the lsq we reach here after raising the extruder due to the loop structure

    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
}

#else // not AUTO_BED_LEVELING_GRID

static void set_bed_level_equation_3pts(float z_at_pt_1, float z_at_pt_2, float z_at_pt_3) {

    plan_bed_level_matrix.set_to_identity();

    vector_3 pt1 = vector_3(ABL_PROBE_PT_1_X, ABL_PROBE_PT_1_Y, z_at_pt_1);
    vector_3 pt2 = vector_3(ABL_PROBE_PT_2_X, ABL_PROBE_PT_2_Y, z_at_pt_2);
    vector_3 pt3 = vector_3(ABL_PROBE_PT_3_X, ABL_PROBE_PT_3_Y, z_at_pt_3);

    vector_3 from_2_to_1 = (pt1 - pt2).get_normal();
    vector_3 from_2_to_3 = (pt3 - pt2).get_normal();
    vector_3 planeNormal = vector_3::cross(from_2_to_1, from_2_to_3).get_normal();
    planeNormal = vector_3(planeNormal.x, planeNormal.y, abs(planeNormal.z));

    plan_bed_level_matrix = matrix_3x3::create_look_at(planeNormal);

    vector_3 corrected_position = plan_get_position();
    current_position[X_AXIS] = corrected_position.x;
    current_position[Y_AXIS] = corrected_position.y;
    current_position[Z_AXIS] = corrected_position.z;

    // put the bed at 0 so we don't go below it.
    current_position[Z_AXIS] = zprobe_zoffset;

    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);

}

#endif // AUTO_BED_LEVELING_GRID

static void run_z_probe() {
    plan_bed_level_matrix.set_to_identity();
    feedrate = FEEDRATE_Z_C;

    // move down until you find the bed
    float zPosition = -10;
    plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], zPosition, current_position[E_AXIS], FEEDRATE_Z_C/60, active_extruder);
    st_synchronize();

        // we have to let the planner know where we are right now as it is not where we said to go.
    zPosition = st_get_position_mm(Z_AXIS);
    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], zPosition, current_position[E_AXIS]);

    // move up the retract distance
    zPosition += home_retract_mm(Z_AXIS);
    plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], zPosition, current_position[E_AXIS], FEEDRATE_Z_D/60, active_extruder);
    st_synchronize();

    // move back down slowly to find bed
    feedrate = homing_feedrate[Z_AXIS]/4;
    zPosition -= home_retract_mm(Z_AXIS) * 2;
    plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], zPosition, current_position[E_AXIS], FEEDRATE_Z_E/60, active_extruder);
    st_synchronize();

    current_position[Z_AXIS] = st_get_position_mm(Z_AXIS);
    // make sure the planner knows where we are as it may be a bit different than we last said to move to
    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
}

static void do_blocking_move_to(float x, float y, float z) {
    float oldFeedRate = feedrate;

    feedrate = FEEDRATE_Z_C;

    current_position[Z_AXIS] = z;
    plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
    st_synchronize();

    //feedrate = XY_TRAVEL_SPEED;
	feedrate =FEEDRATE_Z_C*5;
    current_position[X_AXIS] = x;
    current_position[Y_AXIS] = y;
    plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
    st_synchronize();

    feedrate = oldFeedRate;
}

static void do_blocking_move_relative(float offset_x, float offset_y, float offset_z) {
    do_blocking_move_to(current_position[X_AXIS] + offset_x, current_position[Y_AXIS] + offset_y, current_position[Z_AXIS] + offset_z);
}

static void setup_for_endstop_move() {
    saved_feedrate = feedrate;
    saved_feedmultiply = feedmultiply;
    feedmultiply = 100;
    previous_millis_cmd = millis();

    enable_endstops(true);
}

static void clean_up_after_endstop_move() {
#ifdef ENDSTOPS_ONLY_FOR_HOMING
    enable_endstops(false);
#endif

    feedrate = saved_feedrate;
    feedmultiply = saved_feedmultiply;
    previous_millis_cmd = millis();
}

 void engage_z_probe() {
    // Engage Z Servo endstop if enabled
    #ifdef SERVO_ENDSTOPS
    if (servo_endstops[Z_AXIS] > -1) {
#if defined (ENABLE_AUTO_BED_LEVELING) && (PROBE_SERVO_DEACTIVATION_DELAY > 0)
        servos[servo_endstops[Z_AXIS]].attach(0);
#endif
        servos[servo_endstops[Z_AXIS]].write(servo_endstop_angles[Z_AXIS * 2]);
#if defined (ENABLE_AUTO_BED_LEVELING) && (PROBE_SERVO_DEACTIVATION_DELAY > 0)
        delay(PROBE_SERVO_DEACTIVATION_DELAY);
        servos[servo_endstops[Z_AXIS]].detach();
#endif
    }
    #endif
}

 void retract_z_probe() {
    // Retract Z Servo endstop if enabled
    #ifdef SERVO_ENDSTOPS
    if (servo_endstops[Z_AXIS] > -1) {
#if defined (ENABLE_AUTO_BED_LEVELING) && (PROBE_SERVO_DEACTIVATION_DELAY > 0)
        servos[servo_endstops[Z_AXIS]].attach(0);
#endif
        servos[servo_endstops[Z_AXIS]].write(servo_endstop_angles[Z_AXIS * 2 + 1]);
#if defined (ENABLE_AUTO_BED_LEVELING) && (PROBE_SERVO_DEACTIVATION_DELAY > 0)
        delay(PROBE_SERVO_DEACTIVATION_DELAY);
        servos[servo_endstops[Z_AXIS]].detach();
#endif
    }
    #endif
}

/// Probe bed height at position (x,y), returns the measured z value
static float probe_pt(float x, float y, float z_before, int retract_action=0) {
  // move to right place
  do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], z_before);
  do_blocking_move_to(x - X_PROBE_OFFSET_FROM_EXTRUDER, y - Y_PROBE_OFFSET_FROM_EXTRUDER, current_position[Z_AXIS]);
	//tft_pos_qt_set(floor((X_MAX_POS-destination[X_AXIS])/X_MAX_POS*(qtx_max-qtx_min)+qtx_min),floor(destination[Y_AXIS]/Y_MAX_POS*(qty_max-qty_min)+qty_min));
	//tft_feedrate_set(floor(feedrate));
	//tft_status_monitor_update();
#ifndef Z_PROBE_SLED
   if ((retract_action==0) || (retract_action==1))
     engage_z_probe();   // Engage Z Servo endstop if available
#endif // Z_PROBE_SLED
  run_z_probe();
  float measured_z = current_position[Z_AXIS];
#ifndef Z_PROBE_SLED
  if ((retract_action==0) || (retract_action==3))
     retract_z_probe();
#endif // Z_PROBE_SLED

  SERIAL_PROTOCOLPGM(MSG_BED);
  SERIAL_PROTOCOLPGM(" x: ");
  SERIAL_PROTOCOL(x);
  SERIAL_PROTOCOLPGM(" y: ");
  SERIAL_PROTOCOL(y);
  SERIAL_PROTOCOLPGM(" z: ");
  SERIAL_PROTOCOL(measured_z);
  SERIAL_PROTOCOLPGM("\n");
  return measured_z;
}

#endif // #ifdef ENABLE_AUTO_BED_LEVELING

static void homeaxis(int axis) {
#define HOMEAXIS_DO(LETTER) \
  ((LETTER##_MIN_PIN > -1 && LETTER##_HOME_DIR==-1) || (LETTER##_MAX_PIN > -1 && LETTER##_HOME_DIR==1))

  if (axis==X_AXIS ? HOMEAXIS_DO(X) :
      axis==Y_AXIS ? HOMEAXIS_DO(Y) :
      axis==Z_AXIS ? HOMEAXIS_DO(Z) :
      0) {
    int axis_home_dir = home_dir(axis);
#ifdef DUAL_X_CARRIAGE
    if (axis == X_AXIS)
      axis_home_dir = x_home_dir(active_extruder);
#endif

    current_position[axis] = 0;
    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);


#ifndef Z_PROBE_SLED
    // Engage Servo endstop if enabled
    #ifdef SERVO_ENDSTOPS
      #if defined (ENABLE_AUTO_BED_LEVELING) && (PROBE_SERVO_DEACTIVATION_DELAY > 0)
        if (axis==Z_AXIS) {
          engage_z_probe();
        }
	    else
      #endif
      if (servo_endstops[axis] > -1) {
        servos[servo_endstops[axis]].write(servo_endstop_angles[axis * 2]);
      }
    #endif
#endif // Z_PROBE_SLED
    destination[axis] = 1.5 * max_length(axis) * axis_home_dir;
    feedrate = FEEDRATE_Z_B;
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
    st_synchronize();

    current_position[axis] = 0;
    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
    destination[axis] = -home_retract_mm(axis) * axis_home_dir;
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
    st_synchronize();

    destination[axis] = 2*home_retract_mm(axis) * axis_home_dir;
#ifdef DELTA
    feedrate = homing_feedrate[axis]/10;
#else
    feedrate = FEEDRATE_Z_B ;
#endif
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
    st_synchronize();
#ifdef DELTA
    // retrace by the amount specified in endstop_adj
    if (endstop_adj[axis] * axis_home_dir < 0) {
      plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
      destination[axis] = endstop_adj[axis];
      plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
      st_synchronize();
    }
#endif
    axis_is_at_home(axis);
    destination[axis] = current_position[axis];
    feedrate = 0.0;
    endstops_hit_on_purpose();
    axis_known_position[axis] = true;

    // Retract Servo endstop if enabled
    #ifdef SERVO_ENDSTOPS
      if (servo_endstops[axis] > -1) {
        servos[servo_endstops[axis]].write(servo_endstop_angles[axis * 2 + 1]);
      }
    #endif
#if defined (ENABLE_AUTO_BED_LEVELING) && (PROBE_SERVO_DEACTIVATION_DELAY > 0)
  #ifndef Z_PROBE_SLED
    if (axis==Z_AXIS) retract_z_probe();
  #endif
#endif

  }
}
#define HOMEAXIS(LETTER) homeaxis(LETTER##_AXIS)

#ifdef FWRETRACT
  void retract(bool retracting, bool swapretract = false) {
    if(retracting && !retracted[active_extruder]) {
      destination[X_AXIS]=current_position[X_AXIS];
      destination[Y_AXIS]=current_position[Y_AXIS];
      destination[Z_AXIS]=current_position[Z_AXIS];
      destination[E_AXIS]=current_position[E_AXIS];
      if (swapretract) {
        current_position[E_AXIS]+=retract_length_swap/volumetric_multiplier[active_extruder];
      } else {
        current_position[E_AXIS]+=retract_length/volumetric_multiplier[active_extruder];
      }
      plan_set_e_position(current_position[E_AXIS]);
      float oldFeedrate = feedrate;
      feedrate=retract_feedrate*60;
      retracted[active_extruder]=true;
      prepare_move();
      if(retract_zlift > 0.01) {
         current_position[Z_AXIS]-=retract_zlift;
#ifdef DELTA
         calculate_delta(current_position); // change cartesian kinematic to  delta kinematic;
         plan_set_position(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], current_position[E_AXIS]);
#else
         plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
#endif
         prepare_move();
      }
      feedrate = oldFeedrate;
    } else if(!retracting && retracted[active_extruder]) {
      destination[X_AXIS]=current_position[X_AXIS];
      destination[Y_AXIS]=current_position[Y_AXIS];
      destination[Z_AXIS]=current_position[Z_AXIS];
      destination[E_AXIS]=current_position[E_AXIS];
      if(retract_zlift > 0.01) {
         current_position[Z_AXIS]+=retract_zlift;
#ifdef DELTA
         calculate_delta(current_position); // change cartesian kinematic  to  delta kinematic;
         plan_set_position(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], current_position[E_AXIS]);
#else
         plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
#endif
         //prepare_move();
      }
      if (swapretract) {
        current_position[E_AXIS]-=(retract_length_swap+retract_recover_length_swap)/volumetric_multiplier[active_extruder];
      } else {
        current_position[E_AXIS]-=(retract_length+retract_recover_length)/volumetric_multiplier[active_extruder];
      }
      plan_set_e_position(current_position[E_AXIS]);
      float oldFeedrate = feedrate;
      feedrate=retract_recover_feedrate*60;
      retracted[active_extruder]=false;
      prepare_move();
      feedrate = oldFeedrate;
    }
  } //retract
#endif //FWRETRACT

#ifdef Z_PROBE_SLED
//
// Method to dock/undock a sled designed by Charles Bell.
//
// dock[in]     If true, move to MAX_X and engage the electromagnet
// offset[in]   The additional distance to move to adjust docking location
//
static void dock_sled(bool dock, int offset=0) {
 int z_loc;

 if (!((axis_known_position[X_AXIS]) && (axis_known_position[Y_AXIS]))) {
     //zct:LCD_MESSAGEPGM(MSG_POSITION_UNKNOWN);
   SERIAL_ECHO_START;
   SERIAL_ECHOLNPGM(MSG_POSITION_UNKNOWN);
   return;
 }

 if (dock) {
   do_blocking_move_to(X_MAX_POS + SLED_DOCKING_OFFSET + offset,
                       current_position[Y_AXIS],
                       current_position[Z_AXIS]);
   // turn off magnet
   digitalWrite(SERVO0_PIN, LOW);
 } else {
   if (current_position[Z_AXIS] < (Z_RAISE_BEFORE_PROBING + 5))
     z_loc = Z_RAISE_BEFORE_PROBING;
   else
     z_loc = current_position[Z_AXIS];
   do_blocking_move_to(X_MAX_POS + SLED_DOCKING_OFFSET + offset,
                       Y_PROBE_OFFSET_FROM_EXTRUDER, z_loc);
   // turn on magnet
   digitalWrite(SERVO0_PIN, HIGH);
 }
}
#endif
void w_h_three_piont_leveling_3_17()
{
	#ifdef ENABLE_AUTO_BED_LEVELING
	plan_bed_level_matrix.set_to_identity();  //Reset the plane ("erase" all leveling data)
	#endif //ENABLE_AUTO_BED_LEVELING

	saved_feedrate = feedrate;
	saved_feedmultiply = feedmultiply;
	feedmultiply = 100;
	previous_millis_cmd = millis();

	enable_endstops(true);

	for(int8_t i=0; i < NUM_AXIS; i++) {
		destination[i] = current_position[i];
	}
	feedrate = 0.0;

	#ifdef DELTA
	// A delta can only safely home all axis at the same time
	// all axis have to home at the same time

	// Move all carriages up together until the first endstop is hit.
	current_position[X_AXIS] = 0;
	current_position[Y_AXIS] = 0;
	current_position[Z_AXIS] = 0;
	plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);

	destination[X_AXIS] = 3 * Z_MAX_LENGTH;
	destination[Y_AXIS] = 3 * Z_MAX_LENGTH;
	destination[Z_AXIS] = 3 * Z_MAX_LENGTH;
	feedrate = 1.732 * homing_feedrate[X_AXIS];
	plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
	st_synchronize();
	endstops_hit_on_purpose();

	current_position[X_AXIS] = destination[X_AXIS];
	current_position[Y_AXIS] = destination[Y_AXIS];
	current_position[Z_AXIS] = destination[Z_AXIS];

	// take care of back off and rehome now we are all at the top
	HOMEAXIS(X);
	HOMEAXIS(Y);
	HOMEAXIS(Z);

	calculate_delta(current_position);
	plan_set_position(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], current_position[E_AXIS]);

	#else // NOT DELTA

	home_all_axis = !((code_seen(axis_codes[X_AXIS])) || (code_seen(axis_codes[Y_AXIS])) || (code_seen(axis_codes[Z_AXIS])));

	#if Z_HOME_DIR > 0                      // If homing away from BED do Z first
	if((home_all_axis) || (code_seen(axis_codes[Z_AXIS]))) {
		HOMEAXIS(Z);
	}
	#endif

	#ifdef QUICK_HOME
	if((home_all_axis)||( code_seen(axis_codes[X_AXIS]) && code_seen(axis_codes[Y_AXIS])) )  //first diagonal move
	{
		current_position[X_AXIS] = 0;current_position[Y_AXIS] = 0;

		#ifndef DUAL_X_CARRIAGE
		int x_axis_home_dir = home_dir(X_AXIS);
		#else
		int x_axis_home_dir = x_home_dir(active_extruder);
		extruder_duplication_enabled = false;
		#endif

		plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
		destination[X_AXIS] = 1.5 * max_length(X_AXIS) * x_axis_home_dir;destination[Y_AXIS] = 1.5 * max_length(Y_AXIS) * home_dir(Y_AXIS);
		feedrate = homing_feedrate[X_AXIS];
		if(homing_feedrate[Y_AXIS]<feedrate)
		feedrate = homing_feedrate[Y_AXIS];
		if (max_length(X_AXIS) > max_length(Y_AXIS)) {
			feedrate *= sqrt(pow(max_length(Y_AXIS) / max_length(X_AXIS), 2) + 1);
			} else {
			feedrate *= sqrt(pow(max_length(X_AXIS) / max_length(Y_AXIS), 2) + 1);
		}
		plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
		st_synchronize();

		axis_is_at_home(X_AXIS);
		axis_is_at_home(Y_AXIS);
		plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
		destination[X_AXIS] = current_position[X_AXIS];
		destination[Y_AXIS] = current_position[Y_AXIS];
		plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
		feedrate = 0.0;
		st_synchronize();
		endstops_hit_on_purpose();

		current_position[X_AXIS] = destination[X_AXIS];
		current_position[Y_AXIS] = destination[Y_AXIS];
		#ifndef SCARA
		current_position[Z_AXIS] = destination[Z_AXIS];
		#endif
	}
	#endif

	if((home_all_axis) || (code_seen(axis_codes[X_AXIS])))
	{
		#ifdef DUAL_X_CARRIAGE
		int tmp_extruder = active_extruder;
		extruder_duplication_enabled = false;
		active_extruder = !active_extruder;
		HOMEAXIS(X);
		inactive_extruder_x_pos = current_position[X_AXIS];
		active_extruder = tmp_extruder;
		HOMEAXIS(X);
		// reset state used by the different modes
		memcpy(raised_parked_position, current_position, sizeof(raised_parked_position));
		delayed_move_time = 0;
		active_extruder_parked = true;
		#else
		HOMEAXIS(X);
		#endif
	}

	if((home_all_axis) || (code_seen(axis_codes[Y_AXIS]))) {
		HOMEAXIS(Y);
	}

	if(code_seen(axis_codes[X_AXIS]))
	{
		if(code_value_long() != 0) {
			#ifdef SCARA
			current_position[X_AXIS]=code_value();
			#else
			current_position[X_AXIS]=code_value()+add_homing[X_AXIS];
			#endif
		}
	}

	if(code_seen(axis_codes[Y_AXIS])) {
		if(code_value_long() != 0) {
			#ifdef SCARA
			current_position[Y_AXIS]=code_value();
			#else
			current_position[Y_AXIS]=code_value()+add_homing[Y_AXIS];
			#endif
		}
	}

	#if Z_HOME_DIR < 0                      // If homing towards BED do Z last
	#ifndef Z_SAFE_HOMING
	if((home_all_axis) || (code_seen(axis_codes[Z_AXIS]))) {
		#if defined (Z_RAISE_BEFORE_HOMING) && (Z_RAISE_BEFORE_HOMING > 0)
		destination[Z_AXIS] = Z_RAISE_BEFORE_HOMING * home_dir(Z_AXIS) * (-1);    // Set destination away from bed
		feedrate = max_feedrate[Z_AXIS];
		plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate, active_extruder);
		st_synchronize();
		#endif
		HOMEAXIS(Z);
	}
	#else                      // Z Safe mode activated.
	if(home_all_axis) {
		destination[X_AXIS] = round(Z_SAFE_HOMING_X_POINT - X_PROBE_OFFSET_FROM_EXTRUDER);
		destination[Y_AXIS] = round(Z_SAFE_HOMING_Y_POINT - Y_PROBE_OFFSET_FROM_EXTRUDER);
		destination[Z_AXIS] = Z_RAISE_BEFORE_HOMING * home_dir(Z_AXIS) * (-1);    // Set destination away from bed
		feedrate = XY_TRAVEL_SPEED/60;
		current_position[Z_AXIS] = 0;

		plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
		plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate, active_extruder);
		st_synchronize();
		current_position[X_AXIS] = destination[X_AXIS];
		current_position[Y_AXIS] = destination[Y_AXIS];

		HOMEAXIS(Z);
	}
	// Let's see if X and Y are homed and probe is inside bed area.
	if(code_seen(axis_codes[Z_AXIS])) {
		if ( (axis_known_position[X_AXIS]) && (axis_known_position[Y_AXIS]) \
		&& (current_position[X_AXIS]+X_PROBE_OFFSET_FROM_EXTRUDER >= X_MIN_POS) \
		&& (current_position[X_AXIS]+X_PROBE_OFFSET_FROM_EXTRUDER <= X_MAX_POS) \
		&& (current_position[Y_AXIS]+Y_PROBE_OFFSET_FROM_EXTRUDER >= Y_MIN_POS) \
		&& (current_position[Y_AXIS]+Y_PROBE_OFFSET_FROM_EXTRUDER <= Y_MAX_POS)) {

			current_position[Z_AXIS] = 0;
			plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
			destination[Z_AXIS] = Z_RAISE_BEFORE_HOMING * home_dir(Z_AXIS) * (-1);    // Set destination away from bed
			feedrate = max_feedrate[Z_AXIS];
			plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate, active_extruder);
			st_synchronize();

			HOMEAXIS(Z);
			} else if (!((axis_known_position[X_AXIS]) && (axis_known_position[Y_AXIS]))) {
			  //zct:LCD_MESSAGEPGM(MSG_POSITION_UNKNOWN);
			SERIAL_ECHO_START;
			SERIAL_ECHOLNPGM(MSG_POSITION_UNKNOWN);
			} else {
			//zct:LCD_MESSAGEPGM(MSG_ZPROBE_OUT);
			SERIAL_ECHO_START;
			SERIAL_ECHOLNPGM(MSG_ZPROBE_OUT);
		}
	}
	#endif
	#endif

	if(code_seen(axis_codes[Z_AXIS])) {
		if(code_value_long() != 0) {
			current_position[Z_AXIS]=code_value()+add_homing[Z_AXIS];
		}
	}
	#ifdef ENABLE_AUTO_BED_LEVELING
	if((home_all_axis) || (code_seen(axis_codes[Z_AXIS]))) {
		current_position[Z_AXIS] += zprobe_zoffset;  //Add Z_Probe offset (the distance is negative)
	}
	#endif
	plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
	#endif // else DELTA

	#ifdef SCARA
	calculate_delta(current_position);
	plan_set_position(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], current_position[E_AXIS]);
	#endif // SCARA

	#ifdef ENDSTOPS_ONLY_FOR_HOMING
	enable_endstops(false);
	#endif

	feedrate = saved_feedrate;
	feedmultiply = saved_feedmultiply;
	previous_millis_cmd = millis();
	endstops_hit_on_purpose();

}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//								name :process_commands()
//								funtion闂備浇娉曢崰鎰板几婵犳艾绠柨鐕傛嫹 execute gcode from uart0 for : 1: ready signal 闂備浇娉曢崰鎰板几婵犳艾绠柨鐕傛嫹 2闂備浇娉曢崰鎰板几婵犳艾绠紒灞绢劕inting status , 3: printing code
//								author: Mr. whocares
//								comments: zct 2015/07/04
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void process_commands()
{
  unsigned long codenum; //throw away variable
  char *starpos = NULL;
#ifdef ENABLE_AUTO_BED_LEVELING
  float x_tmp, y_tmp, z_tmp, real_z;
#endif
  if(code_seen('G'))
  {
  //  START_SOUND();
    switch((int)code_value())
    {
    case 0: // G0 -> G1
    case 1: // G1 Coordinated Movement X Y Z E
      if(Stopped == false) {
        get_coordinates(); // For X Y Z E F
          #ifdef FWRETRACT
            if(autoretract_enabled)
            if( !(code_seen('X') || code_seen('Y') || code_seen('Z')) && code_seen('E')) {
              float echange=destination[E_AXIS]-current_position[E_AXIS];
              if((echange<-MIN_RETRACT && !retracted) || (echange>MIN_RETRACT && retracted)) { //move appears to be an attempt to retract or recover
                  current_position[E_AXIS] = destination[E_AXIS]; //hide the slicer-generated retract/recover from calculations
                  plan_set_e_position(current_position[E_AXIS]); //AND from the planner
                  retract(!retracted);
                  return;
              }
            }
          #endif //FWRETRACT
        prepare_move();
        //ClearToSend();
      }
      break;
#ifndef SCARA //disable arc support
    case 2: // G2  - CW ARC
   //  EXTRUDER_HEAT_SOUND();
      if(Stopped == false) {
        get_arc_coordinates();
        prepare_arc_move(true);
      }
      break;
    case 3: // G3  - CCW ARC
      if(Stopped == false) {
        get_arc_coordinates();
        prepare_arc_move(false);
      }
      break;
#endif
    case 4: // G4 dwell
        //zct:LCD_MESSAGEPGM(MSG_DWELL);
      codenum = 0;
      if(code_seen('P')) codenum = code_value(); // milliseconds to wait
      if(code_seen('S')) codenum = code_value() * 1000; // seconds to wait

      st_synchronize();
      codenum += millis();  // keep track of when we started waiting
      previous_millis_cmd = millis();
      while(millis() < codenum) {
        manage_heater();
        manage_inactivity();
        //zct: lcd_update();
      }
      break;
    case 8:
      EXTRUDER_HEAT_SOUND();
      break;
      #ifdef FWRETRACT
    case 10: // G10 retract
       #if EXTRUDERS > 1
        retracted_swap[active_extruder]=(code_seen('S') && code_value_long() == 1); // checks for swap retract argument
        retract(true,retracted_swap[active_extruder]);
       #else
        retract(true);
       #endif
      break;
    case 11: // G11 retract_recover
       #if EXTRUDERS > 1
        retract(false,retracted_swap[active_extruder]);
       #else
        retract(false);
       #endif
      break;
      #endif //FWRETRACT
    case 28: //G28 Home all Axis one at a time
    BEGIN_PRINT_SOUND();
    w_h_EE_check=0;
#ifdef ENABLE_AUTO_BED_LEVELING
      plan_bed_level_matrix.set_to_identity();  //Reset the plane ("erase" all leveling data)
#endif //ENABLE_AUTO_BED_LEVELING

      saved_feedrate = feedrate;
      saved_feedmultiply = feedmultiply;
      feedmultiply = 100;
      previous_millis_cmd = millis();

      enable_endstops(true);

      for(int8_t i=0; i < NUM_AXIS; i++) {
        destination[i] = current_position[i];
      }
      feedrate = 0.0;

#ifdef DELTA
          // A delta can only safely home all axis at the same time
          // all axis have to home at the same time
          // Move all carriages up together until the first endstop is hit.
          current_position[X_AXIS] = 0;
          current_position[Y_AXIS] = 0;
          current_position[Z_AXIS] = 0;
          plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);

          destination[X_AXIS] = 3 * Z_MAX_LENGTH;
          destination[Y_AXIS] = 3 * Z_MAX_LENGTH;
          destination[Z_AXIS] = 3 * Z_MAX_LENGTH;
          feedrate = 1.732 * homing_feedrate[X_AXIS];
          plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
          st_synchronize();
          endstops_hit_on_purpose();
          current_position[X_AXIS] = destination[X_AXIS];
          current_position[Y_AXIS] = destination[Y_AXIS];
          current_position[Z_AXIS] = destination[Z_AXIS];

          // take care of back off and rehome now we are all at the top
          HOMEAXIS(X);
          HOMEAXIS(Y);
          HOMEAXIS(Z);

          calculate_delta(current_position);
          plan_set_position(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], current_position[E_AXIS]);

#else // NOT DELTA

      home_all_axis = !((code_seen(axis_codes[X_AXIS])) || (code_seen(axis_codes[Y_AXIS])) || (code_seen(axis_codes[Z_AXIS])));
      #if Z_HOME_DIR > 0                      // If homing away from BED do Z first
      if((home_all_axis) || (code_seen(axis_codes[Z_AXIS]))) {
        HOMEAXIS(Z);
      }
      #endif

      if((home_all_axis) || (code_seen(axis_codes[X_AXIS])))
      {
      #ifdef DUAL_X_CARRIAGE
        int tmp_extruder = active_extruder;
        extruder_duplication_enabled = false;
        active_extruder = !active_extruder;
        HOMEAXIS(X);
        inactive_extruder_x_pos = current_position[X_AXIS];
        active_extruder = tmp_extruder;
        HOMEAXIS(X);
        // reset state used by the different modes
        memcpy(raised_parked_position, current_position, sizeof(raised_parked_position));
        delayed_move_time = 0;
        active_extruder_parked = true;
      #else
        HOMEAXIS(X);
      #endif
      }

      if((home_all_axis) || (code_seen(axis_codes[Y_AXIS]))) {
        HOMEAXIS(Y);
      }

      if(code_seen(axis_codes[X_AXIS]))
      {
        if(code_value_long() != 0) {
		#ifdef SCARA
		   current_position[X_AXIS]=code_value();
		#else
		   current_position[X_AXIS]=code_value()+add_homing[X_AXIS];
		#endif
        }
      }

      if(code_seen(axis_codes[Y_AXIS])) {
        if(code_value_long() != 0) {
         #ifdef SCARA
		   current_position[Y_AXIS]=code_value();
		#else
		   current_position[Y_AXIS]=code_value()+add_homing[Y_AXIS];
		#endif
        }
      }

      #if Z_HOME_DIR < 0                      // If homing towards BED do Z last
        #ifndef Z_SAFE_HOMING
          if((home_all_axis) || (code_seen(axis_codes[Z_AXIS]))) {
            #if defined (Z_RAISE_BEFORE_HOMING) && (Z_RAISE_BEFORE_HOMING > 0)
              destination[Z_AXIS] = Z_RAISE_BEFORE_HOMING * home_dir(Z_AXIS) * (-1);    // Set destination away from bed
              feedrate = max_feedrate[Z_AXIS];
              plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate, active_extruder);
              st_synchronize();
            #endif
            HOMEAXIS(Z);
          }
        #else                      // Z Safe mode activated.
          if(home_all_axis) {
            destination[X_AXIS] = round(Z_SAFE_HOMING_X_POINT - X_PROBE_OFFSET_FROM_EXTRUDER);
            destination[Y_AXIS] = round(Z_SAFE_HOMING_Y_POINT - Y_PROBE_OFFSET_FROM_EXTRUDER);
            destination[Z_AXIS] = Z_RAISE_BEFORE_HOMING * home_dir(Z_AXIS) * (-1);    // Set destination away from bed
            feedrate = XY_TRAVEL_SPEED/2/60;
            current_position[Z_AXIS] = 0;

            plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
            plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate, active_extruder);
            st_synchronize();
            current_position[X_AXIS] = destination[X_AXIS];
            current_position[Y_AXIS] = destination[Y_AXIS];

            HOMEAXIS(Z);
          }
                                                // Let's see if X and Y are homed and probe is inside bed area.
          if(code_seen(axis_codes[Z_AXIS])) {
            if ( (axis_known_position[X_AXIS]) && (axis_known_position[Y_AXIS]) \
              && (current_position[X_AXIS]+X_PROBE_OFFSET_FROM_EXTRUDER >= X_MIN_POS) \
              && (current_position[X_AXIS]+X_PROBE_OFFSET_FROM_EXTRUDER <= X_MAX_POS) \
              && (current_position[Y_AXIS]+Y_PROBE_OFFSET_FROM_EXTRUDER >= Y_MIN_POS) \
              && (current_position[Y_AXIS]+Y_PROBE_OFFSET_FROM_EXTRUDER <= Y_MAX_POS)) {

              current_position[Z_AXIS] = 0;
              plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
              destination[Z_AXIS] = Z_RAISE_BEFORE_HOMING * home_dir(Z_AXIS) * (-1);    // Set destination away from bed
              feedrate = max_feedrate[Z_AXIS];
              plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate, active_extruder);
              st_synchronize();

              HOMEAXIS(Z);
            } else if (!((axis_known_position[X_AXIS]) && (axis_known_position[Y_AXIS]))) {
                  //zct:LCD_MESSAGEPGM(MSG_POSITION_UNKNOWN);
                SERIAL_ECHO_START;
                SERIAL_ECHOLNPGM(MSG_POSITION_UNKNOWN);
            } else {
                  //zct:LCD_MESSAGEPGM(MSG_ZPROBE_OUT);
                SERIAL_ECHO_START;
                SERIAL_ECHOLNPGM(MSG_ZPROBE_OUT);
            }
          }
        #endif
      #endif

      if(code_seen(axis_codes[Z_AXIS])) {
        if(code_value_long() != 0) {
          current_position[Z_AXIS]=code_value()+add_homing[Z_AXIS];
        }
      }
      #ifdef ENABLE_AUTO_BED_LEVELING
        if((home_all_axis) || (code_seen(axis_codes[Z_AXIS]))) {
          current_position[Z_AXIS] += zprobe_zoffset;  //Add Z_Probe offset (the distance is negative)
        }
      #endif
      plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
#endif // else DELTA

#ifdef SCARA
	  calculate_delta(current_position);
      plan_set_position(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], current_position[E_AXIS]);
#endif // SCARA

      #ifdef ENDSTOPS_ONLY_FOR_HOMING
        enable_endstops(false);
      #endif

      feedrate = saved_feedrate;
      feedmultiply = saved_feedmultiply;
      previous_millis_cmd = millis();
      endstops_hit_on_purpose();
      break;

#ifdef ENABLE_AUTO_BED_LEVELING
    case 29: // G29 Detailed Z-Probe, probes the bed at 3 or more points.
        {
            #if Z_MIN_PIN == -1
            #error "You must have a Z_MIN endstop in order to enable Auto Bed Leveling feature!!! Z_MIN_PIN must point to a valid hardware pin."
            #endif

            // Prevent user from running a G29 without first homing in X and Y
            if (! (axis_known_position[X_AXIS] && axis_known_position[Y_AXIS]) )
            {
                  //zct:LCD_MESSAGEPGM(MSG_POSITION_UNKNOWN);                SERIAL_ECHO_START;
                SERIAL_ECHOLNPGM(MSG_POSITION_UNKNOWN);
                break; // abort G29, since we don't know where we are
            }

#ifdef Z_PROBE_SLED
            dock_sled(false);
#endif // Z_PROBE_SLED
            st_synchronize();
            // make sure the bed_level_rotation_matrix is identity or the planner will get it incorectly
            //vector_3 corrected_position = plan_get_position_mm();
            //corrected_position.debug("position before G29");
            plan_bed_level_matrix.set_to_identity();
            vector_3 uncorrected_position = plan_get_position();
            //uncorrected_position.debug("position durring G29");
            current_position[X_AXIS] = uncorrected_position.x;
            current_position[Y_AXIS] = uncorrected_position.y;
            current_position[Z_AXIS] = uncorrected_position.z;
            plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
            setup_for_endstop_move();

            feedrate = FEEDRATE_Z_C;
#ifdef AUTO_BED_LEVELING_GRID
            // probe at the points of a lattice grid

            int xGridSpacing = (RIGHT_PROBE_BED_POSITION - LEFT_PROBE_BED_POSITION) / (AUTO_BED_LEVELING_GRID_POINTS-1);
            int yGridSpacing = (BACK_PROBE_BED_POSITION - FRONT_PROBE_BED_POSITION) / (AUTO_BED_LEVELING_GRID_POINTS-1);


            // solve the plane equation ax + by + d = z
            // A is the matrix with rows [x y 1] for all the probed points
            // B is the vector of the Z positions
            // the normal vector to the plane is formed by the coefficients of the plane equation in the standard form, which is Vx*x+Vy*y+Vz*z+d = 0
            // so Vx = -a Vy = -b Vz = 1 (we want the vector facing towards positive Z

            // "A" matrix of the linear system of equations
            double eqnAMatrix[AUTO_BED_LEVELING_GRID_POINTS*AUTO_BED_LEVELING_GRID_POINTS*3];
            // "B" vector of Z points
            double eqnBVector[AUTO_BED_LEVELING_GRID_POINTS*AUTO_BED_LEVELING_GRID_POINTS];


            int probePointCounter = 0;
            bool zig = true;

            for (int yProbe=FRONT_PROBE_BED_POSITION; yProbe <= BACK_PROBE_BED_POSITION; yProbe += yGridSpacing)
            {
              int xProbe, xInc;
              if (zig)
              {
                xProbe = LEFT_PROBE_BED_POSITION;
                //xEnd = RIGHT_PROBE_BED_POSITION;
                xInc = xGridSpacing;
                zig = false;
              } else // zag
              {
                xProbe = RIGHT_PROBE_BED_POSITION;
                //xEnd = LEFT_PROBE_BED_POSITION;
                xInc = -xGridSpacing;
                zig = true;
              }

              for (int xCount=0; xCount < AUTO_BED_LEVELING_GRID_POINTS; xCount++)
              {
                float z_before;
                if (probePointCounter == 0)
                {
                  // raise before probing
                  z_before = Z_RAISE_BEFORE_PROBING;
                } else
                {
                  // raise extruder
                  z_before = current_position[Z_AXIS] + Z_RAISE_BETWEEN_PROBINGS;
                }

                float measured_z;
                //Enhanced G29 - Do not retract servo between probes
                if (code_seen('E') || code_seen('e') )
                   {
                   if ((yProbe==FRONT_PROBE_BED_POSITION) && (xCount==0))
                       {
                        measured_z = probe_pt(xProbe, yProbe, z_before,1);
                       } else if ((yProbe==FRONT_PROBE_BED_POSITION + (yGridSpacing * (AUTO_BED_LEVELING_GRID_POINTS-1))) && (xCount == AUTO_BED_LEVELING_GRID_POINTS-1))
                         {
                         measured_z = probe_pt(xProbe, yProbe, z_before,3);
                         } else {
                           measured_z = probe_pt(xProbe, yProbe, z_before,2);
                         }
                    } else {
                    measured_z = probe_pt(xProbe, yProbe, z_before);
                    }

                eqnBVector[probePointCounter] = measured_z;

                eqnAMatrix[probePointCounter + 0*AUTO_BED_LEVELING_GRID_POINTS*AUTO_BED_LEVELING_GRID_POINTS] = xProbe;
                eqnAMatrix[probePointCounter + 1*AUTO_BED_LEVELING_GRID_POINTS*AUTO_BED_LEVELING_GRID_POINTS] = yProbe;
                eqnAMatrix[probePointCounter + 2*AUTO_BED_LEVELING_GRID_POINTS*AUTO_BED_LEVELING_GRID_POINTS] = 1;
                probePointCounter++;
                xProbe += xInc;
              }
            }
            clean_up_after_endstop_move();

            // solve lsq problem
            double *plane_equation_coefficients = qr_solve(AUTO_BED_LEVELING_GRID_POINTS*AUTO_BED_LEVELING_GRID_POINTS, 3, eqnAMatrix, eqnBVector);

            SERIAL_PROTOCOLPGM("Eqn coefficients: a: ");
            SERIAL_PROTOCOL(plane_equation_coefficients[0]);
            SERIAL_PROTOCOLPGM(" b: ");
            SERIAL_PROTOCOL(plane_equation_coefficients[1]);
            SERIAL_PROTOCOLPGM(" d: ");
            SERIAL_PROTOCOLLN(plane_equation_coefficients[2]);


            set_bed_level_equation_lsq(plane_equation_coefficients);

            free(plane_equation_coefficients);

#else // AUTO_BED_LEVELING_GRID not defined

            // Probe at 3 arbitrary points
            // Enhanced G29

            float z_at_pt_1,z_at_pt_2,z_at_pt_3;

            if (code_seen('E') || code_seen('e') )
               {
               // probe 1
                z_at_pt_1 = probe_pt(ABL_PROBE_PT_1_X, ABL_PROBE_PT_1_Y, Z_RAISE_BEFORE_PROBING,1);
               // probe 2
                z_at_pt_2 = probe_pt(ABL_PROBE_PT_2_X, ABL_PROBE_PT_2_Y, current_position[Z_AXIS] + Z_RAISE_BETWEEN_PROBINGS,2);
               // probe 3
                z_at_pt_3 = probe_pt(ABL_PROBE_PT_3_X, ABL_PROBE_PT_3_Y, current_position[Z_AXIS] + Z_RAISE_BETWEEN_PROBINGS,3);
               }
               else
               {
	        // probe 1
	        float z_at_pt_1 = probe_pt(ABL_PROBE_PT_1_X, ABL_PROBE_PT_1_Y, Z_RAISE_BEFORE_PROBING);

                // probe 2
                float z_at_pt_2 = probe_pt(ABL_PROBE_PT_2_X, ABL_PROBE_PT_2_Y, current_position[Z_AXIS] + Z_RAISE_BETWEEN_PROBINGS);

                // probe 3
                float z_at_pt_3 = probe_pt(ABL_PROBE_PT_3_X, ABL_PROBE_PT_3_Y, current_position[Z_AXIS] + Z_RAISE_BETWEEN_PROBINGS);
               }
            clean_up_after_endstop_move();

            set_bed_level_equation_3pts(z_at_pt_1, z_at_pt_2, z_at_pt_3);


#endif // AUTO_BED_LEVELING_GRID
            st_synchronize();

            // The following code correct the Z height difference from z-probe position and hotend tip position.
            // The Z height on homing is measured by Z-Probe, but the probe is quite far from the hotend.
            // When the bed is uneven, this height must be corrected.
            real_z = float(st_get_position(Z_AXIS))/axis_steps_per_unit[Z_AXIS];  //get the real Z (since the auto bed leveling is already correcting the plane)
            x_tmp = current_position[X_AXIS] + X_PROBE_OFFSET_FROM_EXTRUDER;
            y_tmp = current_position[Y_AXIS] + Y_PROBE_OFFSET_FROM_EXTRUDER;
            z_tmp = current_position[Z_AXIS];

            apply_rotation_xyz(plan_bed_level_matrix, x_tmp, y_tmp, z_tmp);         //Apply the correction sending the probe offset
            current_position[Z_AXIS] = z_tmp - real_z + current_position[Z_AXIS];   //The difference is added to current position and sent to planner.
            plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
#ifdef Z_PROBE_SLED
            dock_sled(true, -SLED_DOCKING_OFFSET); // correct for over travel.
#endif // Z_PROBE_SLED
        }
        break;
#ifndef Z_PROBE_SLED
    case 30: // G30 Single Z Probe
        {
            engage_z_probe(); // Engage Z Servo endstop if available
            st_synchronize();
            // TODO: make sure the bed_level_rotation_matrix is identity or the planner will get set incorectly
            setup_for_endstop_move();

            feedrate = homing_feedrate[Z_AXIS];

            run_z_probe();
            SERIAL_PROTOCOLPGM(MSG_BED);
            SERIAL_PROTOCOLPGM(" X: ");
            SERIAL_PROTOCOL(current_position[X_AXIS]);
            SERIAL_PROTOCOLPGM(" Y: ");
            SERIAL_PROTOCOL(current_position[Y_AXIS]);
            SERIAL_PROTOCOLPGM(" Z: ");
            SERIAL_PROTOCOL(current_position[Z_AXIS]);
            SERIAL_PROTOCOLPGM("\n");

            clean_up_after_endstop_move();
            retract_z_probe(); // Retract Z Servo endstop if available
        }
        break;
#else
    case 31: // dock the sled
        dock_sled(true);
        break;
    case 32: // undock the sled
        dock_sled(false);
        break;
#endif // Z_PROBE_SLED
#endif // ENABLE_AUTO_BED_LEVELING
    case 90: // G90
      relative_mode = false;
      break;
    case 91: // G91
      relative_mode = true;
      break;
    case 92: // G92
      if(!code_seen(axis_codes[E_AXIS]))
        st_synchronize();
      for(int8_t i=0; i < NUM_AXIS; i++)
      {
        if(code_seen(axis_codes[i])) {
           if(i == E_AXIS) {
             current_position[i] = code_value();
             plan_set_e_position(current_position[E_AXIS]);
           }
           else {
#ifdef SCARA
		if (i == X_AXIS || i == Y_AXIS) {
                	current_position[i] = code_value();
		}
		else {
                current_position[i] = code_value()+add_homing[i];
            	}
#else
		current_position[i] = code_value()+add_homing[i];
#endif
            plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
           }
        }
      }
      break;
      case 93: //G93 Preparetion for the power off resume
		
	z_value_offset=back_z_value.v;//+0.10;
	current_position[E_AXIS]=-65535;
	analogWrite(FAN_SAN_PIN, 225);
    fanSpeed=255;
        //plan_set_e_position(current_position[E_AXIS]);  // G92 E0;

ECHO("the offset value stored is:");
ECHO(z_value_offset);


	//   Home X/Y Axis one at a time //
       BEGIN_PRINT_SOUND();

      saved_feedrate = feedrate;
      saved_feedmultiply = feedmultiply;
      feedmultiply = 100;
      previous_millis_cmd = millis();

      enable_endstops(true);

      for(int8_t i=0; i < NUM_AXIS; i++) {
        if (i!=2)
        {
			destination[i] = current_position[i];
        }
      }
      feedrate = 0.0;

      home_all_axis = !((code_seen(axis_codes[X_AXIS])) || (code_seen(axis_codes[Y_AXIS])) || (code_seen(axis_codes[Z_AXIS])));

      if((home_all_axis) || (code_seen(axis_codes[X_AXIS])))
      {
        HOMEAXIS(X);
      }

      if((home_all_axis) || (code_seen(axis_codes[Y_AXIS]))) {
        HOMEAXIS(Y);
      }

      if(code_seen(axis_codes[X_AXIS]))
      {
        if(code_value_long() != 0) {
       current_position[X_AXIS]=code_value()+add_homing[X_AXIS];
        }
      }

      if(code_seen(axis_codes[Y_AXIS])) {
        if(code_value_long() != 0) {
       current_position[Y_AXIS]=code_value()+add_homing[Y_AXIS];
        }
      }
      current_position[Z_AXIS] =back_z_value.v;
      plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);




      #ifdef ENDSTOPS_ONLY_FOR_HOMING
        enable_endstops(false);
      #endif

      feedrate = saved_feedrate;
      feedmultiply = saved_feedmultiply;
      previous_millis_cmd = millis();
      endstops_hit_on_purpose();


      break;
    }
  }

  else if(code_seen('M'))
  {
    switch( (int)code_value() )
    {
#ifdef ULTIPANEL
    case 0: // M0 - Unconditional stop - Wait for user button press on LCD
    case 1: // M1 - Conditional stop - Wait for user button press on LCD
    {


	  char *src = strchr_pointer + 2;
      codenum = 0;

      bool hasP = false, hasS = false;
      if (code_seen('P')) {
        codenum = code_value(); // milliseconds to wait
        hasP = codenum > 0;
      }
      if (code_seen('S')) {
        codenum = code_value() * 1000; // seconds to wait
        hasS = codenum > 0;
      }
      starpos = strchr(src, '*');
      if (starpos != NULL) *(starpos) = '\0';
      while (*src == ' ') ++src;
      if (!hasP && !hasS && *src != '\0') {
        //ZCT: lcd_setstatus(src);
      } else {
          //zct:LCD_MESSAGEPGM(MSG_USERWAIT);
      }

      //zct: lcd_ignore_click();
      st_synchronize();
      previous_millis_cmd = millis();
      if (codenum > 0){
        codenum += millis();  // keep track of when we started waiting
        while(millis() < codenum){ //zct: && !lcd_clicked()){
          manage_heater();
          manage_inactivity();
          //zct: lcd_update();
        }
        //zct: lcd_ignore_click(false);
      }else{
          //zct: if (!lcd_detected())
          //zct:   break;
        //zct: while(!lcd_clicked()){
          manage_heater();
          manage_inactivity();
          //zct: lcd_update();
        //zct: }
      }
      //zct:if (IS_SD_PRINTING)
          //zct:LCD_MESSAGEPGM(MSG_RESUMING);
      //zct:else
          //zct:LCD_MESSAGEPGM(WELCOME_MSG);
    }

    break;
#endif
    case 17:
          //zct:LCD_MESSAGEPGM(MSG_NO_MOVE);
        enable_x();
        enable_y();
        enable_z();
        enable_e0();
        enable_e1();
        enable_e2();
      break;

#ifdef SDSUPPORT
    case 20: // M20 - list SD card
      SERIAL_PROTOCOLLNPGM(MSG_BEGIN_FILE_LIST);
      card.ls();
      SERIAL_PROTOCOLLNPGM(MSG_END_FILE_LIST);
      break;
    case 21: // M21 - init SD card

      card.initsd();

      break;
    case 22: //M22 - release SD card
      card.release();

      break;
    case 23: //M23 - Select file
      starpos = (strchr(strchr_pointer + 4,'*'));
      if(starpos!=NULL)
        *(starpos)='\0';
      card.openFile(strchr_pointer + 4,true);
      break;
    case 24: //M24 - Start SD print
      card.startFileprint();
      starttime=millis();
      break;
    case 25: //M25 - Pause SD print
      card.pauseSDPrint();
      break;
    case 26: //M26 - Set SD index
      if(card.cardOK && code_seen('S')) {
        card.setIndex(code_value_long());
      }
      break;
    case 27: //M27 - Get SD status
      card.getStatus();
      break;
    case 28: //M28 - Start SD write
      starpos = (strchr(strchr_pointer + 4,'*'));
      if(starpos != NULL){
        char* npos = strchr(cmdbuffer[bufindr], 'N');
        strchr_pointer = strchr(npos,' ') + 1;
        *(starpos) = '\0';
      }
      card.openFile(strchr_pointer+4,false);
      break;
    case 29: //M29 - Stop SD write
      //processed in write to file routine above
      //card,saving = false;
      break;
    case 30: //M30 <filename> Delete File
      if (card.cardOK){
        card.closefile();
        starpos = (strchr(strchr_pointer + 4,'*'));
        if(starpos != NULL){
          char* npos = strchr(cmdbuffer[bufindr], 'N');
          strchr_pointer = strchr(npos,' ') + 1;
          *(starpos) = '\0';
        }
        card.removeFile(strchr_pointer + 4);
      }
      break;
    case 32: //M32 - Select file and start SD print
    {
      if(card.sdprinting) {
        st_synchronize();
      }
      starpos = (strchr(strchr_pointer + 4,'*'));

      char* namestartpos = (strchr(strchr_pointer + 4,'!'));   //find ! to indicate filename string start.
      if(namestartpos==NULL)
      {
        namestartpos=strchr_pointer + 4; //default name position, 4 letters after the M
      }
      else
        namestartpos++; //to skip the '!'

      if(starpos!=NULL)
        *(starpos)='\0';

      bool call_procedure=(code_seen('P'));

      if(strchr_pointer>namestartpos)
        call_procedure=false;  //false alert, 'P' found within filename

      if( card.cardOK )
      {
        card.openFile(namestartpos,true,!call_procedure);
        if(code_seen('S'))
          if(strchr_pointer<namestartpos) //only if "S" is occuring _before_ the filename
            card.setIndex(code_value_long());
        card.startFileprint();
        if(!call_procedure)
          starttime=millis(); //procedure calls count as normal print time.
      }
    } break;
    case 928: //M928 - Start SD write
      starpos = (strchr(strchr_pointer + 5,'*'));
      if(starpos != NULL){
        char* npos = strchr(cmdbuffer[bufindr], 'N');
        strchr_pointer = strchr(npos,' ') + 1;
        *(starpos) = '\0';
      }
      card.openLogFile(strchr_pointer+5);
      break;

#endif //SDSUPPORT

    case 31: //M31 take time since the start of the SD print or an M109 command
      {
      stoptime=millis();
      char time[30];
      unsigned long t=(stoptime-starttime)/1000;
      int sec,min;
      min=t/60;
      sec=t%60;
      sprintf_P(time, PSTR("%i min, %i sec"), min, sec);
      SERIAL_ECHO_START;
      SERIAL_ECHOLN(time);
      //ZCT: lcd_setstatus(time);
      autotempShutdown();
      }
      break;
    case 42: //M42 -Change pin status via gcode
      if (code_seen('S'))
      {
        int pin_status = code_value();
        int pin_number = LED_PIN;
        if (code_seen('P') && pin_status >= 0 && pin_status <= 255)
          pin_number = code_value();
        for(int8_t i = 0; i < (int8_t)(sizeof(sensitive_pins)/sizeof(int)); i++)
        {
          if (sensitive_pins[i] == pin_number)
          {
            pin_number = -1;
            break;
          }
        }
      #if defined(FAN_PIN) && FAN_PIN > -1
        if (pin_number == FAN_PIN)
          fanSpeed = pin_status;
      #endif
        if (pin_number > -1)
        {
          pinMode(pin_number, OUTPUT);
          digitalWrite(pin_number, pin_status);
          analogWrite(pin_number, pin_status);
        }
      }
     break;
	 case 33://M33
	  digitalWrite(POWER_OFF,HIGH);
	  break;
         case 34://M34
                 break;
        case 35://M35 : stop printing : ZCT
		//enable_endstops(true);
					w_d_x_8_18_check=0;
					w_d_x_8_18_checke=0;
					card.sdprinting = false;
					card.closefile();									
		//									enquecommand_P(PSTR("G0 X10 Y10 Z180 F1000"));
					destination[2]=constrain(PLATE_BOT_POS-z_value_offset,0,PLATE_BOT_POS);
					feedrate=FEEDRATE_Z_G;
					//ECHO("endcheck is");  
					quickStop();
					autotempShutdown();
					cancel_heatup = true;
					setTargetHotend0(0);
					monitormode=false;
					enable_endstops(true);
					prepare_move();	
         break;

// M48 Z-Probe repeatability measurement function.
//
// Usage:   M48 <n #_samples> <X X_position_for_samples> <Y Y_position_for_samples> <V Verbose_Level> <Engage_probe_for_each_reading> <L legs_of_movement_prior_to_doing_probe>
//
// This function assumes the bed has been homed.  Specificaly, that a G28 command
// as been issued prior to invoking the M48 Z-Probe repeatability measurement function.
// Any information generated by a prior G29 Bed leveling command will be lost and need to be
// regenerated.
//
// The number of samples will default to 10 if not specified.  You can use upper or lower case
// letters for any of the options EXCEPT n.  n must be in lower case because Marlin uses a capital
// N for its communication protocol and will get horribly confused if you send it a capital N.
//

#ifdef ENABLE_AUTO_BED_LEVELING
#ifdef Z_PROBE_REPEATABILITY_TEST

    case 48: // M48 Z-Probe repeatability
        {
            #if Z_MIN_PIN == -1
            #error "You must have a Z_MIN endstop in order to enable calculation of Z-Probe repeatability."
            #endif

	double sum=0.0;
	double mean=0.0;
	double sigma=0.0;
	double sample_set[50];
	int verbose_level=1, n=0, j, n_samples = 10, n_legs=0, engage_probe_for_each_reading=0 ;
	double X_current, Y_current, Z_current;
	double X_probe_location, Y_probe_location, Z_start_location, ext_position;

	if (code_seen('V') || code_seen('v')) {
        	verbose_level = code_value();
		if (verbose_level<0 || verbose_level>4 ) {
			SERIAL_PROTOCOLPGM("?Verbose Level not plausable.\n");
			goto Sigma_Exit;
		}
	}

	if (verbose_level > 0)   {
		SERIAL_PROTOCOLPGM("M48 Z-Probe Repeatability test.   Version 2.00\n");
		SERIAL_PROTOCOLPGM("Full support at: http://3dprintboard.com/forum.php\n");
	}

	if (code_seen('n')) {
        	n_samples = code_value();
		if (n_samples<4 || n_samples>50 ) {
			SERIAL_PROTOCOLPGM("?Specified sample size not plausable.\n");
			goto Sigma_Exit;
		}
	}

	X_current = X_probe_location = st_get_position_mm(X_AXIS);
	Y_current = Y_probe_location = st_get_position_mm(Y_AXIS);
	Z_current = st_get_position_mm(Z_AXIS);
	Z_start_location = st_get_position_mm(Z_AXIS) + Z_RAISE_BEFORE_PROBING;
	ext_position	 = st_get_position_mm(E_AXIS);

	if (code_seen('E') || code_seen('e') )
		engage_probe_for_each_reading++;

	if (code_seen('X') || code_seen('x') ) {
        	X_probe_location = code_value() -  X_PROBE_OFFSET_FROM_EXTRUDER;
		if (X_probe_location<X_MIN_POS || X_probe_location>X_MAX_POS ) {
			SERIAL_PROTOCOLPGM("?Specified X position out of range.\n");
			goto Sigma_Exit;
		}
	}

	if (code_seen('Y') || code_seen('y') ) {
        	Y_probe_location = code_value() -  Y_PROBE_OFFSET_FROM_EXTRUDER;
		if (Y_probe_location<Y_MIN_POS || Y_probe_location>Y_MAX_POS ) {
			SERIAL_PROTOCOLPGM("?Specified Y position out of range.\n");
			goto Sigma_Exit;
		}
	}

	if (code_seen('L') || code_seen('l') ) {
        	n_legs = code_value();
		if ( n_legs==1 )
			n_legs = 2;
		if ( n_legs<0 || n_legs>15 ) {
			SERIAL_PROTOCOLPGM("?Specified number of legs in movement not plausable.\n");
			goto Sigma_Exit;
		}
	}

//
// Do all the preliminary setup work.   First raise the probe.
//

        st_synchronize();
        plan_bed_level_matrix.set_to_identity();
	plan_buffer_line( X_current, Y_current, Z_start_location,
			ext_position,
    			homing_feedrate[Z_AXIS]/60,
			active_extruder);
        st_synchronize();

//
// Now get everything to the specified probe point So we can safely do a probe to
// get us close to the bed.  If the Z-Axis is far from the bed, we don't want to
// use that as a starting point for each probe.
//
	if (verbose_level > 2)
		SERIAL_PROTOCOL("Positioning probe for the test.\n");

	plan_buffer_line( X_probe_location, Y_probe_location, Z_start_location,
			ext_position,
    			homing_feedrate[X_AXIS]/60,
			active_extruder);
        st_synchronize();

	current_position[X_AXIS] = X_current = st_get_position_mm(X_AXIS);
	current_position[Y_AXIS] = Y_current = st_get_position_mm(Y_AXIS);
	current_position[Z_AXIS] = Z_current = st_get_position_mm(Z_AXIS);
	current_position[E_AXIS] = ext_position = st_get_position_mm(E_AXIS);

//
// OK, do the inital probe to get us close to the bed.
// Then retrace the right amount and use that in subsequent probes
//

        engage_z_probe();

	setup_for_endstop_move();
	run_z_probe();

	current_position[Z_AXIS] = Z_current = st_get_position_mm(Z_AXIS);
	Z_start_location = st_get_position_mm(Z_AXIS) + Z_RAISE_BEFORE_PROBING;

	plan_buffer_line( X_probe_location, Y_probe_location, Z_start_location,
			ext_position,
    			homing_feedrate[X_AXIS]/60,
			active_extruder);
        st_synchronize();
	current_position[Z_AXIS] = Z_current = st_get_position_mm(Z_AXIS);

	if (engage_probe_for_each_reading)
        	retract_z_probe();

        for( n=0; n<n_samples; n++) {

		do_blocking_move_to( X_probe_location, Y_probe_location, Z_start_location); // Make sure we are at the probe location

		if ( n_legs)  {
		double radius=0.0, theta=0.0, x_sweep, y_sweep;
		int rotational_direction, l;

			rotational_direction = (unsigned long) millis() & 0x0001;			// clockwise or counter clockwise
			radius = (unsigned long) millis() % (long) (X_MAX_LENGTH/4); 			// limit how far out to go
			theta = (float) ((unsigned long) millis() % (long) 360) / (360./(2*3.1415926));	// turn into radians

//SERIAL_ECHOPAIR("starting radius: ",radius);
//SERIAL_ECHOPAIR("   theta: ",theta);
//SERIAL_ECHOPAIR("   direction: ",rotational_direction);
//SERIAL_PROTOCOLLNPGM("");

			for( l=0; l<n_legs-1; l++) {
				if (rotational_direction==1)
					theta += (float) ((unsigned long) millis() % (long) 20) / (360.0/(2*3.1415926)); // turn into radians
				else
					theta -= (float) ((unsigned long) millis() % (long) 20) / (360.0/(2*3.1415926)); // turn into radians

				radius += (float) ( ((long) ((unsigned long) millis() % (long) 10)) - 5);
				if ( radius<0.0 )
					radius = -radius;

				X_current = X_probe_location + cos(theta) * radius;
				Y_current = Y_probe_location + sin(theta) * radius;

				if ( X_current<X_MIN_POS)		// Make sure our X & Y are sane
					 X_current = X_MIN_POS;
				if ( X_current>X_MAX_POS)
					 X_current = X_MAX_POS;

				if ( Y_current<Y_MIN_POS)		// Make sure our X & Y are sane
					 Y_current = Y_MIN_POS;
				if ( Y_current>Y_MAX_POS)
					 Y_current = Y_MAX_POS;

				if (verbose_level>3 ) {
					SERIAL_ECHOPAIR("x: ", X_current);
					SERIAL_ECHOPAIR("y: ", Y_current);
					SERIAL_PROTOCOLLNPGM("");
				}

				do_blocking_move_to( X_current, Y_current, Z_current );
			}
			do_blocking_move_to( X_probe_location, Y_probe_location, Z_start_location); // Go back to the probe location
		}

		if (engage_probe_for_each_reading)  {


        		engage_z_probe();
          		delay(1000);
		}

		setup_for_endstop_move();
                run_z_probe();

		sample_set[n] = current_position[Z_AXIS];

//
// Get the current mean for the data points we have so far
//
		sum=0.0;
		for( j=0; j<=n; j++) {
			sum = sum + sample_set[j];
		}
		mean = sum / (double (n+1));
//
// Now, use that mean to calculate the standard deviation for the
// data points we have so far
//

		sum=0.0;
		for( j=0; j<=n; j++) {
			sum = sum + (sample_set[j]-mean) * (sample_set[j]-mean);
		}
		sigma = sqrt( sum / (double (n+1)) );

		if (verbose_level > 1) {
			SERIAL_PROTOCOL(n+1);
			SERIAL_PROTOCOL(" of ");
			SERIAL_PROTOCOL(n_samples);
			SERIAL_PROTOCOLPGM("   z: ");
			SERIAL_PROTOCOL_F(current_position[Z_AXIS], 6);
		}

		if (verbose_level > 2) {
			SERIAL_PROTOCOL(" mean: ");
			SERIAL_PROTOCOL_F(mean,6);

			SERIAL_PROTOCOL("   sigma: ");
			SERIAL_PROTOCOL_F(sigma,6);
		}

		if (verbose_level > 0)
			SERIAL_PROTOCOLPGM("\n");

		plan_buffer_line( X_probe_location, Y_probe_location, Z_start_location,
				  current_position[E_AXIS], homing_feedrate[Z_AXIS]/60, active_extruder);
        	st_synchronize();

		if (engage_probe_for_each_reading)  {
        		retract_z_probe();
          		delay(1000);
		}
	}

        retract_z_probe();
	delay(1000);

        clean_up_after_endstop_move();

//      enable_endstops(true);

	if (verbose_level > 0) {
		SERIAL_PROTOCOLPGM("Mean: ");
		SERIAL_PROTOCOL_F(mean, 6);
		SERIAL_PROTOCOLPGM("\n");
	}

SERIAL_PROTOCOLPGM("Standard Deviation: ");
SERIAL_PROTOCOL_F(sigma, 6);
SERIAL_PROTOCOLPGM("\n\n");

Sigma_Exit:
        break;
	}
#endif		// Z_PROBE_REPEATABILITY_TEST
#endif		// ENABLE_AUTO_BED_LEVELING

    case 104: // M104
      if(setTargetedHotend(104)){
        break;
      }
      if (code_seen('S')) setTargetHotend(code_value(), tmp_extruder);
#ifdef DUAL_X_CARRIAGE
      if (dual_x_carriage_mode == DXC_DUPLICATION_MODE && tmp_extruder == 0)
        setTargetHotend1(code_value() == 0.0 ? 0.0 : code_value() + duplicate_extruder_temp_offset);
#endif
      setWatch();
      break;
    case 155: //  M155 - Sprout Screen enter the heating blink indication: zct
     tft_heatingloop_enter;
      break;
	 case 156: //  M156 - Sprout Screen enter the leveling blink indication: zct
      tft_leveling_enter;
      break;
	 case 157: //  M157 -Sprout Screen enter the printing : zct
      tft_printing_enter;
      break;
	 case 158: //  M158 -Sprout Screen enter the accomplished printing : zct
     tft_printing_over;
      break;
	 case 159: //  M159 -Sprout Screen total printing time: zct
	 strchr_pointer = strchr(cmdbuffer[bufindr], 'h');
	 ECHO("M159 process....");
	 if (strchr_pointer!=NULL)
	 {
		  total_time+=strtod(&cmdbuffer[bufindr][strchr_pointer-cmdbuffer[bufindr]-3],NULL)*3600;
	 }
	 strchr_pointer = strchr(cmdbuffer[bufindr], 'm');
	 if (strchr_pointer!=NULL)
	 {
		  total_time+=strtod(&cmdbuffer[bufindr][strchr_pointer-cmdbuffer[bufindr]-3],NULL)*60;
	 }
	 ECHO(total_time);
    tft_total_time(total_time);
	total_time=0;
      break;
	case 160: //M160
		tft_resume_enable;
		invalid4resume=false;
	break;
	case 161: //M161
		tft_resume_disable;
	break;
	case 162: // M162: manual leveling process zct	 2015/9/15
		// lift the z probe
   retract_z_probe();
   delay(500);
	// down the bed
  destination[Z_AXIS]=Z_RAISE_BETWEEN_PROBINGS;
  feedrate=300;
  prepare_move();
  //moving the extruder block
  for(int8_t i=0; i < 2; i++) {
    if(code_seen(axis_codes[i]))
    {
      destination[i] = (float)code_value() ;
    }
    //else destination[i] = current_position[i]; //Are these else lines really needed?
  }
  feedrate=1000;
	prepare_move();
	// down the z probe
	st_synchronize();
  engage_z_probe();
  st_synchronize();
// lift the bed
  destination[Z_AXIS]=-1*Z_PROBE_OFFSET_FROM_EXTRUDER;
  feedrate=300;
  prepare_move();
  st_synchronize();
	break;
	case 888 :
 //M888

		ECHO("cluster is:");

	    ECHO(card.curCluster());

		ECHO("resume position is HAHAHAHAHAHAHA");
		ECHO(rePosition);

		ECHO("position is:");
	    ECHO(card.curPosition());

        break;
    case 105 : // M105
      if(setTargetedHotend(105)){
        break;
        }
      #if defined(TEMP_0_PIN) && TEMP_0_PIN > -1
        SERIAL_PROTOCOLPGM("ok T:");
        SERIAL_PROTOCOL_F(degHotend(tmp_extruder),1);
        SERIAL_PROTOCOLPGM(" /");
        SERIAL_PROTOCOL_F(degTargetHotend(tmp_extruder),1);
        #if defined(TEMP_BED_PIN) && TEMP_BED_PIN > -1
          SERIAL_PROTOCOLPGM(" B:");
          SERIAL_PROTOCOL_F(degBed(),1);
          SERIAL_PROTOCOLPGM(" /");
          SERIAL_PROTOCOL_F(degTargetBed(),1);
        #endif //TEMP_BED_PIN
        for (int8_t cur_extruder = 0; cur_extruder < EXTRUDERS; ++cur_extruder) {
          SERIAL_PROTOCOLPGM(" T");
          SERIAL_PROTOCOL(cur_extruder);
          SERIAL_PROTOCOLPGM(":");
          SERIAL_PROTOCOL_F(degHotend(cur_extruder),1);
          SERIAL_PROTOCOLPGM(" /");
          SERIAL_PROTOCOL_F(degTargetHotend(cur_extruder),1);
        }
      #else
        SERIAL_ERROR_START;
        SERIAL_ERRORLNPGM(MSG_ERR_NO_THERMISTORS);
      #endif

        SERIAL_PROTOCOLPGM(" @:");
      #ifdef EXTRUDER_WATTS
        SERIAL_PROTOCOL((EXTRUDER_WATTS * getHeaterPower(tmp_extruder))/127);
        SERIAL_PROTOCOLPGM("W");
      #else
        SERIAL_PROTOCOL(getHeaterPower(tmp_extruder));
      #endif

        SERIAL_PROTOCOLPGM(" B@:");
      #ifdef BED_WATTS
        SERIAL_PROTOCOL((BED_WATTS * getHeaterPower(-1))/127);
        SERIAL_PROTOCOLPGM("W");
      #else
        SERIAL_PROTOCOL(getHeaterPower(-1));
      #endif

        #ifdef SHOW_TEMP_ADC_VALUES
          #if defined(TEMP_BED_PIN) && TEMP_BED_PIN > -1
            SERIAL_PROTOCOLPGM("    ADC B:");
            SERIAL_PROTOCOL_F(degBed(),1);
            SERIAL_PROTOCOLPGM("C->");
            SERIAL_PROTOCOL_F(rawBedTemp()/OVERSAMPLENR,0);
          #endif
          for (int8_t cur_extruder = 0; cur_extruder < EXTRUDERS; ++cur_extruder) {
            SERIAL_PROTOCOLPGM("  T");
            SERIAL_PROTOCOL(cur_extruder);
            SERIAL_PROTOCOLPGM(":");
            SERIAL_PROTOCOL_F(degHotend(cur_extruder),1);
            SERIAL_PROTOCOLPGM("C->");
            SERIAL_PROTOCOL_F(rawHotendTemp(cur_extruder)/OVERSAMPLENR,0);
          }
        #endif

        SERIAL_PROTOCOLLN("");
      return;
      break;
    case 109:
    {// M109 - Wait for extruder heater to reach target.

      if(setTargetedHotend(109)){
        break;
      }
        //zct:LCD_MESSAGEPGM(MSG_HEATING);
      #ifdef AUTOTEMP
        autotemp_enabled=false;
      #endif
      if (code_seen('S')) {
        setTargetHotend(code_value(), tmp_extruder);
#ifdef DUAL_X_CARRIAGE
        if (dual_x_carriage_mode == DXC_DUPLICATION_MODE && tmp_extruder == 0)
          setTargetHotend1(code_value() == 0.0 ? 0.0 : code_value() + duplicate_extruder_temp_offset);
#endif
        CooldownNoWait = true;
      } else if (code_seen('R')) {
        setTargetHotend(code_value(), tmp_extruder);
#ifdef DUAL_X_CARRIAGE
        if (dual_x_carriage_mode == DXC_DUPLICATION_MODE && tmp_extruder == 0)
          setTargetHotend1(code_value() == 0.0 ? 0.0 : code_value() + duplicate_extruder_temp_offset);
#endif
        CooldownNoWait = false;
      }
      #ifdef AUTOTEMP
        if (code_seen('S')) autotemp_min=code_value();
        if (code_seen('B')) autotemp_max=code_value();
        if (code_seen('F'))
        {
          autotemp_factor=code_value();
          autotemp_enabled=true;
        }
      #endif

      setWatch();
      codenum = millis();

      /* See if we are heating up or cooling down */
      target_direction = isHeatingHotend(tmp_extruder); // true if heating, false if cooling

      cancel_heatup = false;
	  // zct闂備浇娉曢崰鎰板几婵犳艾绠紒灞界獖r ps-LCD, update the monitor blink to the heating loop
	  tft_heatingloop_enter;
	  //
      #ifdef TEMP_RESIDENCY_TIME
        long residencyStart;
        residencyStart = -1;
        /* continue to loop until we have reached the target temp
          _and_ until TEMP_RESIDENCY_TIME hasn't passed since we reached it */
        while((!cancel_heatup)&&((residencyStart == -1) ||
              (residencyStart >= 0 && (((unsigned int) (millis() - residencyStart)) < (TEMP_RESIDENCY_TIME * 1000UL)))) ) {
      #else
        while ( target_direction ? (isHeatingHotend(tmp_extruder)) : (isCoolingHotend(tmp_extruder)&&(CooldownNoWait==false)) ) {
      #endif //TEMP_RESIDENCY_TIME
          if( (millis() - codenum) > 1000UL )
          { //Print Temp Reading and remaining time every 1 second while heating up/cooling down
            SERIAL_PROTOCOLPGM("T:");
            SERIAL_PROTOCOL_F(degHotend(tmp_extruder),1);
            SERIAL_PROTOCOLPGM(" E:");
            SERIAL_PROTOCOL((int)tmp_extruder);
            #ifdef TEMP_RESIDENCY_TIME
              SERIAL_PROTOCOLPGM(" W:");
              if(residencyStart > -1)
              {
                 codenum = ((TEMP_RESIDENCY_TIME * 1000UL) - (millis() - residencyStart)) / 1000UL;
                 SERIAL_PROTOCOLLN( codenum );
              }
              else
              {
                 SERIAL_PROTOCOLLN( "?" );
              }
            #else
              SERIAL_PROTOCOLLN("");
            #endif

            codenum = millis();
          }
          manage_heater();
          manage_inactivity();

		 tft_status_temperature_update();

		  tft_rx();



        #ifdef TEMP_RESIDENCY_TIME
            /* start/restart the TEMP_RESIDENCY_TIME timer whenever we reach target temp for the first time
              or when current temp falls outside the hysteresis after target temp was reached */
          if ((residencyStart == -1 &&  target_direction && (degHotend(tmp_extruder) >= (degTargetHotend(tmp_extruder)-TEMP_WINDOW))) ||
              (residencyStart == -1 && !target_direction && (degHotend(tmp_extruder) <= (degTargetHotend(tmp_extruder)+TEMP_WINDOW))) ||
              (residencyStart > -1 && labs(degHotend(tmp_extruder) - degTargetHotend(tmp_extruder)) > TEMP_HYSTERESIS) )
          {
            residencyStart = millis();
          }
        #endif //TEMP_RESIDENCY_TIME
        }
          //zct:LCD_MESSAGEPGM(MSG_HEATING_COMPLETE);
        starttime=millis();
        previous_millis_cmd = millis();
		//tft_printing_enter;
      }
      break;

    #if defined(FAN_PIN) && FAN_PIN > -1
      case 126: //Cool the model
          if (code_seen('R'))
			  {
					analogWrite(FAN_SAN_PIN, constrain(code_value(),0,255));
			  }
			  //else
			  //{
				//	analogWrite(FAN_SAN_PIN, 225);
			  //}

	    if (code_seen('S')){
           fanSpeed=constrain(code_value(),0,255);
        }
        else {
          fanSpeed=255;
        }
        break;
      case 106: //M106 Fan On
        if (code_seen('S')){
           fanSpeed=constrain(code_value(),0,255);
        }
        else {
          fanSpeed=255;
        }
        break;
      case 107: //M107 Fan Off
        fanSpeed = 0;
        analogWrite(FAN_SAN_PIN, 0);
        break;
    #endif //FAN_PIN
    #ifdef BARICUDA
      // PWM for HEATER_1_PIN
      #if defined(HEATER_1_PIN) && HEATER_1_PIN > -1
        case 126: //M126 valve open
          if (code_seen('S')){
             ValvePressure=constrain(code_value(),0,255);
          }
          else {
            ValvePressure=255;
          }
          break;
        case 127: //M127 valve closed
          ValvePressure = 0;
          break;
      #endif //HEATER_1_PIN

      // PWM for HEATER_2_PIN
      #if defined(HEATER_2_PIN) && HEATER_2_PIN > -1
        case 128: //M128 valve open
          if (code_seen('S')){
             EtoPPressure=constrain(code_value(),0,255);
          }
          else {
            EtoPPressure=255;
          }
          break;
        case 129: //M129 valve closed
          EtoPPressure = 0;
          break;
      #endif //HEATER_2_PIN
    #endif

    #if defined(PS_ON_PIN) && PS_ON_PIN > -1
      case 80: // M80 - Turn on Power Supply
        SET_OUTPUT(PS_ON_PIN); //GND
        WRITE(PS_ON_PIN, PS_ON_AWAKE);

        // If you have a switch on suicide pin, this is useful
        // if you want to start another print with suicide feature after
        // a print without suicide...
        #if defined SUICIDE_PIN && SUICIDE_PIN > -1
            SET_OUTPUT(SUICIDE_PIN);
            WRITE(SUICIDE_PIN, HIGH);
        #endif

        #ifdef ULTIPANEL
          powersupply = true;
            //zct:LCD_MESSAGEPGM(WELCOME_MSG);
          //zct: lcd_update();
        #endif
        break;
      #endif

      case 81: // M81 - Turn off Power Supply
        disable_heater();
        st_synchronize();
        disable_e0();
        disable_e1();
        disable_e2();
        finishAndDisableSteppers();
        fanSpeed = 0;
        delay(1000); // Wait a little before to switch off
      #if defined(SUICIDE_PIN) && SUICIDE_PIN > -1
        st_synchronize();
        suicide();
      #elif defined(PS_ON_PIN) && PS_ON_PIN > -1
        SET_OUTPUT(PS_ON_PIN);
        WRITE(PS_ON_PIN, PS_ON_ASLEEP);
      #endif
      #ifdef ULTIPANEL
        powersupply = false;
          //zct:LCD_MESSAGEPGM(MACHINE_NAME" "MSG_OFF".");
        //zct: lcd_update();
      #endif
	  break;

    case 82:
      axis_relative_modes[3] = false;
      break;
    case 83:
      axis_relative_modes[3] = true;
      break;
    case 18: //compatibility
    case 84: // M84
      if(code_seen('S')){
        stepper_inactive_time = code_value() * 1000;
      }
      else
      {
        bool all_axis = !((code_seen(axis_codes[X_AXIS])) || (code_seen(axis_codes[Y_AXIS])) || (code_seen(axis_codes[Z_AXIS]))|| (code_seen(axis_codes[E_AXIS])));
        if(all_axis)
        {
          st_synchronize();
          disable_e0();
          disable_e1();
          disable_e2();
          finishAndDisableSteppers();
        }
        else
        {
          st_synchronize();
          if(code_seen('X')) disable_x();
          if(code_seen('Y')) disable_y();
          if(code_seen('Z')) disable_z();
          #if ((E0_ENABLE_PIN != X_ENABLE_PIN) && (E1_ENABLE_PIN != Y_ENABLE_PIN)) // Only enable on boards that have seperate ENABLE_PINS
            if(code_seen('E')) {
              disable_e0();
              disable_e1();
              disable_e2();
            }
          #endif
        }
      }
      break;
    case 85: // M85
      if(code_seen('S')) {
        max_inactive_time = code_value() * 1000;
      }
      break;
    case 92: // M92
      for(int8_t i=0; i < NUM_AXIS; i++)
      {
        if(code_seen(axis_codes[i]))
        {
          if(i == 3) { // E
            float value = code_value();
            if(value < 20.0) {
              float factor = axis_steps_per_unit[i] / value; // increase e constants if M92 E14 is given for netfab.
              max_e_jerk *= factor;
              max_feedrate[i] *= factor;
              axis_steps_per_sqr_second[i] *= factor;
            }
            axis_steps_per_unit[i] = value;
          }
          else {
            axis_steps_per_unit[i] = code_value();
          }
        }
      }
      break;
    case 115: // M115
      SERIAL_PROTOCOLPGM(MSG_M115_REPORT);
      break;
    case 114: // M114
      SERIAL_PROTOCOLPGM("X:");
      SERIAL_PROTOCOL(current_position[X_AXIS]);
      SERIAL_PROTOCOLPGM(" Y:");
      SERIAL_PROTOCOL(current_position[Y_AXIS]);
      SERIAL_PROTOCOLPGM(" Z:");
      SERIAL_PROTOCOL(current_position[Z_AXIS]);
      SERIAL_PROTOCOLPGM(" E:");
      SERIAL_PROTOCOL(current_position[E_AXIS]);

      SERIAL_PROTOCOLPGM(MSG_COUNT_X);
      SERIAL_PROTOCOL(float(st_get_position(X_AXIS))/axis_steps_per_unit[X_AXIS]);
      SERIAL_PROTOCOLPGM(" Y:");
      SERIAL_PROTOCOL(float(st_get_position(Y_AXIS))/axis_steps_per_unit[Y_AXIS]);
      SERIAL_PROTOCOLPGM(" Z:");
      SERIAL_PROTOCOL(float(st_get_position(Z_AXIS))/axis_steps_per_unit[Z_AXIS]);

      SERIAL_PROTOCOLLN("");
#ifdef SCARA
	  SERIAL_PROTOCOLPGM("SCARA Theta:");
      SERIAL_PROTOCOL(delta[X_AXIS]);
      SERIAL_PROTOCOLPGM("   Psi+Theta:");
      SERIAL_PROTOCOL(delta[Y_AXIS]);
      SERIAL_PROTOCOLLN("");

      SERIAL_PROTOCOLPGM("SCARA Cal - Theta:");
      SERIAL_PROTOCOL(delta[X_AXIS]+add_homing[X_AXIS]);
      SERIAL_PROTOCOLPGM("   Psi+Theta (90):");
      SERIAL_PROTOCOL(delta[Y_AXIS]-delta[X_AXIS]-90+add_homing[Y_AXIS]);
      SERIAL_PROTOCOLLN("");

      SERIAL_PROTOCOLPGM("SCARA step Cal - Theta:");
      SERIAL_PROTOCOL(delta[X_AXIS]/90*axis_steps_per_unit[X_AXIS]);
      SERIAL_PROTOCOLPGM("   Psi+Theta:");
      SERIAL_PROTOCOL((delta[Y_AXIS]-delta[X_AXIS])/90*axis_steps_per_unit[Y_AXIS]);
      SERIAL_PROTOCOLLN("");
      SERIAL_PROTOCOLLN("");
#endif
      break;
    case 120: // M120
      enable_endstops(false) ;
      break;
    case 121: // M121
      enable_endstops(true) ;
      break;
    case 119: // M119
    SERIAL_PROTOCOLLN(MSG_M119_REPORT);
      #if defined(X_MIN_PIN) && X_MIN_PIN > -1
        SERIAL_PROTOCOLPGM(MSG_X_MIN);
        SERIAL_PROTOCOLLN(((READ(X_MIN_PIN)^X_MIN_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
      #endif
      #if defined(X_MAX_PIN) && X_MAX_PIN > -1
        SERIAL_PROTOCOLPGM(MSG_X_MAX);
        SERIAL_PROTOCOLLN(((READ(X_MAX_PIN)^X_MAX_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
      #endif
      #if defined(Y_MIN_PIN) && Y_MIN_PIN > -1
        SERIAL_PROTOCOLPGM(MSG_Y_MIN);
        SERIAL_PROTOCOLLN(((READ(Y_MIN_PIN)^Y_MIN_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
      #endif
      #if defined(Y_MAX_PIN) && Y_MAX_PIN > -1
        SERIAL_PROTOCOLPGM(MSG_Y_MAX);
        SERIAL_PROTOCOLLN(((READ(Y_MAX_PIN)^Y_MAX_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
      #endif
      #if defined(Z_MIN_PIN) && Z_MIN_PIN > -1
        SERIAL_PROTOCOLPGM(MSG_Z_MIN);
        SERIAL_PROTOCOLLN(((READ(Z_MIN_PIN)^Z_MIN_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
      #endif
      #if defined(Z_MAX_PIN) && Z_MAX_PIN > -1
        SERIAL_PROTOCOLPGM(MSG_Z_MAX);
        SERIAL_PROTOCOLLN(((READ(Z_MAX_PIN)^Z_MAX_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
      #endif
      break;
      //TODO: update for all axis, use for loop
    #ifdef BLINKM
    case 150: // M150
      {
        byte red;
        byte grn;
        byte blu;

        if(code_seen('R')) red = code_value();
        if(code_seen('U')) grn = code_value();
        if(code_seen('B')) blu = code_value();

        SendColors(red,grn,blu);
      }
      break;
    #endif //BLINKM

    case 200: // M200 D<millimeters> set filament diameter and set E axis units to cubic millimeters (use S0 to set back to millimeters).
      {

        tmp_extruder = active_extruder;
        if(code_seen('T')) {
          tmp_extruder = code_value();
          if(tmp_extruder >= EXTRUDERS) {
            SERIAL_ECHO_START;
            SERIAL_ECHO(MSG_M200_INVALID_EXTRUDER);
            break;
          }
        }

        float area = .0;
        if(code_seen('D')) {
          float diameter = code_value();
          // setting any extruder filament size disables volumetric on the assumption that
          // slicers either generate in extruder values as cubic mm or as as filament feeds
          // for all extruders
          volumetric_enabled = (diameter != 0.0);
          if (volumetric_enabled) {
            filament_size[tmp_extruder] = diameter;
            // make sure all extruders have some sane value for the filament size
            for (int i=0; i<EXTRUDERS; i++)
              if (! filament_size[i]) filament_size[i] = DEFAULT_NOMINAL_FILAMENT_DIA;
          }
        } else {
          //reserved for setting filament diameter via UFID or filament measuring device
          break;
        }
        calculate_volumetric_multipliers();
      }
      break;
    case 201: // M201
      for(int8_t i=0; i < NUM_AXIS; i++)
      {
        if(code_seen(axis_codes[i]))
        {
          max_acceleration_units_per_sq_second[i] = code_value();
        }
      }
      // steps per sq second need to be updated to agree with the units per sq second (as they are what is used in the planner)
      reset_acceleration_rates();
      break;
    #if 0 // Not used for Sprinter/grbl gen6
    case 202: // M202
      for(int8_t i=0; i < NUM_AXIS; i++) {
        if(code_seen(axis_codes[i])) axis_travel_steps_per_sqr_second[i] = code_value() * axis_steps_per_unit[i];
      }
      break;
    #endif
    case 203: // M203 max feedrate mm/sec
      for(int8_t i=0; i < NUM_AXIS; i++) {
        if(code_seen(axis_codes[i])) max_feedrate[i] = code_value();
      }
      break;
    case 204: // M204 acclereration S normal moves T filmanent only moves
      {
        if(code_seen('S')) acceleration = code_value() ;
        if(code_seen('T')) retract_acceleration = code_value() ;
      }
      break;
    case 205: //M205 advanced settings:  minimum travel speed S=while printing T=travel only,  B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk
    {
      if(code_seen('S')) minimumfeedrate = code_value();
      if(code_seen('T')) mintravelfeedrate = code_value();
      if(code_seen('B')) minsegmenttime = code_value() ;
      if(code_seen('X')) max_xy_jerk = code_value() ;
      if(code_seen('Z')) max_z_jerk = code_value() ;
      if(code_seen('E')) max_e_jerk = code_value() ;
    }
    break;
    case 206: // M206 additional homing offset
      for(int8_t i=0; i < 3; i++)
      {
        if(code_seen(axis_codes[i])) add_homing[i] = code_value();
      }
	  #ifdef SCARA
	   if(code_seen('T'))       // Theta
      {
        add_homing[X_AXIS] = code_value() ;
      }
      if(code_seen('P'))       // Psi
      {
        add_homing[Y_AXIS] = code_value() ;
      }
	  #endif
      break;
    #ifdef DELTA
	case 665: // M665 set delta configurations L<diagonal_rod> R<delta_radius> S<segments_per_sec>
		if(code_seen('L')) {
			delta_diagonal_rod= code_value();
		}
		if(code_seen('R')) {
			delta_radius= code_value();
		}
		if(code_seen('S')) {
			delta_segments_per_second= code_value();
		}

		recalc_delta_settings(delta_radius, delta_diagonal_rod);
		break;
    case 666: // M666 set delta endstop adjustemnt
      for(int8_t i=0; i < 3; i++)
      {
        if(code_seen(axis_codes[i])) endstop_adj[i] = code_value();
      }
      break;
    #endif
    #ifdef FWRETRACT
    case 207: //M207 - set retract length S[positive mm] F[feedrate mm/min] Z[additional zlift/hop]
    {
      if(code_seen('S'))
      {
        retract_length = code_value() ;
      }
      if(code_seen('F'))
      {
        retract_feedrate = code_value()/60 ;
      }
      if(code_seen('Z'))
      {
        retract_zlift = code_value() ;
      }
    }break;
    case 208: // M208 - set retract recover length S[positive mm surplus to the M207 S*] F[feedrate mm/min]
    {
      if(code_seen('S'))
      {
        retract_recover_length = code_value() ;
      }
      if(code_seen('F'))
      {
        retract_recover_feedrate = code_value()/60 ;
      }
    }break;
    case 209: // M209 - S<1=true/0=false> enable automatic retract detect if the slicer did not support G10/11: every normal extrude-only move will be classified as retract depending on the direction.
    {
      if(code_seen('S'))
      {
        int t= code_value() ;
        switch(t)
        {
          case 0:
          case 1:
          {
            autoretract_enabled = (t == 1);
            for (int i=0; i<EXTRUDERS; i++) retracted[i] = false;
          }break;
          default:
            SERIAL_ECHO_START;
            SERIAL_ECHOPGM(MSG_UNKNOWN_COMMAND);
            SERIAL_ECHO(cmdbuffer[bufindr]);
            SERIAL_ECHOLNPGM("\"");
        }
      }

    }break;
    #endif // FWRETRACT
    #if EXTRUDERS > 1
    case 218: // M218 - set hotend offset (in mm), T<extruder_number> X<offset_on_X> Y<offset_on_Y>
    {
      if(setTargetedHotend(218)){
        break;
      }
      if(code_seen('X'))
      {
        extruder_offset[X_AXIS][tmp_extruder] = code_value();
      }
      if(code_seen('Y'))
      {
        extruder_offset[Y_AXIS][tmp_extruder] = code_value();
      }
      #ifdef DUAL_X_CARRIAGE
      if(code_seen('Z'))
      {
        extruder_offset[Z_AXIS][tmp_extruder] = code_value();
      }
      #endif
      SERIAL_ECHO_START;
      SERIAL_ECHOPGM(MSG_HOTEND_OFFSET);
      for(tmp_extruder = 0; tmp_extruder < EXTRUDERS; tmp_extruder++)
      {
         SERIAL_ECHO(" ");
         SERIAL_ECHO(extruder_offset[X_AXIS][tmp_extruder]);
         SERIAL_ECHO(",");
         SERIAL_ECHO(extruder_offset[Y_AXIS][tmp_extruder]);
      #ifdef DUAL_X_CARRIAGE
         SERIAL_ECHO(",");
         SERIAL_ECHO(extruder_offset[Z_AXIS][tmp_extruder]);
      #endif
      }
      SERIAL_ECHOLN("");
    }break;
    #endif
    case 220: // M220 S<factor in percent>- set speed factor override percentage
    {
      if(code_seen('S'))
      {
        feedmultiply = code_value() ;
      }
    }
    break;
    case 221: // M221 S<factor in percent>- set extrude factor override percentage
    {
      if(code_seen('S'))
      {
        int tmp_code = code_value();
        if (code_seen('T'))
        {
          if(setTargetedHotend(221)){
            break;
          }
          extruder_multiply[tmp_extruder] = tmp_code;
        }
        else
        {
          extrudemultiply = tmp_code ;
        }
      }
    }
    break;


    #ifdef PIDTEMP
	case 301: // M301
	{

		// multi-extruder PID patch: M301 updates or prints a single extruder's PID values
		// default behaviour (omitting E parameter) is to update for extruder 0 only
		int e = 0; // extruder being updated
		if (code_seen('E'))
		{
			e = (int)code_value();
		}
		if (e < EXTRUDERS) // catch bad input value
		{

			if (code_seen('P')) PID_PARAM(Kp,e) = code_value();
			if (code_seen('I')) PID_PARAM(Ki,e) = scalePID_i(code_value());
			if (code_seen('D')) PID_PARAM(Kd,e) = scalePID_d(code_value());
			#ifdef PID_ADD_EXTRUSION_RATE
			if (code_seen('C')) PID_PARAM(Kc,e) = code_value();
			#endif

			updatePID();
			SERIAL_PROTOCOL(MSG_OK);
            #ifdef PID_PARAMS_PER_EXTRUDER
			  SERIAL_PROTOCOL(" e:"); // specify extruder in serial output
			  SERIAL_PROTOCOL(e);
            #endif // PID_PARAMS_PER_EXTRUDER
			SERIAL_PROTOCOL(" p:");
			SERIAL_PROTOCOL(PID_PARAM(Kp,e));
			SERIAL_PROTOCOL(" i:");
			SERIAL_PROTOCOL(unscalePID_i(PID_PARAM(Ki,e)));
			SERIAL_PROTOCOL(" d:");
			SERIAL_PROTOCOL(unscalePID_d(PID_PARAM(Kd,e)));
			#ifdef PID_ADD_EXTRUSION_RATE
			SERIAL_PROTOCOL(" c:");
			//Kc does not have scaling applied above, or in resetting defaults
			SERIAL_PROTOCOL(PID_PARAM(Kc,e));
			#endif
			SERIAL_PROTOCOLLN("");

		}
		else
		{
			SERIAL_ECHO_START;
			SERIAL_ECHOLN(MSG_INVALID_EXTRUDER);
		}

      }
      break;
    #endif //PIDTEMP
    #ifdef PIDTEMPBED
    case 304: // M304
      {
        if(code_seen('P')) bedKp = code_value();
        if(code_seen('I')) bedKi = scalePID_i(code_value());
        if(code_seen('D')) bedKd = scalePID_d(code_value());

        updatePID();
        SERIAL_PROTOCOL(MSG_OK);
        SERIAL_PROTOCOL(" p:");
        SERIAL_PROTOCOL(bedKp);
        SERIAL_PROTOCOL(" i:");
        SERIAL_PROTOCOL(unscalePID_i(bedKi));
        SERIAL_PROTOCOL(" d:");
        SERIAL_PROTOCOL(unscalePID_d(bedKd));
        SERIAL_PROTOCOLLN("");
      }
      break;
    #endif //PIDTEMP
    case 240: // M240  Triggers a camera by emulating a Canon RC-1 : http://www.doc-diy.net/photo/rc-1_hacked/
     {
     	#ifdef CHDK

         SET_OUTPUT(CHDK);
         WRITE(CHDK, HIGH);
         chdkHigh = millis();
         chdkActive = true;

       #else

      	#if defined(PHOTOGRAPH_PIN) && PHOTOGRAPH_PIN > -1
	const uint8_t NUM_PULSES=16;
	const float PULSE_LENGTH=0.01524;
	for(int i=0; i < NUM_PULSES; i++) {
        WRITE(PHOTOGRAPH_PIN, HIGH);
        _delay_ms(PULSE_LENGTH);
        WRITE(PHOTOGRAPH_PIN, LOW);
        _delay_ms(PULSE_LENGTH);
        }
        delay(7.33);
        for(int i=0; i < NUM_PULSES; i++) {
        WRITE(PHOTOGRAPH_PIN, HIGH);
        _delay_ms(PULSE_LENGTH);
        WRITE(PHOTOGRAPH_PIN, LOW);
        _delay_ms(PULSE_LENGTH);
        }
      	#endif
      #endif //chdk end if
     }
    break;
#ifdef DOGLCD
    case 250: // M250  Set LCD contrast value: C<value> (value 0..63)
     {
	  if (code_seen('C')) {
	   //zct: lcd_setcontrast( ((int)code_value())&63 );
          }
          SERIAL_PROTOCOLPGM("lcd contrast value: ");
           //zct:SERIAL_PROTOCOL(lcd_contrast);
          SERIAL_PROTOCOLLN("");
     }
    break;
#endif
case 280: // M280 - set servo position absolute. P: servo index, S: angle or microseconds
      {
        int servo_index = -1;
        int servo_position = 0;
        if (code_seen('P'))
          servo_index =
    #if NUM_SERVOS > 0
    code_value();
        if (code_seen('S')) {
          servo_position = code_value();
          if ((servo_index >= 0) && (servo_index < NUM_SERVOS)) {
#if defined (ENABLE_AUTO_BED_LEVELING) && (PROBE_SERVO_DEACTIVATION_DELAY > 0)
		      servos[servo_index].attach(0);
#endif
            servos[servo_index].write(servo_position);
#if defined (ENABLE_AUTO_BED_LEVELING) && (PROBE_SERVO_DEACTIVATION_DELAY > 0)
              delay(PROBE_SERVO_DEACTIVATION_DELAY);
              servos[servo_index].detach();
#endif
          }
          else {
            SERIAL_ECHO_START;
            SERIAL_ECHO("Servo ");
            SERIAL_ECHO(servo_index);
            SERIAL_ECHOLN(" out of range");
          }
        }
        else if (servo_index >= 0) {
          SERIAL_PROTOCOL(MSG_OK);
          SERIAL_PROTOCOL(" Servo ");
          SERIAL_PROTOCOL(servo_index);
          SERIAL_PROTOCOL(": ");
          SERIAL_PROTOCOL(servos[servo_index].read());
          SERIAL_PROTOCOLLN("");
        }
      }
      break;

    #endif // NUM_SERVOS > 0
    #ifdef PREVENT_DANGEROUS_EXTRUDE
    case 302: // allow cold extrudes, or set the minimum extrude temperature
    {
	  float temp = .0;
	  if (code_seen('S')) temp=code_value();
      set_extrude_min_temp(temp);
    }
    break;
	#endif
    case 303: // M303 PID autotune
    {
      float temp = 150.0;
      int e=0;
      int c=5;
      if (code_seen('E')) e=code_value();
        if (e<0)
          temp=70;
      if (code_seen('S')) temp=code_value();
      if (code_seen('C')) c=code_value();
      PID_autotune(temp, e, c);
    }
    break;
	#ifdef SCARA
	case 360:  // M360 SCARA Theta pos1
      SERIAL_ECHOLN(" Cal: Theta 0 ");
      //SoftEndsEnabled = false;              // Ignore soft endstops during calibration
      //SERIAL_ECHOLN(" Soft endstops disabled ");
      if(Stopped == false) {
        //get_coordinates(); // For X Y Z E F
        delta[X_AXIS] = 0;
        delta[Y_AXIS] = 120;
        calculate_SCARA_forward_Transform(delta);
        destination[X_AXIS] = delta[X_AXIS]/axis_scaling[X_AXIS];
        destination[Y_AXIS] = delta[Y_AXIS]/axis_scaling[Y_AXIS];

        prepare_move();
        //ClearToSend();
        return;
      }
    break;

    case 361:  // SCARA Theta pos2
      SERIAL_ECHOLN(" Cal: Theta 90 ");
      //SoftEndsEnabled = false;              // Ignore soft endstops during calibration
      //SERIAL_ECHOLN(" Soft endstops disabled ");
      if(Stopped == false) {
        //get_coordinates(); // For X Y Z E F
        delta[X_AXIS] = 90;
        delta[Y_AXIS] = 130;
        calculate_SCARA_forward_Transform(delta);
        destination[X_AXIS] = delta[X_AXIS]/axis_scaling[X_AXIS];
        destination[Y_AXIS] = delta[Y_AXIS]/axis_scaling[Y_AXIS];

        prepare_move();
        //ClearToSend();
        return;
      }
    break;
    case 362:  // SCARA Psi pos1
      SERIAL_ECHOLN(" Cal: Psi 0 ");
      //SoftEndsEnabled = false;              // Ignore soft endstops during calibration
      //SERIAL_ECHOLN(" Soft endstops disabled ");
      if(Stopped == false) {
        //get_coordinates(); // For X Y Z E F
        delta[X_AXIS] = 60;
        delta[Y_AXIS] = 180;
        calculate_SCARA_forward_Transform(delta);
        destination[X_AXIS] = delta[X_AXIS]/axis_scaling[X_AXIS];
        destination[Y_AXIS] = delta[Y_AXIS]/axis_scaling[Y_AXIS];

        prepare_move();
        //ClearToSend();
        return;
      }
    break;
    case 363:  // SCARA Psi pos2
      SERIAL_ECHOLN(" Cal: Psi 90 ");
      //SoftEndsEnabled = false;              // Ignore soft endstops during calibration
      //SERIAL_ECHOLN(" Soft endstops disabled ");
      if(Stopped == false) {
        //get_coordinates(); // For X Y Z E F
        delta[X_AXIS] = 50;
        delta[Y_AXIS] = 90;
        calculate_SCARA_forward_Transform(delta);
        destination[X_AXIS] = delta[X_AXIS]/axis_scaling[X_AXIS];
        destination[Y_AXIS] = delta[Y_AXIS]/axis_scaling[Y_AXIS];

        prepare_move();
        //ClearToSend();
        return;
      }
    break;
    case 364:  // SCARA Psi pos3 (90 deg to Theta)
      SERIAL_ECHOLN(" Cal: Theta-Psi 90 ");
     // SoftEndsEnabled = false;              // Ignore soft endstops during calibration
      //SERIAL_ECHOLN(" Soft endstops disabled ");
      if(Stopped == false) {
        //get_coordinates(); // For X Y Z E F
        delta[X_AXIS] = 45;
        delta[Y_AXIS] = 135;
        calculate_SCARA_forward_Transform(delta);
        destination[X_AXIS] = delta[X_AXIS]/axis_scaling[X_AXIS];
        destination[Y_AXIS] = delta[Y_AXIS]/axis_scaling[Y_AXIS];

        prepare_move();
        //ClearToSend();
        return;
      }
    break;
    case 365: // M364  Set SCARA scaling for X Y Z
      for(int8_t i=0; i < 3; i++)
      {
        if(code_seen(axis_codes[i]))
        {

            axis_scaling[i] = code_value();

        }
      }
      break;
	#endif
    case 400: // M400 finish all moves
    {
      st_synchronize();
    }
    break;
#if defined(ENABLE_AUTO_BED_LEVELING) && defined(SERVO_ENDSTOPS) && not defined(Z_PROBE_SLED)
    case 401:
    {
        engage_z_probe();    // Engage Z Servo endstop if available
    }
    break;

    case 402:
    {
        retract_z_probe();    // Retract Z Servo endstop if enabled
    }
    break;
#endif

#ifdef FILAMENT_SENSOR
case 404:  //M404 Enter the nominal filament width (3mm, 1.75mm ) N<3.0> or display nominal filament width
    {
    #if (FILWIDTH_PIN > -1)
    if(code_seen('N')) filament_width_nominal=code_value();
    else{
    SERIAL_PROTOCOLPGM("Filament dia (nominal mm):");
    SERIAL_PROTOCOLLN(filament_width_nominal);
    }
    #endif
    }
    break;

    case 405:  //M405 Turn on filament sensor for control
    {


    if(code_seen('D')) meas_delay_cm=code_value();

       if(meas_delay_cm> MAX_MEASUREMENT_DELAY)
       	meas_delay_cm = MAX_MEASUREMENT_DELAY;

       if(delay_index2 == -1)  //initialize the ring buffer if it has not been done since startup
    	   {
    	   int temp_ratio = widthFil_to_size_ratio();

       	    for (delay_index1=0; delay_index1<(MAX_MEASUREMENT_DELAY+1); ++delay_index1 ){
       	              measurement_delay[delay_index1]=temp_ratio-100;  //subtract 100 to scale within a signed byte
       	        }
       	    delay_index1=0;
       	    delay_index2=0;
    	   }

    filament_sensor = true ;

    //SERIAL_PROTOCOLPGM("Filament dia (measured mm):");
    //SERIAL_PROTOCOL(filament_width_meas);
    //SERIAL_PROTOCOLPGM("Extrusion ratio(%):");
    //SERIAL_PROTOCOL(extrudemultiply);
    }
    break;

    case 406:  //M406 Turn off filament sensor for control
    {
    filament_sensor = false ;
    }
    break;

    case 407:   //M407 Display measured filament diameter
    {



    SERIAL_PROTOCOLPGM("Filament dia (measured mm):");
    SERIAL_PROTOCOLLN(filament_width_meas);
    }
    break;
    #endif

    #ifdef ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED
    case 540:
    {
        if(code_seen('S')) abort_on_endstop_hit = code_value() > 0;
    }
    break;
    #endif

    #ifdef CUSTOM_M_CODE_SET_Z_PROBE_OFFSET
    case CUSTOM_M_CODE_SET_Z_PROBE_OFFSET:
    {
      float value;
      if (code_seen('Z'))
      {
        value = code_value();
        if ((Z_PROBE_OFFSET_RANGE_MIN <= value) && (value <= Z_PROBE_OFFSET_RANGE_MAX))
        {
          zprobe_zoffset = -value; // compare w/ line 278 of ConfigurationStore.cpp
          SERIAL_ECHO_START;
          SERIAL_ECHOLNPGM(MSG_ZPROBE_ZOFFSET " " MSG_OK);
          SERIAL_PROTOCOLLN("");
        }
        else
        {
          SERIAL_ECHO_START;
          SERIAL_ECHOPGM(MSG_ZPROBE_ZOFFSET);
          SERIAL_ECHOPGM(MSG_Z_MIN);
          SERIAL_ECHO(Z_PROBE_OFFSET_RANGE_MIN);
          SERIAL_ECHOPGM(MSG_Z_MAX);
          SERIAL_ECHO(Z_PROBE_OFFSET_RANGE_MAX);
          SERIAL_PROTOCOLLN("");
        }
      }
      else
      {
          SERIAL_ECHO_START;
          SERIAL_ECHOLNPGM(MSG_ZPROBE_ZOFFSET " : ");
          SERIAL_ECHO(-zprobe_zoffset);
          SERIAL_PROTOCOLLN("");
      }
      break;
    }
    #endif // CUSTOM_M_CODE_SET_Z_PROBE_OFFSET

    case 999: // M999: Restart after being stopped
      Stopped = false;
      //zct: lcd_reset_alert_level();
      gcode_LastN = Stopped_gcode_LastN;
      FlushSerialRequestResend();
    break;
    }
  }

  else if(code_seen('T'))
  {
    tmp_extruder = code_value();
    if(tmp_extruder >= EXTRUDERS) {
      SERIAL_ECHO_START;
      SERIAL_ECHO("T");
      SERIAL_ECHO(tmp_extruder);
      SERIAL_ECHOLN(MSG_INVALID_EXTRUDER);
    }
    else {
      boolean make_move = false;
      if(code_seen('F')) {
        make_move = true;
        next_feedrate = code_value();
        if(next_feedrate > 0.0) {
          feedrate = next_feedrate;
        }
      }
      #if EXTRUDERS > 1
      if(tmp_extruder != active_extruder) {
        // Save current position to return to after applying extruder offset
        memcpy(destination, current_position, sizeof(destination));
      #ifdef DUAL_X_CARRIAGE
        if (dual_x_carriage_mode == DXC_AUTO_PARK_MODE && Stopped == false &&
            (delayed_move_time != 0 || current_position[X_AXIS] != x_home_pos(active_extruder)))
        {
          // Park old head: 1) raise 2) move to park position 3) lower
          plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS] + TOOLCHANGE_PARK_ZLIFT,
                current_position[E_AXIS], max_feedrate[Z_AXIS], active_extruder);
          plan_buffer_line(x_home_pos(active_extruder), current_position[Y_AXIS], current_position[Z_AXIS] + TOOLCHANGE_PARK_ZLIFT,
                current_position[E_AXIS], max_feedrate[X_AXIS], active_extruder);
          plan_buffer_line(x_home_pos(active_extruder), current_position[Y_AXIS], current_position[Z_AXIS],
                current_position[E_AXIS], max_feedrate[Z_AXIS], active_extruder);
          st_synchronize();
        }

        // apply Y & Z extruder offset (x offset is already used in determining home pos)
        current_position[Y_AXIS] = current_position[Y_AXIS] -
                     extruder_offset[Y_AXIS][active_extruder] +
                     extruder_offset[Y_AXIS][tmp_extruder];
        current_position[Z_AXIS] = current_position[Z_AXIS] -
                     extruder_offset[Z_AXIS][active_extruder] +
                     extruder_offset[Z_AXIS][tmp_extruder];

        active_extruder = tmp_extruder;

        // This function resets the max/min values - the current position may be overwritten below.
        axis_is_at_home(X_AXIS);

        if (dual_x_carriage_mode == DXC_FULL_CONTROL_MODE)
        {
          current_position[X_AXIS] = inactive_extruder_x_pos;
          inactive_extruder_x_pos = destination[X_AXIS];
        }
        else if (dual_x_carriage_mode == DXC_DUPLICATION_MODE)
        {
          active_extruder_parked = (active_extruder == 0); // this triggers the second extruder to move into the duplication position
          if (active_extruder == 0 || active_extruder_parked)
            current_position[X_AXIS] = inactive_extruder_x_pos;
          else
            current_position[X_AXIS] = destination[X_AXIS] + duplicate_extruder_x_offset;
          inactive_extruder_x_pos = destination[X_AXIS];
          extruder_duplication_enabled = false;
        }
        else
        {
          // record raised toolhead position for use by unpark
          memcpy(raised_parked_position, current_position, sizeof(raised_parked_position));
          raised_parked_position[Z_AXIS] += TOOLCHANGE_UNPARK_ZLIFT;
          active_extruder_parked = true;
          delayed_move_time = 0;
        }
      #else
        // Offset extruder (only by XY)
        int i;
        for(i = 0; i < 2; i++) {
           current_position[i] = current_position[i] -
                                 extruder_offset[i][active_extruder] +
                                 extruder_offset[i][tmp_extruder];
        }
        // Set the new active extruder and position
        active_extruder = tmp_extruder;
      #endif //else DUAL_X_CARRIAGE
#ifdef DELTA

  calculate_delta(current_position); // change cartesian kinematic  to  delta kinematic;
   //sent position to plan_set_position();
  plan_set_position(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS],current_position[E_AXIS]);

#else
        plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);

#endif
        // Move to the old position if 'F' was in the parameters
        if(make_move && Stopped == false) {
           prepare_move();
        }
      }
      #endif
      SERIAL_ECHO_START;
      SERIAL_ECHO(MSG_ACTIVE_EXTRUDER);
      SERIAL_PROTOCOLLN((int)active_extruder);
    }
  }

  else
  {
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM(MSG_UNKNOWN_COMMAND);
    SERIAL_ECHO(cmdbuffer[bufindr]);
    SERIAL_ECHOLNPGM("\"");
  }

  ClearToSend();
}

void FlushSerialRequestResend()
{
  //char cmdbuffer[bufindr][100]="Resend:";
  MYSERIAL.flush();
  SERIAL_PROTOCOLPGM(MSG_RESEND);
  SERIAL_PROTOCOLLN(gcode_LastN + 1);
  ClearToSend();
}

void ClearToSend()
{
  previous_millis_cmd = millis();
  #ifdef SDSUPPORT
  if(fromsd[bufindr])
    return;
  #endif //SDSUPPORT
  SERIAL_PROTOCOLLNPGM(MSG_OK);
}

void get_coordinates()
{
  bool seen[4]={false,false,false,false};
  for(int8_t i=0; i < NUM_AXIS; i++) {
    if(code_seen(axis_codes[i]))
    {
      destination[i] = (float)code_value() + (axis_relative_modes[i] || relative_mode)*current_position[i];
      seen[i]=true;
   if(i==Z_AXIS){
			tft_flash_write;	// save the resume timepoints in the sprout screen. (everytime z++)
			w_d_x_5_16=destination[Z_AXIS] ; // resume printing should be out of function ,when stopping at the Z updating line.
			if (w_d_x_8_18_check==1){
						destination[Z_AXIS] =destination[Z_AXIS]-z_value_offset; //compensation
						ECHO("Z has been compensated by:");
						ECHO(z_value_offset);

			}
   }
			if (w_d_x_8_18_checke==1){
			current_position[E_AXIS] =destination[E_AXIS]; //compensation
                        w_d_x_8_18_checke==0;
                        ECHO("E has been compensated");
//ECHO(e_value_offset);
			}

    }
    else destination[i] = current_position[i]; //Are these else lines really needed?
  }
  //if (w_d_x_8_18_check==1){
    //destination[Z_AXIS] =destination[Z_AXIS]-back_z_value.v;//compensation
  //}
  if(code_seen('F')) {
    next_feedrate = code_value();
    if(next_feedrate > 0.0) feedrate = next_feedrate;
  }
}

void get_arc_coordinates()
{
#ifdef SF_ARC_FIX
   bool relative_mode_backup = relative_mode;
   relative_mode = true;
#endif
   get_coordinates();
#ifdef SF_ARC_FIX
   relative_mode=relative_mode_backup;
#endif

   if(code_seen('I')) {
     offset[0] = code_value();
   }
   else {
     offset[0] = 0.0;
   }
   if(code_seen('J')) {
     offset[1] = code_value();
   }
   else {
     offset[1] = 0.0;
   }
}

void clamp_to_software_endstops(float target[3])
{
  if (min_software_endstops) {
    if (target[X_AXIS] < min_pos[X_AXIS]) target[X_AXIS] = min_pos[X_AXIS];
    if (target[Y_AXIS] < min_pos[Y_AXIS]) target[Y_AXIS] = min_pos[Y_AXIS];


    #ifdef ENABLE_AUTO_BED_LEVELING
      if (Z_PROBE_OFFSET_FROM_EXTRUDER < 0) negative_z_offset = negative_z_offset + Z_PROBE_OFFSET_FROM_EXTRUDER;
      if (add_homing[Z_AXIS] < 0) negative_z_offset = negative_z_offset + add_homing[Z_AXIS];
    #endif

    if (target[Z_AXIS] < min_pos[Z_AXIS]+negative_z_offset) target[Z_AXIS] = min_pos[Z_AXIS]+negative_z_offset;
  }

  if (max_software_endstops) {
    if (target[X_AXIS] > max_pos[X_AXIS]) target[X_AXIS] = max_pos[X_AXIS];
    if (target[Y_AXIS] > max_pos[Y_AXIS]) target[Y_AXIS] = max_pos[Y_AXIS];
    if (target[Z_AXIS] > max_pos[Z_AXIS]) target[Z_AXIS] = max_pos[Z_AXIS];
  }
}
void prepare_move()
{
  clamp_to_software_endstops(destination);
  previous_millis_cmd = millis();

  #ifdef SCARA //for now same as delta-code

float difference[NUM_AXIS];
for (int8_t i=0; i < NUM_AXIS; i++) {
	difference[i] = destination[i] - current_position[i];
}

float cartesian_mm = sqrt(	sq(difference[X_AXIS]) +
							sq(difference[Y_AXIS]) +
							sq(difference[Z_AXIS]));
if (cartesian_mm < 0.000001) { cartesian_mm = abs(difference[E_AXIS]); }
if (cartesian_mm < 0.000001) { return; }
float seconds = 6000 * cartesian_mm / feedrate / feedmultiply;
int steps = max(1, int(scara_segments_per_second * seconds));
 //SERIAL_ECHOPGM("mm="); SERIAL_ECHO(cartesian_mm);
 //SERIAL_ECHOPGM(" seconds="); SERIAL_ECHO(seconds);
 //SERIAL_ECHOPGM(" steps="); SERIAL_ECHOLN(steps);
for (int s = 1; s <= steps; s++) {
	float fraction = float(s) / float(steps);
	for(int8_t i=0; i < NUM_AXIS; i++) {
		destination[i] = current_position[i] + difference[i] * fraction;
	}


	calculate_delta(destination);
         //SERIAL_ECHOPGM("destination[X_AXIS]="); SERIAL_ECHOLN(destination[X_AXIS]);
         //SERIAL_ECHOPGM("destination[Y_AXIS]="); SERIAL_ECHOLN(destination[Y_AXIS]);
         //SERIAL_ECHOPGM("destination[Z_AXIS]="); SERIAL_ECHOLN(destination[Z_AXIS]);
         //SERIAL_ECHOPGM("delta[X_AXIS]="); SERIAL_ECHOLN(delta[X_AXIS]);
         //SERIAL_ECHOPGM("delta[Y_AXIS]="); SERIAL_ECHOLN(delta[Y_AXIS]);
         //SERIAL_ECHOPGM("delta[Z_AXIS]="); SERIAL_ECHOLN(delta[Z_AXIS]);

	plan_buffer_line(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS],destination[E_AXIS], feedrate*feedmultiply/60/100.0,active_extruder);
}
#endif // SCARA

#ifdef DELTA
  float difference[NUM_AXIS];
  for (int8_t i=0; i < NUM_AXIS; i++) {
    difference[i] = destination[i] - current_position[i];
  }
  float cartesian_mm = sqrt(sq(difference[X_AXIS]) +
                            sq(difference[Y_AXIS]) +
                            sq(difference[Z_AXIS]));
  if (cartesian_mm < 0.000001) { cartesian_mm = abs(difference[E_AXIS]); }
  if (cartesian_mm < 0.000001) { return; }
  float seconds = 6000 * cartesian_mm / feedrate / feedmultiply;
  int steps = max(1, int(delta_segments_per_second * seconds));
  // SERIAL_ECHOPGM("mm="); SERIAL_ECHO(cartesian_mm);
  // SERIAL_ECHOPGM(" seconds="); SERIAL_ECHO(seconds);
  // SERIAL_ECHOPGM(" steps="); SERIAL_ECHOLN(steps);
  for (int s = 1; s <= steps; s++) {
    float fraction = float(s) / float(steps);
    for(int8_t i=0; i < NUM_AXIS; i++) {
      destination[i] = current_position[i] + difference[i] * fraction;
    }
    calculate_delta(destination);
    plan_buffer_line(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS],
                     destination[E_AXIS], feedrate*feedmultiply/60/100.0,
                     active_extruder);
  }

#endif // DELTA

#ifdef DUAL_X_CARRIAGE
  if (active_extruder_parked)
  {
    if (dual_x_carriage_mode == DXC_DUPLICATION_MODE && active_extruder == 0)
    {
      // move duplicate extruder into correct duplication position.
      plan_set_position(inactive_extruder_x_pos, current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
      plan_buffer_line(current_position[X_AXIS] + duplicate_extruder_x_offset, current_position[Y_AXIS], current_position[Z_AXIS],
          current_position[E_AXIS], max_feedrate[X_AXIS], 1);
      plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
      st_synchronize();
      extruder_duplication_enabled = true;
      active_extruder_parked = false;
    }
    else if (dual_x_carriage_mode == DXC_AUTO_PARK_MODE) // handle unparking of head
    {
      if (current_position[E_AXIS] == destination[E_AXIS])
      {
        // this is a travel move - skit it but keep track of current position (so that it can later
        // be used as start of first non-travel move)
        if (delayed_move_time != 0xFFFFFFFFUL)
        {
          memcpy(current_position, destination, sizeof(current_position));
          if (destination[Z_AXIS] > raised_parked_position[Z_AXIS])
            raised_parked_position[Z_AXIS] = destination[Z_AXIS];
          delayed_move_time = millis();
          return;
        }
      }
      delayed_move_time = 0;
      // unpark extruder: 1) raise, 2) move into starting XY position, 3) lower
      plan_buffer_line(raised_parked_position[X_AXIS], raised_parked_position[Y_AXIS], raised_parked_position[Z_AXIS],    current_position[E_AXIS], max_feedrate[Z_AXIS], active_extruder);
      plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], raised_parked_position[Z_AXIS],
          current_position[E_AXIS], min(max_feedrate[X_AXIS],max_feedrate[Y_AXIS]), active_extruder);
      plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS],
          current_position[E_AXIS], max_feedrate[Z_AXIS], active_extruder);
      active_extruder_parked = false;
    }
  }
#endif //DUAL_X_CARRIAGE

#if ! (defined DELTA || defined SCARA)
  // Do not use feedmultiply for E or Z only moves
  if( (current_position[X_AXIS] == destination [X_AXIS]) && (current_position[Y_AXIS] == destination [Y_AXIS])) {
      plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
  }
  else {
    plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate*feedmultiply/60/100.0, active_extruder);
  }
#endif // !(DELTA || SCARA)

  for(int8_t i=0; i < NUM_AXIS; i++) {
    current_position[i] = destination[i];
  }
}

void prepare_arc_move(char isclockwise) {
  float r = hypot(offset[X_AXIS], offset[Y_AXIS]); // Compute arc radius for mc_arc

  // Trace the arc
  mc_arc(current_position, destination, offset, X_AXIS, Y_AXIS, Z_AXIS, feedrate*feedmultiply/60/100.0, r, isclockwise, active_extruder);

  // As far as the parser is concerned, the position is now == target. In reality the
  // motion control system might still be processing the action and the real tool position
  // in any intermediate location.
  for(int8_t i=0; i < NUM_AXIS; i++) {
    current_position[i] = destination[i];
  }
  previous_millis_cmd = millis();
}

#if defined(CONTROLLERFAN_PIN) && CONTROLLERFAN_PIN > -1

#if defined(FAN_PIN)
  #if CONTROLLERFAN_PIN == FAN_PIN
    #error "You cannot set CONTROLLERFAN_PIN equal to FAN_PIN"
  #endif
#endif

unsigned long lastMotor = 0; //Save the time for when a motor was turned on last
unsigned long lastMotorCheck = 0;

void controllerFan()
{
  if ((millis() - lastMotorCheck) >= 2500) //Not a time critical function, so we only check every 2500ms
  {
    lastMotorCheck = millis();

    if(!READ(X_ENABLE_PIN) || !READ(Y_ENABLE_PIN) || !READ(Z_ENABLE_PIN) || (soft_pwm_bed > 0)
    #if EXTRUDERS > 2
       || !READ(E2_ENABLE_PIN)
    #endif
    #if EXTRUDER > 1
      #if defined(X2_ENABLE_PIN) && X2_ENABLE_PIN > -1
       || !READ(X2_ENABLE_PIN)
      #endif
       || !READ(E1_ENABLE_PIN)
    #endif
       || !READ(E0_ENABLE_PIN)) //If any of the drivers are enabled...
    {
      lastMotor = millis(); //... set time to NOW so the fan will turn on
    }

    if ((millis() - lastMotor) >= (CONTROLLERFAN_SECS*1000UL) || lastMotor == 0) //If the last time any driver was enabled, is longer since than CONTROLLERSEC...
    {
        digitalWrite(CONTROLLERFAN_PIN, 0);
        analogWrite(CONTROLLERFAN_PIN, 0);
    }
    else
    {
        // allows digital or PWM fan output to be used (see M42 handling)
        digitalWrite(CONTROLLERFAN_PIN, CONTROLLERFAN_SPEED);
        analogWrite(CONTROLLERFAN_PIN, CONTROLLERFAN_SPEED);
    }
  }
}
#endif

#ifdef TEMP_STAT_LEDS
static bool blue_led = false;
static bool red_led = false;
static uint32_t stat_update = 0;

void handle_status_leds(void) {
  float max_temp = 0.0;
  if(millis() > stat_update) {
    stat_update += 500; // Update every 0.5s
    for (int8_t cur_extruder = 0; cur_extruder < EXTRUDERS; ++cur_extruder) {
       max_temp = max(max_temp, degHotend(cur_extruder));
       max_temp = max(max_temp, degTargetHotend(cur_extruder));
    }
    #if defined(TEMP_BED_PIN) && TEMP_BED_PIN > -1
      max_temp = max(max_temp, degTargetBed());
      max_temp = max(max_temp, degBed());
    #endif
    if((max_temp > 55.0) && (red_led == false)) {
      digitalWrite(STAT_LED_RED, 1);
      digitalWrite(STAT_LED_BLUE, 0);
      red_led = true;
      blue_led = false;
    }
    if((max_temp < 54.0) && (blue_led == false)) {
      digitalWrite(STAT_LED_RED, 0);
      digitalWrite(STAT_LED_BLUE, 1);
      red_led = false;
      blue_led = true;
    }
  }
}
#endif

void manage_inactivity(bool ignore_stepper_queue/*=false*/) //default argument set in Marlin.h
{

#if defined(KILL_PIN) && KILL_PIN > -1
	static int killCount = 0;   // make the inactivity button a bit less responsive
   const int KILL_DELAY = 10000;
#endif

#if defined(HOME_PIN) && HOME_PIN > -1
   static int homeDebounceCount = 0;   // poor man's debouncing count
   const int HOME_DEBOUNCE_DELAY = 10000;
#endif


  if(buflen < (BUFSIZE-1))
    get_command();

  if( (millis() - previous_millis_cmd) >  max_inactive_time )
    if(max_inactive_time)
      kill();
  if(stepper_inactive_time)  {
    if( (millis() - previous_millis_cmd) >  stepper_inactive_time )
    {
      if(blocks_queued() == false && ignore_stepper_queue == false) {
        disable_x();
        disable_y();
        disable_z();
        disable_e0();
        disable_e1();
        disable_e2();
      }
    }
  }

  #ifdef CHDK //Check if pin should be set to LOW after M240 set it to HIGH
    if (chdkActive && (millis() - chdkHigh > CHDK_DELAY))
    {
      chdkActive = false;
      WRITE(CHDK, LOW);
    }
  #endif

  #if defined(KILL_PIN) && KILL_PIN > -1

    // Check if the kill button was pressed and wait just in case it was an accidental
    // key kill key press
    // -------------------------------------------------------------------------------
    if( 0 == READ(KILL_PIN) )
    {
       killCount++;
    }
    else if (killCount > 0)
    {
       killCount--;
    }
    // Exceeded threshold and we can confirm that it was not accidental
    // KILL the machine
    // ----------------------------------------------------------------
    if ( killCount >= KILL_DELAY)
    {
       kill();
    }
  #endif

#if defined(HOME_PIN) && HOME_PIN > -1
    // Check to see if we have to home, use poor man's debouncer
    // ---------------------------------------------------------
    if ( 0 == READ(HOME_PIN) )
    {
       if (homeDebounceCount == 0)
       {
          enquecommand_P((PSTR("G28")));
          homeDebounceCount++;
          //zct: LCD_ALERTMESSAGEPGM(MSG_AUTO_HOME);
       }
       else if (homeDebounceCount < HOME_DEBOUNCE_DELAY)
       {
          homeDebounceCount++;
       }
       else
       {
          homeDebounceCount = 0;
       }
    }
#endif

  #if defined(CONTROLLERFAN_PIN) && CONTROLLERFAN_PIN > -1
    controllerFan(); //Check if fan should be turned on to cool stepper drivers down
  #endif
  #ifdef EXTRUDER_RUNOUT_PREVENT
    if( (millis() - previous_millis_cmd) >  EXTRUDER_RUNOUT_SECONDS*1000 )
    if(degHotend(active_extruder)>EXTRUDER_RUNOUT_MINTEMP)
    {
     bool oldstatus=READ(E0_ENABLE_PIN);
     enable_e0();
     float oldepos=current_position[E_AXIS];
     float oldedes=destination[E_AXIS];
     plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS],
                      destination[E_AXIS]+EXTRUDER_RUNOUT_EXTRUDE*EXTRUDER_RUNOUT_ESTEPS/axis_steps_per_unit[E_AXIS],
                      EXTRUDER_RUNOUT_SPEED/60.*EXTRUDER_RUNOUT_ESTEPS/axis_steps_per_unit[E_AXIS], active_extruder);
     current_position[E_AXIS]=oldepos;
     destination[E_AXIS]=oldedes;
     plan_set_e_position(oldepos);
     previous_millis_cmd=millis();
     st_synchronize();
     WRITE(E0_ENABLE_PIN,oldstatus);
    }
  #endif
  #if defined(DUAL_X_CARRIAGE)
    // handle delayed move timeout
    if (delayed_move_time != 0 && (millis() - delayed_move_time) > 1000 && Stopped == false)
    {
      // travel moves have been received so enact them
      delayed_move_time = 0xFFFFFFFFUL; // force moves to be done
      memcpy(destination,current_position,sizeof(destination));
      prepare_move();
    }
  #endif
  #ifdef TEMP_STAT_LEDS
      handle_status_leds();
  #endif
	  //if(!block_buffer_pause)
  check_axes_activity();
}

void kill()
{
  cli(); // Stop interrupts
  disable_heater();

  disable_x();
  disable_y();
  disable_z();
  disable_e0();
  disable_e1();
  disable_e2();

#if defined(PS_ON_PIN) && PS_ON_PIN > -1
  pinMode(PS_ON_PIN,INPUT);
#endif
  SERIAL_ERROR_START;
  SERIAL_ERRORLNPGM(MSG_ERR_KILLED);
  //zct: LCD_ALERTMESSAGEPGM(MSG_KILLED);

  // FMC small patch to update the LCD before ending
  sei();   // enable interrupts
  //zct: for ( int i=5; i--; lcd_update())
   //zct:{
     //zct: delay(200);
   //zct:}
  cli();   // disable interrupts
  suicide();
  while(1) { /* Intentionally left empty */ } // Wait for reset
}

void Stop()
{
  disable_heater();
  if(Stopped == false) {
    Stopped = true;
    Stopped_gcode_LastN = gcode_LastN; // Save last g_code for restart
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM(MSG_ERR_STOPPED);
      //zct:LCD_MESSAGEPGM(MSG_STOPPED);
  }
}

bool IsStopped() { return Stopped; };

#ifdef FAST_PWM_FAN
void setPwmFrequency(uint8_t pin, int val)
{
  val &= 0x07;
  switch(digitalPinToTimer(pin))
  {

    #if defined(TCCR0A)
    case TIMER0A:
    case TIMER0B:
//         TCCR0B &= ~(_BV(CS00) | _BV(CS01) | _BV(CS02));
//         TCCR0B |= val;
         break;
    #endif

    #if defined(TCCR1A)
    case TIMER1A:
    case TIMER1B:
//         TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
//         TCCR1B |= val;
         break;
    #endif

    #if defined(TCCR2)
    case TIMER2:
    case TIMER2:
         TCCR2 &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
         TCCR2 |= val;
         break;
    #endif

    #if defined(TCCR2A)
    case TIMER2A:
    case TIMER2B:
         TCCR2B &= ~(_BV(CS20) | _BV(CS21) | _BV(CS22));
         TCCR2B |= val;
         break;
    #endif

    #if defined(TCCR3A)
    case TIMER3A:
    case TIMER3B:
    case TIMER3C:
         TCCR3B &= ~(_BV(CS30) | _BV(CS31) | _BV(CS32));
         TCCR3B |= val;
         break;
    #endif

    #if defined(TCCR4A)
    case TIMER4A:
    case TIMER4B:
    case TIMER4C:
         TCCR4B &= ~(_BV(CS40) | _BV(CS41) | _BV(CS42));
         TCCR4B |= val;
         break;
   #endif

    #if defined(TCCR5A)
    case TIMER5A:
    case TIMER5B:
    case TIMER5C:
         TCCR5B &= ~(_BV(CS50) | _BV(CS51) | _BV(CS52));
         TCCR5B |= val;
         break;
   #endif

  }
}
#endif //FAST_PWM_FAN

bool setTargetedHotend(int code){
  tmp_extruder = active_extruder;
  if(code_seen('T')) {
    tmp_extruder = code_value();
    if(tmp_extruder >= EXTRUDERS) {
      SERIAL_ECHO_START;
      switch(code){
        case 104:
          SERIAL_ECHO(MSG_M104_INVALID_EXTRUDER);
          break;
        case 105:
          SERIAL_ECHO(MSG_M105_INVALID_EXTRUDER);
          break;
        case 109:
          SERIAL_ECHO(MSG_M109_INVALID_EXTRUDER);
          break;
        case 218:
          SERIAL_ECHO(MSG_M218_INVALID_EXTRUDER);
          break;
        case 221:
          SERIAL_ECHO(MSG_M221_INVALID_EXTRUDER);
          break;
      }
      SERIAL_ECHOLN(tmp_extruder);
      return true;
    }
  }
  return false;
}


float calculate_volumetric_multiplier(float diameter) {
	float area = .0;
	float radius = .0;

	radius = diameter * .5;
	if (! volumetric_enabled || radius == 0) {
		area = 1;
	}
	else {
		area = M_PI * pow(radius, 2);
	}

	return 1.0 / area;
}

void calculate_volumetric_multipliers() {
  for (int i=0; i<EXTRUDERS; i++)
  	volumetric_multiplier[i] = calculate_volumetric_multiplier(filament_size[i]);
}

/* filament load and unload */






static void _axis_move(const char *name, int axis, int min, int max) {
	//ECHO("moving...");
    destination[axis] += movedirect*axisstep;
    if (min_software_endstops && destination[axis] < min) destination[axis] = min;
    if (max_software_endstops && destination[axis] > max) destination[axis] = max;
	//movingcount[axis]=0;
	//ECHO("here is the moving");
	//ECHO("distance x moving");
	//ECHO(current_position[X_AXIS]);
	//ECHO("distance y moving");
	//ECHO(current_position[Y_AXIS]);
	//ECHO("distance z moving");
	//ECHO(current_position[Z_AXIS]);

	plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], FEEDRATE_Z_A/60, active_extruder);
    current_position[axis]=destination[axis];
}


static void axis_move_x() { _axis_move(PSTR("X"), X_AXIS, X_MIN_POS, X_MAX_POS); }
static void axis_move_y() { _axis_move(PSTR("Y"), Y_AXIS, Y_MIN_POS, Y_MAX_POS); }
static void axis_move_z() { _axis_move(PSTR("Z"), Z_AXIS, Z_MIN_POS, Z_MAX_POS); }


uint8_t tft_rx()
	{
		
		char cmd[30];
		rx_msg=xserial.hearfromtft();
		if (rx_msg.length()<1)
		{
			return -1;
		}
		else if (rx_msg.startsWith("D"))
		{
		process_calibration();
			return -1;
		}
		else if (rx_msg.startsWith("E"))
		{
			ECHO(rx_msg);
										if(rx_msg==TFT_MSG_B_READY2GO)
										{
												uart2ready=true;
												ECHO("touch screen iniatial have been finished :)");
												tft_sys_welcome;
												delay(1500);
												tft_sys_start;
												return TFTCONNECTED;
										}

										else if (rx_msg==TFT_MSG_V_STOPPRINT)
										{ // the same with TFT_MSG_V_ABANDONR
											ECHO("STOP HERE");
											w_d_x_8_18_check=0;
											w_d_x_8_18_checke=0;
											card.sdprinting = false; // stop reading from sd card into buff
											card.closefile();
											quickStop();
											block_buffer_tail=block_buffer_head;
											//enquecommand_P(PSTR("G0 X10 Y10 Z100 F1800"));

											destination[0]=10;
											destination[1]=10;

											destination[2]=PLATE_BOT_POS;
										
											
											for (int i=0;i<4;i++)
												{
													back_z_value.zchar[i]=EEPROM.read(i+55);
												}
													z_value_offset=back_z_value.v;//+0.10;
					destination[2]=constrain(PLATE_BOT_POS-z_value_offset,0,PLATE_BOT_POS);
					feedrate=FEEDRATE_Z_G;
													//ECHO("endcheck is");                                                                             
													enable_endstops(true);
													prepare_move();	
										
											buflen=0; // stop processing from buff
											bufindw=0;
											bufindr=0;										
											autotempShutdown();
											cancel_heatup = true; //  Jump out of the heating loop (if any)
											setTargetHotend0(0);  // cool the extruder
											monitormode=false; // Jump out of the monitor mode loop
											invalid4resume=false;// invalid the resume printing entrance

											return PRINTSTOPED;
										}
										else if (rx_msg==TFT_MSG_V_ABANDONR)
										{// the same with TFT_MSG_V_STOPPRINT
											ECHO("STOP HERE");
											w_d_x_8_18_check=0;
											w_d_x_8_18_checke=0;
											card.sdprinting = false; // stop reading from sd card into buff
											card.closefile();
											quickStop();
											block_buffer_tail=block_buffer_head;
											//enquecommand_P(PSTR("G0 X10 Y10 Z100 F1800"));										
											destination[0]=10;
											destination[1]=10;
											
											for (int i=0;i<4;i++)
												{
													back_z_value.zchar[i]=EEPROM.read(i+55);
												}
													z_value_offset=back_z_value.v;//+0.10;
					destination[2]=constrain(PLATE_BOT_POS-z_value_offset,0,PLATE_BOT_POS);
					feedrate=FEEDRATE_Z_G;
													//ECHO("endcheck is");
                                                                                  // ECHO(whatisendstop());
													enable_endstops(true);
													prepare_move();
											buflen=0; // stop processing from buff
											bufindw=0;
											bufindr=0;

											autotempShutdown();
											cancel_heatup = true; //  Jump out of the heating loop (if any)
											setTargetHotend0(0);  // cool the extruder
											monitormode=false; // Jump out of the monitor mode loop
											invalid4resume=false;// invalid the resume printing entrance

											return PRINTSTOPED;
										}

										else if (rx_msg==TFT_MSG_V_PAUSEPRINT)
										{
                                               	  monitormode=false; // exit monitormode in case of fill or retract
												  card.pauseSDPrint();
                                                  //quickStop();
                                                  buflen_stack=buflen;
												  bufindr_stack=bufindr;
												  bufindw_stack=bufindw;
												  feedrate_stack=feedrate;
												  saved_feedmultiply = feedmultiply;
												  feedmultiply = 100;
												  buflen=0;
												  bufindr=0;
												  bufindw=0;
                                                  st_synchronize();
												  tft_pause_done;
                                                  x_pos_stack=current_position[0];
												  y_pos_stack=current_position[1];
													//ECHO("stack:");
													//ECHO(current_position[X_AXIS]);
													//ECHO(current_position[Y_AXIS]);
												  feedrate=180;
                                                  destination[2]+=10.0;
                                                  prepare_move();
                                                  st_synchronize();
												  delay(800);
                                                  feedrate=homing_feedrate[X_AXIS];
                                                  destination[0]=15.0;//X AXIS
												  destination[1]=15.0;//Y AXIS
                                                  enable_endstops(true);
                                                  prepare_move();
                                                  enable_x();
                                                  enable_y();
                                                  st_synchronize();
												  tft_resume_valid;
												return PRINTPAUSED;
										}
									   else if (rx_msg==TFT_MSG_V_RESUMEPRINT)
										{
												monitormode=true;
                                                feedrate=800;
                                                destination[0]=x_pos_stack; //X AXIS
                                                destination[1]=y_pos_stack;// Y AXIS
												prepare_move();
												st_synchronize();
                                                delay(800);
												feedrate=180;
												destination[Z_AXIS]-=10;
											    prepare_move();
												st_synchronize();
                                                card.startFileprint();
                                                buflen=buflen_stack;
												bufindr=bufindr_stack;
												bufindw=bufindw_stack;
												feedrate=feedrate_stack;
												feedmultiply=saved_feedmultiply;
												return PRINTRESUMED;
										}
// The rx below is about the filament operation
											else if (rx_msg==TFT_MSG_V_filafil_H)
												{// START HEATING FOR filling FILA
														ECHO("heating");
														filamove=fillmove;
														plan_heat_and_sound();
														tft_heatingloop_set;
														heat4fila=true;
														fanSpeed=degHotend(active_extruder)/plaPreheatHotendTemp;
														return FILAOPERATE;
												}
											 else if (rx_msg==TFT_MSG_V_filaret_H)
												{// START HEATING FOR retracting FILA
														ECHO("heating");
														filamove=retrmove;
														plan_heat_and_sound();
														tft_heatingloop_set;
														heat4fila=true;
														fanSpeed=degHotend(active_extruder)/plaPreheatHotendTemp;
															return FILAOPERATE;
													}
												else if (rx_msg==TFT_MSG_V_fila_E)
															{
																	ECHO("stop here");
																	quickStop(); //stop all the stepper
																	emovable=false;
																	filamove=nomove;//stop the servo in the extrude
																	tft_heatingloop_clear;
																	return FILAOPERATE;
																}
											else if (rx_msg==TFT_MSG_V_fila_COOL)
																{
																	ECHO("let us cool the extruder");
																	setTargetHotend0(0);
																	return FILAOPERATE;
																}
		return -1;
		}
		else if (rx_msg.startsWith("N"))
		{
				fileselected=rx_msg;
				B_fileselected=true;
				return FILESELECTED;
		}
		else if (rx_msg.startsWith("R"))
		{ // initiate the states when power off resume printing
				fileselected=rx_msg;
				B_fileselected=true;
				
				w_d_x_8_18_check=1;
				w_d_x_8_18_checke=1;				
				w_h_4_check=1;
				w_h_4_2_0=EEPROM.read(0);
				w_h_4_2_1=EEPROM.read(1);
				w_h_4_2_2=EEPROM.read(2);
				w_h_4_2_3=EEPROM.read(3);
				w_h_4_2_4=EEPROM.read(4);
				w_h_4_2_5=EEPROM.read(5);
				w_h_4_2_6=EEPROM.read(6);
				w_h_4_2_7=EEPROM.read(7);
         //current_position[Z_AXIS]=EEPROM.read(8);
				w_h_4_1=w_h_4_2_0+w_h_4_2_1*256+w_h_4_2_2*65536+w_h_4_2_3*16777216;
				w_h_4_2=w_h_4_2_4+w_h_4_2_5*256+w_h_4_2_6*65536+w_h_4_2_7*16777216;

				for (int i=0;i<4;i++){
					back_z_value.zchar[i]=EEPROM.read(i+55);
				}
				if(!w_h_4_1&&!w_h_4_2)
			{	return tft_sd_memory_fail;   }
			    else
			 {   	pwroff_re_proc();
				return FILESELECTED;    }
		}
		else if (rx_msg.startsWith("A"))
		{
			ECHO(rx_msg);
										if (rx_msg==TFT_MSG_A_AXIS_UP)
										{
													ECHO("up");
													movedirect=-1;
													axis_move_z();
													return AXISMOVING;
										}
										else if (rx_msg==TFT_MSG_A_AXIS_DOWN)
										{
														ECHO("down");
														movedirect=1;
														axis_move_z();
													return AXISMOVING;
										}
										else if (rx_msg==TFT_MSG_A_AXIS_XRIGHT)
										{
															ECHO("XRIGHT");
															movedirect=-1;
															axis_move_x();
													return AXISMOVING;
										}

										else if (rx_msg==TFT_MSG_A_AXIS_XLEFT)
										{
																ECHO("XLEFT");
																movedirect=1;
																axis_move_x();
													return AXISMOVING;
										}
										else if (rx_msg==TFT_MSG_A_AXIS_YRIGHT)
										{
																ECHO("YRIGHT");
																movedirect=-1;
																axis_move_y();
													return AXISMOVING;
										}

										else if (rx_msg==TFT_MSG_A_AXIS_YLEFT)
										{
																	ECHO("YLEFT");
																	movedirect=1;
																	axis_move_y();
													return AXISMOVING;
										}
			return -1;
		}
		else if (rx_msg.startsWith("F"))
		{
			ECHO(rx_msg);
								 if (rx_msg==TFT_MSG_F_AXIS_HOME)
									{
											enquecommand_P(PSTR("G28"));
											return AUXFCTDONE;
									}
									else if (rx_msg==TFT_MSG_F_SHUT)
									{
												enquecommand_P(PSTR("M33"));
												return AUXFCTDONE;
									}
									else if (rx_msg==TFT_MSG_F_XHOME)
									{
													enable_endstops(true);
													HOMEAXIS(X);
													return AUXFCTDONE;
									}
									else if (rx_msg==TFT_MSG_F_YHOME)
									{
													enable_endstops(true);
													HOMEAXIS(Y);
													return AUXFCTDONE;
									}
									else if (rx_msg==TFT_MSG_F_ZHOME)
									{
													 enable_endstops(true);
													HOMEAXIS(Z);
													return AUXFCTDONE;
									}
									else if (rx_msg==TFT_MSG_F_COOLDOWN)
									{
										  fanSpeed=255;
										   setTargetHotend0(0);
										  tft_heatingloop_set;
										  fanop=true;
										  return AUXFCTDONE;
									}
									else if (rx_msg==TFT_MSG_F_COOLDONE)
									{
										  fanSpeed=0;
										  tft_heatingloop_clear;
										   fanop=false;
										  return AUXFCTDONE;
									}
									else if (rx_msg==TFT_MSG_F_PREHEAT)
									{
											setTargetHotend0(plaPreheatHotendTemp);
											tft_heatingloop_set;
											fanSpeed = plaPreheatFanSpeed;
											fanop=true;
										   return AUXFCTDONE;
									}
									else if (rx_msg==TFT_MSG_F_PREHEATDONE)
									{
										  tft_heatingloop_clear;
										  fanop=false;
										  return AUXFCTDONE;
									}
									return -1;
		}
		else if (rx_msg.startsWith("L"))
		{ // manual leveling 
			ECHO(rx_msg);
										if (rx_msg==TFT_MSG_L_A)
										{
													sprintf_P(cmd, PSTR("M162 X%i Y%i"), LEVEL_AX,LEVEL_AY);
													enquecommand(cmd);						//moving the extruder block
													return AXISMOVING;
										}
										else if (rx_msg==TFT_MSG_L_B)
										{
													sprintf_P(cmd, PSTR("M162 X%i Y%i"), LEVEL_BX,LEVEL_BY);
													enquecommand(cmd);						//moving the extruder block
													return AXISMOVING;
										}
										else if (rx_msg==TFT_MSG_L_C)
										{
													sprintf_P(cmd, PSTR("M162 X%i Y%i"), LEVEL_CX,LEVEL_CY);
													enquecommand(cmd);						//moving the extruder block
													return AXISMOVING;
										}
										else if (rx_msg==TFT_MSG_L_Z)
										{
													retract_z_probe();
													enquecommand_P(PSTR("G28"));
													return AXISMOVING;
										}
			return -1;
		}
		else if (rx_msg.startsWith("S"))
		{
			ECHO(rx_msg);
									if (rx_msg==TFT_MSG_S_AXIS_STEP1)
									{
											ECHO("HIGH ACCURACY");
											axisstep=steplength1;
											return SETTINGDONE;
									}
									else	if (rx_msg==TFT_MSG_S_AXIS_STEP2)
									{
												ECHO("low ACCURACY");
												axisstep=steplength2;
												return SETTINGDONE;
									}
									return -1;
		}
			return -1;
}

void tft_status_temperature_update()
	{
	uint16_t extrudetemp;
	static uint16_t temp_old;
	extrudetemp=degHotend(0);
	if (temp_old-extrudetemp>0.99||extrudetemp-temp_old>0.99)
	{
		tft_thermo_set(extrudetemp);
		temp_old=extrudetemp;
	}
	return;
}

void tft_status_monitor_update()
{
	uint16_t percentage=card.percentDone();
	static uint16_t percentage_old;
	static double x_old;
	static double y_old;
	static double x_buf;
	static double y_buf;
	static double f_buf;
	static double f_old;
	//destination
	double x=floor((X_MAX_POS-destination[X_AXIS])/X_MAX_POS*(qtx_max-qtx_min)+qtx_min);
	double y=floor(destination[Y_AXIS]/Y_MAX_POS*(qty_max-qty_min)+qty_min);
	double f=floor(feedrate);
	//double x=floor((X_MAX_POS-current_position[X_AXIS])/X_MAX_POS*(qtx_max-qtx_min)+qtx_min);
	//double y=floor(current_position[Y_AXIS]/Y_MAX_POS*(qty_max-qty_min)+qty_min);
		if (x-x_old>1.99||x_old-x>1.99||y_old-y>1.99||y-y_old>1.99)
	{
						if((x_buf+y_buf)>0){
								//tft_pos_qt_set(x_buf,y_buf,f_buf);
						}
						x_buf=x;
						y_buf=y;
						f_buf=f;

						y_old=y;
						x_old=x;
						f_old=f;
	}

// the finished process is defined here;
		if (percentage-percentage_old>0.99)
	{
			if (percentage>99)
			{
					tft_progressbar_set(100);
					return;
			}
			else
			{
					tft_progressbar_set(percentage);
					percentage_old=percentage;
			}
	}
	return;
}

void tft_update(){
	tft_status_temperature_update(); 
	if (monitormode)
	{
		//ECHO("Monitormode");
			tft_status_monitor_update();
			//tft_status_temperature_update();
			tft_rx();
			return;
	}
	// not monitor mode is listed as below
	uint8_t rxmode=tft_rx(); // read from the touch screen;
	bool z_min_detected=READ(Z_MIN_PIN) != Z_MIN_ENDSTOP_INVERTING;
	bool y_min_detected=READ(Y_MIN_PIN) != Y_MIN_ENDSTOP_INVERTING;
	bool x_min_detected=READ(X_MIN_PIN) != X_MIN_ENDSTOP_INVERTING;
	if(z_min_detected!=old_z_detected||x_min_detected!=old_x_detected||y_min_detected!=old_y_detected)
	{
		if (z_min_detected||y_min_detected||x_min_detected)// z switch turning on : ZCT
		{
				tft_light_on;
		}
		else
		{
				tft_light_off;
		}
		old_x_detected=x_min_detected;
		old_y_detected=y_min_detected;
		old_z_detected=z_min_detected;
	}
	if (fanop)
	{
		//tft_status_temperature_update();
		return;
	}
	// filament fill and retraction updated is as below
   if (heat4fila)
   {
	   if (degHotend(0)<plaPreheatHotendTemp)
	   {
		      //tft_status_temperature_update();// should be a loop
			  emovable=false;
	   }
	   else
	   {
		   tft_filamove_enter;
		   heat4fila=false;
		   emovable=true;
		//   enquecommand_P(PSTR("G91"));
           //enquecommand_P(PSTR("G1 Z2 F5000"));
		  // enquecommand_P(PSTR("G90"));
	   }

   }
   else // close the fan when extruder is below 45 celsius.
	{
   				if (fanSpeed>0&&degHotend(0)<45)
				{
						fanSpeed==0;
				}
   }
   	if (emovable)
	{
	emovecount=emovecount-1;
	//ECHO(emovecount);
	}
      if (emovable&&filamove==fillmove&&emovecount==0)
   {
	    enquecommand_P((PSTR("M82")));
		enquecommand_P((PSTR("G92  E0 ")));
		enquecommand_P((PSTR("G1  F300 E2")));   // should be a loop
		emovecount=EINTERVAL;
		return; // for the filament fill
   }
      else if (emovable&&filamove==retrmove&&emovecount==0)
   {
	    enquecommand_P((PSTR("M82")));
		enquecommand_P((PSTR("G92  E0 ")));
		enquecommand_P((PSTR("G1  F300 E-2")));   // should be a loop
		emovecount=EINTERVAL;
		return;// for the filament retract
   }
// the main stream is listed as below; actually, the monitormode should be in this stream, maybe it will be in the future.
	if (!uart2ready)//&&0) // hand shake happened here
	{
		return;
	// when the tft not ready;
	//1. reset will be this state
	//2. no tft will be this state
	}

	// when the tft  ready;
	//1. SD card dir updated(C)
	//2. process state updated(C)
	//3.
	else{
		//SCAN THE SD States ;
		uint16_t fileCnt=0;

		bool sdpindetect = IS_SD_INSERTED;
		//delay(555);
		//ECHO(sdpindetect);
		//sdpindetect= IS_SD_INSERTED && sdpindetect; // tricky: prevent shock: only "detected 3 times" counts
		//delay(55);
		//sdpindetect= IS_SD_INSERTED && sdpindetect; // tricky: prevent shock: only "detected 3 times" counts
		if(sdpindetect != oldcardstatus){
			oldcardstatus=sdpindetect;
			//ECHO("oldcardstatus is:");
			//ECHO(oldcardstatus);

			if(oldcardstatus)
			{
			    //when insert
				delay(800);
				tft_sd_detected;
				card.initsd();
				ECHO("card inserted");
				fileCnt = card.getnrfilenames(); //update the file count when inserted
			}
			else if(!oldcardstatus)
			{
				//when release
				tft_sd_released;
				card.release();
				ECHO("card released");
				fileCnt=0;
				tft_sd_del;
			}
// update the filenames array in the PS-LCD
			for(uint16_t i=0;i<fileCnt;i++)
			{
				if(oldcardstatus){
					card.getfilename(fileCnt-1-i);
					if(strstr(card.longFilename,".gcode")!=NULL){// filter off the file not gcode type
					// filter the gcode among all files should be here
						tft_sd_add(card.longFilename);
						//echo_sd_add(card.longFilename);
					}
					//MYSERIAL.print("add item ");
					//MYSERIAL.println(i+1);
				}
			}
			if(oldcardstatus) {tft_sd_detected;}
		}//IS_SD_INSERTED != oldcardstatus

	// selectd the file and start to print;
		if (B_fileselected&&!invalid4resume)
		{
			B_fileselected=false;
			 ECHO("file selected, start print, oh yeer XD");
			// tft_getfilename;
			 char* autoname=&fileselected[2];
			//ECHO("index is:");
			//ECHO(String(autoname).indexOf("."));
			//ECHO("last index is:");
			//ECHO(String(autoname).lastIndexOf("."));
			 //bool waiting4rx=true;
			// querytype=queryfilename;
			 //while(waiting4rx){waiting4rx=(tft_rx()!=RESPONSED);}

				//for(int8_t i=2;i<fileselected.length();i++)
				//	autoname[i-2]=fileselected[i];
				//zct: check for the short filename
	// Emm, the file name should be selected from the value from the Mega2560 rather than the PS-LCD;
	// The index should be from the PS-LCD, should be corrected someday xD : ZCT
			 char* shortname;
			 uint16_t fileCnt = card.getnrfilenames();
			 bool found=false;
	         for(uint16_t i=0;(i<fileCnt)&&!found;i++)
			 {
					//card.getfilename(i);
					#ifndef SDCARD_RATHERRECENTFIRST
							card.getfilename(i);
					#else
							card.getfilename(fileCnt-1-i);
					#endif
					if (strncmp(card.longFilename,autoname,String(autoname).lastIndexOf("."))==0)
					{
						shortname=&card.filename[0];
						found=true;
					}
			 }

			 if(found)
			 {
					card.click_sdprint(shortname);
					monitormode=true;
			 }
			 else
			 {ECHO("the selected filename is not found on the sd card");
			 }
			 return;
		}
	// refresh the status monitor;

	}

}

void pwroff_re_proc()
	{
	enquecommand_P(PSTR("M109 T0 S200"));
	buflen = (BUFSIZE-1);
	process_commands(); 
	
	
	z_value_offset=back_z_value.v;//+0.10;
	current_position[E_AXIS]=-65535;
	analogWrite(FAN_SAN_PIN, 225);
    fanSpeed=255;
        //plan_set_e_position(current_position[E_AXIS]);  // G92 E0;
	 ECHO("the offset value stored is:");
	 ECHO(z_value_offset);
	//   Home X/Y Axis one at a time //
     BEGIN_PRINT_SOUND();

     saved_feedrate = feedrate;
     saved_feedmultiply = feedmultiply;
     feedmultiply = 100;
     previous_millis_cmd = millis();

     enable_endstops(true);

      for(int8_t i=0; i < NUM_AXIS; i++) {
        if (i!=2)
        {
			destination[i] = current_position[i];
        }
      }
      feedrate = 0.0;

      home_all_axis = !((code_seen(axis_codes[X_AXIS])) || (code_seen(axis_codes[Y_AXIS])) || (code_seen(axis_codes[Z_AXIS])));

      if((home_all_axis) || (code_seen(axis_codes[X_AXIS])))
      {
        HOMEAXIS(X);
      }

      if((home_all_axis) || (code_seen(axis_codes[Y_AXIS]))) {
        HOMEAXIS(Y);
      }

      if(code_seen(axis_codes[X_AXIS]))
      {
        if(code_value_long() != 0) {
       current_position[X_AXIS]=code_value()+add_homing[X_AXIS];
        }
      }

      if(code_seen(axis_codes[Y_AXIS])) {
        if(code_value_long() != 0) {
       current_position[Y_AXIS]=code_value()+add_homing[Y_AXIS];
        }
      }
      current_position[Z_AXIS] =back_z_value.v;
      plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);

      #ifdef ENDSTOPS_ONLY_FOR_HOMING
        enable_endstops(false);
      #endif

      feedrate = saved_feedrate;
      feedmultiply = saved_feedmultiply;
      previous_millis_cmd = millis();
      endstops_hit_on_purpose();
	
	  enquecommand_P(PSTR("M109 T0 S200"));
	  buflen = (BUFSIZE-1);
	  process_commands(); 

	  tft_printing_enter;
	  tft_resume_enable;
	  invalid4resume=false;
}

bool cali_code_seen(char code)
{
  xserial.print("main_label_2.text=");
  strchr_position=rx_msg.indexOf(code);
  xserial.print(strchr_position);
  xserial.print("\r");
  return (strchr_position!= -1);//Return True if a character was found
}

String cali_code_value()
{
  return rx_msg.substring(strchr_position+1,rx_msg.indexOf("\r",strchr_position));
}

void process_calibration()
	{
 if(cali_code_seen('D'))
  {
  //  START_SOUND();
 // xserial.print("main_label_2.text=");
  //xserial.print(cali_code_value());
  //xserial.print("\r");
    switch((int)cali_code_value().toFloat())
    {
	case 11: //X axis move test in positive direction
		movedirect=-1;
	    axisstep=5*steplength2;
		manual_feedrate[0]=feedrate_test;
		axis_move_x();
	break;
	case 22: //X axis move test in negative direction
		movedirect=1;
	    axisstep=5*steplength2;
		manual_feedrate[0]=feedrate_test;
		axis_move_x();
	break;
	case 33: 
		enable_endstops(true);
		HOMEAXIS(X);
	break;
	case 44: //Y axis move test in positive direction
		movedirect=-1;
	    axisstep=5*steplength2;
		manual_feedrate[1]=feedrate_test;
		axis_move_y();
	break;
	case 55: //Y axis move test in negative direction
		movedirect=1;
	    axisstep=5*steplength2;
		manual_feedrate[1]=feedrate_test;
		axis_move_y();
	break;
	case 66: 
		enable_endstops(true);
		HOMEAXIS(Y);
	break;
	case 77: //Z axis move test in positive direction
		movedirect=-1;
	    axisstep=5*steplength1;
		manual_feedrate[2]=feedrate_test;
		axis_move_z();
	break;
	case 6039: //Z axis move test in negative direction
		movedirect=1;
	    axisstep=5*steplength1;
		manual_feedrate[2]=feedrate_test;
		axis_move_z();
	break;
	case 7018: 
		enable_endstops(true);
		HOMEAXIS(Z);
	break;
	case 1012: //Z axis move test in positive direction
		enquecommand_P((PSTR("M82")));
        enquecommand_P((PSTR("G92  E0 ")));
        enquecommand_P((PSTR("G1  F400 E2")));
	break;
	case 1023: //Z axis move test in negative direction
		enquecommand_P((PSTR("M82")));
        enquecommand_P((PSTR("G92  E0 ")));
        enquecommand_P((PSTR("G1  F400 E-2")));
	break;
	case 1034: 
        enquecommand_P((PSTR("G28")));
	break;
	case 1045: //Z axis move test in negative direction
        fanSpeed=255;
	break;
	case 1056: 
   
        fanSpeed=0;
	break;
	case 1067: //Z axis move test in negative direction
        analogWrite(FAN_SAN_PIN, 225);
	break;
	case 1078: 
        analogWrite(FAN_SAN_PIN, 0);
	break;
	case 1089: //Z axis move test in negative direction
        setTargetHotend0(plaPreheatHotendTemp);
	break;
	case 2013: 
        setTargetHotend0(0);
	break;
	case 2024: 
        enquecommand_P(PSTR("G28"));
	    enquecommand_P(PSTR("G29 E"));
	break;
	case 2035: // down the platform
		enquecommand_P(PSTR("G92"));
		enable_endstops(true);
		HOMEAXIS(Z);
        enquecommand_P(PSTR("G1 F300 Z150"));
	break;
	case 2046: // read feedrate from the 2560
		xserial.print("feedrate_read(");
		xserial.print(feedrate_test);
		xserial.print(")\r");
	   
	break;
	case 2057: 
        if(cali_code_seen('F'))
		  feedrate_test=cali_code_value().toFloat();
	break;
	case 2068: // read the compensation from the pin to the extruder 
		xserial.print("Z_offset(");
		xserial.print(zprobe_zoffset);
		xserial.print(")\r");
	break;
	case 2079: //write the compensation from the pin to the extruder 
		if(cali_code_seen('Z')){
			zprobe_zoffset=cali_code_value().toFloat();
			p2p_3mm_std_error.v=cali_code_value().toFloat();
		for (int i=0;i<4;i++){
			EEPROM.write(i+200,p2p_3mm_std_error.echar[i]);
		}
			xserial.print("indication(\"z_offset updated!\")\r");
		}
        
	break;
	case 3014: 
		enquecommand_P((PSTR("G91")));
        enquecommand_P((PSTR("G1 F100 Z-0.01")));
	break;
	case 3025: 
		enquecommand_P((PSTR("G91")));
        enquecommand_P((PSTR("G1 F100 Z0.01")));
	break;
	case 3036: 
      {
		int servo_index = -1;
		int servo_position = 0;
		if (code_seen('P'))
		servo_index =
		#if NUM_SERVOS > 0
			code_value();
				if (code_seen('S')) {
				  servo_position = code_value();
				  if ((servo_index >= 0) && (servo_index < NUM_SERVOS)) {
			#if defined (ENABLE_AUTO_BED_LEVELING) && (PROBE_SERVO_DEACTIVATION_DELAY > 0)
						  servos[servo_index].attach(0);
			#endif
						servos[servo_index].write(servo_position);
			#if defined (ENABLE_AUTO_BED_LEVELING) && (PROBE_SERVO_DEACTIVATION_DELAY > 0)
						  delay(PROBE_SERVO_DEACTIVATION_DELAY);
						  servos[servo_index].detach();
			#endif
				  }
				  else {
					SERIAL_ECHO_START;
					SERIAL_ECHO("Servo ");
					SERIAL_ECHO(servo_index);
					SERIAL_ECHOLN(" out of range");
				  }
				}
				else if (servo_index >= 0) {
				  SERIAL_PROTOCOL(MSG_OK);
				  SERIAL_PROTOCOL(" Servo ");
				  SERIAL_PROTOCOL(servo_index);
				  SERIAL_PROTOCOL(": ");
				  SERIAL_PROTOCOL(servos[servo_index].read());
				  SERIAL_PROTOCOLLN("");
				}
			  }
		#endif // NUM_SERVOS > 0
	break;
	case 3047: 
        enquecommand_P((PSTR("M402")));
	break;
	case 3058: 
        enquecommand_P((PSTR("M401")));
	break;
	case 3069: 
		xserial.print("servo_angle(");
		xserial.print(servo_endstop_angles[4]);
		xserial.print(")\r");
	break;
	case 4015: 
	
	if (cali_code_seen('A')){
		  servo_endstop_angles[4]=cali_code_value().toFloat();
		  servo_set_angle.v=cali_code_value().toFloat();
		for (int i=0;i<2;i++){
			EEPROM.write(i+204,servo_set_angle.schar[i]);
		}
		xserial.print("indication(\"servo angle updated!\")\r");
		}
	break;
	case 4026: // read P I D parameter from the 2560
		xserial.print("PID_read(");
		xserial.print(Kp);
		xserial.print(",");
		xserial.print(Ki);
		xserial.print(",");
		xserial.print(Kd);
		xserial.print(")\r");
	break;
	case 4037: // write P I D parameter to the 2560, violatile when power off.
                if(cali_code_seen('P'))
		  Kp=cali_code_value().toFloat();
				if(cali_code_seen('I'))
		  Ki=cali_code_value().toFloat();
			    if(cali_code_seen('D'))
		  Kd=cali_code_value().toFloat();
	break;
	case 4048: 
		xserial.print("indication(\"heating, pls wait...\")\r");
        enquecommand_P((PSTR("M109 T0 S200")));
		xserial.print("indication(\"heating is done\")\r");
	break;
	case 4059: 
        for (int i=0;i<6;i++){
			EEPROM.write(i+200,0);
		}
		xserial.print("indication(\"E2 vars was reset!\")\r");
	break;
	case 5016: // read the machine serias number from EEPROM 
		for (int i=0;i<16;i++){
			machine_series[i]=EEPROM.read(i+183);
			xserial.print("machine_series(\"");
			xserial.print(machine_series[i]);
			xserial.print("\")\r");
		}
        xserial.print("indication(\"machine series is there!\")\r");
	break;
	case 5027:  // write the machine serias number  
		if(cali_code_seen('S')){
				for (int i=0;i<16;i++){
					EEPROM.write(i+183,cali_code_value().charAt(i));
				}
        xserial.print("indication(\"machine series is updated!\")\r");
	}
	break;
	case 5038: // test the handshake

		if (!uart2ready)
		{
			xserial.print("indication(\"handshake failed!\")\r");
		}

	break;
	case 5049: // test the jam condition of the uart2 
        for(int i=0;i<10;i++)
        {
			tft_thermo_set(degHotend(0));
			tft_progressbar_set(9999);
			tft_pos_qt_set(9999,9999,9999);
        }
		xserial.print("indication(\"jam test passed!\")\r");

	break;
	case 6017: // sprout rx test 
        xserial.print("indication(\"2560 just said hello!\")\r");
	break;
	case 6028:  // sprout tx test , when 2560 received done, there is a beep for indication.
        TEST_SOUND();
	break;
  case 8019: // draw a line 
         char linecmd[30];
         sprintf_P(linecmd,PSTR("G1 F%i X200 Y200 Z0.80"),feedrate_test);
         enquecommand(linecmd);
         sprintf_P(linecmd,PSTR("G1 F%i X30 E20 Z0.30"),feedrate_test);
         enquecommand(linecmd);
  break;
}

}
	}




