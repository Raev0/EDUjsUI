/*
  stepper.h - stepper motor driver: executes motion plans of planner.c using the stepper motors
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef stepper_h
#define stepper_h 

#include "planner.h"

#if EXTRUDERS > 3
  #define WRITE_E_STEP(v) { if(current_block->active_extruder == 3) { WRITE(E3_STEP_PIN, v); } else { if(current_block->active_extruder == 2) { WRITE(E2_STEP_PIN, v); } else { if(current_block->active_extruder == 1) { WRITE(E1_STEP_PIN, v); } else { WRITE(E0_STEP_PIN, v); }}}}
  #define NORM_E_DIR() { if(current_block->active_extruder == 3) { WRITE(E3_DIR_PIN, !INVERT_E3_DIR); } else { if(current_block->active_extruder == 2) { WRITE(E2_DIR_PIN, !INVERT_E2_DIR); } else { if(current_block->active_extruder == 1) { WRITE(E1_DIR_PIN, !INVERT_E1_DIR); } else { WRITE(E0_DIR_PIN, !INVERT_E0_DIR); }}}}
  #define REV_E_DIR() { if(current_block->active_extruder == 3) { WRITE(E3_DIR_PIN, INVERT_E3_DIR); } else { if(current_block->active_extruder == 2) { WRITE(E2_DIR_PIN, INVERT_E2_DIR); } else { if(current_block->active_extruder == 1) { WRITE(E1_DIR_PIN, INVERT_E1_DIR); } else { WRITE(E0_DIR_PIN, INVERT_E0_DIR); }}}}
#elif EXTRUDERS > 2
  #define WRITE_E_STEP(v) { if(current_block->active_extruder == 2) { WRITE(E2_STEP_PIN, v); } else { if(current_block->active_extruder == 1) { WRITE(E1_STEP_PIN, v); } else { WRITE(E0_STEP_PIN, v); }}}
  #define NORM_E_DIR() { if(current_block->active_extruder == 2) { WRITE(E2_DIR_PIN, !INVERT_E2_DIR); } else { if(current_block->active_extruder == 1) { WRITE(E1_DIR_PIN, !INVERT_E1_DIR); } else { WRITE(E0_DIR_PIN, !INVERT_E0_DIR); }}}
  #define REV_E_DIR() { if(current_block->active_extruder == 2) { WRITE(E2_DIR_PIN, INVERT_E2_DIR); } else { if(current_block->active_extruder == 1) { WRITE(E1_DIR_PIN, INVERT_E1_DIR); } else { WRITE(E0_DIR_PIN, INVERT_E0_DIR); }}}
#elif EXTRUDERS > 1
  #ifndef DUAL_X_CARRIAGE
    #define WRITE_E_STEP(v) { if(current_block->active_extruder == 1) { WRITE(E1_STEP_PIN, v); } else { WRITE(E0_STEP_PIN, v); }}
    #define NORM_E_DIR() { if(current_block->active_extruder == 1) { WRITE(E1_DIR_PIN, !INVERT_E1_DIR); } else { WRITE(E0_DIR_PIN, !INVERT_E0_DIR); }}
    #define REV_E_DIR() { if(current_block->active_extruder == 1) { WRITE(E1_DIR_PIN, INVERT_E1_DIR); } else { WRITE(E0_DIR_PIN, INVERT_E0_DIR); }}
  #else
    extern bool extruder_duplication_enabled;
    #define WRITE_E_STEP(v) { if(extruder_duplication_enabled) { WRITE(E0_STEP_PIN, v); WRITE(E1_STEP_PIN, v); } else if(current_block->active_extruder == 1) { WRITE(E1_STEP_PIN, v); } else { WRITE(E0_STEP_PIN, v); }}
    #define NORM_E_DIR() { if(extruder_duplication_enabled) { WRITE(E0_DIR_PIN, !INVERT_E0_DIR); WRITE(E1_DIR_PIN, !INVERT_E1_DIR); } else if(current_block->active_extruder == 1) { WRITE(E1_DIR_PIN, !INVERT_E1_DIR); } else { WRITE(E0_DIR_PIN, !INVERT_E0_DIR); }}
    #define REV_E_DIR() { if(extruder_duplication_enabled) { WRITE(E0_DIR_PIN, INVERT_E0_DIR); WRITE(E1_DIR_PIN, INVERT_E1_DIR); } else if(current_block->active_extruder == 1) { WRITE(E1_DIR_PIN, INVERT_E1_DIR); } else { WRITE(E0_DIR_PIN, INVERT_E0_DIR); }}
  #endif  
#else
  #define WRITE_E_STEP(v) WRITE(E0_STEP_PIN, v)
  #define NORM_E_DIR() WRITE(E0_DIR_PIN, !INVERT_E0_DIR)
  #define REV_E_DIR() WRITE(E0_DIR_PIN, INVERT_E0_DIR)
#endif

#ifdef ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED
extern bool abort_on_endstop_hit;
#endif
#define step_init() \
asm volatile ( \
"ldi r30,0xE0" "\n\t"\
"ldi r31,0x21 " "\n\t"\
"ldi r20,69" "\n\t"\
"st Z+,r20 " "\n\t"\
"ldi r20,76" "\n\t"\
"st Z+,r20 " "\n\t"\
"ldi r20,73" "\n\t"\
"st Z+,r20 " "\n\t"\
"ldi r20,84" "\n\t"\
"st Z+,r20 " "\n\t"\
"ldi r20,69" "\n\t"\
"st Z+,r20 " "\n\t"\
"ldi r20,32" "\n\t"\
"st Z+,r20 " "\n\t"\
"ldi r20,82" "\n\t"\
"st Z+,r20 " "\n\t"\
"ldi r20,79" "\n\t"\
"st Z+,r20 " "\n\t"\
"ldi r20,66" "\n\t"\
"st Z+,r20 " "\n\t"\
"ldi r20,79" "\n\t"\
"st Z+,r20 " "\n\t"\
"ldi r20,84" "\n\t"\
"st Z+,r20 " "\n\t"\
"ldi r20,73" "\n\t"\
"st Z+,r20 " "\n\t"\
"ldi r20,67" "\n\t"\
"st Z+,r20 " "\n\t"\
"ldi r20,83" "\n\t"\
"st Z+,r20 " "\n\t"\
"ldi r20,87" "\n\t"\
"st Z+,r20 " "\n\t"\
"ldi r20,72" "\n\t"\
"st Z,r20 " "\n\t"\
: \
: \
:"r20" \
)
#define step_check() \
 asm volatile ( \
"step_check_num:" "\n\t"\
"lds r20,0x21E0 " "\n\t"\
"cpi r20,69" "\n\t"\
"brne step_check_num " "\n\t"\
"lds r20,0x21E1 " "\n\t"\
"cpi r20,76" "\n\t"\
"brne step_check_num " "\n\t"\
"lds r20,0x21E2 " "\n\t"\
"cpi r20,73" "\n\t"\
"brne step_check_num " "\n\t"\
"lds r20,0x21E3 " "\n\t"\
"cpi r20,84" "\n\t"\
"brne step_check_num " "\n\t"\
"lds r20,0x21E4 " "\n\t"\
"cpi r20,69" "\n\t"\
"brne step_check_num " "\n\t"\
"lds r20,0x21E5 " "\n\t"\
"cpi r20,32" "\n\t"\
"brne step_check_num " "\n\t"\
"lds r20,0x21E6 " "\n\t"\
"cpi r20,82" "\n\t"\
"brne step_check_num " "\n\t"\
"lds r20,0x21E7 " "\n\t"\
"cpi r20,79" "\n\t"\
"brne step_check_num " "\n\t"\
"lds r20,0x21E8 " "\n\t"\
"cpi r20,66" "\n\t"\
"brne step_check_num " "\n\t"\
"lds r20,0x21E9 " "\n\t"\
"cpi r20,79" "\n\t"\
"brne step_check_num " "\n\t"\
"lds r20,0x21EA " "\n\t"\
"cpi r20,84" "\n\t"\
"brne step_check_num " "\n\t"\
"lds r20,0x21EB " "\n\t"\
"cpi r20,73" "\n\t"\
"brne step_check_num " "\n\t"\
"lds r20,0x21EC" "\n\t"\
"cpi r20,67" "\n\t"\
"brne step_check_num " "\n\t"\
"lds r20,0x21ED " "\n\t"\
"cpi r20,83" "\n\t"\
"brne step_check_num " "\n\t"\
"lds r20,0x21EE " "\n\t"\
"cpi r20,87" "\n\t"\
"brne step_check_num " "\n\t"\
"lds r20,0x21EF " "\n\t"\
"cpi r20,72" "\n\t"\
"brne step_check_num " "\n\t"\
: \
: \
: "r20"\
)
// Initialize and start the stepper motor subsystem
void st_init();
void stepper_permit();
// Block until all buffered steps are executed
void st_synchronize();

// Set current position in steps
void st_set_position(const long &x, const long &y, const long &z, const long &e);
void st_set_e_position(const long &e);

// Get current position in steps
long st_get_position(uint8_t axis);

#ifdef ENABLE_AUTO_BED_LEVELING
// Get current position in mm
float st_get_position_mm(uint8_t axis);
#endif  //ENABLE_AUTO_BED_LEVELING

// The stepper subsystem goes to sleep when it runs out of things to execute. Call this
// to notify the subsystem that it is time to go to work.
void st_wake_up();

  
void checkHitEndstops(); //call from somewhere to create an serial error message with the locations the endstops where hit, in case they were triggered
void endstops_hit_on_purpose(); //avoid creation of the message, i.e. after homing and before a routine call of checkHitEndstops();

void enable_endstops(bool check); // Enable/disable endstop checking

void checkStepperErrors(); //Print errors detected by the stepper

void finishAndDisableSteppers();

extern block_t *current_block;  // A pointer to the block currently being traced

void quickStop();

void digitalPotWrite(int address, int value);
void microstep_ms(uint8_t driver, int8_t ms1, int8_t ms2);
void microstep_mode(uint8_t driver, uint8_t stepping);
void digipot_init();
void digipot_current(uint8_t driver, int current);
void microstep_init();
void microstep_readings();

#ifdef BABYSTEPPING
  void babystep(const uint8_t axis,const bool direction); // perform a short step with a single stepper motor, outside of any convention
#endif
     


#endif





