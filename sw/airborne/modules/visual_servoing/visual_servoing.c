/*
 * Copyright (C) Kirk Scheper <kirkscheper@gmail.com>
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider_guided.c"
 * @author Kirk Scheper
 * This module is an example module for the course AE4317 Autonomous Flight of Micro Air Vehicles at the TU Delft.
 * This module is used in combination with a color filter (cv_detect_color_object) and the guided mode of the autopilot.
 * The avoidance strategy is to simply count the total number of orange pixels. When above a certain percentage threshold,
 * (given by color_count_frac) we assume that there is an obstacle and we turn.
 *
 * The color filter settings are set using the cv_detect_color_object. This module can run multiple filters simultaneously
 * so you have to define which filter to use with the ORANGE_AVOIDER_VISUAL_DETECTION_ID setting.
 * This module differs from the simpler orange_avoider.xml in that this is flown in guided mode. This flight mode is
 * less dependent on a global positioning estimate as witht the navigation mode. This module can be used with a simple
 * speed estimate rather than a global position.
 *
 * Here we also need to use our onboard sensors to stay inside of the cyberzoo and not collide with the nets. For this
 * we employ a simple color detector, similar to the orange poles but for green to detect the floor. When the total amount
 * of green drops below a given threshold (given by floor_count_frac) we assume we are near the edge of the zoo and turn
 * around. The color detection is done by the cv_detect_color_object module, use the FLOOR_VISUAL_DETECTION_ID setting to
 * define which filter to use.
 */

#include <math.h>
#include "modules/visual_servoing/visual_servoing.h"
#include "autopilot.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/stabilization/stabilization_indi_simple.h"
#include "math/pprz_algebra_float.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include <stdio.h>
#include <time.h>

#define ORANGE_AVOIDER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[orange_avoider_guided->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

uint8_t chooseRandomIncrementAvoidance(void);

// define and initialise global variables
int32_t color_count = 0;                // orange color count from color filter for obstacle detection
int32_t last_color_count = 1;
float divergence = 0;
int16_t box_centroid_x = 0;
int16_t box_centroid_y = 0;
float nominal_thrust = 0;
float sum_pitch_error;
float sum_roll_error;
const int16_t max_trajectory_confidence = 5;  // number of consecutive negative object detections to be sure we are obstacle free

// define visual servo controller variables
float divergence_setpoint = 1;
float vx_setpoint = 0;
float vy_setpoint = 0;
float vz_setpoint = 0;
float visual_servo_x_kp = 0;
float visual_servo_y_kp = 0.1;
float visual_servo_z_kp = 1;
float P_hor = 0.1;
float I_hor = 0.001;

// This call back will be used to receive the color count from the orange detector
#ifndef ORANGE_AVOIDER_VISUAL_DETECTION_ID
#error This module requires two color filters, as such you have to define ORANGE_AVOIDER_VISUAL_DETECTION_ID to the orange filter
#error Please define ORANGE_AVOIDER_VISUAL_DETECTION_ID to be COLOR_OBJECT_DETECTION1_ID or COLOR_OBJECT_DETECTION2_ID in your airframe
#endif
static abi_event color_detection_ev;
static void color_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t pixel_x, int16_t pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t __attribute__((unused)) extra)
{
  color_count = quality;
  box_centroid_x = pixel_x;
  box_centroid_y = pixel_y;
}

void visual_servoing_module_init(void);

void visual_servoing_module_run(bool in_flight);

/*
 * Initialisation function
 */
void visual_servoing_module_init(void)
{
  visual_servoing.divergence_sp = 0;
  visual_servoing.acc_x_sp = 0;                 
  visual_servoing.acc_y_sp = 0;              
  visual_servoing.ol_x_pgain = 0;                    
  visual_servoing.ol_y_pgain = 0;    
  visual_servoing.ol_z_pgain = 0;                  
  visual_servoing.ol_x_dgain = 0;                  
  visual_servoing.ol_y_dgain = 0;                  
  visual_servoing.ol_z_dgain = 0;             
  visual_servoing.il_h_pgain = 0;           
  visual_servoing.il_h_pgain = 0;
  visual_servoing.il_h_pgain = 0;

  // bind our colorfilter callbacks to receive the color filter outputs
  AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);
}

/*
 * Function that checks it is safe to move forwards, and then sets a forward velocity setpoint or changes the heading
 */
void visual_servoing_module_run(bool in_flight)
{ 
  // // VERBOSE_PRINT("Box_centroid_x %d, Box_centroid_y %d\n", box_centroid_x, box_centroid_y);
  // // VERBOSE_PRINT("Floor centroid: %f\n", floor_centroid_frac);

  // move x, y
  struct FloatEulers *attitude = stateGetNedToBodyEulers_f();
  struct EnuCoor_f *position = stateGetPositionEnu_f();

  // Compute divergence
  divergence = (color_count - last_color_count) / last_color_count;

  // Derotate box_centroid


  // desired accelerations inertial
  float mu_x = visual_servoing.acc_x_sp; // visual_servo_x_kp * (divergence_setpoint - divergence);
  float mu_y = visual_servoing.acc_y_sp;
  float mu_z = 9.81 + visual_servo_z_kp * (1.5 - position->z);
  VERBOSE_PRINT("centroid_y: %d", box_centroid_y);


  // set the desired thrust
  float mass = (guidance_v_nominal_throttle * MAX_PPRZ) / 9.81;

  // // VERBOSE_PRINT("Color_count: %d  threshold: %d state: %d \n", color_count, color_count_threshold);
  // // VERBOSE_PRINT("Box_centroid_x %d, Box_centroid_y %d\n", box_centroid_x, box_centroid_y);

  float thrust_set = sqrtf(pow(mu_x, 2) + pow(mu_y, 2) + pow(mu_z, 2)) * mass;

  if (in_flight) {
    Bound(thrust_set, 0.25 * guidance_v_nominal_throttle, MAX_PPRZ);
    stabilization_cmd[COMMAND_THRUST] = thrust_set;
  }

  // set desired angles
  float pitch_sp = atan2f(mu_x, mu_z);
  float roll_sp = asinf(mass * mu_y/thrust_set);

  float error_pitch = pitch_sp - attitude->theta;
  float error_roll = roll_sp - attitude->psi;
  sum_pitch_error += error_pitch;
  sum_roll_error += error_roll;
  float pitch_cmd;
  float roll_cmd;

  pitch_cmd = error_pitch *  P_hor;
  roll_cmd = error_roll *  P_hor;

  // VERBOSE_PRINT("pitch_sp: %f", pitch_sp);

  BoundAbs(pitch_cmd, RadOfDeg(10.0));
  BoundAbs(roll_cmd, RadOfDeg(10.0));

  float psi_cmd = attitude->psi;

  struct Int32Eulers rpy = { .phi = (int32_t)ANGLE_BFP_OF_REAL(roll_cmd),
        .theta = (int32_t)ANGLE_BFP_OF_REAL(pitch_cmd), .psi = (int32_t)ANGLE_BFP_OF_REAL(psi_cmd)
  };

  // set the desired roll pitch and yaw:
  stabilization_indi_set_rpy_setpoint_i(&rpy);
  // execute attitude stabilization:
  stabilization_attitude_run(in_flight);
  // VERBOSE_PRINT("vy_setpoint: %f, Box_centroid_y %d\n", vy_setpoint, box_centroid_y);
  // VERBOSE_PRINT("divergence: %f, color_count %d last_color_count %d\n", divergence, color_count, last_color_count);

  // // // move z
  // // vz_setpoint = visual_servo_z_kp * box_centroid_x;
  // // guidance_v_set_vz(vz_setpoint);
  // // VERBOSE_PRINT("vz_setpoint: %f, Box_centroid_x %d\n", vz_setpoint, box_centroid_x);

  // visual_servo_x_kp = 0.6;
  // last_color_count = color_count;
}

////////////////////////////////////////////////////////////////////
// Call our controller
// Implement own Horizontal loops
void guidance_h_module_init(void)
{
  visual_servoing_module_init();
}

void guidance_h_module_enter(void)
{
  visual_servoing_module_init();
}

void guidance_h_module_read_rc(void)
{
  
}

void guidance_h_module_run(bool in_flight)
{
  // Call full inner-/outerloop / horizontal-/vertical controller:
  visual_servoing_module_run(in_flight);
}

void guidance_v_module_init(void)
{
  // initialization of your custom vertical controller goes here
}

// Implement own Vertical loops
void guidance_v_module_enter(void)
{
  // your code that should be executed when entering this vertical mode goes here
}

void guidance_v_module_run(UNUSED bool in_flight)
{
  // your vertical controller goes here
}
