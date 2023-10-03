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
#include "modules/datalink/telemetry.h"
#include <stdio.h>
#include <time.h>

#define ORANGE_AVOIDER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[orange_avoider_guided->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

#ifndef VS_NOM_THROTTLE
#define VS_NOM_THROTTLE 0.66
#endif

#ifndef VS_DIVERGENCE_SP
#define VS_DIVERGENCE_SP 0.05
#endif

#ifndef VS_DIV_FACTOR
#define VS_DIV_FACTOR 600
#endif

#ifndef VS_OL_X_PGAIN
#define VS_OL_X_PGAIN 3
#endif

#ifndef VS_OL_Y_PGAIN
#define VS_OL_Y_PGAIN 2.4
#endif

#ifndef VS_OL_Z_PGAIN
#define VS_OL_Z_PGAIN 0.5
#endif

#ifndef VS_OL_X_DGAIN
#define VS_OL_X_DGAIN 0.2
#endif

#ifndef VS_OL_Y_DGAIN
#define VS_OL_Y_DGAIN 1.2
#endif

#ifndef VS_OL_Z_DGAIN
#define VS_OL_Z_DGAIN 0
#endif

#ifndef VS_IL_H_PGAIN
#define VS_IL_H_PGAIN 0.9
#endif

#ifndef VS_IL_H_IGAIN
#define VS_IL_H_IGAIN 0.005
#endif

#ifndef VS_IL_H_DGAIN
#define VS_IL_H_DGAIN 0
#endif

#ifndef VS_LP_CONST
#define VS_LP_CONST 0.3
#endif

#ifndef VS_SWITCH_TIME_CONSTANT
#define VS_SWITCH_TIME_CONSTANT 0.01
#endif

// define and initialise global variables
float vs_dt = 0.065;
float fps = 0;
float divergence_sp = 0;
float distance_est = 0;
float measurement_time = 0;
float set_point_time = 0;
float m_dt;
float pitch_sp = 0;
float roll_sp = 0;
float vision_time, prev_vision_time;    // time stamp of the color object detector
float color_count = 0;                // orange color count from color filter for obstacle detection
float last_color_count = 1;
float new_divergence;
float box_centroid_x = 0;
float box_centroid_y = 0;
float true_distance = 0;
int8_t set_point_count = 0;

static void send_vs_attitude(struct transport_tx *trans, struct link_device *dev)
{
  struct FloatEulers *attitude = stateGetNedToBodyEulers_f();
  pprz_msg_send_VISUAL_SERVOING(trans, dev, AC_ID,
                                &(attitude->theta),
                                &(attitude->phi),
                                &visual_servoing.divergence,
                                &visual_servoing.true_divergence,
                                &fps,
                                &box_centroid_x,
                                &box_centroid_y,
                                &distance_est,
                                &true_distance);
}

// This call back will be used to receive the color count from the orange detector
#ifndef ORANGE_AVOIDER_VISUAL_DETECTION_ID
#error This module requires two color filters, as such you have to define ORANGE_AVOIDER_VISUAL_DETECTION_ID to the orange filter
#error Please define ORANGE_AVOIDER_VISUAL_DETECTION_ID to be COLOR_OBJECT_DETECTION1_ID or COLOR_OBJECT_DETECTION2_ID in your airframe
#endif
static abi_event color_detection_ev;
static void color_detection_cb(uint8_t __attribute__((unused)) sender_id, uint32_t stamp,
                               int16_t pixel_x, int16_t pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t __attribute__((unused)) extra)
{
  vision_time = (float)stamp / 1e6;
  color_count = (float)quality;
  box_centroid_x = (float)pixel_x;
  box_centroid_y = (float)pixel_y;
}

// struct containing most relevant parameters
struct VisualServoing visual_servoing;

void visual_servoing_module_init(void);

void visual_servoing_module_run(bool in_flight);

static void update_errors(float box_x_err, float box_y_err, float div_err, float vs_dt);

static void vs_init_filters(void);


/*
 * Initialisation function
 */
void visual_servoing_module_init(void)
{
  visual_servoing.nominal_throttle = (float)guidance_v_nominal_throttle;
  visual_servoing.divergence_sp = VS_DIVERGENCE_SP;
  visual_servoing.divergence = 0;
  visual_servoing.true_divergence = 0;
  visual_servoing.acc_y_sp = 0;                 
  visual_servoing.div_factor = VS_DIV_FACTOR;              
  visual_servoing.ol_x_pgain = VS_OL_X_PGAIN;                    
  visual_servoing.ol_y_pgain = VS_OL_Y_PGAIN / VISUAL_SERVOING_CAMERA.output_size.w;    
  visual_servoing.ol_z_pgain = VS_OL_Z_PGAIN / VISUAL_SERVOING_CAMERA.output_size.h;                  
  visual_servoing.ol_x_dgain = VS_OL_X_DGAIN;                  
  visual_servoing.ol_y_dgain = VS_OL_Y_DGAIN / VISUAL_SERVOING_CAMERA.output_size.w;                  
  visual_servoing.ol_z_dgain = VS_OL_Z_DGAIN;             
  visual_servoing.il_h_pgain = VS_IL_H_PGAIN;           
  visual_servoing.il_h_igain = VS_IL_H_IGAIN;
  visual_servoing.il_h_dgain = VS_IL_H_DGAIN;
  visual_servoing.previous_box_x_err = 0;
  visual_servoing.box_x_err_sum = 0;
  visual_servoing.box_x_err_d = 0;
  visual_servoing.previous_box_y_err = 0;
  visual_servoing.box_y_err_sum = 0;
  visual_servoing.box_y_err_d = 0;
  visual_servoing.div_err = 0;
  visual_servoing.previous_div_err = 0;
  visual_servoing.div_err_sum = 0;
  visual_servoing.div_err_d = 0;
  visual_servoing.lp_const = VS_LP_CONST;
  visual_servoing.switch_time_constant = VS_SWITCH_TIME_CONSTANT;

  // bind our colorfilter callbacks to receive the color filter outputs
  AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);

  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_VISUAL_SERVOING, send_vs_attitude);

  vs_init_filters();
} 

void vs_init_filters(void)
{
  // tau = 1/(2*pi*Fc)
  float tau = 1.0 / (2.0 * M_PI * 108.0f);
  float sample_time = 1.0 / 50.0;
  // Filtering of divergenge
  init_butterworth_2_low_pass(&visual_servoing.lpf, tau, sample_time, 0.0);
}

static void reset_all_vars(void)
{
  pitch_sp = 0;
  roll_sp = 0;
  color_count = 0;                
  last_color_count = 1;
  box_centroid_x = 0;
  box_centroid_y = 0;
  set_point_count = 0;
  set_point_time = get_sys_time_usec();
  divergence_sp = 0;
  vs_init_filters();
}


void visual_servoing_module_run(bool in_flight)
{ 
  // // Vision update fps
  // if (color_count != last_color_count) {
  //   uint32_t new_measurement_time = get_sys_time_usec();
  //   m_dt = (float)new_measurement_time / 1e6 - measurement_time;
  //   measurement_time = (float)new_measurement_time / 1e6;
  //   VERBOSE_PRINT("dt: %f", m_dt);
  // }

  // move x, y
  struct FloatEulers *attitude = stateGetNedToBodyEulers_f();
  struct NedCoor_f *position = stateGetPositionNed_f();
  struct NedCoor_f *speed = stateGetSpeedNed_f();

  // Derotate box_centroid
  // float focal_y = VISUAL_SERVOING_CAMERA.camera_intrinsics.focal_y;
  // struct FloatRMat *world_to_body_rmat = stateGetNedToBodyRMat_f();
  // struct FloatRMat body_to_world_rmat;
  // float_rmat_inv(&body_to_world_rmat, world_to_body_rmat);
  // struct FloatVect3 true_centroid;
  // struct FloatVect3 relative_centroid = {focal_y, -box_centroid_y, -box_centroid_x};
  // float_rmat_vmult(&true_centroid, &body_to_world_rmat, &relative_centroid);
  // true_centroid.x = -true_centroid.z;
  // true_centroid.y = -true_centroid.y;


  // check if new measurement received
  vs_dt = vision_time - prev_vision_time;
  prev_vision_time = vision_time;
  
  if (vs_dt > 1e-5){
    fps = 1/vs_dt;
    Bound(visual_servoing.lp_const, 0.001f, 1.f);
    float lp_factor = vs_dt / visual_servoing.lp_const;
    Bound(lp_factor, 0.f, 1.f);

    // Compute divergence
    // new_divergence = pow(fabsf(color_count - last_color_count), 0.33) / (dt * visual_servoing.div_factor);
    // new_divergence = (color_count - last_color_count) / (dt * visual_servoing.div_factor * sqrt(color_count));
    if (last_color_count && color_count != 0){
      float a1 = atan2f(sqrt(last_color_count), 2 * visual_servoing.div_factor);
      float a2 = atan2f(sqrt(color_count), 2 * visual_servoing.div_factor);
      float flow = (a2 - a1) / vs_dt;
      new_divergence = flow / (sqrt(color_count) / (2 * visual_servoing.div_factor));
    }
    else {new_divergence = visual_servoing.divergence;}

    true_distance = sqrt(pow(4 - position->x, 2) + pow(0 - position->y, 2) + pow(1.5 + position->z, 2));

    visual_servoing.true_divergence = speed->x / true_distance;

    // deal with (unlikely) fast changes in divergence:
    static const float max_div_dt = 0.20f;
    if (fabsf(new_divergence - visual_servoing.divergence) > max_div_dt) {
      if (new_divergence < visual_servoing.divergence) { new_divergence = visual_servoing.divergence - max_div_dt; }
      else { new_divergence = visual_servoing.divergence + max_div_dt; }
    }

    // low-pass filter the divergence:
    // visual_servoing.divergence = update_butterworth_2_low_pass(&visual_servoing.lpf, new_divergence);
    visual_servoing.divergence += (new_divergence - visual_servoing.divergence) * lp_factor;

    // Distance Estimate
    if (visual_servoing.divergence != 0){
      distance_est = roundf((-17.2*attitude->theta * pow(visual_servoing.divergence, -0.808)) * 100) / 100;
    }

    // uint32_t m_time = get_sys_time_usec();
    // float m_time_s = ((float)m_time) / 1e6;
    // float fps = 1/(m_time_s - prev_m_time);

    // prev_m_time = m_time_s;

    // update control errors
    // 2 [1/s] ramp to setpoint
    if (fabsf(visual_servoing.divergence_sp - divergence_sp) > 2.*vs_dt){
      divergence_sp += 2*vs_dt * visual_servoing.divergence_sp / fabsf(visual_servoing.divergence_sp);
    } else {
      divergence_sp = visual_servoing.divergence_sp;
    }

    visual_servoing.div_err = divergence_sp - visual_servoing.divergence;

    // float current_time = get_sys_time_usec();
    // float time_after_switch = (current_time - set_point_time) / 1e6;
    // // VERBOSE_PRINT("time after switch: %f", time_after_switch);
    // if (time_after_switch < 6) {
    //   visual_servoing.lp_const += visual_servoing.switch_time_constant * time_after_switch;
    // }

    update_errors(box_centroid_x, box_centroid_y, visual_servoing.div_err, vs_dt);
  }

  // change divergence set-point
  if (color_count > 100 && set_point_count == 0) {
    visual_servoing.divergence_sp = 0.1;
    visual_servoing.lp_const = VS_LP_CONST;
    set_point_time = get_sys_time_usec();
    set_point_count += 1;
    VERBOSE_PRINT("divergence set to: %f", 0.1);
  }
  if (color_count > 20000 && set_point_count == 3) {
    visual_servoing.divergence_sp = 0.15;
    visual_servoing.lp_const = VS_LP_CONST;
    set_point_time = get_sys_time_usec();
    set_point_count += 1;
    VERBOSE_PRINT("divergence set to: %f", 0.15);
  }
  if (color_count > 100000 && set_point_count == 3) {
    visual_servoing.divergence_sp = 0.2;
    visual_servoing.lp_const = VS_LP_CONST;
    set_point_time = get_sys_time_usec();
    set_point_count += 1;
    VERBOSE_PRINT("divergence set to: %f", 0.3);
  }
  // visual_servoing.ol_x_pgain * (1 - position->x)
  // desired accelerations inertial
  // float mu_x = - visual_servoing.ol_x_pgain * visual_servoing.div_err - visual_servoing.ol_x_dgain * visual_servoing.div_err_sum;
  // float mu_y = -visual_servoing.ol_y_pgain * position->y;
  // float mu_z = 9.81 + 0.1 * (position->z + 1);

  float mu_x = - visual_servoing.ol_x_pgain * visual_servoing.div_err - visual_servoing.ol_x_dgain * visual_servoing.div_err_sum;
  float mu_y = -visual_servoing.ol_y_pgain * box_centroid_y - visual_servoing.ol_y_dgain * visual_servoing.box_y_err_d;
  float mu_z = 9.81 + visual_servoing.ol_z_pgain * box_centroid_x;

  // set the desired thrust
  float mass = (visual_servoing.nominal_throttle * MAX_PPRZ) / 9.81;
  float thrust_set = sqrtf(pow(mu_x, 2) + pow(mu_y, 2) + pow(mu_z, 2)) * mass;

  if (in_flight) {
    Bound(thrust_set, 0.25 * guidance_v_nominal_throttle, MAX_PPRZ);
    stabilization_cmd[COMMAND_THRUST] = thrust_set;
  }

  // set desired angles
  pitch_sp = atan2f(mu_x, mu_z);
  roll_sp = asinf(mass * mu_y/thrust_set);
  BoundAbs(pitch_sp, RadOfDeg(10.0));
  BoundAbs(roll_sp, RadOfDeg(10.0));

  float psi_cmd = attitude->psi;

  struct Int32Eulers rpy = { .phi = (int32_t)ANGLE_BFP_OF_REAL(roll_sp),
        .theta = (int32_t)ANGLE_BFP_OF_REAL(pitch_sp), .psi = (int32_t)ANGLE_BFP_OF_REAL(psi_cmd)
  };

  // set the desired roll pitch and yaw:
  stabilization_indi_set_rpy_setpoint_i(&rpy);
  // execute attitude stabilization:
  stabilization_attitude_run(in_flight);

  last_color_count = color_count;
}

/**
 * Updates the integral and differential errors for PID control and sets the previous error
 * @param[in] err: the error of the divergence and divergence setpoint
 * @param[in] vs_dt:  time difference since last update
 */
void update_errors(float box_x_err, float box_y_err, float div_err, float vs_dt)
{
  float lp_factor = vs_dt / 0.02;
  Bound(lp_factor, 0.f, 1.f);

  // maintain the controller errors:
  // Error of box x coordinate
  visual_servoing.box_x_err_sum += box_x_err;
  visual_servoing.box_x_err_d = (((box_x_err - visual_servoing.previous_box_x_err) / vs_dt) - visual_servoing.box_x_err_d) * lp_factor;
  visual_servoing.previous_box_x_err = box_x_err;

  // Error of box y coordinate
  visual_servoing.box_y_err_sum += box_y_err;
  visual_servoing.box_y_err_d = (((box_y_err - visual_servoing.previous_box_y_err) / vs_dt)); // - visual_servoing.box_y_err_d) * lp_factor;
  visual_servoing.previous_box_y_err = box_y_err;

  // Error of divergence
  visual_servoing.div_err_sum += div_err;
  visual_servoing.div_err_d += (((div_err - visual_servoing.previous_div_err) / vs_dt) - visual_servoing.div_err_d) * lp_factor;
  visual_servoing.previous_div_err = div_err;
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
  reset_all_vars();
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
