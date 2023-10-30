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
#include "math/pprz_stat.h"
#include "math/pprz_algebra_float.h"
#include "modules/visual_servoing/visual_servoing.h"
#include "autopilot.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/stabilization/stabilization_indi_simple.h"
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
#define VS_DIVERGENCE_SP 0.15
#endif

#ifndef VS_DIV_FACTOR
#define VS_DIV_FACTOR 600
#endif

#ifndef VS_OL_X_PGAIN
#define VS_OL_X_PGAIN 3
#endif

#ifndef VS_OL_Y_PGAIN
#define VS_OL_Y_PGAIN 0.015
#endif

#ifndef VS_OL_Z_PGAIN
#define VS_OL_Z_PGAIN 0.01
#endif

#ifndef VS_OL_X_DGAIN
#define VS_OL_X_DGAIN 0.0
#endif

#ifndef VS_OL_Y_DGAIN
#define VS_OL_Y_DGAIN 0.015
#endif

#ifndef VS_OL_Z_DGAIN
#define VS_OL_Z_DGAIN 0.005
#endif

// 0.04
#ifndef VS_OL_X_IGAIN
#define VS_OL_X_IGAIN 0.1
#endif

#ifndef VS_LP_CONST
#define VS_LP_CONST 0.3
#endif

#ifndef VS_WINDOW_SIZE
#define VS_WINDOW_SIZE 5
#endif

#ifndef VS_THETA_OFFSET
#define VS_THETA_OFFSET -0.03
#endif

#ifndef VS_SWITCH_TIME_CONSTANT
#define VS_SWITCH_TIME_CONSTANT 0.01
#endif

#ifndef VS_DISTANCE_EST_THRESHOLD
#define VS_DISTANCE_EST_THRESHOLD 0.1
#endif

#ifndef VS_CD
#define VS_CD 0.9
#endif

#ifndef VS_DIV_CUTOFF_FREQ
#define VS_DIV_CUTOFF_FREQ 0.1
#endif



// define and initialise global variables
float vs_dt = 0.065;
float fps = 0;
float divergence_sp = 0;
float divergence_history[VS_WINDOW_SIZE];
float theta_history[VS_WINDOW_SIZE];
float divergence_variance = 0;
float divergence_mean = 0;
float set_point_error = 0;
uint32_t index_hist = 0;
bool history_array_filled = FALSE;
float distance_est = 0;
float set_point_time = 0;
float end_time = 0;
float m_dt;
float pitch_sp = 0;
float roll_sp = 0;
float thrust_set = 0;
float vision_time, prev_vision_time;    // time stamp of the color object detector
float color_count = 0;                // orange color count from color filter for obstacle detection
float last_color_count = 1;
float new_divergence;
float box_centroid_x = 0;
float box_centroid_y = 0;
float true_distance = 0;
int8_t set_point_count = 0;
bool landing = FALSE;
bool switching = FALSE;
float accel_est = 0;
float filtered_divergence = 0;
float switch_time_start = 0;
uint16_t count = 0;
float initial_distance = 0;

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
                                &true_distance,
                                &divergence_variance,
                                &guidance_v_nominal_throttle,
                                &divergence_mean,
                                &color_count,
                                &(stateGetAccelNed_f()->x),
                                &accel_est,
                                &divergence_sp,
                                &filtered_divergence,
                                &visual_servoing.div_err_sum);
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
static struct FirstOrderLowPass div_filter;

void visual_servoing_module_init(void);

void visual_servoing_module_enter(void);

void visual_servoing_module_run(bool in_flight);

static void update_errors(float box_x_err, float box_y_err, float div_err, float div_err_filt, float vs_dt);

static void final_land_in_box(float start_time);

static float divergence_step(float switch_time);

static void vs_init_filters(void);


/*
 * Initialisation function
 */
void visual_servoing_module_init(void)
{
  visual_servoing.nominal_throttle = (float)guidance_v_nominal_throttle;
  visual_servoing.divergence_sp = 0;
  visual_servoing.divergence = 0;
  visual_servoing.true_divergence = 0;
  visual_servoing.window_size = VS_WINDOW_SIZE;                
  visual_servoing.div_factor = VS_DIV_FACTOR;              
  visual_servoing.ol_x_pgain = VS_OL_X_PGAIN;                    
  visual_servoing.ol_y_pgain = VS_OL_Y_PGAIN; // / VISUAL_SERVOING_CAMERA.output_size.w;    
  visual_servoing.ol_z_pgain = VS_OL_Z_PGAIN; // / VISUAL_SERVOING_CAMERA.output_size.h;                  
  visual_servoing.ol_x_dgain = VS_OL_X_DGAIN;                  
  visual_servoing.ol_y_dgain = VS_OL_Y_DGAIN; // / VISUAL_SERVOING_CAMERA.output_size.w;                  
  visual_servoing.ol_z_dgain = VS_OL_Z_DGAIN;             
  visual_servoing.ol_x_igain = VS_OL_X_IGAIN;             
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
  visual_servoing.distance_est_threshold = VS_DISTANCE_EST_THRESHOLD;
  visual_servoing.theta_offset = VS_THETA_OFFSET;
  visual_servoing.cd = VS_CD;
  visual_servoing.div_cutoff_freq = VS_DIV_CUTOFF_FREQ;

  // bind our colorfilter callbacks to receive the color filter outputs
  AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);

  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_VISUAL_SERVOING, send_vs_attitude);

  vs_init_filters();
} 

void vs_init_filters(void)
{
  // tau = 1/(2*pi*Fc)
  float tau = 1.0 / (2.0 * M_PI * visual_servoing.div_cutoff_freq);
  float sample_time = 1.0 / 15;
  // Filtering of divergenge
  init_first_order_low_pass(&div_filter, tau, sample_time, visual_servoing.divergence);
}

static void reset_all_vars(void)
{
  visual_servoing.nominal_throttle = (float)guidance_v_nominal_throttle;
  visual_servoing.divergence = 0;
  visual_servoing.true_divergence = 0;
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
  pitch_sp = 0;
  roll_sp = 0;
  color_count = 0;                
  last_color_count = 1;
  box_centroid_x = 0;
  box_centroid_y = 0;
  set_point_count = 0;
  set_point_time = get_sys_time_usec();
  divergence_sp = 0;
  uint32_t i;
  for (i = 0; i < VS_WINDOW_SIZE; i++) {
    theta_history[i] = 0;
    divergence_history[i] = 0;
  }
  vs_init_filters();
  landing = FALSE;
  switch_time_start = 0;
  count = 0;
}


void visual_servoing_module_run(bool in_flight)
{ 
  // Get the current state of the quadrotor
  struct FloatEulers *attitude = stateGetNedToBodyEulers_f();
  // attitude->theta += visual_servoing.theta_offset;
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

  vs_dt = vision_time - prev_vision_time;
  prev_vision_time = vision_time;

  // // Initiate final landing maneuver
  // if (color_count > 45000 && !landing){
  //   landing = TRUE;
  //   end_time = (float)get_sys_time_usec() / 1e6;
  // }

  // Initiate divergence step
  if (color_count > 0 && !switching && set_point_count == 0){
    switching = TRUE;
    switch_time_start = (float)get_sys_time_usec() / 1e6;
    set_point_count += 1;
  }

  // Initiate divergence step
  if (color_count > 5000 && !switching && set_point_count == 1){
    switching = TRUE;
    switch_time_start = (float)get_sys_time_usec() / 1e6;
    set_point_count += 1;
  }
  
  // check if new measurement received
  if (vs_dt > 1e-5 && !landing){
    fps = 1/vs_dt;
    Bound(visual_servoing.lp_const, 0.001f, 1.f);
    float lp_factor = vs_dt / visual_servoing.lp_const;
    Bound(lp_factor, 0.f, 1.f);

    // Compute divergence
    if (last_color_count && color_count != 0){
      float a1 = atan2f(sqrt(last_color_count), 2 * visual_servoing.div_factor);
      float a2 = atan2f(sqrt(color_count), 2 * visual_servoing.div_factor);
      float flow = (a2 - a1) / vs_dt;
      new_divergence = flow / (sqrtf(color_count) / (2 * visual_servoing.div_factor));
    }
    else {new_divergence = visual_servoing.divergence;}

    // deal with (unlikely) fast changes in divergence:
    static const float max_div_dt = 0.20f;
    if (fabsf(new_divergence - visual_servoing.divergence) > max_div_dt) {
      if (new_divergence < visual_servoing.divergence) { new_divergence = visual_servoing.divergence - max_div_dt; }
      else { new_divergence = visual_servoing.divergence + max_div_dt; }
    }

    // low-pass filter the divergence:
    visual_servoing.divergence += (new_divergence - visual_servoing.divergence) * lp_factor;

    // Ground truth divergence
    true_distance = sqrtf(pow(4 - position->x, 2) + pow(0 - position->y, 2) + pow(0.7 + position->z, 2));
    visual_servoing.true_divergence = speed->x / true_distance;

    // // 2 [1/s] ramp to setpoint
    // if (fabsf(visual_servoing.divergence_sp - divergence_sp) > 0.05*vs_dt){
    //   divergence_sp += 0.05*vs_dt * visual_servoing.divergence_sp / fabsf(visual_servoing.divergence_sp);
    // } else {
    //   divergence_sp = visual_servoing.divergence_sp;
    // }

    visual_servoing.div_err = divergence_sp - visual_servoing.divergence;

    // Distance Estimate
    // Compute mean and variance of divergence and pitch angle
    divergence_history[index_hist] = visual_servoing.divergence;
    theta_history[index_hist] = attitude->theta;
    divergence_variance = variance_f(divergence_history, visual_servoing.window_size); // below 0.001 is good enough
    divergence_mean = mean_f(divergence_history, visual_servoing.window_size);
    float theta_mean = mean_f(theta_history, visual_servoing.window_size) + visual_servoing.theta_offset;
    accel_est = -9.81 * theta_mean / (1.0f + visual_servoing.cd / divergence_mean);
    float true_acceleration = stateGetAccelNed_f()->x;

    // Accuracy of the distance estimate
    set_point_error = fabsf(visual_servoing.divergence_sp - divergence_mean); // below 0.04 is good enough
    visual_servoing.distance_accuracy = set_point_error + 40 * divergence_variance;

    // d = d0 * e^-rt
    // Compute distance estimate
    if (history_array_filled && visual_servoing.distance_accuracy < visual_servoing.distance_est_threshold){
      // distance_est = roundf((-17.2*theta_mean * pow(divergence_mean, -0.808)) * 100) / 100;
      distance_est = roundf((accel_est / (divergence_mean*divergence_mean))*100) / 100;
    }
    else {distance_est = 0;}

    if (index_hist + 1 == visual_servoing.window_size) {
      history_array_filled = TRUE;
    }

    index_hist = (index_hist + 1) % visual_servoing.window_size;

    filtered_divergence = update_first_order_low_pass(&div_filter, new_divergence);
    float filtered_div_err = visual_servoing.divergence_sp - filtered_divergence;

    // update control errors
    update_errors(box_centroid_x, box_centroid_y, visual_servoing.div_err, filtered_div_err, vs_dt);
  }

  // // change divergence set-point
  // if (distance_est > 0 && set_point_count == 3) {
  //   float new_sp = 0.25;
  //   visual_servoing.divergence_sp = new_sp;
  //   set_point_time = get_sys_time_usec();
  //   uint32_t j;
  //   for (j = 0; j < VS_WINDOW_SIZE; j++) {
  //     theta_history[j] = 0;
  //     divergence_history[j] = 0;
  //     index_hist = 0;
  //   }
  //   set_point_count += 1;
  //   VERBOSE_PRINT("divergence set to: %f", new_sp);
  // }
  // if (color_count > 20000 && set_point_count == 3) {
  //   visual_servoing.divergence_sp = 0.15;
  //   visual_servoing.lp_const = VS_LP_CONST;
  //   set_point_time = get_sys_time_usec();
  //   set_point_count += 1;
  //   VERBOSE_PRINT("divergence set to: %f", 0.15);
  // }
  // if (color_count > 100000 && set_point_count == 3) {
  //   visual_servoing.divergence_sp = 0.2;
  //   visual_servoing.lp_const = VS_LP_CONST;
  //   set_point_time = get_sys_time_usec();
  //   set_point_count += 1;
  //   VERBOSE_PRINT("divergence set to: %f", 0.3);
  // }

  // visual_servoing.ol_x_pgain * (1 - position->x)
  // desired accelerations inertial
  // float mu_x = - visual_servoing.ol_x_pgain * visual_servoing.div_err - visual_servoing.ol_x_dgain * visual_servoing.div_err_sum;
  // float mu_y = -0.1 * position->y;
  // float mu_z = 9.81 + 0.1 * (position->z + 0.5);

  // Desired accelerations
  float mu_x;
  if (!switching){   
    mu_x = - visual_servoing.ol_x_pgain * visual_servoing.div_err 
           - visual_servoing.ol_x_igain * visual_servoing.div_err_sum
           - visual_servoing.ol_x_dgain * visual_servoing.div_err_d;
  }
  else{
    mu_x = divergence_step(switch_time_start);
  }
  float mu_y = - visual_servoing.ol_y_pgain * (box_centroid_y - 2) - visual_servoing.ol_y_dgain * visual_servoing.box_y_err_d;
  float mu_z = 9.81 + visual_servoing.ol_z_pgain * box_centroid_x + visual_servoing.ol_z_dgain * visual_servoing.box_x_err_d;

  if (!landing){
    // set the desired thrust
    float mass = (visual_servoing.nominal_throttle * MAX_PPRZ) / 9.81;
    thrust_set = sqrtf(pow(mu_x, 2) + pow(mu_y, 2) + pow(mu_z, 2)) * mass;

    // set desired angles
    pitch_sp = atan2f(mu_x, mu_z);
    roll_sp = asinf(mass * mu_y/thrust_set);
    BoundAbs(pitch_sp, RadOfDeg(10.0));
    BoundAbs(roll_sp, RadOfDeg(10.0));
  }
  else{
    final_land_in_box(end_time);
  }
  
  if (in_flight) {
    Bound(thrust_set, 0.25 * guidance_v_nominal_throttle, MAX_PPRZ);
    stabilization_cmd[COMMAND_THRUST] = thrust_set;
  }

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
void update_errors(float box_x_err, float box_y_err, float div_err, float div_err_filt, float vs_dt)
{
  float lp_factor = vs_dt / 0.02;
  Bound(lp_factor, 0.f, 1.f);

  // maintain the controller errors:
  // Error of box x coordinate
  visual_servoing.box_x_err_sum += box_x_err;
  visual_servoing.box_x_err_d = ((box_x_err - visual_servoing.previous_box_x_err) / vs_dt);
  visual_servoing.previous_box_x_err = box_x_err;

  // Error of box y coordinate
  visual_servoing.box_y_err_sum += box_y_err;
  visual_servoing.box_y_err_d = ((box_y_err - visual_servoing.previous_box_y_err) / vs_dt);
  visual_servoing.previous_box_y_err = box_y_err;

  // Error of divergence
  visual_servoing.div_err_sum += div_err;
  visual_servoing.div_err_d = ((div_err_filt - visual_servoing.previous_div_err) / vs_dt);
  visual_servoing.previous_div_err = div_err_filt;
}

/**
 * Execute the final landing procedure to land in the box when a certain distance from the target is reached
 */
void final_land_in_box(float start_time)
{
  float current_time = (float)get_sys_time_usec() / 1e6;
  float delta_time = current_time - start_time;
  // first 2 seconds accelerate forward
  if (delta_time <= 1.5f){
    pitch_sp = -0.04;
    roll_sp = 0.001;
    thrust_set = visual_servoing.nominal_throttle * MAX_PPRZ * 0.99;
  }
  // then, 1 second descending
  if (1.5f < delta_time && delta_time <= 1.75f){
    pitch_sp = 0;
    roll_sp = 0;
    thrust_set = visual_servoing.nominal_throttle * MAX_PPRZ * 0.85;
  }
  // kill throttle
  if (delta_time > 1.75f){
    autopilot_set_kill_throttle(true);
  }
}

/**
 * Make a standardized step in divergence
 */
float divergence_step(float switch_time)
{
  float current_time = (float)get_sys_time_usec() / 1e6;
  float delta_time = current_time - switch_time;

  // Give a sin input to the forward acceleration 
  float accel_x = -1 * expf(-0.8 * delta_time);
  switching = TRUE;

  // if (delta_time >= 1.5 && count == 0){
  //   float new_sp = divergence_mean;
  //   visual_servoing.div_err_sum = (new_sp - divergence_sp) * 5;
  //   divergence_sp = new_sp;
  //   count += 1;
  // }
  int16_t end = 2;
  if (delta_time >= end){
    float new_sp = divergence_mean;
    initial_distance = (new_sp - divergence_sp) / end;
    printf("initial_distance: %f distance %f \n", initial_distance, true_distance);
    divergence_sp = new_sp;
    visual_servoing.div_err = 0.1;
    visual_servoing.div_err_sum = 2;
    switching = FALSE;
  }  

  return accel_x;
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
