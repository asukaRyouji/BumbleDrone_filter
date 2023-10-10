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

#ifndef VISUAL_SERVOING_H
#define VISUAL_SERVOING_H

#include <std.h>
#include "filters/low_pass_filter.h"

struct VisualServoing {
  float nominal_throttle;
  float divergence_sp;
  float divergence;
  float true_divergence;
  uint32_t window_size;                       
  float ol_x_pgain;                    
  float ol_y_pgain;   
  float ol_z_pgain;                 
  float ol_x_dgain;                 
  float ol_y_dgain;               
  float ol_z_dgain;                             
  float previous_box_x_err;
  float box_x_err_sum;
  float box_x_err_d;
  float previous_box_y_err;
  float box_y_err_sum;
  float box_y_err_d;
  float div_err;
  float previous_div_err;
  float div_err_sum;
  float div_err_d;
  float lp_const;
  float div_factor;
  float switch_time_constant;
  float distance_accuracy;
  float distance_est_threshold;
  Butterworth2LowPass lpf;
};

extern struct VisualServoing visual_servoing;

#define GUIDANCE_H_MODE_MODULE_SETTING GUIDANCE_H_MODE_MODULE
#define GUIDANCE_V_MODE_MODULE_SETTING GUIDANCE_V_MODE_MODULE

// Implement own horizontal loop:
extern void guidance_h_module_init(void);
extern void guidance_h_module_enter(void);
extern void guidance_h_module_run(bool in_flight);
extern void guidance_h_module_read_rc(void);

// Implement own Vertical loops
extern void guidance_v_module_init(void);
extern void guidance_v_module_enter(void);
extern void guidance_v_module_run(bool in_flight);

#endif
