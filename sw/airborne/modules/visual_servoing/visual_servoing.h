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

struct VisualServoing {
  float divergence_sp;          ///< agl = height from sonar (only used when using "fake" divergence)
  float acc_x_sp;                 ///< low-pass version of agl
  float acc_y_sp;               ///< low-pass filter constant
  float ol_x_pgain;                    ///< vertical velocity as determined with sonar (only used when using "fake" divergence)
  float ol_y_pgain;    ///< setpoint for constant divergence approach
  float ol_z_pgain;                  ///< P-gain for constant divergence control (from divergence error to thrust)
  float ol_x_dgain;                  ///< I-gain for constant divergence control
  float ol_y_dgain;                  ///< D-gain for constant divergence control
  float ol_z_dgain;             ///< Divergence estimate
  float il_h_pgain;           ///< Previous divergence tracking error
  float il_h_pgain;                ///< integration of the error for I-gain
  float il_h_pgain;                  ///< difference of error for the D-gain
};

extern struct VisualServoing visual_servoing;

// settings
extern float oag_color_count_frac;  // obstacle detection threshold as a fraction of total of image
extern float oag_floor_count_frac;  // floor detection threshold as a fraction of total of image
extern float oag_max_speed;         // max flight speed [m/s]
extern float oag_heading_rate;      // heading rate setpoint [rad/s]

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
