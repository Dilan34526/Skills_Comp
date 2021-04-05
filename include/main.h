/**
 * \file main.h
 *
 * Contains common definitions and header files used throughout your PROS
 * project.
 *
 * Copyright (c) 2017-2020, Purdue University ACM SIGBots.
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef _PROS_MAIN_H_
#define _PROS_MAIN_H_

/**
 * If defined, some commonly used enums will have preprocessor macros which give
 * a shorter, more convenient naming pattern. If this isn't desired, simply
 * comment the following line out.
 *
 * For instance, E_CONTROLLER_MASTER has a shorter name: CONTROLLER_MASTER.
 * E_CONTROLLER_MASTER is pedantically correct within the PROS styleguide, but
 * not convienent for most student programmers.
 */
#define PROS_USE_SIMPLE_NAMES

/**
 * If defined, C++ literals will be available for use. All literals are in the
 * pros::literals namespace.
 *
 * For instance, you can do `4_mtr = 50` to set motor 4's target velocity to 50
 */
#define PROS_USE_LITERALS

#include "api.h"
#include "auton.hpp"
#include "base.hpp"
#include "count.hpp"
#include "correct.hpp"
#include "controller.hpp"
#include "drive.hpp"
#include "intake.hpp"
#include "indexer.hpp"
#include "sensor.hpp"
#include "shooter.hpp"
#include "turn.hpp"
#include "util.hpp"

using namespace pros;

//ports
//motors
#define DLF 11
#define DLB 12
#define DRF 20
#define DRB 19
#define DIMU 8
#define INL 13
#define INR 18
#define INDX 16
#define SHOT 15

//optical
#define INDX_OPT 14
#define SHOT_OPT 21

//distance
#define LFT 1
#define EMPTY 9
#define DOWN 4
#define BCK 10
#define DIAG 7

//v4 sensors
#define LINE 'A'
#define SIDE_1 'G'
#define SIDE_2 'H'

extern Motor dlf;
extern Motor dlb;
extern Motor drf;
extern Motor drb;
extern Motor inl;
extern Motor inr;
extern Motor indx;
extern Motor shot;

extern Controller m;

extern Imu imu;
extern Distance down;
extern Distance left;
extern Distance back;
extern Distance empty;
extern Distance diag;
extern Optical indxOpt;

extern ADIAnalogIn line;
extern ADIEncoder track;

#define WHEEL_CIRCMF (3.25 * M_PI)
#define TRACK_CIRCMF (2.75 * M_PI)

/**
 * You should add more #includes here
 */
//#include "okapi/api.hpp"
//#include "pros/api_legacy.h"

/**
 * If you find doing pros::Motor() to be tedious and you'd prefer just to do
 * Motor, you can use the namespace with the following commented out line.
 *
 * IMPORTANT: Only the okapi or pros namespace may be used, not both
 * concurrently! The okapi namespace will export all symbols inside the pros
 * namespace.
 */
// using namespace pros;
// using namespace pros::literals;
// using namespace okapi;

/**
 * Prototypes for the competition control tasks are redefined here to ensure
 * that they can be called from user code (i.e. calling autonomous from a
 * button press in opcontrol() for testing purposes).
 */
#ifdef __cplusplus
extern "C" {
#endif
void autonomous(void);
void initialize(void);
void disabled(void);
void competition_initialize(void);
void opcontrol(void);
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
/**
 * You can add C++-only headers here
 */
//#include <iostream>
#endif

#endif  // _PROS_MAIN_H_
