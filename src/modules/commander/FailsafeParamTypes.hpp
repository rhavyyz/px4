/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include <stdint.h>

/**
 * Named enum types for Commander failsafe action parameters.
 *
 * Integer values are transcribed from commander_params.yaml and
 * geofence_params.yaml — the YAML files are authoritative. Enums without
 * a continuous value range have gaps noted inline.
 *
 * Included by both failsafe/failsafe.h (replacing its former private enum
 * definitions) and aop_engine/DefaultRules.cpp so that both translation
 * units share the same constants and any drift produces a compile error.
 */

// NAV_DLL_ACT (GCS connection loss) and NAV_RCL_ACT (RC / manual control loss).
// Note: NAV_RCL_ACT has no Disabled option (minimum selectable value is 1).
enum class gcs_connection_loss_failsafe_mode : int32_t {
	Disabled    = 0,
	Hold_mode   = 1,
	Return_mode = 2,
	Land_mode   = 3,
	// 4 is absent from the YAML
	Terminate   = 5,
	Disarm      = 6,
};

// GF_ACTION (geofence breach).
// Integer layout is identical to command_after_high_wind_failsafe and
// command_after_pos_low_failsafe.
enum class geofence_violation_action : int32_t {
	None        = 0,
	Warning     = 1,
	Hold_mode   = 2,
	Return_mode = 3,
	Terminate   = 4,
	Land_mode   = 5,
};

// COM_WIND_MAX_ACT (high-wind failsafe). Layout identical to geofence_violation_action.
enum class command_after_high_wind_failsafe : int32_t {
	None        = 0,
	Warning     = 1,
	Hold_mode   = 2,
	Return_mode = 3,
	Terminate   = 4,
	Land_mode   = 5,
};

// COM_POS_LOW_ACT (low position-accuracy failsafe). Layout identical to geofence_violation_action.
enum class command_after_pos_low_failsafe : int32_t {
	None        = 0,
	Warning     = 1,
	Hold_mode   = 2,
	Return_mode = 3,
	Terminate   = 4,
	Land_mode   = 5,
};

// COM_LOW_BAT_ACT (battery failsafe).
// Value 1 (Return) is deprecated and absent from current YAML but retained
// for param stores that still hold it.
enum class LowBatteryAction : int32_t {
	Warning      = 0,
	Return       = 1, // deprecated
	Land         = 2,
	ReturnOrLand = 3,
};

// COM_FLTT_LOW_ACT (remaining flight time low). Value 2 is absent from the YAML.
enum class command_after_remaining_flight_time_low : int32_t {
	None        = 0,
	Warning     = 1,
	// 2 absent
	Return_mode = 3,
};

// COM_ACT_FAIL_ACT (actuator / motor failure).
enum class actuator_failure_failsafe_mode : int32_t {
	Warning_only = 0,
	Hold_mode    = 1,
	Land_mode    = 2,
	Return_mode  = 3,
	Terminate    = 4,
};

// COM_GNSSLOSS_ACT (GNSS redundancy loss).
enum class gps_redundancy_failsafe_mode : int32_t {
	Warning     = 0,
	Return_mode = 1,
	Land_mode   = 2,
	Terminate   = 3,
};

// COM_QC_ACT (VTOL quadchute / fixed-wing system failure).
enum class command_after_quadchute : int32_t {
	Warning_only = -1,
	Return_mode  =  0,
	Land_mode    =  1,
	Hold_mode    =  2,
};

// COM_OBL_RC_ACT (offboard control signal loss).
enum class offboard_loss_failsafe_mode : int32_t {
	Position_mode = 0,
	Altitude_mode = 1,
	Stabilized    = 2,
	Return_mode   = 3,
	Land_mode     = 4,
	Hold_mode     = 5,
	Terminate     = 6,
	Disarm        = 7,
};

// COM_RC_IN_MODE (manual control input source).
enum class RcInMode : int32_t {
	RcOnly                          = 0,
	MavLinkOnly                     = 1,
	RcOrMavlinkWithFallback         = 2,
	RcOrMavlinkKeepFirst            = 3,
	DisableManualControl            = 4,
	PriorityRcThenMavlinkAscending  = 5,
	PriorityMavlinkAscendingThenRc  = 6,
	PriorityRcThenMavlinkDescending = 7,
	PriorityMavlinkDescendingThenRc = 8,
};
