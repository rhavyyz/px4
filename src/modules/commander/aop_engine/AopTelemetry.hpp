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

#include <uORB/topics/airspeed.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/failsafe_flags.h>
#include <uORB/topics/safe_land_topic.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>

/**
 * Snapshot of all vehicle telemetry for a single AOP evaluation cycle.
 *
 * Populated by Commander each cycle from uORB subscriptions and passed to
 * AopEngine::update(). Condition functions receive a const reference and read
 * only what they need; all other fields are zero-initialised.
 */
struct AopTelemetry {
	failsafe_flags_s          failsafe_flags{};
	vehicle_local_position_s  local_position{};
	vehicle_attitude_s        attitude{};
	vehicle_global_position_s global_position{};
	battery_status_s          battery{};
	airspeed_s                airspeed{};
	safe_land_topic_s         safe_land{};
	uint8_t                   nav_state{0};   ///< user_intended_nav_state, for mode-gated conditions
};

/**
 * Snapshot of all failsafe-relevant parameter values.
 *
 * Populated by AopEngine::updateParams() from the DEFINE_PARAMETERS members.
 * Passed by const reference to every condition function so that param-based
 * filtering (e.g. "skip when action == Warning") can happen inside the function
 * without coupling it to the ModuleParams hierarchy.
 */
struct AopEnv {
	int32_t nav_rcl_act{};
	int32_t nav_dll_act{};
	int32_t com_rc_in_mode{};
	int32_t gf_action{};
	int32_t com_wind_max_act{};
	int32_t com_pos_low_act{};
	int32_t com_low_bat_act{};
	int32_t com_fltt_low_act{};
	int32_t com_act_fail_act{};
	int32_t com_gnssloss_act{};
	int32_t cbrk_flightterm{};
	int32_t com_qc_act{};
	int32_t com_obl_rc_act{};
	float   com_spoolup_time{};
	int32_t com_rcl_except{};   ///< COM_RCL_EXCEPT bitmask — RC-loss exception modes
	int32_t com_dll_except{};   ///< COM_DLL_EXCEPT bitmask — GCS-loss exception modes
};
