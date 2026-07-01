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

/**
 * @file DefaultRules.cpp
 *
 * AopEngine::load_rules() — default safety rule definitions.
 *
 * Each rule is an AspectDefinition with a single ConditionFn:
 *   bool my_condition(const AopTelemetry &t, const AopEnv &env, const AopMission *)
 *
 * Only rules that produce a nav_state behaviour change (Hold, RTL, Land,
 * Terminate, Disarm) are registered. Warn-only outcomes are excluded.
 *
 * Reference: src/modules/commander/failsafe/failsafe.cpp — Failsafe::checkStateAndMode()
 * That file is kept untouched as authoritative reference.
 *
 * Known limitations vs. the reference (see plan file for details):
 *   - VTOL_TAKEOFF link-loss suppression during forward-flight transition not implemented
 *   - Parachute (COM_PARACHUTE) and Remote ID (COM_ARM_ODID) rules not implemented
 *   - Mode fallback Pos→Alt→Stab cascade not implemented (engine DESCEND fallback applies)
 *   - Offboard loss + RC lost → NAV_RCL_ACT chained logic only partially covered
 */

#include "AopEngine.hpp"
#include "AopTelemetry.hpp"
#include "AopTransitions.hpp"
#include "../FailsafeParamTypes.hpp"
#include "px4_platform_common/log.h"

#include <lib/circuit_breaker/circuit_breaker.h>
#include <lib/geo/geo.h>
#include <math.h>
#include <stdio.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/vehicle_status.h>

using namespace px4_aop;
using vs = vehicle_status_s;

// ---------------------------------------------------------------------------
// Helper: mirrors Failsafe::isFailsafeIgnored() (failsafe.cpp:497).
// Bit positions match LinkLossExceptionBits in failsafe/failsafe.h.
// ---------------------------------------------------------------------------
static bool link_loss_ignored(uint8_t nav_state, int32_t except_mask)
{
	switch (nav_state) {
	case vs::NAVIGATION_STATE_AUTO_MISSION:
		return except_mask & (1 << 0);

	case vs::NAVIGATION_STATE_AUTO_LOITER:
	case vs::NAVIGATION_STATE_AUTO_TAKEOFF:
	case vs::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF:
	case vs::NAVIGATION_STATE_AUTO_LAND:
	case vs::NAVIGATION_STATE_AUTO_RTL:
	case vs::NAVIGATION_STATE_DESCEND:
	case vs::NAVIGATION_STATE_AUTO_FOLLOW_TARGET:
	case vs::NAVIGATION_STATE_AUTO_PRECLAND:
	case vs::NAVIGATION_STATE_ORBIT:
	case vs::NAVIGATION_STATE_GUIDED_COURSE:
		return except_mask & (1 << 1);

	case vs::NAVIGATION_STATE_OFFBOARD:
		return except_mask & (1 << 2);

	case vs::NAVIGATION_STATE_EXTERNAL1:
	case vs::NAVIGATION_STATE_EXTERNAL2:
	case vs::NAVIGATION_STATE_EXTERNAL3:
	case vs::NAVIGATION_STATE_EXTERNAL4:
	case vs::NAVIGATION_STATE_EXTERNAL5:
	case vs::NAVIGATION_STATE_EXTERNAL6:
	case vs::NAVIGATION_STATE_EXTERNAL7:
	case vs::NAVIGATION_STATE_EXTERNAL8:
		return except_mask & (1 << 3);

	case vs::NAVIGATION_STATE_ALTITUDE_CRUISE:
		return except_mask & (1 << 4);

	default:
		return false;
	}
}

// ---------------------------------------------------------------------------
// Action helpers — translate param value to AspectDefinition advice + policy.
// Return false when the param maps to no behaviour change (Disabled/None/Warning).
// Caller must skip _add_rule() when the helper returns false.
// ---------------------------------------------------------------------------

static bool apply_nav_dll_rcl_action(AspectDefinition &def, int32_t val)
{
	using mode = gcs_connection_loss_failsafe_mode;

	switch (mode(val)) {
	default:
	case mode::Disabled:
		return false;

	case mode::Hold_mode:
		def.transition      = go_to_state(vs::NAVIGATION_STATE_AUTO_LOITER);
		def.clear_condition = ClearOn::OnModeChange;
		return true;

	case mode::Return_mode:
		def.transition      = go_to_state(vs::NAVIGATION_STATE_AUTO_RTL);
		def.clear_condition = ClearOn::OnModeChange;
		return true;

	case mode::Land_mode:
		def.transition      = go_to_state(vs::NAVIGATION_STATE_AUTO_LAND);
		def.clear_condition = ClearOn::OnModeChange;
		return true;

	case mode::Terminate:
		def.transition          = go_to_terminate();
		def.clear_condition     = ClearOn::Never;
		def.allow_user_takeover = false;
		def.can_be_deferred     = false;
		return true;

	case mode::Disarm:
		def.transition          = go_to_disarm();
		def.allow_user_takeover = false;
		return true;
	}
}

static bool apply_gf_action(AspectDefinition &def, int32_t val)
{
	using mode = geofence_violation_action;

	switch (mode(val)) {
	default:
	case mode::None:
	case mode::Warning:
		return false;

	case mode::Hold_mode:
		def.transition                    = go_to_state(vs::NAVIGATION_STATE_AUTO_LOITER);
		def.clear_condition               = ClearOn::OnModeChange;
		def.allow_user_takeover           = true;
		def.takeover_requires_mode_switch = true;
		return true;

	case mode::Return_mode:
		def.transition      = go_to_state(vs::NAVIGATION_STATE_AUTO_RTL);
		def.clear_condition = ClearOn::OnModeChange;
		return true;

	case mode::Terminate:
		def.transition          = go_to_terminate();
		def.clear_condition     = ClearOn::Never;
		def.allow_user_takeover = false;
		def.can_be_deferred     = false;
		return true;

	case mode::Land_mode:
		def.transition      = go_to_state(vs::NAVIGATION_STATE_AUTO_LAND);
		def.clear_condition = ClearOn::OnModeChange;
		return true;
	}
}

static bool apply_wind_pos_action(AspectDefinition &def, int32_t val)
{
	// Same enum layout as geofence_violation_action; no mode-switch-only escape.
	using mode = geofence_violation_action;

	switch (mode(val)) {
	default:
	case mode::None:
	case mode::Warning:
		return false;

	case mode::Hold_mode:
		def.transition      = go_to_state(vs::NAVIGATION_STATE_AUTO_LOITER);
		def.clear_condition = ClearOn::OnModeChange;
		return true;

	case mode::Return_mode:
		def.transition      = go_to_state(vs::NAVIGATION_STATE_AUTO_RTL);
		def.clear_condition = ClearOn::OnModeChange;
		return true;

	case mode::Terminate:
		def.transition          = go_to_terminate();
		def.clear_condition     = ClearOn::Never;
		def.allow_user_takeover = false;
		def.can_be_deferred     = false;
		return true;

	case mode::Land_mode:
		def.transition      = go_to_state(vs::NAVIGATION_STATE_AUTO_LAND);
		def.clear_condition = ClearOn::OnModeChange;
		return true;
	}
}

static bool apply_quadchute_action(AspectDefinition &def, int32_t val)
{
	using mode = command_after_quadchute;

	switch (mode(val)) {
	default:
	case mode::Warning_only:
		return false;

	case mode::Return_mode:
		def.transition      = go_to_state(vs::NAVIGATION_STATE_AUTO_RTL);
		def.clear_condition = ClearOn::OnModeChange;
		return true;

	case mode::Land_mode:
		def.transition      = go_to_state(vs::NAVIGATION_STATE_AUTO_LAND);
		def.clear_condition = ClearOn::OnModeChange;
		return true;

	case mode::Hold_mode:
		def.transition      = go_to_state(vs::NAVIGATION_STATE_AUTO_LOITER);
		def.clear_condition = ClearOn::OnModeChange;
		return true;
	}
}

static bool apply_actuator_fail_action(AspectDefinition &def, int32_t val)
{
	using mode = actuator_failure_failsafe_mode;

	switch (mode(val)) {
	default:
	case mode::Warning_only:
		return false;

	case mode::Hold_mode:
		def.transition = go_to_state(vs::NAVIGATION_STATE_AUTO_LOITER);
		return true;

	case mode::Land_mode:
		def.transition      = go_to_state(vs::NAVIGATION_STATE_AUTO_LAND);
		def.clear_condition = ClearOn::OnModeChange;
		return true;

	case mode::Return_mode:
		def.transition      = go_to_state(vs::NAVIGATION_STATE_AUTO_RTL);
		def.clear_condition = ClearOn::OnModeChange;
		return true;

	case mode::Terminate:
		def.transition      = go_to_terminate();
		def.clear_condition = ClearOn::Never;
		def.can_be_deferred = false;
		return true;
	}
}

static bool apply_gnss_loss_action(AspectDefinition &def, int32_t val)
{
	using mode = gps_redundancy_failsafe_mode;

	switch (mode(val)) {
	default:
	case mode::Warning:
		return false;

	case mode::Return_mode:
		def.transition      = go_to_state(vs::NAVIGATION_STATE_AUTO_RTL);
		def.clear_condition = ClearOn::OnModeChange;
		return true;

	case mode::Land_mode:
		def.transition      = go_to_state(vs::NAVIGATION_STATE_AUTO_LAND);
		def.clear_condition = ClearOn::OnModeChange;
		return true;

	case mode::Terminate:
		def.transition          = go_to_terminate();
		def.clear_condition     = ClearOn::Never;
		def.allow_user_takeover = false;
		def.can_be_deferred     = false;
		return true;
	}
}

static bool apply_offboard_loss_action(AspectDefinition &def, int32_t val)
{
	using mode = offboard_loss_failsafe_mode;

	switch (mode(val)) {
	default:
	case mode::Position_mode:
		def.transition      = go_to_state(vs::NAVIGATION_STATE_POSCTL);
		def.clear_condition = ClearOn::OnModeChange;
		return true;

	case mode::Altitude_mode:
		def.transition      = go_to_state(vs::NAVIGATION_STATE_ALTCTL);
		def.clear_condition = ClearOn::OnModeChange;
		return true;

	case mode::Stabilized:
		def.transition      = go_to_state(vs::NAVIGATION_STATE_STAB);
		def.clear_condition = ClearOn::OnModeChange;
		return true;

	case mode::Return_mode:
		def.transition      = go_to_state(vs::NAVIGATION_STATE_AUTO_RTL);
		def.clear_condition = ClearOn::OnModeChange;
		return true;

	case mode::Land_mode:
		def.transition      = go_to_state(vs::NAVIGATION_STATE_AUTO_LAND);
		def.clear_condition = ClearOn::OnModeChange;
		return true;

	case mode::Hold_mode:
		def.transition      = go_to_state(vs::NAVIGATION_STATE_AUTO_LOITER);
		def.clear_condition = ClearOn::OnModeChange;
		return true;

	case mode::Terminate:
		def.transition          = go_to_terminate();
		def.clear_condition     = ClearOn::Never;
		def.allow_user_takeover = false;
		return true;

	case mode::Disarm:
		def.transition          = go_to_disarm();
		def.allow_user_takeover = false;
		return true;
	}
}

static bool apply_fltt_low_action(AspectDefinition &def, int32_t val)
{
	using mode = command_after_remaining_flight_time_low;

	switch (mode(val)) {
	default:
	case mode::None:
	case mode::Warning:
		return false;

	case mode::Return_mode:
		def.transition          = go_to_state(vs::NAVIGATION_STATE_AUTO_RTL);
		def.clear_condition     = ClearOn::OnModeChange;
		def.allow_user_takeover = true;
		return true;
	}
}

// ---------------------------------------------------------------------------
// Condition functions — one per logical failsafe trigger.
// ---------------------------------------------------------------------------

static bool cond_rc_loss(const AopTelemetry &t, const AopEnv &env, const AopMission *)
{
	return t.failsafe_flags.manual_control_signal_lost
	       && env.com_rc_in_mode != int32_t(RcInMode::DisableManualControl)
	       && !link_loss_ignored(t.nav_state, env.com_rcl_except);
}

static bool cond_gcs_loss(const AopTelemetry &t, const AopEnv &env, const AopMission *)
{
	if (!t.failsafe_flags.gcs_connection_lost) { return false; }

	if (env.nav_dll_act == int32_t(gcs_connection_loss_failsafe_mode::Disabled)) { return false; }

	// GCS loss is suppressed during AUTO_LAND and AUTO_PRECLAND (failsafe.cpp:562)
	if (t.nav_state == vs::NAVIGATION_STATE_AUTO_LAND
	    || t.nav_state == vs::NAVIGATION_STATE_AUTO_PRECLAND) { return false; }

	return !link_loss_ignored(t.nav_state, env.com_dll_except);
}

static bool cond_vtol_quadchute(const AopTelemetry &t, const AopEnv &, const AopMission *)
{
	// Fires only in the four modes checked by legacy code (failsafe.cpp:573-578)
	if (!t.failsafe_flags.vtol_fixed_wing_system_failure) { return false; }

	return t.nav_state == vs::NAVIGATION_STATE_AUTO_MISSION
	       || t.nav_state == vs::NAVIGATION_STATE_AUTO_LOITER
	       || t.nav_state == vs::NAVIGATION_STATE_AUTO_TAKEOFF
	       || t.nav_state == vs::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF;
}

static bool cond_mission_failure(const AopTelemetry &t, const AopEnv &, const AopMission *)
{
	return t.failsafe_flags.mission_failure;
}

static bool cond_wind_limit(const AopTelemetry &t, const AopEnv &env, const AopMission *)
{
	return t.failsafe_flags.wind_limit_exceeded
	       && env.com_wind_max_act != int32_t(command_after_high_wind_failsafe::None)
	       && env.com_wind_max_act != int32_t(command_after_high_wind_failsafe::Warning);
}

static bool cond_flight_time_limit(const AopTelemetry &t, const AopEnv &, const AopMission *)
{
	return t.failsafe_flags.flight_time_limit_exceeded;
}

static bool cond_pos_accuracy_low(const AopTelemetry &t, const AopEnv &env, const AopMission *)
{
	if (!t.failsafe_flags.position_accuracy_low) { return false; }

	// Only in AUTO_MISSION and AUTO_LOITER (failsafe.cpp:600-602)
	if (t.nav_state != vs::NAVIGATION_STATE_AUTO_MISSION
	    && t.nav_state != vs::NAVIGATION_STATE_AUTO_LOITER) { return false; }

	return env.com_pos_low_act != int32_t(command_after_pos_low_failsafe::None)
	       && env.com_pos_low_act != int32_t(command_after_pos_low_failsafe::Warning);
}

// navigator_failure in TAKEOFF or AUTO_RTL — Land action (failsafe.cpp:605-608)
static bool cond_nav_fail_land_modes(const AopTelemetry &t, const AopEnv &, const AopMission *)
{
	return t.failsafe_flags.navigator_failure
	       && (t.nav_state == vs::NAVIGATION_STATE_AUTO_TAKEOFF
		   || t.nav_state == vs::NAVIGATION_STATE_AUTO_RTL);
}

// navigator_failure in any other mode — Hold action (failsafe.cpp:610-612)
static bool cond_navigator_failure(const AopTelemetry &t, const AopEnv &, const AopMission *)
{
	return t.failsafe_flags.navigator_failure;
}

static bool cond_geofence(const AopTelemetry &t, const AopEnv &env, const AopMission *)
{
	return t.failsafe_flags.geofence_breached
	       && env.gf_action != int32_t(geofence_violation_action::None)
	       && env.gf_action != int32_t(geofence_violation_action::Warning);
}

static bool cond_battery_time_low(const AopTelemetry &t, const AopEnv &env, const AopMission *)
{
	if (t.safe_land.valid) { return false; }
	return t.failsafe_flags.battery_low_remaining_time
	       && env.com_fltt_low_act != int32_t(command_after_remaining_flight_time_low::None)
	       && env.com_fltt_low_act != int32_t(command_after_remaining_flight_time_low::Warning);
}

// ---------------------------------------------------------------------------
// Mission-achievability helper used by cond_battery_smart_reroute.
// Returns estimated seconds to complete the remaining mission waypoints,
// or INFINITY when no valid mission data is available.
// ---------------------------------------------------------------------------

static float mission_time_estimate_s(const AopTelemetry &t, const AopMission *mission)
{
	if (!mission || mission->count == 0 || mission->current_seq < 0) {
		return INFINITY;
	}

	float dist = 0.f;
	const int start = static_cast<int>(mission->current_seq);

	for (int i = start; i < static_cast<int>(mission->count) - 1; i++) {
		dist += get_distance_to_next_waypoint(
				mission->waypoints[i].lat, mission->waypoints[i].lon,
				mission->waypoints[i + 1].lat, mission->waypoints[i + 1].lon);
	}

	const float spd = fmaxf(sqrtf(t.local_position.vx * t.local_position.vx
				      + t.local_position.vy * t.local_position.vy), 1.f);
	return dist / spd;
}

static bool cond_battery_smart_reroute(const AopTelemetry &t, const AopEnv &env, const AopMission *mission)
{
	if (LowBatteryAction(env.com_low_bat_act) == LowBatteryAction::Warning) { return false; }

	const uint8_t warn       = t.failsafe_flags.battery_warning;
	const bool bat_critical  = warn == battery_status_s::WARNING_CRITICAL;
	const bool bat_emergency = warn == battery_status_s::WARNING_EMERGENCY;

	if (!bat_critical && !bat_emergency) { return false; }

	// Must be actively running a mission
	if (t.nav_state != vs::NAVIGATION_STATE_AUTO_MISSION) { return false; }

	// Safe-land zone must be active
	if (!t.safe_land.valid) { return false; }

	// At EMERGENCY: always reroute — no time estimate needed or reliable
	if (bat_emergency) { return true; }

	// At CRITICAL: only reroute if battery can't complete the remaining mission
	if (!PX4_ISFINITE(t.battery.time_remaining_s) || t.battery.time_remaining_s < 0.f) { return false; }
	return t.battery.time_remaining_s < mission_time_estimate_s(t, mission);
}

static bool cond_battery_unhealthy(const AopTelemetry &t, const AopEnv &, const AopMission *)
{
	return t.failsafe_flags.battery_unhealthy;
}

static bool cond_battery_critical(const AopTelemetry &t, const AopEnv &env, const AopMission *)
{
	if (t.safe_land.valid) { return false; }
	return t.failsafe_flags.battery_warning == battery_status_s::WARNING_CRITICAL
	       && LowBatteryAction(env.com_low_bat_act) != LowBatteryAction::Warning;
}

static bool cond_battery_emergency(const AopTelemetry &t, const AopEnv &env, const AopMission *)
{
	if (t.safe_land.valid) { return false; }
	return t.failsafe_flags.battery_warning == battery_status_s::WARNING_EMERGENCY
	       && LowBatteryAction(env.com_low_bat_act) != LowBatteryAction::Warning;
}

static bool cond_esc_arming_fail(const AopTelemetry &t, const AopEnv &, const AopMission *)
{
	return t.failsafe_flags.fd_esc_arming_failure;
}

static bool cond_fd_critical(const AopTelemetry &t, const AopEnv &, const AopMission *)
{
	return t.failsafe_flags.fd_critical_failure;
}

static bool cond_fd_alt_loss(const AopTelemetry &t, const AopEnv &, const AopMission *)
{
	return t.failsafe_flags.fd_alt_loss;
}

static bool cond_fd_motor_failure(const AopTelemetry &t, const AopEnv &env, const AopMission *)
{
	return t.failsafe_flags.fd_motor_failure
	       && env.com_act_fail_act != int32_t(actuator_failure_failsafe_mode::Warning_only);
}

static bool cond_gnss_lost(const AopTelemetry &t, const AopEnv &env, const AopMission *)
{
	return t.failsafe_flags.gnss_lost
	       && env.com_gnssloss_act != int32_t(gps_redundancy_failsafe_mode::Warning);
}

static bool cond_offboard_loss(const AopTelemetry &t, const AopEnv &, const AopMission *)
{
	// Fires only when the current mode actually requires offboard signal (failsafe.cpp:725)
	return t.failsafe_flags.offboard_control_signal_lost
	       && (t.failsafe_flags.mode_req_offboard_signal & (1u << t.nav_state));
}

// ---------------------------------------------------------------------------
// Rule registration
// ---------------------------------------------------------------------------

void AopEngine::load_rules()
{
	_reset_rules();

	// Priority scheme (higher = fires first):
	//   200  — spoolup-window Disarm/Terminate (irrecoverable hardware failures)
	//   150  — post-spoolup Terminate (fd_critical, fd_alt_loss)
	//   100  — battery emergency
	//    80  — battery critical
	//    70  — RC / GCS link loss
	//    60  — geofence, flight-time-limit, wind (cannot be deferred)
	//    51  — VTOL quadchute, navigator failure → Land
	//    50  — mission failure, navigator failure → Hold
	//    40  — offboard loss, battery flight-time remaining
	//    30  — position accuracy, motor failure, GNSS loss

	const float spoolup = _env.com_spoolup_time;

	// -----------------------------------------------------------------------
	// Priority 200 — spoolup-window hardware failures → Disarm
	// -----------------------------------------------------------------------

	{
		AspectDefinition def{};
		snprintf(def.name, MAX_NAME_LEN, "bat_unhealthy_spoolup");
		def.match_any_mode          = true;
		def.condition               = &cond_battery_unhealthy;
		def.transition              = go_to_disarm();
		def.priority                = 200;
		def.can_be_deferred         = false;
		def.clear_condition         = ClearOn::OnDisarm;
		def.max_time_since_arm_s    = spoolup;
		_add_rule(def);
	}

	{
		AspectDefinition def{};
		snprintf(def.name, MAX_NAME_LEN, "esc_arming_spoolup");
		def.match_any_mode          = true;
		def.condition               = &cond_esc_arming_fail;
		def.transition              = go_to_disarm();
		def.priority                = 200;
		def.can_be_deferred         = false;
		def.clear_condition         = ClearOn::OnDisarm;
		def.max_time_since_arm_s    = spoolup;
		_add_rule(def);
	}

	{
		AspectDefinition def{};
		snprintf(def.name, MAX_NAME_LEN, "fd_critical_spoolup");
		def.match_any_mode          = true;
		def.condition               = &cond_fd_critical;
		def.transition              = go_to_disarm();
		def.priority                = 200;
		def.can_be_deferred         = false;
		def.clear_condition         = ClearOn::OnDisarm;
		def.max_time_since_arm_s    = spoolup + 3.0f;
		_add_rule(def);
	}

	{
		AspectDefinition def{};
		snprintf(def.name, MAX_NAME_LEN, "fd_alt_loss_spoolup");
		def.match_any_mode          = true;
		def.condition               = &cond_fd_alt_loss;
		def.transition              = go_to_disarm();
		def.priority                = 200;
		def.can_be_deferred         = false;
		def.clear_condition         = ClearOn::OnDisarm;
		def.max_time_since_arm_s    = spoolup + 3.0f;
		_add_rule(def);
	}

	// -----------------------------------------------------------------------
	// Priority 150 — post-spoolup fd_critical / fd_alt_loss → Terminate
	// Only registered when CBRK_FLIGHTTERM circuit-breaker is NOT enabled.
	// When the breaker is enabled, these degrade to Warn (not registered).
	// -----------------------------------------------------------------------

	if (!circuit_breaker_enabled_by_val(_env.cbrk_flightterm, CBRK_FLIGHTTERM_KEY)) {
		{
			AspectDefinition def{};
			snprintf(def.name, MAX_NAME_LEN, "fd_critical_normal");
			def.match_any_mode          = true;
			def.condition               = &cond_fd_critical;
			def.transition              = go_to_terminate();
			def.priority                = 150;
			def.can_be_deferred         = false;
			def.allow_user_takeover     = false;
			def.clear_condition         = ClearOn::Never;
			def.min_time_since_arm_s    = spoolup + 3.0f;
			_add_rule(def);
		}

		{
			AspectDefinition def{};
			snprintf(def.name, MAX_NAME_LEN, "fd_alt_loss_normal");
			def.match_any_mode          = true;
			def.condition               = &cond_fd_alt_loss;
			def.transition              = go_to_terminate();
			def.priority                = 150;
			def.can_be_deferred         = false;
			def.allow_user_takeover     = false;
			def.clear_condition         = ClearOn::Never;
			def.min_time_since_arm_s    = spoolup + 3.0f;
			_add_rule(def);
		}
	}

	// -----------------------------------------------------------------------
	// Priority 110 — smart battery reroute to safe_land zone
	// Pre-empts both battery_emergency and battery_critical when:
	//   • battery is at CRITICAL or EMERGENCY level
	//   • vehicle is in AUTO_MISSION
	//   • a safe_land coordinate has been published (non-zero lat/lon)
	//   • remaining battery time < estimated time to finish the mission
	// Flies the drone to the safe_land zone (60 s loiter window) then lands.
	// -----------------------------------------------------------------------

	{
		AspectDefinition def{};
		snprintf(def.name, MAX_NAME_LEN, "batt_reroute_safe");
		def.match_any_mode  = true;
		def.condition       = &cond_battery_smart_reroute;
		def.priority        = 110;
		def.clear_condition = ClearOn::WhenConditionClears;
		def.transition      = go_to_safe_land_then_land();
		def.delay_s         = 60.f;
		_add_rule(def);
	}

	// -----------------------------------------------------------------------
	// Priority 100 — battery emergency
	// Action depends on COM_LOW_BAT_ACT (failsafe.cpp:193-212).
	// -----------------------------------------------------------------------

	{
		const auto bat_act = LowBatteryAction(_env.com_low_bat_act);

		if (bat_act != LowBatteryAction::Warning) {
			AspectDefinition def{};
			snprintf(def.name, MAX_NAME_LEN, "battery_emergency");
			def.match_any_mode  = true;
			def.condition       = &cond_battery_emergency;
			def.priority        = 100;
			def.clear_condition = ClearOn::WhenConditionClears;

			if (bat_act == LowBatteryAction::Return) {
				// deprecated Return → RTL with Land fallback
				def.transition = go_to_state(vs::NAVIGATION_STATE_AUTO_RTL,
							     vs::NAVIGATION_STATE_AUTO_LAND);
			} else {
				// Land or ReturnOrLand → Land (failsafe.cpp:201-204)
				def.transition = go_to_state(vs::NAVIGATION_STATE_AUTO_LAND);
			}

			_add_rule(def);
		}
	}

	// -----------------------------------------------------------------------
	// Priority 80 — battery critical
	// Action depends on COM_LOW_BAT_ACT (failsafe.cpp:172-191).
	// -----------------------------------------------------------------------

	{
		const auto bat_act = LowBatteryAction(_env.com_low_bat_act);

		if (bat_act != LowBatteryAction::Warning) {
			AspectDefinition def{};
			snprintf(def.name, MAX_NAME_LEN, "battery_critical");
			def.match_any_mode  = true;
			def.condition       = &cond_battery_critical;
			def.priority        = 80;
			def.clear_condition = ClearOn::WhenConditionClears;

			if (bat_act == LowBatteryAction::Land) {
				def.transition = go_to_safe_land();

			} else {
				// Return (deprecated) or ReturnOrLand → RTL with Land fallback
				def.transition = go_to_state(vs::NAVIGATION_STATE_AUTO_RTL,
							     vs::NAVIGATION_STATE_AUTO_LAND);
			}

			_add_rule(def);
		}
	}

	// -----------------------------------------------------------------------
	// Priority 70 — RC loss / GCS loss
	// -----------------------------------------------------------------------

	{
		AspectDefinition def{};
		snprintf(def.name, MAX_NAME_LEN, "rc_loss");
		def.match_any_mode = true;
		def.condition      = &cond_rc_loss;
		def.priority       = 70;

		if (apply_nav_dll_rcl_action(def, _env.nav_rcl_act)) {
			_add_rule(def);
		}
	}

	{
		AspectDefinition def{};
		snprintf(def.name, MAX_NAME_LEN, "gcs_loss");
		def.match_any_mode = true;
		def.condition      = &cond_gcs_loss;
		def.priority       = 70;

		if (apply_nav_dll_rcl_action(def, _env.nav_dll_act)) {
			_add_rule(def);
		}
	}

	// -----------------------------------------------------------------------
	// Priority 60 — cannot be deferred: geofence, flight-time-limit, wind
	// -----------------------------------------------------------------------

	{
		AspectDefinition def{};
		snprintf(def.name, MAX_NAME_LEN, "geofence_breach");
		def.match_any_mode  = true;
		def.condition       = &cond_geofence;
		def.priority        = 60;
		def.can_be_deferred = false;

		if (apply_gf_action(def, _env.gf_action)) {
			_add_rule(def);
		}
	}

	{
		AspectDefinition def{};
		snprintf(def.name, MAX_NAME_LEN, "flight_time_limit");
		def.match_any_mode          = true;
		def.condition               = &cond_flight_time_limit;
		def.transition              = go_to_state(vs::NAVIGATION_STATE_AUTO_RTL,
							   vs::NAVIGATION_STATE_AUTO_LAND);
		def.priority                = 60;
		def.can_be_deferred         = false;
		def.clear_condition         = ClearOn::OnModeChange;
		_add_rule(def);
	}

	{
		AspectDefinition def{};
		snprintf(def.name, MAX_NAME_LEN, "wind_limit");
		def.match_any_mode  = true;
		def.condition       = &cond_wind_limit;
		def.priority        = 60;
		def.can_be_deferred = false;

		if (apply_wind_pos_action(def, _env.com_wind_max_act)) {
			_add_rule(def);
		}
	}

	// -----------------------------------------------------------------------
	// Priority 51/50 — VTOL quadchute, mission failure, navigator failure
	// navigator_failure has two rules: priority 51 for TAKEOFF/RTL → Land,
	// priority 50 for all other modes → Hold. The engine's priority selection
	// ensures the Land rule wins when the vehicle is in TAKEOFF or AUTO_RTL.
	// -----------------------------------------------------------------------

	{
		AspectDefinition def{};
		snprintf(def.name, MAX_NAME_LEN, "vtol_quadchute");
		def.match_any_mode = true;
		def.condition      = &cond_vtol_quadchute;
		def.priority       = 51;

		if (apply_quadchute_action(def, _env.com_qc_act)) {
			_add_rule(def);
		}
	}

	{
		// navigator_failure in TAKEOFF / AUTO_RTL → Land (failsafe.cpp:605-608)
		AspectDefinition def{};
		snprintf(def.name, MAX_NAME_LEN, "nav_fail_land");
		def.match_any_mode          = true;
		def.condition               = &cond_nav_fail_land_modes;
		def.transition              = go_to_state(vs::NAVIGATION_STATE_AUTO_LAND);
		def.priority                = 51;
		def.clear_condition         = ClearOn::OnModeChange;
		_add_rule(def);
	}

	{
		// mission failure → RTL (failsafe.cpp:582)
		AspectDefinition def{};
		snprintf(def.name, MAX_NAME_LEN, "mission_failure");
		def.match_any_mode          = true;
		def.condition               = &cond_mission_failure;
		def.transition              = go_to_state(vs::NAVIGATION_STATE_AUTO_RTL,
							   vs::NAVIGATION_STATE_AUTO_LAND);
		def.priority                = 50;
		def.clear_condition         = ClearOn::OnModeChange;
		_add_rule(def);
	}

	{
		// navigator_failure in any other mode → Hold (failsafe.cpp:610-612)
		AspectDefinition def{};
		snprintf(def.name, MAX_NAME_LEN, "nav_fail_hold");
		def.match_any_mode          = true;
		def.condition               = &cond_navigator_failure;
		def.transition              = go_to_state(vs::NAVIGATION_STATE_AUTO_LOITER,
							   vs::NAVIGATION_STATE_DESCEND);
		def.priority                = 50;
		def.clear_condition         = ClearOn::OnModeChange;
		_add_rule(def);
	}

	// -----------------------------------------------------------------------
	// Priority 40 — offboard loss, battery flight-time remaining
	// -----------------------------------------------------------------------

	{
		AspectDefinition def{};
		snprintf(def.name, MAX_NAME_LEN, "offboard_loss");
		def.match_any_mode = true;
		def.condition      = &cond_offboard_loss;
		def.priority       = 40;

		if (apply_offboard_loss_action(def, _env.com_obl_rc_act)) {
			_add_rule(def);
		}
	}

	{
		AspectDefinition def{};
		snprintf(def.name, MAX_NAME_LEN, "battery_time_low");
		def.match_any_mode = true;
		def.condition      = &cond_battery_time_low;
		def.priority       = 40;

		if (apply_fltt_low_action(def, _env.com_fltt_low_act)) {
			_add_rule(def);
		}
	}

	// -----------------------------------------------------------------------
	// Priority 30 — position accuracy, motor failure, GNSS loss
	// -----------------------------------------------------------------------

	{
		AspectDefinition def{};
		snprintf(def.name, MAX_NAME_LEN, "pos_accuracy_low");
		def.match_any_mode    = true;
		def.condition         = &cond_pos_accuracy_low;
		def.priority          = 30;
		def.allow_user_takeover = true;

		if (apply_wind_pos_action(def, _env.com_pos_low_act)) {
			// pos_accuracy_low uses WhenConditionClears (failsafe.cpp:353-370)
			def.clear_condition = ClearOn::WhenConditionClears;
			_add_rule(def);
		}
	}

	{
		AspectDefinition def{};
		snprintf(def.name, MAX_NAME_LEN, "motor_failure");
		def.match_any_mode = true;
		def.condition      = &cond_fd_motor_failure;
		def.priority       = 30;

		if (apply_actuator_fail_action(def, _env.com_act_fail_act)) {
			_add_rule(def);
		}
	}

	{
		AspectDefinition def{};
		snprintf(def.name, MAX_NAME_LEN, "gnss_lost");
		def.match_any_mode = true;
		def.condition      = &cond_gnss_lost;
		def.priority       = 30;

		if (apply_gnss_loss_action(def, _env.com_gnssloss_act)) {
			_add_rule(def);
		}
	}
}
