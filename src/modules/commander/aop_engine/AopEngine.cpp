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

#include "AopEngine.hpp"

#include <px4_platform_common/log.h>
#include <uORB/topics/vehicle_status.h>

using namespace px4_aop;
using namespace time_literals;

px4_aop::AopResult AopEngine::update(uint8_t user_intended_nav_state,
				     uint8_t prev_nav_state,
				     const AopTelemetry &telemetry,
				     const AopMission *mission,
				     bool user_takeover_request,
				     bool armed,
				     hrt_abstime dt_us)
{
	cont++;
	if (cont % 50  == 0) {

		PX4_INFO("safe_land: valid=%d lat=%.6f lon=%.6f alt=%.2f",
			(int)telemetry.safe_land.valid,
			(double)telemetry.safe_land.lat,
			(double)telemetry.safe_land.lon,
			(double)telemetry.safe_land.alt);
		
	}


	AopResult result{};
	result.nav_state = user_intended_nav_state;

	const failsafe_flags_s &flags = telemetry.failsafe_flags;

	// --- Arm-transition tracking (matches Failsafe::_manual_control_lost_at_arming) ---
	// If RC was already lost when the vehicle armed, suppress rc_loss rules until RC is
	// regained at least once. This prevents false failsafes in SITL / MAVLink-only ops.
	if (armed && !_was_armed) {
		_manual_control_lost_at_arming = flags.manual_control_signal_lost;
		_battery_warning_at_arming     = flags.battery_warning; // matches Failsafe::updateArmingState (failsafe.cpp:709)
		_time_since_arm_us             = 0;
	}

	if (!flags.manual_control_signal_lost) {
		_manual_control_lost_at_arming = false; // RC regained — lift suppression
	}

	_was_armed = armed;

	if (!armed) {
		_in_failsafe = false;
		_user_takeover_active = false;
		_failsafe_deferred = false;
		return result;
	}

	// Accumulate time since arming for time-windowed rules (spool-up / early-takeoff phases)
	_time_since_arm_us += dt_us;

	// Synthesize effective telemetry: mask manual_control_signal_lost when it was already
	// lost at arming time (pilot never had RC — not a real in-flight loss).
	AopTelemetry effective_telemetry = telemetry;
	failsafe_flags_s &effective_flags = effective_telemetry.failsafe_flags;

	if (_manual_control_lost_at_arming) {
		effective_flags.manual_control_signal_lost = false;
	}

	// Mask battery_warning when it is no worse than it was at arming, so arming with an
	// already low/critical battery does not immediately escalate to RTL/Land. Mirrors
	// legacy warning_worse_than_at_arming gate (failsafe.cpp:640-642).
	if (effective_flags.battery_warning <= _battery_warning_at_arming) {
		effective_flags.battery_warning = 0;
	}

	// Update deferral timeout
	if (_defer_failsafes && _defer_started_us > 0 && _defer_timeout_us > 0) {
		if (dt_us >= _defer_timeout_us) {
			_defer_failsafes = false;
			_defer_timeout_us = DEFAULT_DEFER_TIMEOUT_US;
			_defer_started_us = 0;

		} else {
			_defer_timeout_us -= dt_us;
		}
	}

	// On nav_state transition: clear OnModeChange rules
	const bool mode_changed = (user_intended_nav_state != prev_nav_state);

	if (mode_changed) {
		_user_takeover_active = false;

		for (int i = 0; i < _count; ++i) {
			if (_rules[i].clear_condition == ClearOn::OnModeChange) {
				_rule_state[i].latched       = false;
				_rule_state[i].in_delay_phase = false;
				_rule_state[i].delay_remaining_us = 0;
			}
		}
	}

	// Evaluate all rules, find the highest-priority match
	int     winner_idx      = -1;
	uint8_t winner_priority = 0;
	bool    any_deferred    = false;

	for (int i = 0; i < _count; ++i) {
		const AspectDefinition &rule  = _rules[i];
		RuleState              &state = _rule_state[i];

		// --- Pointcut check ---
		if (!rule.match_any_mode && rule.trigger_nav_state != user_intended_nav_state) {
			// Rule not applicable to current mode — clear if condition-based
			if (!state.latched) {
				state.in_delay_phase      = false;
				state.delay_remaining_us  = 0;
			}

			continue;
		}

		if (rule.on_entry_only && !mode_changed) {
			continue;
		}

		// --- Time-since-arm window ---
		// Rules constrained to a spool-up / early-takeoff window are inactive outside it.
		if (rule.min_time_since_arm_s > 0.f || rule.max_time_since_arm_s > 0.f) {
			const float t_arm_s = static_cast<float>(_time_since_arm_us) / 1e6f;

			if ((rule.max_time_since_arm_s > 0.f && t_arm_s >= rule.max_time_since_arm_s) ||
			    (rule.min_time_since_arm_s > 0.f && t_arm_s <  rule.min_time_since_arm_s)) {
				if (!state.latched) {
					state.in_delay_phase     = false;
					state.delay_remaining_us = 0;
				}

				continue;
			}
		}

		// --- Condition evaluation ---
		const bool conditions_pass = _eval_conditions(rule, effective_telemetry, mission);

		if (conditions_pass) {
			// Activate and set latching state for subsequent cycles
			if (rule.clear_condition == ClearOn::OnModeChange ||
			    rule.clear_condition == ClearOn::OnDisarm ||
			    rule.clear_condition == ClearOn::Never) {
				state.latched = true;
			}

		} else if (state.latched) {
			// Condition cleared but rule is latched — keep it active

		} else {
			// Condition cleared and not latched — deactivate
			state.in_delay_phase     = false;
			state.delay_remaining_us = 0;
			continue;
		}

		// --- Deferral ---
		if (_defer_failsafes && rule.can_be_deferred) {
			any_deferred = true;
			continue;
		}

		// --- Priority selection ---
		if (winner_idx < 0 || rule.priority > winner_priority) {
			winner_idx      = i;
			winner_priority = rule.priority;
		}
	}

	_failsafe_deferred = any_deferred;

	if (winner_idx < 0) {
		// No rule fired
		_in_failsafe          = false;
		_user_takeover_active = false;
		_prev_winner_idx      = -1;
		return result;
	}

	const AspectDefinition &winner       = _rules[winner_idx];
	RuleState              &winner_state = _rule_state[winner_idx];

	// --- Notification fields ---
	result.newly_fired = (winner_idx != _prev_winner_idx);
	_prev_winner_idx   = winner_idx;

	if (result.newly_fired) {
		PX4_WARN("AOP rule fired: '%s'", winner.name);
	}

	__builtin_strncpy(result.fired_name, winner.name, MAX_NAME_LEN - 1);
	result.fired_name[MAX_NAME_LEN - 1] = '\0';
	result.in_delay_phase    = winner_state.in_delay_phase;
	result.delay_remaining_s = static_cast<uint16_t>(winner_state.delay_remaining_us / 1_s);

	// --- Terminal actions ---
	if (winner.transition.nav_state == OVERRIDE_DISARM) {
		result.request_disarm = true;
		result.in_failsafe    = true;
		_in_failsafe          = true;
		return result;
	}

	if (winner.transition.nav_state == OVERRIDE_TERMINATE) {
		result.request_terminate = true;
		result.in_failsafe       = true;
		_in_failsafe             = true;
		return result;
	}

	// --- Feasibility: primary → fallback → DESCEND → terminate ---
	uint8_t override_nav = winner.transition.nav_state;

	if (!_mode_can_run(override_nav, effective_flags)) {
		const uint8_t fallback = winner.transition.fallback_nav_state;
		override_nav = (fallback != 0 && _mode_can_run(fallback, effective_flags))
			       ? fallback
			       : vehicle_status_s::NAVIGATION_STATE_DESCEND;

		if (!_mode_can_run(override_nav, effective_flags)) {
			result.request_terminate = true;
			result.in_failsafe       = true;
			_in_failsafe             = true;
			return result;
		}
	}

	// --- Commit: fire on_enter once when the rule first wins ---
	if (result.newly_fired && winner.transition.on_enter) {
		winner.transition.on_enter(effective_telemetry, _env, mission);
	}

	// --- Delay phase ---
	if (winner.delay_s > 0.f) {
		if (!winner_state.in_delay_phase) {
			winner_state.in_delay_phase      = true;
			winner_state.delay_remaining_us  = static_cast<hrt_abstime>(winner.delay_s * 1e6f);
		}

		if (winner_state.delay_remaining_us > dt_us) {
			winner_state.delay_remaining_us -= dt_us;

			// During delay: hold in AUTO_LOITER if available
			const uint8_t hold = vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;
			result.nav_state   = _mode_can_run(hold, flags) ? hold : override_nav;
			result.in_failsafe = true;
			_in_failsafe       = true;
			return result;

		} else {
			winner_state.delay_remaining_us = 0;
			winner_state.in_delay_phase     = false;
		}
	}

	// --- User takeover ---
	// takeover_requires_mode_switch rules (legacy AlwaysModeSwitchOnly, e.g. geofence Hold)
	// only let the pilot escape via a mode switch — handled by the OnModeChange un-latch path —
	// so a stick request must NOT cancel them here.
	if (winner.allow_user_takeover && !winner.takeover_requires_mode_switch
	    && (user_takeover_request || _user_takeover_active)) {
		const uint8_t posctl = vehicle_status_s::NAVIGATION_STATE_POSCTL;
		result.nav_state       = _mode_can_run(posctl, flags) ? posctl : user_intended_nav_state;
		result.user_takeover   = true;
		result.in_failsafe     = false;
		_user_takeover_active  = true;
		_in_failsafe           = false;
		return result;
	}

	_user_takeover_active = false;
	result.nav_state   = override_nav;
	result.in_failsafe = true;
	_in_failsafe       = true;
	return result;
}

void AopEngine::onDisarm()
{
	for (int i = 0; i < _count; ++i) {
		const ClearOn c = _rules[i].clear_condition;

		if (c == ClearOn::OnDisarm || c == ClearOn::OnModeChange || c == ClearOn::WhenConditionClears) {
			_rule_state[i].latched           = false;
			_rule_state[i].in_delay_phase    = false;
			_rule_state[i].delay_remaining_us = 0;
		}

		// ClearOn::Never survives disarm intentionally
	}

	_in_failsafe                   = false;
	_user_takeover_active          = false;
	_failsafe_deferred             = false;
	_defer_failsafes               = false;
	_defer_started_us              = 0;
	_defer_timeout_us              = DEFAULT_DEFER_TIMEOUT_US;
	_prev_winner_idx               = -1;
	_was_armed                     = false;
	_manual_control_lost_at_arming = false;
	_battery_warning_at_arming     = 0;
	_time_since_arm_us             = 0;
}

void AopEngine::deferFailsafes(bool enabled, int timeout_s)
{
	if (enabled && !_defer_failsafes) {
		_defer_failsafes  = true;
		_defer_started_us = hrt_absolute_time();
		_defer_timeout_us = (timeout_s == 0)  ? DEFAULT_DEFER_TIMEOUT_US :
				    (timeout_s < 0)   ? 0 :
				    static_cast<hrt_abstime>(timeout_s) * 1_s;

	} else if (!enabled) {
		_defer_failsafes  = false;
		_defer_started_us = 0;
		_defer_timeout_us = DEFAULT_DEFER_TIMEOUT_US;
	}
}

void AopEngine::updateParams()
{
	ModuleParams::updateParams();

	_env.nav_rcl_act      = _param_nav_rcl_act.get();
	_env.nav_dll_act      = _param_nav_dll_act.get();
	_env.com_rc_in_mode   = _param_com_rc_in_mode.get();
	_env.gf_action        = _param_gf_action.get();
	_env.com_wind_max_act = _param_com_wind_max_act.get();
	_env.com_pos_low_act  = _param_com_pos_low_act.get();
	_env.com_low_bat_act  = _param_com_low_bat_act.get();
	_env.com_fltt_low_act = _param_com_fltt_low_act.get();
	_env.com_act_fail_act = _param_com_act_fail_act.get();
	_env.com_gnssloss_act = _param_com_gnssloss_act.get();
	_env.cbrk_flightterm  = _param_cbrk_flightterm.get();
	_env.com_qc_act       = _param_com_qc_act.get();
	_env.com_obl_rc_act   = _param_com_obl_rc_act.get();
	_env.com_spoolup_time = _param_com_spoolup_time.get();
	_env.com_rcl_except   = _param_com_rcl_except.get();
	_env.com_dll_except   = _param_com_dll_except.get();

	load_rules();
}

void AopEngine::_reset_rules()
{
	for (int i = 0; i < px4_aop::MAX_RULES; ++i) {
		_rules[i]      = px4_aop::AspectDefinition{};
		_rule_state[i] = RuleState{};
	}

	_count           = 0;
	_prev_winner_idx = -1;
}

bool AopEngine::_add_rule(const AspectDefinition &def)
{
	if (_count >= MAX_RULES) {
		PX4_ERR("AopEngine: rule registry full (max %d), cannot add '%s'", MAX_RULES, def.name);
		return false;
	}

	_rules[_count] = def;
	_count++;
	return true;
}

bool AopEngine::_eval_conditions(const AspectDefinition &def, const AopTelemetry &telemetry,
				 const AopMission *mission) const
{
	if (def.condition == nullptr) { return false; }

	return def.condition(telemetry, _env, mission);
}

bool AopEngine::_mode_can_run(uint8_t nav_state, const failsafe_flags_s &flags) const
{
	// Copied verbatim from FailsafeBase::modeCanRun() — failsafe/framework.cpp:703
	// mode_req_wind_and_flight_time_compliance: handled as separate rules
	// mode_req_manual_control: handled separately
	const uint32_t mode_mask = 1u << nav_state;
	return
		(!flags.angular_velocity_invalid      || ((flags.mode_req_angular_velocity & mode_mask) == 0)) &&
		(!flags.attitude_invalid              || ((flags.mode_req_attitude & mode_mask) == 0)) &&
		(!flags.local_position_invalid        || ((flags.mode_req_local_position & mode_mask) == 0)) &&
		(!flags.local_position_invalid_relaxed || ((flags.mode_req_local_position_relaxed & mode_mask) == 0)) &&
		(!flags.global_position_invalid       || ((flags.mode_req_global_position & mode_mask) == 0)) &&
		(!flags.global_position_invalid_relaxed || ((flags.mode_req_global_position_relaxed & mode_mask) == 0)) &&
		(!flags.local_altitude_invalid        || ((flags.mode_req_local_alt & mode_mask) == 0)) &&
		(!flags.auto_mission_missing          || ((flags.mode_req_mission & mode_mask) == 0)) &&
		(!flags.offboard_control_signal_lost  || ((flags.mode_req_offboard_signal & mode_mask) == 0)) &&
		(!flags.home_position_invalid         || ((flags.mode_req_home_position & mode_mask) == 0)) &&
		((flags.mode_req_other & mode_mask) == 0);
}

void AopEngine::printStatus() const
{
	PX4_INFO("AopEngine: %d rules loaded, in_failsafe=%s, takeover=%s, deferred=%s",
		 _count,
		 _in_failsafe ? "yes" : "no",
		 _user_takeover_active ? "yes" : "no",
		 _failsafe_deferred ? "yes" : "no");

	for (int i = 0; i < _count; ++i) {
		PX4_INFO("  [%d] '%s' pri=%u latched=%s delay_ms=%.0f",
			 i, _rules[i].name, _rules[i].priority,
			 _rule_state[i].latched ? "yes" : "no",
			 static_cast<double>(_rule_state[i].delay_remaining_us) / 1000.0);
	}
}
