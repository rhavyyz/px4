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

#include "AopTransitions.hpp"

#include <stdint.h>

namespace px4_aop
{

static constexpr int MAX_RULES    = 32;
static constexpr int MAX_NAME_LEN = 24;

/**
 * Sentinel values embedded in Transition::nav_state to request terminal actions.
 * Safely above the valid NAVIGATION_STATE range (< 32).
 */
static constexpr uint8_t OVERRIDE_DISARM    = 0xFE;
static constexpr uint8_t OVERRIDE_TERMINATE = 0xFF;

/**
 * Condition predicate type. Each rule owns exactly one function with this
 * signature. Returns true when the rule's condition is met.
 * May be nullptr — the engine skips any rule whose condition is nullptr.
 */
using ConditionFn = bool (*)(const AopTelemetry &, const AopEnv &, const AopMission *);

/**
 * When the condition that triggered a rule clears, this controls whether
 * the rule keeps its active/override state.
 */
enum class ClearOn : uint8_t {
	WhenConditionClears, ///< deactivate immediately when condition clears
	OnModeChange,        ///< keep active until a nav_state transition happens
	OnDisarm,            ///< keep active until vehicle disarms
	Never,               ///< never deactivate (requires reboot)
};

/**
 * One declarative safety rule: pointcut + condition + advice + policy.
 *
 * The engine evaluates all rules each cycle and applies the highest-priority
 * match. Each rule has exactly one condition function (ConditionFn).
 *
 * The transition field is produced by a generator function at definition time
 * and carries the target nav_state, an optional fallback, and an on_enter
 * callback. OVERRIDE_DISARM / OVERRIDE_TERMINATE in nav_state request
 * terminal actions from Commander.
 */
struct AspectDefinition {
	// --- Pointcut ---
	bool    match_any_mode{false};      ///< true → fires regardless of current nav_state
	uint8_t trigger_nav_state{0};       ///< NAVIGATION_STATE_* (ignored if match_any_mode)
	bool    on_entry_only{false};       ///< true → fires only on the cycle nav_state changes to trigger

	// --- Condition ---
	ConditionFn condition{nullptr};

	// --- Transition ---
	Transition transition{};

	// --- Policy ---
	uint8_t  priority{0};                ///< higher wins among simultaneously firing rules
	float    delay_s{0.f};              ///< hold in AUTO_LOITER for this duration before applying
	bool     allow_user_takeover{false}; ///< sticks/mode-switch can cancel the override
	// When allow_user_takeover, escape only via a mode switch — stick input does NOT cancel.
	// Mirrors legacy UserTakeoverAllowed::AlwaysModeSwitchOnly.
	bool     takeover_requires_mode_switch{false};
	bool     can_be_deferred{true};    ///< rule can be suppressed by deferFailsafes()
	ClearOn  clear_condition{ClearOn::WhenConditionClears};

	// --- Time-since-arm window (0 = no constraint) ---
	// Mirrors the legacy spool-up / early-takeoff windows that key off _armed_time
	// and COM_SPOOLUP_TIME (failsafe.cpp:621,668,678). The rule only fires while
	// time-since-arm is within [min_time_since_arm_s, max_time_since_arm_s).
	float    min_time_since_arm_s{0.f}; ///< rule inactive until this many seconds after arming
	float    max_time_since_arm_s{0.f}; ///< rule inactive once this many seconds have elapsed since arming

	// --- Identity ---
	char name[MAX_NAME_LEN] {};
};

/**
 * Result returned by AopEngine::update() each cycle.
 */
struct AopResult {
	uint8_t nav_state{0};             ///< effective nav_state to apply
	bool    in_failsafe{false};       ///< a rule is active and overriding the user intent
	bool    user_takeover{false};     ///< user took over from an overriding rule
	bool    request_disarm{false};    ///< engine requests Commander to disarm
	bool    request_terminate{false}; ///< engine requests Commander to set NAVIGATION_STATE_TERMINATION
	bool    failsafe_deferred{false}; ///< at least one deferrable rule would fire but is suppressed

	// Notification support — mirrors FailsafeBase::notifyUser() trigger data
	bool     newly_fired{false};          ///< true only on the first cycle a rule becomes active (or switches)
	bool     in_delay_phase{false};       ///< true while hold-before-apply delay is running
	uint16_t delay_remaining_s{0};        ///< seconds remaining in delay phase
	char     fired_name[MAX_NAME_LEN] {}; ///< name of the winning rule
};

} // namespace px4_aop
