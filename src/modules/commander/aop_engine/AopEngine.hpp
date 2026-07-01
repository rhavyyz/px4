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

#include "AopMission.hpp"
#include "AopTelemetry.hpp"
#include "AspectDefinition.hpp"

#include <drivers/drv_hrt.h>
#include <px4_platform_common/module_params.h>

/**
 * AopEngine — standalone Aspect-Oriented Programming safety engine.
 *
 * Replaces the legacy Failsafe class in Commander. All safety behavior —
 * including conditions that the old failsafe hardcoded in checkStateAndMode()
 * — is expressed as AspectDefinition rules loaded by load_rules() at startup.
 *
 * The engine is completely independent: it does not inherit from Failsafe or
 * FailsafeBase. The original failsafe source (failsafe/) is kept as reference.
 *
 * Lifecycle (called from Commander):
 *   1. Construction with ModuleParams *parent — registers in the param tree
 *   2. load_rules()   — once at startup; also called automatically by updateParams()
 *   3. update()       — every Commander loop (~10 Hz), returns AopResult
 *   4. onDisarm()     — called when vehicle disarms
 *   5. deferFailsafes() — called when config_overrides requests deferral
 */
class AopEngine : public ModuleParams
{
public:
	AopEngine(ModuleParams *parent) : ModuleParams(parent) {}

	/**
	 * Populate _rules[] from current param values. Defined in DefaultRules.cpp.
	 * Called automatically by updateParams() when PX4 parameters change.
	 * Safe to call multiple times — resets the registry before rebuilding.
	 */
	void load_rules();

	/** Called by ModuleParams cascade when parameters change; rebuilds rules. */
	void updateParams() override;

	/**
	 * Main per-cycle update. Replaces _failsafe.update() + modeFromAction()
	 * in Commander::handleModeIntentionAndFailsafe().
	 *
	 * @param user_intended_nav_state  what the user (or last failsafe cycle) set
	 * @param prev_nav_state           nav_state from the previous cycle
	 * @param telemetry                full vehicle telemetry snapshot for this cycle
	 * @param user_takeover_request    true if pilot moved sticks requesting takeover
	 * @param armed                    true if vehicle is armed
	 * @param dt_us                    elapsed time since last call (microseconds)
	 * @return AopResult with effective nav_state and meta flags
	 */
	px4_aop::AopResult update(uint8_t user_intended_nav_state,
				  uint8_t prev_nav_state,
				  const AopTelemetry &telemetry,
				  const AopMission *mission,
				  bool user_takeover_request,
				  bool armed,
				  hrt_abstime dt_us);

	/** Called when the vehicle disarms — clears latched rules and delay timers. */
	void onDisarm();

	/**
	 * Enable or disable deferral of deferrable rules.
	 * When enabled, rules with can_be_deferred=true do not fire.
	 * @param enabled     true to defer
	 * @param timeout_s   max deferral seconds (0 = default 30s, -1 = no timeout)
	 */
	void deferFailsafes(bool enabled, int timeout_s);

	bool failsafeDeferred() const { return _failsafe_deferred; }
	bool getDeferFailsafes() const { return _defer_failsafes; }
	bool inFailsafe() const { return _in_failsafe; }
	bool userTakeoverActive() const { return _user_takeover_active; }

	void printStatus() const;

protected:
	/** Called by load_rules(). Returns false if the rule registry is full. */
	bool _add_rule(const px4_aop::AspectDefinition &def);

	/** Zero the rule registry; called at the top of load_rules() so it is idempotent. */
	void _reset_rules();

private:
	long long int cont = 0;
	/** Evaluate the rule's condition function against the current telemetry frame. */
	bool _eval_conditions(const px4_aop::AspectDefinition &def,
			      const AopTelemetry &telemetry,
			      const AopMission *mission) const;

	/**
	 * Check whether a given mode's sensor requirements are met.
	 * Logic copied verbatim from FailsafeBase::modeCanRun()
	 * (failsafe/framework.cpp:703) — no inheritance needed.
	 */
	bool _mode_can_run(uint8_t nav_state, const failsafe_flags_s &flags) const;

	/** Per-rule runtime state, index-parallel to _rules[]. */
	struct RuleState {
		bool        latched{false};           ///< rule fired once and clear_condition keeps it active
		bool        in_delay_phase{false};    ///< delay timer is running
		hrt_abstime delay_remaining_us{0};    ///< remaining delay in microseconds
	};

	px4_aop::AspectDefinition _rules[px4_aop::MAX_RULES] {};
	RuleState                 _rule_state[px4_aop::MAX_RULES] {};
	int                       _count{0};

	bool _in_failsafe{false};
	bool _user_takeover_active{false};
	bool _failsafe_deferred{false};
	bool _defer_failsafes{false};
	int  _prev_winner_idx{-1};   ///< rule index that won last cycle; -1 = no winner

	static constexpr hrt_abstime DEFAULT_DEFER_TIMEOUT_US{30000000ULL}; // 30 seconds in microseconds
	hrt_abstime _defer_timeout_us{DEFAULT_DEFER_TIMEOUT_US};
	hrt_abstime _defer_started_us{0};

	// Matches Failsafe::_manual_control_lost_at_arming: suppresses rc_loss rules
	// when RC was already disconnected at the moment of arming (e.g. SITL, MAVLink-only ops).
	bool _was_armed{false};
	bool _manual_control_lost_at_arming{false};

	// Matches Failsafe::_battery_warning_at_arming (failsafe.cpp:709): battery rules only
	// escalate when the warning is worse than it was at arming, so arming with an already
	// low/critical battery does not immediately trigger RTL/Land.
	uint8_t _battery_warning_at_arming{0};

	// Matches Failsafe::_armed_time: time elapsed since the vehicle armed, accumulated from
	// the per-cycle dt. Used to gate rules with min/max_time_since_arm_s (spool-up windows).
	hrt_abstime _time_since_arm_us{0};

	// Param-value snapshot, refreshed in updateParams(). Passed to condition functions.
	AopEnv _env{};

	DEFINE_PARAMETERS(
		// RC / GCS loss
		(ParamInt<px4::params::NAV_RCL_ACT>)            _param_nav_rcl_act,
		(ParamInt<px4::params::NAV_DLL_ACT>)            _param_nav_dll_act,
		(ParamInt<px4::params::COM_RC_IN_MODE>)         _param_com_rc_in_mode,
		// Geofence / wind / position
		(ParamInt<px4::params::GF_ACTION>)              _param_gf_action,
		(ParamInt<px4::params::COM_WIND_MAX_ACT>)       _param_com_wind_max_act,
		(ParamInt<px4::params::COM_POS_LOW_ACT>)        _param_com_pos_low_act,
		// Battery
		(ParamInt<px4::params::COM_LOW_BAT_ACT>)        _param_com_low_bat_act,
		(ParamInt<px4::params::COM_FLTT_LOW_ACT>)       _param_com_fltt_low_act,
		// Failure detection
		(ParamInt<px4::params::COM_ACT_FAIL_ACT>)       _param_com_act_fail_act,
		(ParamInt<px4::params::COM_GNSSLOSS_ACT>)       _param_com_gnssloss_act,
		(ParamInt<px4::params::CBRK_FLIGHTTERM>)        _param_cbrk_flightterm,
		// VTOL / offboard
		(ParamInt<px4::params::COM_QC_ACT>)             _param_com_qc_act,
		(ParamInt<px4::params::COM_OBL_RC_ACT>)         _param_com_obl_rc_act,
		// Spool-up window
		(ParamFloat<px4::params::COM_SPOOLUP_TIME>)     _param_com_spoolup_time,
		// Link-loss exception bitmasks
		(ParamInt<px4::params::COM_RCL_EXCEPT>)         _param_com_rcl_except,
		(ParamInt<px4::params::COM_DLL_EXCEPT>)         _param_com_dll_except
	)
};
