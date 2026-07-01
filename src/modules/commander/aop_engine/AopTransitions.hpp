#pragma once

#include "AopTelemetry.hpp"
#include "AopMission.hpp"

#include <functional>
#include <stdint.h>

/**
 * Callback type invoked once when the engine commits to a transition
 * (on newly_fired). Receives the full snapshot so lambdas can read
 * telemetry without capturing runtime state.
 */
using OnEnterFn = std::function<void(const AopTelemetry &, const AopEnv &, const AopMission *)>;

/**
 * A geographic coordinate used by go_to_coord().
 */
struct GeoCoord {
	double lat{0.0};
	double lon{0.0};
	float  alt{0.f};
};

/**
 * Result of a generator call — stored in AspectDefinition::transition.
 *
 * The engine reads nav_state every cycle to maintain the override.
 * If nav_state is not feasible the engine tries fallback_nav_state
 * (0 = let engine fall to DESCEND).
 * on_enter is called once, on newly_fired.
 */
struct Transition {
	uint8_t   nav_state{0};
	uint8_t   fallback_nav_state{0};
	OnEnterFn on_enter{};
};

// ---------------------------------------------------------------------------
// Generator functions — any signature, always return Transition.
// Call these at rule-definition time inside DefaultRules.cpp.
// ---------------------------------------------------------------------------

/** Simple state switch, no side effects. Optional fallback state. */
Transition go_to_state(uint8_t nav_state, uint8_t fallback = 0);

/**
 * Switch to AUTO_LOITER and publish a DO_REPOSITION command using the
 * current safe_land_topic coordinates read from telemetry at call time.
 * Fallback: DESCEND.
 */
Transition go_to_safe_land();

/**
 * Switch to nav_state and publish a DO_REPOSITION with hardcoded
 * coordinates captured at definition time. Fallback: DESCEND.
 */
Transition go_to_coord(uint8_t nav_state, GeoCoord coord);

/** Request vehicle disarm (sentinel 0xFE). */
Transition go_to_disarm();

/** Request flight termination (sentinel 0xFF). */
Transition go_to_terminate();

/**
 * Switch to AUTO_LAND and publish a DO_REPOSITION command using the
 * current safe_land_topic coordinates read from telemetry at call time.
 * During delay_s the engine holds AUTO_LOITER, giving the drone time to
 * fly to the safe zone. Fallback: DESCEND.
 */
Transition go_to_safe_land_then_land();
