/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include "gpsRedundancyCheck.hpp"

#include <lib/geo/geo.h>

using namespace time_literals;

// eph is firmware-dependent and not a rigorous 1-sigma, so 3 is a heuristic consistency gate
// rather than a precise statistical claim. It relaxes the gate automatically when receivers degrade.
static constexpr float    GPS_DIVERGENCE_SIGMA   = 3.0f;
static constexpr uint64_t GPS_DIVERGENCE_SUSTAIN = 2_s;    // must be sustained before warning

void GpsRedundancyChecks::checkAndReport(const Context &context, Report &reporter)
{
	// Always reset — will be set below only when the condition is active
	reporter.failsafeFlags().gps_redundancy_lost = false;

	// Count GPS instances that are present, providing fresh data, and have a valid 3D fix.
	// A receiver with fix_type < 3 (no fix or 2D only) is treated the same as one gone silent.
	int active_count = 0;
	sensor_gps_s active_gps[GPS_MAX_INSTANCES] {};

	for (int i = 0; i < _sensor_gps_sub.size(); i++) {
		sensor_gps_s gps;

		if (_sensor_gps_sub[i].copy(&gps)
		    && (gps.device_id != 0)
		    && (hrt_elapsed_time(&gps.timestamp) < 2_s)
		    && (gps.fix_type >= 3)) {
			active_gps[active_count++] = gps;
		}
	}

	// Position divergence check: warn if two active receivers disagree beyond their combined uncertainty.
	if (active_count >= 2) {
		float north, east;
		get_vector_to_next_waypoint(active_gps[0].latitude_deg, active_gps[0].longitude_deg,
					    active_gps[1].latitude_deg, active_gps[1].longitude_deg,
					    &north, &east);

		const float divergence_m = sqrtf(north * north + east * east);
		const float rms_eph = sqrtf(active_gps[0].eph * active_gps[0].eph + active_gps[1].eph * active_gps[1].eph);
		const float dx = _param_gps0_offx.get() - _param_gps1_offx.get();
		const float dy = _param_gps0_offy.get() - _param_gps1_offy.get();
		const float expected_d = sqrtf(dx * dx + dy * dy);
		const float gate_m = expected_d + rms_eph * GPS_DIVERGENCE_SIGMA;

		// Pre-arm: warn immediately so the operator can decide before takeoff.
		// In-flight: require sustained divergence to avoid false alarms from transient multipath.
		const uint64_t sustain = context.isArmed() ? GPS_DIVERGENCE_SUSTAIN : 0_s;

		if (divergence_m > gate_m) {
			if (_divergence_since == 0) {
				_divergence_since = hrt_absolute_time();
			}

			if (hrt_elapsed_time(&_divergence_since) >= sustain) {
				/* EVENT
				 * @description
				 * Two GPS receivers report positions that are inconsistent with their reported accuracy.
				 */
				reporter.healthFailure<float, float>(NavModes::None, health_component_t::gps,
								     events::ID("check_gps_position_divergence"),
								     events::Log::Warning,
								     "GPS receivers disagree: {1:.1}m apart (gate {2:.1}m)",
								     (double)divergence_m, (double)gate_m);
			}

		} else {
			_divergence_since = 0;
		}

	}

	// Track the highest count seen — used to detect any GPS loss regardless of SYS_HAS_NUM_GPS
	if (active_count > _peak_active_count) {
		_peak_active_count = active_count;
	}

	const int required = _param_sys_has_num_gps.get();
	const bool below_required    = (required > 0 && active_count < required);
	const bool dropped_below_peak = (_peak_active_count > 1 && active_count < _peak_active_count);

	if (!below_required && !dropped_below_peak) {
		return;
	}

	reporter.failsafeFlags().gps_redundancy_lost = below_required;

	// act==0: warn only, never blocks arming; act>0: blocks arming and shows red
	const bool block_arming     = below_required && (_param_com_gps_loss_act.get() > 0);
	const NavModes nav_modes    = block_arming ? NavModes::All : NavModes::None;
	const events::Log log_level = block_arming ? events::Log::Error : events::Log::Warning;
	const uint8_t expected      = below_required ? (uint8_t)required : (uint8_t)_peak_active_count;

	/* EVENT
	 * @description
	 * <profile name="dev">
	 * Configure the minimum required GPS count with <param>SYS_HAS_NUM_GPS</param>.
	 * Configure the failsafe action with <param>COM_GPS_LOSS_ACT</param>.
	 * </profile>
	 */
	reporter.healthFailure<uint8_t, uint8_t>(nav_modes, health_component_t::gps,
			events::ID("check_gps_redundancy_lost"),
			log_level,
			"GPS receiver count dropped: {1} of {2} expected active",
			(uint8_t)active_count, expected);
}
