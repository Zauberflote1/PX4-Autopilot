/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
 * @file ev2_control.cpp
 * Control functions for ekf external vision control
 */

#include "ekf.h"

void Ekf::controlExternalVision2Fusion()
{
	_ev2_pos_b_est.predict(_dt_ekf_avg);

	// Check for new external vision data
	extVision2Sample ev2_sample;

	if (_ext_vision2_buffer && _ext_vision2_buffer->pop_first_older_than(_time_delayed_us, &ev2_sample)) {

		bool ev2_reset = (ev2_sample.reset_counter != _ev2_sample_prev.reset_counter);

		// determine if we should use the horizontal position observations
		bool quality_sufficient = (_params.ev2_quality_minimum <= 0) || (ev2_sample.quality >= _params.ev2_quality_minimum);

		const bool starting_conditions_passing = quality_sufficient
				&& ((ev2_sample.time_us - _ev2_sample_prev.time_us) < EV2_MAX_INTERVAL)
				&& ((_params.ev2_quality_minimum <= 0) || (_ev2_sample_prev.quality >= _params.ev2_quality_minimum)) // previous quality sufficient
				&& ((_params.ev2_quality_minimum <= 0) || (_ext_vision2_buffer->get_newest().quality >= _params.ev2_quality_minimum)) // newest quality sufficient
				&& isNewestSampleRecent(_time_last_ext_vision2_buffer_push, EV2_MAX_INTERVAL);

		updateEv2AttitudeErrorFilter(ev2_sample, ev2_reset);
		controlEv2YawFusion(ev2_sample, starting_conditions_passing, ev2_reset, quality_sufficient, _aid_src_ev2_yaw);
		controlEv2VelFusion(ev2_sample, starting_conditions_passing, ev2_reset, quality_sufficient, _aid_src_ev2_vel);
		controlEv2PosFusion(ev2_sample, starting_conditions_passing, ev2_reset, quality_sufficient, _aid_src_ev2_pos);
		controlEv2HeightFusion(ev2_sample, starting_conditions_passing, ev2_reset, quality_sufficient, _aid_src_ev2_hgt);

		if (quality_sufficient) {
			_ev2_sample_prev = ev2_sample;
		}

		// record corresponding yaw state for future EV delta heading innovation (logging only)
		_ev2_yaw_pred_prev = getEulerYaw(_state.quat_nominal);

	} else if ((_control_status.flags.ev2_pos || _control_status.flags.ev2_vel || _control_status.flags.ev2_yaw
		    || _control_status.flags.ev2_hgt)
		   && isTimedOut(_ev2_sample_prev.time_us, 2 * EV2_MAX_INTERVAL)) {

		// Turn off EV fusion mode if no data has been received
		stopEv2PosFusion();
		stopEv2VelFusion();
		stopEv2YawFusion();
		stopEv2HgtFusion();

		_ev2_q_error_initialized = false;

		_warning_events.flags.vision_data_stopped = true;
		ECL_WARN("vision data stopped");
	}
}

void Ekf::updateEv2AttitudeErrorFilter(extVision2Sample &ev2_sample, bool ev2_reset)
{
	const Quatf q_error((_state.quat_nominal * ev2_sample.quat.inversed()).normalized());

	if (!q_error.isAllFinite()) {
		return;
	}

	if (!_ev2_q_error_initialized || ev2_reset) {
		_ev2_q_error_filt.reset(q_error);
		_ev2_q_error_initialized = true;

	} else {
		_ev2_q_error_filt.update(q_error);
	}
}
