/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @file ActuatorEffectivenessTailsitterVTOL.cpp
 *
 * Actuator effectiveness for tailsitter VTOL
 */

#include "ActuatorEffectivenessTailsitterVTOL.hpp"

#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/log.h>
#include <uORB/topics/actuator_servos.h>

using namespace matrix;
using namespace time_literals;

ActuatorEffectivenessTailsitterVTOL::ActuatorEffectivenessTailsitterVTOL(ModuleParams *parent)
	: ModuleParams(parent), _mc_rotors(this), _control_surfaces(this)
{
	setFlightPhase(FlightPhase::HOVER_FLIGHT);
}
bool
ActuatorEffectivenessTailsitterVTOL::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}

	// MC motors
	configuration.selected_matrix = 0;
	// enable MC yaw control if more than 3 rotors
	_mc_rotors.enableYawByDifferentialThrust(_mc_rotors.geometry().num_rotors > 3);
	const bool mc_rotors_added_successfully = _mc_rotors.addActuators(configuration);

	// Control Surfaces
	configuration.selected_matrix = 1;
	_first_control_surface_idx = configuration.num_actuators_matrix[configuration.selected_matrix];
	const bool surfaces_added_successfully = _control_surfaces.addActuators(configuration);

	// Register the tilt outputs after the control surfaces, preserving their servo numbers.
	// Their zero effectiveness keeps them out of the native motor allocation.
	_first_tilt_idx = -1;

	if (_param_tilt_count.get() == 2) {
		if (configuration.num_actuators[(int)ActuatorType::SERVOS] + 2 > actuator_servos_s::NUM_CONTROLS
		    || configuration.totalNumActuators() + 2 > NUM_ACTUATORS) {
			PX4_ERR("tailsitter tilt: too many actuators");

		} else {
			configuration.selected_matrix = 0;
			_first_tilt_idx = configuration.num_actuators_matrix[0];
			configuration.addActuator(ActuatorType::SERVOS, Vector3f{}, Vector3f{});
			configuration.addActuator(ActuatorType::SERVOS, Vector3f{}, Vector3f{});
		}

	} else if (_param_tilt_count.get() != 0) {
		PX4_ERR("tailsitter tilt count must be 0 or 2");
	}

	return (mc_rotors_added_successfully && surfaces_added_successfully);
}

void ActuatorEffectivenessTailsitterVTOL::allocateAuxilaryControls(const float dt, int matrix_index,
		ActuatorVector &actuator_sp)
{
	if (matrix_index == 1) {
		// apply flaps
		normalized_unsigned_setpoint_s flaps_setpoint;

		if (_flaps_setpoint_sub.copy(&flaps_setpoint)) {
			_control_surfaces.applyFlaps(flaps_setpoint.normalized_setpoint, _first_control_surface_idx, dt, actuator_sp);
		}

		// apply spoilers
		normalized_unsigned_setpoint_s spoilers_setpoint;

		if (_spoilers_setpoint_sub.copy(&spoilers_setpoint)) {
			_control_surfaces.applySpoilers(spoilers_setpoint.normalized_setpoint, _first_control_surface_idx, dt, actuator_sp);
		}
	}
}

void ActuatorEffectivenessTailsitterVTOL::updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp,
		int matrix_index, ActuatorVector &actuator_sp, const ActuatorVector &actuator_min, const ActuatorVector &actuator_max)
{
	if (matrix_index == 0) {
		stopMaskedMotorsWithZeroThrust(_forwards_motors_mask, actuator_sp);

		if (_first_tilt_idx < 0) {
			return;
		}

		manual_control_setpoint_s manual{};
		float aux = NAN;

		if (_manual_control_setpoint_sub.copy(&manual) && manual.valid && manual.timestamp != 0
		    && hrt_elapsed_time(&manual.timestamp) < 1_s) {
			switch (_param_ts_aux.get()) {
			case 1: aux = manual.aux1; break;
			case 2: aux = manual.aux2; break;
			case 3: aux = manual.aux3; break;
			case 4: aux = manual.aux4; break;
			case 5: aux = manual.aux5; break;
			case 6: aux = manual.aux6; break;
			default: break;
			}
		}

		float left = 0.f;
		float right = 0.f;

		if (_flight_phase != FlightPhase::FORWARD_FLIGHT && PX4_ISFINITE(aux) && aux > 0.5f
		    && PX4_ISFINITE(control_sp(ControlAxis::PITCH)) && PX4_ISFINITE(control_sp(ControlAxis::YAW))
		    && PX4_ISFINITE(_param_ts_pitch_gain.get()) && PX4_ISFINITE(_param_ts_yaw_gain.get())) {
			const float pitch = control_sp(ControlAxis::PITCH) * _param_ts_pitch_gain.get();
			const float yaw = control_sp(ControlAxis::YAW) * _param_ts_yaw_gain.get();
			left = pitch - yaw;
			right = pitch + yaw;
		}

		actuator_sp(_first_tilt_idx) = math::constrain(left, actuator_min(_first_tilt_idx), actuator_max(_first_tilt_idx));
		actuator_sp(_first_tilt_idx + 1) = math::constrain(right, actuator_min(_first_tilt_idx + 1),
								 actuator_max(_first_tilt_idx + 1));
	}
}

void ActuatorEffectivenessTailsitterVTOL::setFlightPhase(const FlightPhase &flight_phase)
{
	if (_flight_phase == flight_phase) {
		return;
	}

	ActuatorEffectiveness::setFlightPhase(flight_phase);

	// update stopped motors
	switch (flight_phase) {
	case FlightPhase::FORWARD_FLIGHT:
		_forwards_motors_mask = _mc_rotors.getUpwardsMotors(); // allocation frame they stay upwards
		break;

	case FlightPhase::HOVER_FLIGHT:
	case FlightPhase::TRANSITION_FF_TO_HF:
	case FlightPhase::TRANSITION_HF_TO_FF:
		_forwards_motors_mask = 0;
		_stopped_motors_mask = 0;
		break;
	}
}
