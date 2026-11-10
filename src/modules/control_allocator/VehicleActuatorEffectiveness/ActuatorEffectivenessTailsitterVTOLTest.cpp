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

#include <gtest/gtest.h>
#include <drivers/drv_hrt.h>
#include <uORB/Publication.hpp>

#include "ActuatorEffectivenessTailsitterVTOL.hpp"
#include <control_allocation/control_allocation/ControlAllocationSequentialDesaturation.hpp>

using Effectiveness = ActuatorEffectiveness;

class TestAllocation : public ControlAllocationSequentialDesaturation
{
public:
	ActuatorVector &actuatorSetpoint() { return _actuator_sp; }
};

static void setInt(const char *name, int32_t value)
{
	const param_t param = param_find(name);
	ASSERT_NE(param, PARAM_INVALID);
	ASSERT_EQ(param_set(param, &value), 0);
}

static void setFloat(const char *name, float value)
{
	const param_t param = param_find(name);
	ASSERT_NE(param, PARAM_INVALID);
	ASSERT_EQ(param_set(param, &value), 0);
}

TEST(ActuatorEffectivenessTailsitterVTOLTest, TiltAssistAndFlightPhases)
{
	param_control_autosave(false);
	setInt("CA_ROTOR_COUNT", 2);
	setInt("CA_SV_CS_COUNT", 2);
	setInt("CA_SV_TL_COUNT", 2);
	setInt("CA_TS_AUX", 1);
	setFloat("CA_TS_P_GAIN", 0.5f);
	setFloat("CA_TS_Y_GAIN", 0.25f);

	ActuatorEffectivenessTailsitterVTOL tailsitter(nullptr);
	Effectiveness::Configuration config{};
	ASSERT_TRUE(tailsitter.getEffectivenessMatrix(config, EffectivenessUpdateReason::CONFIGURATION_UPDATE));
	ASSERT_EQ(config.num_actuators_matrix[0], 4);
	ASSERT_EQ(config.num_actuators_matrix[1], 2);
	ASSERT_EQ(config.num_actuators[(int)ActuatorType::SERVOS], 4);

	uORB::Publication<manual_control_setpoint_s> manual_pub{ORB_ID(manual_control_setpoint)};
	manual_control_setpoint_s manual{};
	manual.valid = true;
	manual.aux1 = 1.f;
	manual.timestamp = hrt_absolute_time();
	ASSERT_TRUE(manual_pub.publish(manual));

	matrix::Vector<float, Effectiveness::NUM_AXES> control{};
	Effectiveness::ActuatorVector output{};
	Effectiveness::ActuatorVector minimum{};
	Effectiveness::ActuatorVector maximum{};
	minimum.setAll(-1.f);
	maximum.setAll(1.f);

	control(Effectiveness::PITCH) = 0.4f;
	tailsitter.updateSetpoint(control, 0, output, minimum, maximum);
	EXPECT_NEAR(output(2), 0.2f, 1e-6f);
	EXPECT_NEAR(output(3), 0.2f, 1e-6f);

	control(Effectiveness::PITCH) = 0.f;
	control(Effectiveness::YAW) = 0.4f;
	tailsitter.updateSetpoint(control, 0, output, minimum, maximum);
	EXPECT_NEAR(output(2), -0.1f, 1e-6f);
	EXPECT_NEAR(output(3), 0.1f, 1e-6f);

	control(Effectiveness::PITCH) = 4.f;
	control(Effectiveness::YAW) = 4.f;
	tailsitter.updateSetpoint(control, 0, output, minimum, maximum);
	EXPECT_FLOAT_EQ(output(2), 1.f);
	EXPECT_FLOAT_EQ(output(3), 1.f);

	control(Effectiveness::PITCH) = 0.4f;
	control(Effectiveness::YAW) = 0.4f;
	for (const auto phase : {Effectiveness::FlightPhase::HOVER_FLIGHT,
				 Effectiveness::FlightPhase::TRANSITION_HF_TO_FF,
				 Effectiveness::FlightPhase::TRANSITION_FF_TO_HF}) {
		tailsitter.setFlightPhase(phase);
		tailsitter.updateSetpoint(control, 0, output, minimum, maximum);
		EXPECT_NEAR(output(2), 0.1f, 1e-6f);
		EXPECT_NEAR(output(3), 0.3f, 1e-6f);
	}

	tailsitter.setFlightPhase(Effectiveness::FlightPhase::FORWARD_FLIGHT);
	tailsitter.updateSetpoint(control, 0, output, minimum, maximum);
	EXPECT_FLOAT_EQ(output(2), 0.f);
	EXPECT_FLOAT_EQ(output(3), 0.f);

	tailsitter.setFlightPhase(Effectiveness::FlightPhase::HOVER_FLIGHT);
	manual.aux1 = -1.f;
	manual.timestamp = hrt_absolute_time();
	ASSERT_TRUE(manual_pub.publish(manual));
	tailsitter.updateSetpoint(control, 0, output, minimum, maximum);
	EXPECT_FLOAT_EQ(output(2), 0.f);
	EXPECT_FLOAT_EQ(output(3), 0.f);

	manual.aux1 = 1.f;
	manual.valid = false;
	manual.timestamp = hrt_absolute_time();
	ASSERT_TRUE(manual_pub.publish(manual));
	tailsitter.updateSetpoint(control, 0, output, minimum, maximum);
	EXPECT_FLOAT_EQ(output(2), 0.f);
	EXPECT_FLOAT_EQ(output(3), 0.f);

	manual.valid = true;
	manual.timestamp = hrt_absolute_time();
	ASSERT_TRUE(manual_pub.publish(manual));
	tailsitter.updateSetpoint(control, 0, output, minimum, maximum);
	EXPECT_NEAR(output(2), 0.1f, 1e-6f);
	EXPECT_NEAR(output(3), 0.3f, 1e-6f);

	manual.aux1 = NAN;
	manual.timestamp = hrt_absolute_time();
	ASSERT_TRUE(manual_pub.publish(manual));
	tailsitter.updateSetpoint(control, 0, output, minimum, maximum);
	EXPECT_FLOAT_EQ(output(2), 0.f);
	EXPECT_FLOAT_EQ(output(3), 0.f);

	manual.aux1 = 1.f;
	manual.timestamp = 1;
	ASSERT_TRUE(manual_pub.publish(manual));
	tailsitter.updateSetpoint(control, 0, output, minimum, maximum);
	EXPECT_FLOAT_EQ(output(2), 0.f);
	EXPECT_FLOAT_EQ(output(3), 0.f);

	manual.timestamp = hrt_absolute_time();
	ASSERT_TRUE(manual_pub.publish(manual));
	control(Effectiveness::PITCH) = NAN;
	tailsitter.updateSetpoint(control, 0, output, minimum, maximum);
	EXPECT_FLOAT_EQ(output(2), 0.f);
	EXPECT_FLOAT_EQ(output(3), 0.f);
}

TEST(ActuatorEffectivenessTailsitterVTOLTest, DisabledPreservesNativeMatrices)
{
	param_control_autosave(false);
	setInt("CA_ROTOR_COUNT", 2);
	setInt("CA_SV_CS_COUNT", 2);
	setInt("CA_SV_TL_COUNT", 0);
	setFloat("CA_ROTOR0_PY", -0.2f);
	setFloat("CA_ROTOR1_PY", 0.2f);

	ActuatorEffectivenessTailsitterVTOL disabled(nullptr);
	Effectiveness::Configuration native{};
	ASSERT_TRUE(disabled.getEffectivenessMatrix(native, EffectivenessUpdateReason::CONFIGURATION_UPDATE));
	ASSERT_EQ(native.num_actuators_matrix[0], 2);
	ASSERT_EQ(native.num_actuators_matrix[1], 2);

	setInt("CA_SV_TL_COUNT", 2);
	ActuatorEffectivenessTailsitterVTOL enabled(nullptr);
	Effectiveness::Configuration extended{};
	ASSERT_TRUE(enabled.getEffectivenessMatrix(extended, EffectivenessUpdateReason::CONFIGURATION_UPDATE));

	for (int matrix = 0; matrix < 2; ++matrix) {
		for (int axis = 0; axis < Effectiveness::NUM_AXES; ++axis) {
			for (int actuator = 0; actuator < Effectiveness::NUM_ACTUATORS; ++actuator) {
				EXPECT_FLOAT_EQ(native.effectiveness_matrices[matrix](axis, actuator),
						extended.effectiveness_matrices[matrix](axis, actuator));
			}
		}
	}

	matrix::Vector<float, Effectiveness::NUM_AXES> control{};
	Effectiveness::ActuatorVector output{};
	Effectiveness::ActuatorVector minimum{};
	Effectiveness::ActuatorVector maximum{};
	output.setAll(0.25f);
	minimum.setAll(-1.f);
	maximum.setAll(1.f);
	disabled.updateSetpoint(control, 0, output, minimum, maximum);
	EXPECT_FLOAT_EQ(output(2), 0.25f);
	EXPECT_FLOAT_EQ(output(3), 0.25f);
}

TEST(ActuatorEffectivenessTailsitterVTOLTest, InvalidTiltCountDoesNotAddPartialPair)
{
	param_control_autosave(false);
	setInt("CA_ROTOR_COUNT", 2);
	setInt("CA_SV_CS_COUNT", 2);

	for (int32_t invalid_count : {1, 3, 4, 5}) {
		setInt("CA_SV_TL_COUNT", invalid_count);
		ActuatorEffectivenessTailsitterVTOL tailsitter(nullptr);
		Effectiveness::Configuration config{};
		ASSERT_TRUE(tailsitter.getEffectivenessMatrix(config, EffectivenessUpdateReason::CONFIGURATION_UPDATE));
		EXPECT_EQ(config.num_actuators_matrix[0], 2);
		EXPECT_EQ(config.num_actuators_matrix[1], 2);
		EXPECT_EQ(config.num_actuators[(int)ActuatorType::SERVOS], 2);
	}
}

TEST(ActuatorEffectivenessTailsitterVTOLTest, ConfiguredServoSlewLimitsSwitchOffAndReturn)
{
	param_control_autosave(false);
	setInt("CA_ROTOR_COUNT", 2);
	setInt("CA_SV_CS_COUNT", 2);
	setInt("CA_SV_TL_COUNT", 2);
	setInt("CA_TS_AUX", 1);
	setFloat("CA_TS_P_GAIN", 1.f);
	setFloat("CA_TS_Y_GAIN", 0.f);

	ActuatorEffectivenessTailsitterVTOL tailsitter(nullptr);
	Effectiveness::Configuration config{};
	ASSERT_TRUE(tailsitter.getEffectivenessMatrix(config, EffectivenessUpdateReason::CONFIGURATION_UPDATE));

	TestAllocation allocation;
	Effectiveness::ActuatorVector minimum{};
	Effectiveness::ActuatorVector maximum{};
	Effectiveness::ActuatorVector slew{};
	minimum.setAll(-1.f);
	maximum.setAll(1.f);
	slew(2) = 2.f;
	slew(3) = 2.f;
	allocation.setActuatorMin(minimum);
	allocation.setActuatorMax(maximum);
	allocation.setSlewRateLimit(slew);
	allocation.setEffectivenessMatrix(config.effectiveness_matrices[0], config.trim[0], config.linearization_point[0],
					  config.num_actuators_matrix[0], true);

	uORB::Publication<manual_control_setpoint_s> manual_pub{ORB_ID(manual_control_setpoint)};
	manual_control_setpoint_s manual{};
	manual.valid = true;
	manual.aux1 = 1.f;
	manual.timestamp = hrt_absolute_time();
	ASSERT_TRUE(manual_pub.publish(manual));

	matrix::Vector<float, Effectiveness::NUM_AXES> control{};
	control(Effectiveness::PITCH) = 1.f;
	allocation.setControlSetpoint(control);

	for (int step = 1; step <= 2; ++step) {
		allocation.allocate();
		tailsitter.updateSetpoint(control, 0, allocation.actuatorSetpoint(), minimum, maximum);
		allocation.applySlewRateLimit(0.1f);
		EXPECT_NEAR(allocation.getActuatorSetpoint()(2), 0.1f * step, 1e-6f);
		EXPECT_NEAR(allocation.getActuatorSetpoint()(3), 0.1f * step, 1e-6f);
	}

	manual.aux1 = -1.f;
	manual.timestamp = hrt_absolute_time();
	ASSERT_TRUE(manual_pub.publish(manual));
	allocation.allocate();
	tailsitter.updateSetpoint(control, 0, allocation.actuatorSetpoint(), minimum, maximum);
	allocation.applySlewRateLimit(0.1f);
	EXPECT_NEAR(allocation.getActuatorSetpoint()(2), 0.1f, 1e-6f);
	EXPECT_NEAR(allocation.getActuatorSetpoint()(3), 0.1f, 1e-6f);
}
