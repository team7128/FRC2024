// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/length.h>

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace OperatorConstants
{
	inline constexpr int kDriverControllerPort = 0;

}  // namespace OperatorConstants

namespace HardwareConstants
{
	/**
	 * Motor controller IDs for drivebase motors
	*/
	inline constexpr int kDrivebaseMotorIDs[4] = {
		1,	// Front right motor
		2,	// Back right motor
		3,	// Front left motor
		4	// Back left motor
	};

	/**
	 * Drivebase encoder A and B channel DIO ports
	*/
	inline constexpr int kDrivebaseEncoderPorts[4] {
		2, 3,	// Left encoder A and B channels
		4, 5	// Right encoder A and B channels
	};

	// Counts per revolution of drivebase encoders
	inline constexpr int kDrivebaseEncoderCountsPerRev = 400;

}	// namespace HardwareConstants

namespace RobotDimensions
{
	inline constexpr units::meter_t kWheelbaseWidth = 0.6_m;
	inline constexpr units::meter_t kWheelDiameter = 6_in;
}	// namespace RobotDimensions