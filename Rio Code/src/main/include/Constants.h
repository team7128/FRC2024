// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/length.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>

#include <frc/trajectory/TrapezoidProfile.h>

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

	/**
	 * Max driving and turn speeds for teleop players
	*/
	inline constexpr units::meters_per_second_t kMaxTeleopSpeed = 4_mps;
	inline constexpr units::degrees_per_second_t kMaxTeleopTurnSpeed = 40_deg_per_s;
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
	inline constexpr int kDrivebaseEncoderPorts[4] = {
		2, 3,	// Left encoder A and B channels
		4, 5	// Right encoder A and B channels
	};

	// Counts per revolution of drivebase encoders
	inline constexpr int kDrivebaseEncoderCountsPerRev = 400;

}	// namespace HardwareConstants

namespace RobotConstants
{
	inline constexpr units::meter_t kWheelbaseWidth = 0.6_m;
	inline constexpr units::meter_t kWheelDiameter = 6_in;

	/// Maximum wheel speed if motors are running at full throttle
	inline constexpr units::meters_per_second_t kMaxWheelSpeed = 3_mps;
}	// namespace RobotDimensions

namespace AutoConstants
{
	inline constexpr double kDriveP = 1,
		kDriveI = 0.02,
		kDriveD = 0.1;

	inline constexpr units::meters_per_second_t kMaxDriveVelocity = 1_mps;
	inline constexpr units::meters_per_second_squared_t kMaxDriveAccel = 0.5_mps_sq;
	inline const frc::TrapezoidProfile<units::meters> kDriveProfile = { { kMaxDriveVelocity, kMaxDriveAccel } };

	inline constexpr double kTurnP = 1,
		kTurnI = 0.02,
		kTurnD = 0.1;

	inline constexpr units::degrees_per_second_t kMaxTurnVelocity = 30_deg_per_s;
	inline constexpr units::degrees_per_second_squared_t kMaxTurnAccel = 10_deg_per_s_sq;
	inline const frc::TrapezoidProfile<units::degrees> kTurnProfile = { { kMaxTurnVelocity, kMaxTurnAccel } };
}