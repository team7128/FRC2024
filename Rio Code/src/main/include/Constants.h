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
	inline constexpr units::degrees_per_second_t kMaxTeleopTurnSpeed = 360_deg_per_s;
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
	 * Spark MAX IDs for shooter motors
	*/
	inline constexpr int kShooterSparkIDs[2] = {
		1,	// Left Spark
		2	// Right Spark
	};

	/**
	 * Spark MAX IDs for intake motors
	*/
	inline constexpr int kIntakeSparkIDs[2] = {
		3,	// Top Spark
		4	// Bottom Spark
	};

	/**
	 * Drivebase encoder A and B channel DIO ports
	*/
	inline constexpr int kDrivebaseEncoderPorts[4] = {
		2, 3,	// Left encoder A and B channels
		0, 1	// Right encoder A and B channels
	};

	/**
	 * Solenoid forward and back channels for amp ramp extender
	*/
	inline constexpr int kAmpSolenoidChannels[4] = {
		1, 2,	// Left ramp solenoid
		3, 4	// Right ramp solenoid
	};

	/**
	 * Solenoid forward and back channels for intake deployers
	*/
	inline constexpr int kIntakeSolenoidChannels[4] = {
		5, 6,	// Left ramp solenoid
		7, 8	// Right ramp solenoid
	};

	// Counts per revolution of drivebase encoders
	inline constexpr int kDrivebaseEncoderCountsPerRev = 360;
}	// namespace HardwareConstants

namespace RobotConstants
{
	inline constexpr units::meter_t kWheelbaseWidth = 0.6_m;
	inline constexpr units::meter_t kWheelDiameter = 6_in;

	/// Maximum wheel speed if motors are running at full throttle
	inline constexpr units::meters_per_second_t kMaxWheelSpeed = 4_mps;
}	// namespace RobotDimensions

namespace AutoConstants
{
	inline constexpr double kDriveP = 4,
		kDriveI = 0.12,
		kDriveD = 0.03;

	inline constexpr units::meters_per_second_t kMaxDriveVelocity = 3_mps;
	inline constexpr units::meters_per_second_squared_t kMaxDriveAccel = 0.5_mps_sq;
	inline const frc::TrapezoidProfile<units::meters> kDriveProfile = { { kMaxDriveVelocity, kMaxDriveAccel } };

	inline constexpr double kTurnP = 1,
		kTurnI = 0.02,
		kTurnD = 0.03;

	inline constexpr units::degrees_per_second_t kMaxTurnVelocity = 210_deg_per_s;
	inline constexpr units::degrees_per_second_squared_t kMaxTurnAccel = 10_deg_per_s_sq;
	inline const frc::TrapezoidProfile<units::degrees> kTurnProfile = { { kMaxTurnVelocity, kMaxTurnAccel } };
}