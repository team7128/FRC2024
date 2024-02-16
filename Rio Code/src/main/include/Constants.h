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
#include <frc/controller/ProfiledPIDController.h>

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

	///	Max driving and turn speeds for teleop players
	inline constexpr units::meters_per_second_t kMaxTeleopSpeed = 4_mps;
	inline constexpr units::degrees_per_second_t kMaxTeleopTurnSpeed = 500_deg_per_s;
}  // namespace OperatorConstants

namespace HardwareConstants
{
	///	Motor controller IDs for drivebase motors
	inline constexpr int kDrivebaseMotorIDs[4] = {
		1,	// Front right motor
		2,	// Back right motor
		3,	// Front left motor
		4	// Back left motor
	};

	///	Spark MAX IDs for shooter motors
	inline constexpr int kShooterSparkIDs[2] = {
		1,	// Left Spark
		2	// Right Spark
	};

	///	Talon SRX CAN ID for intake roller (indexer) motor
	inline constexpr int kIntakeRollerTalonID = 5;

	///	DIO port for amp ramp limit switch
	inline constexpr int kAmpRampSwitchPort = 4;
	/// Talon SRX CAN ID for amp ramp motor
	inline constexpr int kAmpRampTalonID = 6;

	///	Drivebase encoder A and B channel DIO ports
	inline constexpr int kDrivebaseEncoderPorts[4] = {
		2, 3,	// Left encoder A and B channels
		0, 1	// Right encoder A and B channels
	};

	// Counts per revolution of drivebase encoders
	inline constexpr int kDrivebaseEncoderCountsPerRev = 360;

	/// Counts per revolution of versaplanetary encoders
	inline constexpr int kVersaEncoderCPR = 1024;
}	// namespace HardwareConstants

namespace RobotConstants
{
	inline constexpr units::meter_t kWheelbaseWidth = 0.6_m;
	inline constexpr units::meter_t kWheelDiameter = 6_in;

	/// Maximum wheel speed if motors are running at full throttle
	inline constexpr units::meters_per_second_t kMaxWheelSpeed = 4_mps;
}	// namespace RobotDimensions

/**
 * Constants used to operate subsystems, such as setpoints and speeds
*/
namespace SubsystemConstants
{
	///	PID values for forward/back driving PID controllers
	inline constexpr double kDriveP = 22.0,
		kDriveI = 0.0,
		kDriveD = 6.0;

	///	Max velocity/acceleration for forward/back driving in auto
	inline constexpr units::meters_per_second_t kMaxDriveVelocity = 4_mps;
	inline constexpr units::meters_per_second_squared_t kMaxDriveAccel = 10_mps_sq;

	inline const frc::ProfiledPIDController<units::meters> kAutoDriveController{ kDriveP, kDriveI, kDriveD, { kMaxDriveVelocity, kMaxDriveAccel } };

	///	PID values for turning PID controllers
	inline constexpr double kTurnP = 60.0,
		kTurnI = 2.2,
		kTurnD = 1.0;

	///	Max velocity/acceleration for turning in auto
	inline constexpr units::degrees_per_second_t kMaxTurnVelocity = 300_deg_per_s;
	inline constexpr units::degrees_per_second_squared_t kMaxTurnAccel = 800_deg_per_s_sq;

	/// Intake deploy and stow angles
	inline constexpr units::degree_t kIntakeDownAngle = -53_deg,
		kIntakeUpAngle = 160_deg;

	/// Amp ramp deploy and stow angles
	inline constexpr units::degree_t kAmpRampDeployAngle = 0_deg,
		kAmpRampStowAngle = 60_deg,
		kAmpRampHomeAngle = 60_deg;
}