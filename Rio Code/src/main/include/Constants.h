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
#include <frc/controller/ArmFeedforward.h>
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
}	// namespace OperatorConstants

namespace HardwareConstants
{
	/// Counts per revolution of versaplanetary encoders
	inline constexpr int kVersaEncoderCPR = 1024;
}	// namespace HardwareConstants

/**
 * CAN bus IDs for various components
*/
namespace CANConstants
{
	/// Victor SPX CAN IDs

	///	Motor controller IDs for drivebase motors
	inline constexpr int kDrivebaseMotorIDs[4] = {
		1,	// Front left motor
		2,	// Back left motor
		3,	// Front right motor
		4	// Back right motor
	};

	/// @brief Victor SPX CAN ID for intake lift motor
	inline constexpr int kIntakeLiftVictorID = 5;

	/// @brief Victor SPX CAN IDs for climb motors
	inline constexpr int kClimbVictorIDs[2] = {
		6,	// Left climb motor
		7	// Right climb motor
	};

	/// Talon SRX CAN IDs

	///	@brief Talon SRX CAN ID for intake roller (indexer) motor
	inline constexpr int kIntakeRollerTalonID = 1;

	/// @brief Talon SRX CAN ID for amp ramp motor
	inline constexpr int kAmpRampTalonID = 2;

	// Spark MAX CAN IDs

	///	Spark MAX IDs for shooter motors
	inline constexpr int kShooterSparkIDs[2] = {
		1,	// Left NEO
		2	// Right NEO
	};
}	// namespace CANConstants

/**
 * DIO ports for various components such as encoders and limit switches
*/
namespace DIOConstants
{
	///	@brief Drivebase encoder A and B channel DIO ports
	inline constexpr int kDrivebaseEncoderPorts[4] = {
		2, 3,	// Left encoder A and B channels
		0, 1	// Right encoder A and B channels
	};

	///	@brief DIO port for amp ramp limit switch
	inline constexpr int kAmpRampSwitchPort = 4;

	/// @brief Encoder ports for lift motor
	inline constexpr int kIntakeLiftEncoderPorts[2] = { 5, 6 };

	/// @brief Limit switch port for homing the intake arms
	inline constexpr int kIntakeLimitSwitchPort = 7;

}	// namespace DIOConstants

/**
 * Constants for drivebase operation
*/
namespace DriveConstants
{
	// Counts per revolution of drivebase encoders
	inline constexpr int kEncoderCountsPerRev = 360;
	
	/// Robot dimensions
	inline constexpr units::meter_t kWheelbaseWidth = 0.6_m;
	inline constexpr units::meter_t kWheelDiameter = 6_in;

	/// Maximum wheel speed if motors are running at full throttle
	inline constexpr units::meters_per_second_t kMaxWheelSpeed = 4_mps;

	///	PID values for forward/back driving PID controllers
	inline constexpr double kDriveP = 22.0,
		kDriveI = 0.0,
		kDriveD = 6.0;

	///	Max velocity/acceleration for forward/back driving in auto
	inline constexpr units::meters_per_second_t kMaxDriveVel = 4_mps;
	inline constexpr units::meters_per_second_squared_t kMaxDriveAccel = 10_mps_sq;

	inline const frc::ProfiledPIDController<units::meters> kAutoDriveController{ kDriveP, kDriveI, kDriveD, { kMaxDriveVel, kMaxDriveAccel } };

	///	PID values for turning PID controllers
	inline constexpr double kTurnP = 60.0,
		kTurnI = 2.2,
		kTurnD = 1.0;

	///	Max velocity/acceleration for turning in auto
	inline constexpr units::degrees_per_second_t kMaxTurnVel = 300_deg_per_s;
	inline constexpr units::degrees_per_second_squared_t kMaxTurnAccel = 800_deg_per_s_sq;

	inline const frc::ProfiledPIDController<units::degrees> kAutoTurnController{ kTurnP, kTurnI, kTurnD, { kMaxTurnVel, kMaxTurnAccel } };
}

/**
 * Constants for intake operation
*/
namespace IntakeConstants
{
	/// @brief Encoder counts per rev for lift motor
	inline constexpr int kLiftEncoderCPR = 7;

	/**
	 * @brief Ratio between motor and intake arms
	 * Motor is a 71:1 reduction
	 * Sprockets are in a 12t:33t
	*/
	inline constexpr double kLiftRatio = 1.0 / 71.0 / (33.0 / 12.0);

	/// @brief Homing speed
	inline constexpr double kHomeSpeed = -0.2;
	
	/**
	 * @brief Homed angle of the arm
	 * this is measured clockwise from the horizontal
	*/
	inline constexpr units::degree_t kHomeAngle = 148_deg;

	/**
	 * @brief Deploy angle for the intake
	 * Intake resets to 0 upon homing, so this is measured from the home position
	*/
	inline constexpr units::degree_t kDeployAngle = 238_deg;

	/// Maximum velocity and acceleration for lifting the intake
	inline constexpr units::degrees_per_second_t kMaxVel = 500_deg_per_s;
	inline constexpr units::degrees_per_second_squared_t kMaxAccel = 2000_deg_per_s_sq;

	/// Intake lift PID constants
	inline double kP = 1.0;

	/// Intake lift arm feedforward constants
	inline units::volt_t kS = 0.05_V;
	inline units::volt_t kG = 0.42_V;
	inline units::unit_t<frc::ArmFeedforward::kv_unit> kV{ 0.02 };
}

namespace AmpRampConstants
{
	/// Amp ramp deploy and stow angles
	inline constexpr units::degree_t kDeployAngle = 0_deg,
		kStowAngle = 60_deg,
		kHomeAngle = kStowAngle;
}