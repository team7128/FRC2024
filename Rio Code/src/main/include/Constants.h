// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>
#include <math.h>

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
	inline constexpr int kShooterControllerPort = 1;

	///	Max driving and turn speeds for teleop players
	inline constexpr units::meters_per_second_t kMaxTeleopSpeed = 3_mps;
	inline constexpr units::degrees_per_second_t kMaxTeleopTurnSpeed = 500_deg_per_s;
}	// namespace OperatorConstants

namespace HardwareConstants
{
	/// Counts per revolution of versaplanetary encoders
	/// DO NOT TOUCH
	inline constexpr int kVersaEncoderCPR = 4096;
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

 Preferrably, refer to these when reconnecting any electronics instead of changing these values
*/
namespace DIOConstants
{
	///	@brief Drivebase encoder A and B channel DIO ports
	inline constexpr int kDrivebaseEncoderPorts[4] = {
		2, 3,	// Left encoder A and B channels
		0, 1	// Right encoder A and B channels
	};

	/// @brief Encoder ports for lift motor
	inline constexpr int kIntakeLiftEncoderPorts[2] = { 4, 5 };

	///	@brief DIO port for amp ramp limit switch
	inline constexpr int kAmpRampSwitchPort = 6;

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
	/// DO NOT CHANGE
	inline constexpr units::meters_per_second_t kMaxWheelSpeed = 4_mps;

	///	PID values for forward/back driving PID controllers
	inline constexpr double kDriveP = 4.0,
		kDriveI = 0.05,
		kDriveD = 0.5;

	///	Max velocity/acceleration for forward/back driving in auto
	inline constexpr units::meters_per_second_t kMaxDriveVel = 3_mps;
	inline constexpr units::meters_per_second_squared_t kMaxDriveAccel = 30_mps_sq;

	/// DO NOT EDIT THIS
	inline const frc::ProfiledPIDController<units::meters> kAutoDriveController{ kDriveP, kDriveI, kDriveD, { kMaxDriveVel, kMaxDriveAccel } };

	///	PID values for turning PID controllers
	inline constexpr double kTurnP = 12.0,
		kTurnI = 0.4,
		kTurnD = 0.8;

	///	Max velocity/acceleration for turning in auto
	inline constexpr units::degrees_per_second_t kMaxTurnVel = 500_deg_per_s;
	inline constexpr units::degrees_per_second_squared_t kMaxTurnAccel = 1200_deg_per_s_sq;

	/// DO NOT EDIT THIS
	inline const frc::ProfiledPIDController<units::degrees> kAutoTurnController{ kTurnP, kTurnI, kTurnD, { kMaxTurnVel, kMaxTurnAccel } };
}

/**
 * Constants for intake operation
*/
namespace IntakeConstants
{
	/// @brief Encoder counts per rev for lift motor
	/// DO NOT TOUCH
	inline constexpr int kLiftEncoderCPR = 7;

	/**
	 * @brief Ratio between motor and intake arms
	 * Motor is a 71:1 reduction
	 * Sprockets are in a 12t:33t
	 * L+Ratio
	*/
	inline constexpr double kLiftRatio = 1.0 / 71.0 / (33.0 / 12.0);

	/// @brief Homing speed
	inline constexpr double kHomeSpeed = 0.4;
	
	/**
	 * @brief Homed angle of the arm
	 * This is measured clockwise from the horizontal
	*/
	inline constexpr units::degree_t kHomeAngle = 40_deg;

	/**
	 * @brief Deploy angle for the intake
	 * Intake resets to 0 upon homing, so this is measured from the home position
	*/
	inline constexpr units::degree_t kDeployAngle = 170_deg;

	/**
	 * @brief Angle of intake to lock in climb arms
	 * This is measured from the home position, not vertical or horizontal
	 */
	inline constexpr units::degree_t kClimbAngle = 25_deg;

	inline constexpr units::degree_t kClimbIdleAngle = 55_deg;

	/// Maximum velocity and acceleration for lifting the intake
	inline constexpr units::degrees_per_second_t kMaxVel = 400_deg_per_s;
	inline constexpr units::degrees_per_second_squared_t kMaxAccel = 400_deg_per_s_sq;

	/// Intake lift PID constants
	inline constexpr double kP = 0.04;
}

namespace AmpRampConstants
{
	inline constexpr double kP = 5.0;

	inline constexpr double khomeSpeed = 0.2;

	/// Amp ramp deploy and stow angles
	inline constexpr units::degree_t kDeployAngle = 70_deg,
		kStowAngle = -70_deg,
		kHomeAngle = -90_deg;
}

namespace ClimbConstants
{
	inline const std::string kCLimbSpeedKey = "ClimbSpeed";
	inline constexpr double kClimbSpeedDefault = 0.65;
}

namespace ShooterConstants
{
	inline const std::string kSpeakerSpeedKey = "SpeakerShotSpeed";
	inline constexpr double kSpeakerSpeedDefault = 1.0;
	
	inline const std::string kAmpSpeedKey = "SpeakerShotSpeed";
	inline constexpr double kAmpSpeedDefault = 0.15;
}