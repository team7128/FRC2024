#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>

#include <frc/drive/DifferentialDrive.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/XboxController.h>

#include <ctre/phoenix/motorcontrol/can/WPI_VictorSPX.h>

#include <units/velocity.h>
#include <units/angular_velocity.h>

class RobotDrive : public frc2::SubsystemBase
{
public:
	RobotDrive();

	/**
	 * Command factory to provide the default human-driven arcade drive behaviour
	 * 
	 * @param velocity Forward/back velocity in meters/second
	 * @param rotational Rotational velocity, in degrees/second. CCW is positive.
	*/
	void ArcadeDrive(units::meters_per_second_t velocity, units::degrees_per_second_t rotational, bool squareInputs);

	/**
	 * Stops the drivebase
	*/
	void Stop();

private:
	/// Victor SPX motor controllers
	ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_motorFR, m_motorBR, m_motorFL, m_motorBL;

	/// Differential drive for controlling motors
	frc::DifferentialDrive m_diffDrive;

	/// Handles limiting wheel speeds to reasonable values
	frc::DifferentialDriveKinematics m_diffDriveKinematics;
};