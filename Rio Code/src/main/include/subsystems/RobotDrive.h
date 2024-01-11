#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>

#include <frc/drive/DifferentialDrive.h>
#include <frc/XboxController.h>

#include <ctre/Phoenix.h>

#include <units/length.h>
#include <units/angle.h>

class RobotDrive : public frc2::SubsystemBase
{
public:
	RobotDrive();

	/**
	 * Command factory to provide the default human-driven arcade drive behaviour
	 * 
	 * @param controller The Xbox controller used by the driver
	*/
	frc2::CommandPtr ArcadeDrive(frc::XboxController &controller);

	/**
	 * Stops the drivebase
	*/
	frc2::CommandPtr Stop();

	/**
	 * Drives forward/backward a given distance
	 * 
	 * @param distance Distance to drive, in meters
	*/
	frc2::CommandPtr DriveDistanceAuto(units::meter_t distance);

	/**
	 * Turns drivebase by a given angle
	 * 
	 * @param angle Amount to turn in degrees
	*/
	frc2::CommandPtr TurnAngleAuto(units::degree_t angle);

	/**
	 * Turns the drivebase to face a given point on the field
	 * 
	 * @param fieldX Target X coordinate, in meters. 0 is at the blue alliance driver stations, measured towards the red driver stations
	 * @param fieldY Target Y coordinate, in meters. 0 is at the source wall, measured towards the amp wall
	*/
	frc2::CommandPtr FacePoint(units::meter_t fieldX, units::meter_t fieldY);

	/**
	 * Navigates in a straight line to a given point on the field
	 * 
	 * @param fieldX Target X coordinate, in meters. 0 is at the blue alliance driver stations, measured towards the red driver stations
	 * @param fieldY Target Y coordinate, in meters. 0 is at the source wall, measured towards the amp wall
	*/
	frc2::CommandPtr GoToPoint(units::meter_t fieldX, units::meter_t fieldY);

private:
	/// Victor SPX motor controllers
	WPI_VictorSPX m_motorFR, m_motorBR, m_motorFL, m_motorBL;

	/// Differential drive for controlling motors
	frc::DifferentialDrive m_diffDrive;
};