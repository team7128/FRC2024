#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/ProfiledPIDCommand.h>

#include "subsystems/RobotDrive.h"
#include "subsystems/Odometry.h"

class DriveAutos
{
public:
	DriveAutos(RobotDrive *driveSubsystem, Odometry *odometrySubsystem);

	/**
	 * Command factory for command that drives a given distance
	 * 
	 * @param driveSubsystem RobotDrive subsystem
	 * @param odometrySubsystem Odometry subsystem
	 * @param distance Distance to drive, in meters
	*/
	frc2::CommandPtr DriveDistance(units::meter_t distance);

	/**
	 * Turns drivebase by a given angle
	 * 
	 * @param angle Direction to face, in degrees
	*/
	frc2::CommandPtr TurnToAngle(units::degree_t angle);

	/**
	 * Turns drivebase by a given angle
	 * 
	 * @param angle Amount to turn, in degrees
	*/
	frc2::CommandPtr TurnByAngle(units::degree_t angle);

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
	 * @param deadzone Allows you to stop some distance away from target point, regardless of approach angle
	*/
	frc2::CommandPtr GoToPoint(units::meter_t fieldX, units::meter_t fieldY, units::meter_t deadzone = 0_m);

private:
	RobotDrive *m_driveSubsystem;
	Odometry *m_odometrySubsystem;

	class DriveDistanceCmd
		: public frc2::CommandHelper<frc2::ProfiledPIDCommand<units::meters>, DriveDistanceCmd>
	{
	public:
		DriveDistanceCmd(RobotDrive *driveSubsystem, Odometry *odometrySubsystem, units::meter_t distance);

		bool IsFinished() override;
	};

	class TurnToAngleCmd
		: public frc2::CommandHelper<frc2::ProfiledPIDCommand<units::degrees>, TurnToAngleCmd>
	{
	public:
		TurnToAngleCmd(RobotDrive *driveSubsystem, Odometry *odometrySubsystem, units::degree_t distance);

		bool IsFinished() override;
	};

	class GoToPointCmd
		: public frc2::CommandHelper<frc2::Command, GoToPointCmd>
	{
	public:
		GoToPointCmd(
			RobotDrive *driveSubsystem, Odometry *odometrySubsystem,
			units::meter_t fieldX, units::meter_t fieldY,
			units::meter_t deadzone
		);

		void Execute() override;

		bool IsFinished() override;
	
	private:
		units::meter_t m_targetX, m_targetY;
		units::meter_t m_deadzone;

		RobotDrive *m_driveSubsystem;
		Odometry *m_odometrySubsystem;

		frc::ProfiledPIDController<units::meters> m_driveController;
		frc::ProfiledPIDController<units::degrees> m_turnController;
	};
};