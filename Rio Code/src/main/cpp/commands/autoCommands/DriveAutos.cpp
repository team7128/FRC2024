#include "commands/autoCommands/DriveAutos.h"

#include <frc2/command/TrapezoidProfileCommand.h>

#include "Constants.h"

DriveAutos::DriveAutos(RobotDrive *driveSubsystem, Odometry *odometrySubsystem)
	: m_driveSubsystem(driveSubsystem), m_odometrySubsystem(odometrySubsystem)
{}

frc2::CommandPtr DriveAutos::DriveDistance(units::meter_t distance)
{
	return DriveDistanceCmd(m_driveSubsystem, m_odometrySubsystem, distance).ToPtr();
	// units::meter_t currentDistance = m_odometrySubsystem->GetCenterDistance();

	// return frc2::TrapezoidProfileCommand<units::meters>(
	// 	AutoConstants::kDriveProfile,
	// 	[this](frc::TrapezoidProfile<units::meters>::State state)
	// 	{
	// 		m_driveSubsystem->ArcadeDrive(state.velocity, 0_deg_per_s, false);
	// 	},
	// 	[distance, currentDistance]() -> frc::TrapezoidProfile<units::meters>::State { return { distance + currentDistance, 0_mps }; },
	// 	[this]() -> frc::TrapezoidProfile<units::meters>::State {
	// 		return { m_odometrySubsystem->GetCenterDistance(), m_odometrySubsystem->GetVelocity() };
	// 	},
	// 	{ m_driveSubsystem }
	// ).ToPtr();
}

frc2::CommandPtr DriveAutos::TurnToAngle(units::degree_t angle)
{
	return TurnToAngleCmd(m_driveSubsystem, m_odometrySubsystem, angle).ToPtr();
}

frc2::CommandPtr DriveAutos::TurnByAngle(units::degree_t angle)
{
	return TurnToAngleCmd(m_driveSubsystem, m_odometrySubsystem, m_odometrySubsystem->GetAngle() + angle).ToPtr();
	// return frc2::TrapezoidProfileCommand<units::degrees>(
	// 	AutoConstants::kTurnProfile,
	// 	[this](frc::TrapezoidProfile<units::degrees>::State state)
	// 	{
	// 		m_driveSubsystem->ArcadeDrive(0_mps, state.velocity, false);
	// 	},
	// 	[angle]() -> frc::TrapezoidProfile<units::degrees>::State { return { angle, 0_deg_per_s }; },
	// 	[this]() -> frc::TrapezoidProfile<units::degrees>::State {
	// 		return { m_odometrySubsystem->GetAngle(), m_odometrySubsystem->GetAngularVelocity() };
	// 	},
	// 	{ m_driveSubsystem }
	// ).ToPtr();
}

frc2::CommandPtr DriveAutos::FacePoint(units::meter_t fieldX, units::meter_t fieldY)
{
	units::degree_t targetAngle = units::math::atan2<units::meter_t, units::meter_t>(
		fieldY - m_odometrySubsystem->GetY(),
		fieldX - m_odometrySubsystem->GetX()
	);

	return TurnToAngle(targetAngle);
}

frc2::CommandPtr DriveAutos::GoToPoint(units::meter_t fieldX, units::meter_t fieldY, units::meter_t deadzone)
{
	return GoToPointCmd(m_driveSubsystem, m_odometrySubsystem, fieldX, fieldY, deadzone).ToPtr();
}

DriveAutos::DriveDistanceCmd::DriveDistanceCmd(RobotDrive *driveSubsystem, Odometry *odometrySubsystem, units::meter_t distance)
	: CommandHelper(
		frc::ProfiledPIDController<units::meters>{
			AutoConstants::kDriveP, AutoConstants::kDriveI, AutoConstants::kDriveD,
			{ AutoConstants::kMaxDriveVelocity, AutoConstants::kMaxDriveAccel }
		},
		[odometrySubsystem] { return odometrySubsystem->GetCenterDistance(); },
		distance,
		[driveSubsystem](double output, auto state) {
			driveSubsystem->ArcadeDrive(units::meters_per_second_t(output), 0_deg_per_s, false);
		},
		{ driveSubsystem }
	)
{
	m_controller.SetTolerance(10_cm, 0.1_mps);
}

bool DriveAutos::DriveDistanceCmd::IsFinished()
{
	return m_controller.AtSetpoint();
}

DriveAutos::TurnToAngleCmd::TurnToAngleCmd(RobotDrive *driveSubsystem, Odometry *odometrySubsystem, units::degree_t targetAngle)
	: CommandHelper(
		frc::ProfiledPIDController<units::degrees>{
			AutoConstants::kTurnP, AutoConstants::kTurnI, AutoConstants::kTurnD,
			{ AutoConstants::kMaxTurnVelocity, AutoConstants::kMaxTurnAccel }
		},
		[odometrySubsystem] { return odometrySubsystem->GetAngle(); },
		targetAngle,
		[driveSubsystem](double output, auto state) {
			driveSubsystem->ArcadeDrive(0_mps, units::degrees_per_second_t(output), false);
		},
		{ driveSubsystem }
	)
{
	m_controller.EnableContinuousInput(0_deg, 360_deg);
	m_controller.SetTolerance(3_deg, 0.1_deg_per_s);
}

bool DriveAutos::TurnToAngleCmd::IsFinished()
{
	return m_controller.AtSetpoint();
}

DriveAutos::GoToPointCmd::GoToPointCmd(
		RobotDrive *driveSubsystem, Odometry *odometrySubsystem,
		units::meter_t fieldX, units::meter_t fieldY,
		units::meter_t deadzone
	)
	: m_targetX(fieldX), m_targetY(fieldY), m_deadzone(deadzone),
	m_driveSubsystem(driveSubsystem), m_odometrySubsystem(odometrySubsystem),
	m_driveController{
		AutoConstants::kDriveP, AutoConstants::kDriveI, AutoConstants::kDriveD,
		{ AutoConstants::kMaxDriveVelocity, AutoConstants::kMaxDriveAccel }
	},
	m_turnController{
		AutoConstants::kTurnP, AutoConstants::kTurnI, AutoConstants::kTurnD,
		{ AutoConstants::kMaxTurnVelocity, AutoConstants::kMaxTurnAccel }
	}
{
	AddRequirements(driveSubsystem);
	m_turnController.EnableContinuousInput(0_deg, 360_deg);
	m_driveController.SetGoal(deadzone);

	m_driveController.SetTolerance(10_cm, 0.1_mps);
	m_turnController.SetTolerance(3_deg, 0.1_deg_per_s);
}

void DriveAutos::GoToPointCmd::Execute()
{
	static bool facingCorrect = false;

	units::degree_t targetAngle = units::math::atan2(
		m_targetY - m_odometrySubsystem->GetY(),
		m_targetX - m_odometrySubsystem->GetX()
	);

	units::degrees_per_second_t turnRate{ m_turnController.Calculate(targetAngle, m_odometrySubsystem->GetAngle()) };
	units::meters_per_second_t driveRate{ 0 };

	units::meter_t distanceToTarget = units::math::sqrt(
		units::math::pow<2>(m_targetX - m_odometrySubsystem->GetX()) +
		units::math::pow<2>(m_targetY - m_odometrySubsystem->GetY())	
	);

	if (!facingCorrect)
	{
		if (m_turnController.AtSetpoint())
		{
			facingCorrect = true;
			m_driveController.Reset(distanceToTarget);
		}
	}
	
	if (facingCorrect)
	{
		driveRate = units::meters_per_second_t(m_driveController.Calculate(distanceToTarget));
	}

	m_driveSubsystem->ArcadeDrive(driveRate, turnRate, false);
}

bool DriveAutos::GoToPointCmd::IsFinished()
{
	return m_driveController.AtGoal();
}