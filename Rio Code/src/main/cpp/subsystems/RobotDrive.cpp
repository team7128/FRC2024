#include "subsystems/RobotDrive.h"

#include <frc/Preferences.h>
#include <frc/RobotController.h>

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/BuiltInWidgets.h>

#include <frc2/command/InstantCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/ProfiledPIDCommand.h>

#include <numbers>

#include "Constants.h"

using namespace DriveConstants;

//drive base can IDs
RobotDrive::RobotDrive() :
	m_motorFL(CANConstants::kDrivebaseMotorIDs[0]),
	m_motorBL(CANConstants::kDrivebaseMotorIDs[1]),
	m_motorFR(CANConstants::kDrivebaseMotorIDs[2]),
	m_motorBR(CANConstants::kDrivebaseMotorIDs[3]),
	m_diffDrive(m_motorFL, m_motorFR),
	m_diffDriveKinematics(kWheelbaseWidth),
	m_odometry{ frc::Rotation2d(0_deg), 0_m, 0_m }
{
	// Invert right side of drivebase
	m_motorFR.SetInverted(true);
	m_motorBR.SetInverted(true);

	// Back motors follow front motors
	m_motorBR.Follow(m_motorFR);
	m_motorBL.Follow(m_motorFL);

	// Place a drivebase and field widget on the Shuffleboard
	auto &tab = frc::Shuffleboard::GetTab("Main");
	tab.Add(m_diffDrive);
	tab.Add(m_field);

	frc::Preferences::InitBoolean(DriveConstants::kUseEncoderKey, DriveConstants::kUseEncoderDefault);
}

void RobotDrive::Periodic()
{
	// Update internal position and upload it to the field widget
	m_position = m_odometry.Update(m_odometryComponents.GetAngle(), m_odometryComponents.GetLeftDistance(), m_odometryComponents.GetRightDistance());
	m_field.SetRobotPose(m_position);

	m_odometryComponents.Periodic();
}

void RobotDrive::SimulationPeriodic()
{
	m_driveSim.SetInputs(
		m_motorFL.Get() * units::volt_t(frc::RobotController::GetInputVoltage()),
		m_motorFR.Get() * units::volt_t(frc::RobotController::GetInputVoltage())
	);

	m_driveSim.Update(20_ms);

	m_odometryComponents.m_leftEncoderSim.SetDistance(m_driveSim.GetLeftPosition().value());
	m_odometryComponents.m_leftEncoderSim.SetRate(m_driveSim.GetLeftVelocity().value());
	m_odometryComponents.m_rightEncoderSim.SetDistance(m_driveSim.GetRightPosition().value());
	m_odometryComponents.m_rightEncoderSim.SetRate(m_driveSim.GetRightVelocity().value());

	m_odometryComponents.leftSpeed = m_driveSim.GetLeftVelocity();
	m_odometryComponents.rightSpeed = m_driveSim.GetRightVelocity();
	m_odometryComponents.leftDistance = m_driveSim.GetLeftPosition();
	m_odometryComponents.rightDistance = m_driveSim.GetRightPosition();

	m_odometryComponents.m_angleSim.Set(m_driveSim.GetHeading().Radians().value());
}

void RobotDrive::ArcadeDrive(units::meters_per_second_t velocity, units::degrees_per_second_t rotational, bool squareInputs)
{
	// Convert forward/rotational speed to wheel velocities
	auto wheelSpeeds = m_diffDriveKinematics.ToWheelSpeeds({ velocity, 0_mps, rotational });
	wheelSpeeds.Desaturate(kMaxWheelSpeed);

	// Using tank drive to simply provide wheel speeds, instead of converting back to chassis speeds for arcade
	m_diffDrive.TankDrive(
		wheelSpeeds.left / kMaxWheelSpeed,
		wheelSpeeds.right / kMaxWheelSpeed,
		squareInputs
	); 

	m_odometryComponents.leftSpeed = m_motorFL.Get() * kMaxWheelSpeed;
	m_odometryComponents.rightSpeed = m_motorFR.Get() * kMaxWheelSpeed;
}

void RobotDrive::Stop()
{
	m_diffDrive.StopMotor();
	
	m_odometryComponents.leftSpeed = 0_mps;
	m_odometryComponents.rightSpeed = 0_mps;
}

void RobotDrive::ResetPosition()
{
	m_odometry.ResetPosition({ m_odometryComponents.GetAngle() }, m_odometryComponents.GetLeftDistance(), m_odometryComponents.GetRightDistance(), frc::Pose2d());
}

void RobotDrive::UpdateParams()
{
	m_odometryComponents.useEncoders = frc::Preferences::GetBoolean(DriveConstants::kUseEncoderKey, m_odometryComponents.useEncoders);
}

frc2::CommandPtr RobotDrive::DriveDistanceCmd(units::meter_t distance)
{
	return DriveDistanceCmd_t(distance, this).ToPtr();
}

frc2::CommandPtr RobotDrive::TurnToAngleCmd(units::degree_t angle)
{
	return TurnToAngleCmd_t(angle, this).ToPtr();
}

frc2::CommandPtr RobotDrive::TurnByAngleCmd(units::degree_t angle)
{
	return TurnByAngleCmd_t(angle, this).ToPtr();
}

frc2::CommandPtr RobotDrive::FacePointCmd(units::meter_t fieldX, units::meter_t fieldY)
{
	auto angle = units::math::atan2(fieldY - m_position.Y(), fieldX - m_position.X());
	return TurnToAngleCmd(angle);
}

frc2::CommandPtr RobotDrive::GoToPointCmd(units::meter_t fieldX, units::meter_t fieldY, units::meter_t stopDistance)
{
	return GoToPointCmd_t(fieldX, fieldY, stopDistance, this).ToPtr();
}

void RobotDrive::ConfigureDriveController(frc::ProfiledPIDController<units::meters> &controller)
{
	controller.SetTolerance(5_cm, 0.1_mps);
}

void RobotDrive::ConfigureTurnController(frc::ProfiledPIDController<units::degrees> &controller)
{
	controller.EnableContinuousInput(0_deg, 360_deg);
	controller.SetTolerance(5_deg, 50_deg_per_s);
}

RobotDrive::OdometryComponents::OdometryComponents() :
	m_gyro(frc::SPI::Port::kMXP),
	m_leftEncoder(DIOConstants::kDrivebaseEncoderPorts[0], DIOConstants::kDrivebaseEncoderPorts[1]),
	m_rightEncoder(DIOConstants::kDrivebaseEncoderPorts[2], DIOConstants::kDrivebaseEncoderPorts[3])
{
	// Set up encoder distance per pulse (wheel circumference / pulses per revolution)
	m_leftEncoder.SetDistancePerPulse(kWheelDiameter.value() * std::numbers::pi / kEncoderCountsPerRev);
	m_rightEncoder.SetDistancePerPulse(kWheelDiameter.value() * std::numbers::pi / kEncoderCountsPerRev);
	
	m_leftEncoder.SetReverseDirection(true);

	// Initialize all components to 0
	Reset();

	auto &mainTab = frc::Shuffleboard::GetTab("Main");

	mainTab.Add("Left encoder", m_leftEncoder);
	mainTab.Add("Right encoder", m_rightEncoder);

	mainTab.Add("Gyro", m_gyro);
}

void RobotDrive::OdometryComponents::Periodic()
{
	leftDistance += leftSpeed * 20_ms;
	rightDistance += rightSpeed * 20_ms;
}

void RobotDrive::OdometryComponents::Reset()
{
	m_gyro.Reset();
	m_leftEncoder.Reset();
	m_rightEncoder.Reset();

	leftSpeed = 0_mps;
	rightSpeed = 0_mps;
	leftDistance = 0_m;
	rightDistance = 0_m;
}

int RobotDrive::ToSensorUnits(double value)
{
	return std::round(value / (DriveConstants::kWheelDiameter.value() * std::numbers::pi) * DriveConstants::kEncoderCountsPerRev * 10.0);
}

RobotDrive::DriveDistanceCmd_t::DriveDistanceCmd_t(units::meter_t distance, RobotDrive *drive) :
	CommandHelper(
		kAutoDriveController,
		[] { return 0_m; }, // Overwritten in Initialize()
		distance,
		[drive] (double output, auto) { drive->ArcadeDrive(units::meters_per_second_t(output), 0_deg_per_s, false); },
		{ drive }
	),
	m_robotDriveSub(drive)
{
	RobotDrive::ConfigureDriveController(GetController());

	AddRequirements(drive);
}

void RobotDrive::DriveDistanceCmd_t::Initialize()
{
	CommandHelper::Initialize();
	auto initialDistance = m_robotDriveSub->m_odometryComponents.GetAvgDistance();
	GetController().Reset(m_robotDriveSub->m_odometryComponents.GetAvgDistance() - initialDistance);
	m_measurement = [this, initialDistance] { return m_robotDriveSub->m_odometryComponents.GetAvgDistance() - initialDistance; };
}

bool RobotDrive::DriveDistanceCmd_t::IsFinished()
{
	return GetController().AtGoal();
}

RobotDrive::TurnToAngleCmd_t::TurnToAngleCmd_t(units::degree_t angle, RobotDrive *drive) :
	CommandHelper(
		kAutoTurnController,
		[drive] { return drive->m_odometryComponents.GetAngle(); },
		angle,
		[drive] (double output, auto) { drive->ArcadeDrive(0_mps, units::degrees_per_second_t(output), false); },
		{ drive }
	),
	m_robotDriveSub(drive)
{
	RobotDrive::ConfigureTurnController(GetController());

	AddRequirements(drive);
}

void RobotDrive::TurnToAngleCmd_t::Initialize()
{
	CommandHelper::Initialize();
	GetController().Reset(m_robotDriveSub->m_odometryComponents.GetAngle());
}

bool RobotDrive::TurnToAngleCmd_t::IsFinished()
{
	return GetController().AtGoal();
}

RobotDrive::TurnByAngleCmd_t::TurnByAngleCmd_t(units::degree_t angle, RobotDrive *drive) :
	CommandHelper(
		kAutoTurnController,
		[] { return 0_deg; },	// Overwritten in Initialize()
		angle,
		[drive] (double output, auto) { drive->ArcadeDrive(0_mps, units::degrees_per_second_t(output), false); },
		{ drive }
	),
	m_robotDriveSub(drive)
{
	RobotDrive::ConfigureTurnController(GetController());
}

void RobotDrive::TurnByAngleCmd_t::Initialize()
{
	CommandHelper::Initialize();
	auto initialAngle = m_robotDriveSub->m_odometryComponents.GetAngle();
	GetController().Reset(m_robotDriveSub->m_odometryComponents.GetAngle() - initialAngle);
	m_measurement = [this, initialAngle] { return m_robotDriveSub->m_odometryComponents.GetAngle() - initialAngle; };
}

bool RobotDrive::TurnByAngleCmd_t::IsFinished()
{
	return GetController().AtGoal();
}

RobotDrive::GoToPointCmd_t::GoToPointCmd_t(units::meter_t fieldX, units::meter_t fieldY, units::meter_t stopDistance, RobotDrive *drive) :
	m_targetX(fieldX),
	m_targetY(fieldY),
	m_driveController(kAutoDriveController),
	m_turnController(kAutoTurnController),
	m_robotDriveSub(drive)
{
	AddRequirements(drive);

	RobotDrive::ConfigureDriveController(m_driveController);
	RobotDrive::ConfigureTurnController(m_turnController);
	m_driveController.SetGoal(stopDistance);
}

void RobotDrive::GoToPointCmd_t::Initialize()
{
	CommandHelper::Initialize();
	m_turnController.Reset(m_robotDriveSub->m_odometryComponents.GetAngle());

	// auto &tab = frc::Shuffleboard::GetTab("Main");
	// tab.Add("GoTo PID", m_driveController);
}

void RobotDrive::GoToPointCmd_t::Execute()
{
	CommandHelper::Execute();

	units::degree_t targetAngle = units::math::atan2(
		m_targetY - m_robotDriveSub->GetPosition().Y(),
		m_targetX - m_robotDriveSub->GetPosition().X()
	);

	units::degrees_per_second_t turnRate{ m_turnController.Calculate(m_robotDriveSub->m_odometryComponents.GetAngle(), targetAngle) };
	units::meters_per_second_t driveRate{ 0 };

	units::meter_t distanceToTarget = units::math::sqrt(
		units::math::pow<2>(m_targetX - m_robotDriveSub->GetPosition().X()) +
		units::math::pow<2>(m_targetY - m_robotDriveSub->GetPosition().Y())
	);

	if (!m_facingCorrect && m_turnController.AtGoal())
	{
		m_facingCorrect = true;
		m_driveController.Reset(distanceToTarget);
	}
	
	if (m_facingCorrect)
	{
		auto targetAngleOffset = targetAngle - m_robotDriveSub->m_odometryComponents.GetAngle();
		auto speedMult = units::math::cos(targetAngleOffset);
		// Negative because we are going from a positive distance towards zero, which gives a negative velocity
		driveRate = -units::meters_per_second_t(m_driveController.Calculate(distanceToTarget)) * speedMult;
	}

	m_robotDriveSub->ArcadeDrive(driveRate, turnRate, false);
}

bool RobotDrive::GoToPointCmd_t::IsFinished()
{
	return m_driveController.AtGoal();
}