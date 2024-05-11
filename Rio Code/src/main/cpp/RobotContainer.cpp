// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/Preferences.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>

#include <frc2/command/CommandScheduler.h>
#include <frc2/command/button/Trigger.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/InstantCommand.h>

#include "commands/Autos.h"
#include "commands/ShooterCommands.h"
#include "commands/IntakeCommands.h"

using namespace OperatorConstants;

RobotContainer::RobotContainer() :
	m_subsystems(Subsystems::GetInstance()),
	m_maxTeleopSpeed(kTeleopSpeedDefault),
	m_maxTeleopTurnSpeed(kTeleopTurnSpeedDefault),
	m_maxTeleopAccel(kMaxTeleopAccelDefault),
	m_maxTeleopTurnAccel(kMaxTeleopTurnAccelDefault),
	m_driveLimiter(m_maxTeleopAccel),
	m_turnLimiter(m_maxTeleopTurnAccel)
{
	frc::CameraServer::StartAutomaticCapture();

	m_autoChooser.SetDefaultOption("None", autos::None);
	m_autoChooser.AddOption("Mobility", autos::Mobility);
	m_autoChooser.AddOption("Speaker Shot", autos::SpeakerShot);
	m_autoChooser.AddOption("Speaker Center", autos::SpeakerCenter);
	m_autoChooser.AddOption("Speaker Left", autos::SpeakerLeft);
	m_autoChooser.AddOption("Speaker Right", autos::SpeakerRight);
	m_autoChooser.AddOption("Custom", autos::Custom);

	frc::SmartDashboard::PutData("Auto Selector", &m_autoChooser);

	frc::Preferences::InitDouble(kTeleopSpeedKey, m_maxTeleopSpeed.value());
	frc::Preferences::InitDouble(kTeleopTurnSpeedKey, m_maxTeleopTurnSpeed.value());
	frc::Preferences::InitDouble(kMaxTeleopAccelKey, m_maxTeleopAccel.value());
	frc::Preferences::InitDouble(kMaxTeleopTurnAccelKey, m_maxTeleopTurnAccel.value());

	// Set shooter to not run when not in use
	m_subsystems.shooterSub.SetDefaultCommand(m_subsystems.shooterSub.DisableCmd());
	
	m_subsystems.ampRampSub.SetDefaultCommand(m_subsystems.ampRampSub.StopCmd());

	m_subsystems.intakeSub.m_rollerSub.SetDefaultCommand(m_subsystems.intakeSub.m_rollerSub.DisableCmd());

	m_subsystems.intakeSub.m_liftSub.SetDefaultCommand(m_subsystems.intakeSub.m_liftSub.DisableCmd());

	m_subsystems.climbSub.SetDefaultCommand(m_subsystems.climbSub.StopCmd());

	m_climbModeTrigger = frc2::Trigger([this] { return this->m_climbMode; });

	// Extend the intake when beginning climb
	m_climbModeTrigger.OnTrue(m_subsystems.intakeSub.m_liftSub.ClimbIdleCmd());
	// Pull in the intake if we ever exit climb mode
	m_climbModeTrigger.OnFalse(m_subsystems.intakeSub.m_liftSub.StowCmd());

	// Configure the button bindings
	ConfigureBindings();
}

void RobotContainer::Reset()
{
	m_climbMode = false;
	m_subsystems.robotDriveSub.ResetPosition();
}

void RobotContainer::ConfigureBindings()
{
	// ===== Driver Controls =====

	// Bind drivebase controls to left stick up/down and right stick left/right
	m_subsystems.robotDriveSub.SetDefaultCommand(frc2::RunCommand([this] {
			auto driveSpeed = m_driveLimiter.Calculate(-m_driverController.GetLeftY() * m_maxTeleopSpeed);
			auto turnSpeed = m_turnLimiter.Calculate(-m_driverController.GetRightX() * m_maxTeleopTurnSpeed);

			m_subsystems.robotDriveSub.ArcadeDrive(
				driveSpeed,
				turnSpeed,
				true
			);
		},
		{ &m_subsystems.robotDriveSub }
	));
	
	// Bind climb up to POV (D-pad) up
	// This also enters climb mode, removing the shooter's control of the intake
	frc2::Trigger([this] { return this->m_driverController.GetPOV() == 0; })
		.WhileTrue(m_subsystems.climbSub.UpCmd())
		.OnTrue(frc2::InstantCommand([this] { this->m_climbMode = true; }).ToPtr());
	// Bind climb down to POV (D-pad) down
	frc2::Trigger([this] { return this->m_driverController.GetPOV() == 180; }).WhileTrue(m_subsystems.climbSub.DownCmd());
	
	// Bind intake climb position to A button
	// This moves the intake into position to lock in the climb arms
	// Will only happen if in climb mode
	m_driverController.A().OnTrue(m_subsystems.intakeSub.m_liftSub.DriveTimedCmd(0.4, 2_s).OnlyIf([this] { return this->m_climbMode; }));

	m_driverController.B().OnTrue(m_subsystems.intakeSub.m_liftSub.ClimbIdleCmd().OnlyIf([this] { return this->m_climbMode; }));


	// ===== Shooter Controls =====

	// Shooter sequence commands
	// A for amp shot (also deploys AmpRamp)
	// B for speaker shot
	// Will not work once climb mode is active
	m_shooterController.A().OnTrue(AmpSequence().Unless([this] { return this->m_climbMode; }));
	m_shooterController.B().OnTrue(SpeakerShotSequence().Unless([this] { return this->m_climbMode; }));

	// Intake sequence commands, left bumper
	// Deploys and intakes while held, retracts when released
	// IMPORTANT NOTE: loses control once climb has begun
	m_shooterController.LeftBumper().OnTrue(std::move(IntakeDeploySequence().Unless([this] { return this->m_climbMode; })));
	m_shooterController.LeftBumper().OnFalse(std::move(IntakeStowSequence().Unless([this] { return this->m_climbMode; })));

	// Exit climb mode, START button
	// Emergency button to get back intake control if climb mode gets entered early
	// Make sure that the climb arms are out of the way if you press this
	m_shooterController.Start().OnTrue(frc2::InstantCommand([this] { this->m_climbMode = false; }).ToPtr());
	
	// AmpRamp controls
	// Y to raise
	// X to lower
	// m_shooterController.Y().OnTrue(m_subsystems.ampRampSub.DeployCmd());
	// m_shooterController.X().OnTrue(m_subsystems.ampRampSub.StowCmd());

	// m_driverController.Start().OnTrue(m_subsystems.ampRampSub.HomeCmd());
	// m_driverController.Back().OnTrue(m_subsystems.intakeSub.m_liftSub.HomeCmd());
}

void RobotContainer::UpdateParams()
{
	if (frc::Preferences::GetDouble(kTeleopSpeedKey, m_maxTeleopSpeed.value()) != m_maxTeleopSpeed.value())
	{
		m_maxTeleopSpeed = units::meters_per_second_t(frc::Preferences::GetDouble(kTeleopSpeedKey));
	}
	
	if (frc::Preferences::GetDouble(kTeleopTurnSpeedKey, m_maxTeleopTurnSpeed.value()) != m_maxTeleopTurnSpeed.value())
	{
		m_maxTeleopTurnSpeed = units::degrees_per_second_t(frc::Preferences::GetDouble(kTeleopTurnSpeedKey));
	}
	
	if (frc::Preferences::GetDouble(kMaxTeleopAccelKey, m_maxTeleopAccel.value()) != m_maxTeleopAccel.value())
	{
		m_maxTeleopAccel = units::meters_per_second_squared_t(frc::Preferences::GetDouble(kMaxTeleopAccelKey));
		m_driveLimiter = frc::SlewRateLimiter<units::meters_per_second>(m_maxTeleopAccel);
	}
	
	if (frc::Preferences::GetDouble(kMaxTeleopTurnAccelKey, m_maxTeleopTurnAccel.value()) != m_maxTeleopTurnAccel.value())
	{
		m_maxTeleopTurnAccel = units::degrees_per_second_squared_t(frc::Preferences::GetDouble(kMaxTeleopTurnAccelKey));
		m_turnLimiter = frc::SlewRateLimiter<units::degrees_per_second>(m_maxTeleopTurnAccel);
	}
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
	// Basic auto sequence to shoot into speaker and drive out of starting zone
	return autos::CompAuto(m_autoChooser.GetSelected());
}