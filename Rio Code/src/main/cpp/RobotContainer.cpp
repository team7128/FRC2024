// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/shuffleboard/Shuffleboard.h>

#include <frc2/command/CommandScheduler.h>
#include <frc2/command/button/Trigger.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/InstantCommand.h>

#include "commands/Autos.h"
#include "commands/ShooterCommands.h"
#include "commands/IntakeCommands.h"

RobotContainer::RobotContainer()
	: m_subsystems(Subsystems::GetInstance())
{
	// Set shooter to not run when not in use
	m_subsystems.shooterSub.SetDefaultCommand(m_subsystems.shooterSub.DisableCmd());
	
	// m_subsystems.ampRampSub.SetDefaultCommand(m_subsystems.ampRampSub.StopCmd());

	m_subsystems.intakeSub.m_rollerSub.SetDefaultCommand(m_subsystems.intakeSub.m_rollerSub.DisableCmd());

	// m_subsystems.intakeSub.m_liftSub.SetDefaultCommand(m_subsystems.intakeSub.m_liftSub.DisableCmd());

	m_subsystems.climbSub.SetDefaultCommand(m_subsystems.climbSub.StopCmd());

	m_climbModeTrigger = frc2::Trigger([this] { return this->m_climbMode; });

	// Extend the intake when beginning climb
	m_climbModeTrigger.OnTrue(m_subsystems.intakeSub.m_liftSub.DeployCmd());
	// Pull in the intake if we ever exit climb mode
	m_climbModeTrigger.OnFalse(m_subsystems.intakeSub.m_liftSub.StowCmd());

	// Configure the button bindings
	ConfigureBindings();
}

void RobotContainer::ConfigureBindings()
{
	// ----- Driver Controls -----
	m_subsystems.robotDriveSub.SetDefaultCommand(frc2::RunCommand([this] {
			m_subsystems.robotDriveSub.ArcadeDrive(
				-m_driverController.GetLeftY() * OperatorConstants::kMaxTeleopSpeed,
				-m_driverController.GetRightX() * OperatorConstants::kMaxTeleopTurnSpeed,
				true
			);
		},
		{ &m_subsystems.robotDriveSub }
	));
	
	// Bind climb up to POV (D-pad) up
	// This also enters climb mode, removing the shooter's control of the intake
	frc2::Trigger([this] { return this->m_driverController.GetPOV() == 0; })
		.WhileTrue(m_subsystems.climbSub.DriveCmd(0.35))
		.OnTrue(frc2::InstantCommand([this] { this->m_climbMode = true; }).ToPtr());
	// Bind climb down to POV (D-pad) down
	frc2::Trigger([this] { return this->m_driverController.GetPOV() == 180; }).WhileTrue(m_subsystems.climbSub.DriveCmd(-0.35));


	// ----- Shooter Controls -----

	// Shooter sequence commands
	// A for amp shot (also deploys AmpRamp)
	// B for speaker shot
	m_shooterController.A().OnTrue(AmpSequence(0.12));
	m_shooterController.B().OnTrue(ShootSequence(1.0));

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

frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
	// Simple testing auto to drive around a bit
	return autos::BasicAuto();
}