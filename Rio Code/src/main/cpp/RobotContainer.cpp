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
	
	m_subsystems.ampRampSub.SetDefaultCommand(m_subsystems.ampRampSub.StopCmd());

	m_subsystems.intakeSub.m_rollerSub.SetDefaultCommand(m_subsystems.intakeSub.m_rollerSub.DisableCmd());

	// m_subsystems.intakeSub.m_liftSub.SetDefaultCommand(m_subsystems.intakeSub.m_liftSub.DisableCmd());

	m_subsystems.climbSub.SetDefaultCommand(m_subsystems.climbSub.StopCmd());

	// Configure the button bindings
	ConfigureBindings();
}

void RobotContainer::ConfigureBindings()
{
	// Initialize all of your commands and subsystems here
	m_subsystems.robotDriveSub.SetDefaultCommand(frc2::RunCommand([this] {
			m_subsystems.robotDriveSub.ArcadeDrive(
				-m_driverController.GetLeftY() * OperatorConstants::kMaxTeleopSpeed,
				-m_driverController.GetRightX() * OperatorConstants::kMaxTeleopTurnSpeed,
				true
			);
		},
		{ &m_subsystems.robotDriveSub }
	));

	// Configure your trigger bindings here

	// m_driverController.A().WhileTrue(m_subsystems.shooterSub.EnableCmd(0.1));
	// m_driverController.B().WhileTrue(m_subsystems.shooterSub.EnableCmd(1.0));

	// Experimental shooter sequence commands
	m_driverController.A().OnTrue(std::move(ShootSequence(0.12)));
	m_driverController.B().OnTrue(std::move(ShootSequence(1.0)));

	// m_driverController.LeftBumper().WhileTrue(m_subsystems.intakeSub.m_rollerSub.EnableCmd(1.0));
	// m_driverController.RightBumper().WhileTrue(m_subsystems.intakeSub.m_rollerSub.EnableCmd(-1.0));

	// m_driverController.LeftTrigger().WhileTrue(frc2::RunCommand([this] { this->m_subsystems.intakeSub.m_liftSub.Drive(-this->m_driverController.GetLeftTriggerAxis()); }, { &m_subsystems.intakeSub.m_liftSub}).ToPtr());
	// m_driverController.RightTrigger().WhileTrue(frc2::RunCommand([this] { this->m_subsystems.intakeSub.m_liftSub.Drive(this->m_driverController.GetRightTriggerAxis()); }, { &m_subsystems.intakeSub.m_liftSub}).ToPtr());

	// m_driverController.LeftTrigger().OnTrue(m_subsystems.intakeSub.m_liftSub.StowCmd());
	// m_driverController.RightTrigger().OnTrue(m_subsystems.intakeSub.m_liftSub.DeployCmd());

	// Experimental intake sequence commands
	m_driverController.LeftBumper().OnTrue(std::move(IntakeDeploySequence()));
	m_driverController.LeftBumper().OnFalse(std::move(IntakeStowSequence()));

	frc2::Trigger([this] { return this->m_driverController.GetPOV() == 0; }).WhileTrue(m_subsystems.climbSub.DriveCmd(0.35));
	frc2::Trigger([this] { return this->m_driverController.GetPOV() == 180; }).WhileTrue(m_subsystems.climbSub.DriveCmd(-0.35));
	
	/*m_driverController.Y().WhileTrue(frc2::RunCommand([this] {
			this->m_subsystems.ampRampSub.Drive(0.4);
	}, { &m_subsystems.ampRampSub }).ToPtr());
	m_driverController.X().WhileTrue(frc2::RunCommand([this] {
			this->m_subsystems.ampRampSub.Drive(-0.4);
	}, { &m_subsystems.ampRampSub }).ToPtr());*/

	m_driverController.Y().OnTrue(m_subsystems.ampRampSub.DeployCmd());
	m_driverController.X().OnTrue(m_subsystems.ampRampSub.StowCmd());

	m_driverController.Start().OnTrue(m_subsystems.ampRampSub.HomeCmd());
	m_driverController.Back().OnTrue(m_subsystems.intakeSub.m_liftSub.HomeCmd());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
	// Simple testing auto to drive around a bit
	return autos::BasicAuto();
}