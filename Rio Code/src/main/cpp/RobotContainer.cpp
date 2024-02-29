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

RobotContainer::RobotContainer()
	: m_subsystems(Subsystems::GetInstance())
{
	// Set shooter to not run when not in use
	m_subsystems.shooterSub.SetDefaultCommand(m_subsystems.shooterSub.DisableCmd());

	m_subsystems.intakeSub.m_rollerSub.SetDefaultCommand(m_subsystems.intakeSub.m_rollerSub.DisableCmd());

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

	m_subsystems.intakeSub.m_liftSub.SetDefaultCommand(frc2::RunCommand([this] {
			m_subsystems.intakeSub.m_liftSub.DriveRaw(-m_driverController.GetRightY());
		},
		{ &m_subsystems.intakeSub.m_liftSub }
	));

	// Configure your trigger bindings here

	m_driverController.A().WhileTrue(m_subsystems.shooterSub.EnableCmd(0.15));
	m_driverController.B().WhileTrue(m_subsystems.shooterSub.EnableCmd(1.0));

	m_driverController.LeftBumper().WhileTrue(m_subsystems.intakeSub.m_rollerSub.EnableCmd(-1.0));
	m_driverController.RightBumper().WhileTrue(m_subsystems.intakeSub.m_rollerSub.EnableCmd(1.0));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
	// Simple testing auto to drive around a bit
	return autos::TestAuto();
}