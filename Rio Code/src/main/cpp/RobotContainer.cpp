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
#include "commands/ExampleCommand.h"

RobotContainer::RobotContainer()
{
	// Initialize all of your commands and subsystems here
	m_driveSubsystem.SetDefaultCommand(frc2::RunCommand([this] {
			m_driveSubsystem.ArcadeDrive(
				-m_driverController.GetLeftY() * OperatorConstants::kMaxTeleopSpeed,
				m_driverController.GetRightX() * OperatorConstants::kMaxTeleopTurnSpeed,
				true
			);
		},
		{ &m_driveSubsystem }
	));

	// Configure the button bindings
	ConfigureBindings();
}

void RobotContainer::ConfigureBindings()
{
	// Configure your trigger bindings here

	// Schedule `ExampleCommand` when `exampleCondition` changes to `true`
	frc2::Trigger([this] {
		return m_subsystem.ExampleCondition();
	}).OnTrue(ExampleCommand(&m_subsystem).ToPtr());

	// Schedule `ExampleMethodCommand` when the Xbox controller's B button is
	// pressed, cancelling on release.
	m_driverController.B().WhileTrue(m_subsystem.ExampleMethodCommand());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
	// Simple testing auto to drive around a bit
	return autos::TestAuto(&m_driveSubsystem, &m_odometry);
}