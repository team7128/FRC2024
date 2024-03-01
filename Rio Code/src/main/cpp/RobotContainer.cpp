// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/CommandScheduler.h>
#include <frc2/command/button/Trigger.h>

#include "commands/Autos.h"
#include "commands/ExampleCommand.h"

<<<<<<< Updated upstream
RobotContainer::RobotContainer() {
	// Initialize all of your commands and subsystems here
	m_driveSubsystem.SetDefaultCommand(m_driveSubsystem.ArcadeDrive(m_driverController));
=======
RobotContainer::RobotContainer()
	: m_subsystems(Subsystems::GetInstance())
{
	// Set shooter to not run when not in use
	m_subsystems.shooterSub.SetDefaultCommand(m_subsystems.shooterSub.DisableCmd());
	m_subsystems.ampRampSub.SetDefaultCommand(m_subsystems.ampRampSub.StopCmd());
	m_subsystems.intakeSub.m_rollerSub.SetDefaultCommand(m_subsystems.intakeSub.m_rollerSub.DisableCmd());
	//m_subsystems.climb.SetDefaultCommand(m_subsystems.climb.StopCmd());
>>>>>>> Stashed changes

	// Configure the button bindings
	ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
	// Configure your trigger bindings here

	// Schedule `ExampleCommand` when `exampleCondition` changes to `true`
	frc2::Trigger([this] {
		return m_subsystem.ExampleCondition();
	}).OnTrue(ExampleCommand(&m_subsystem).ToPtr());

<<<<<<< Updated upstream
	// Schedule `ExampleMethodCommand` when the Xbox controller's B button is
	// pressed, cancelling on release.
	m_driverController.B().WhileTrue(m_subsystem.ExampleMethodCommand());
=======
	m_driverController.LeftBumper().WhileTrue(m_subsystems.intakeSub.m_rollerSub.EnableCmd(-1.0));
	m_driverController.RightBumper().WhileTrue(m_subsystems.intakeSub.m_rollerSub.EnableCmd(1.0));

	/*m_driverController.POVUp().WhileTrue(m_subsystems.climbSub.EnableCmd(-1.0));
	m_driverController.POVDown().WhileTrue(m_subsystems.climbSub.EnableCmd(1.0));*/
	

	m_driverController.Y().WhileTrue(frc2::RunCommand([this] {
			this->m_subsystems.ampRampSub.Drive(0.4);
	}, { &m_subsystems.ampRampSub }).ToPtr());
	m_driverController.X().WhileTrue(frc2::RunCommand([this] {
			this->m_subsystems.ampRampSub.Drive(-0.4);
	}, { &m_subsystems.ampRampSub }).ToPtr());
>>>>>>> Stashed changes
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
	// An example command will be run in autonomous
	return autos::ExampleAuto(&m_subsystem);
}
