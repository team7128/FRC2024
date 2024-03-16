// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <cameraserver/CameraServer.h>

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/filter/SlewRateLimiter.h>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include "commands/Autos.h"
#include "Constants.h"
#include "subsystems/Subsystems.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer
{
public:
	RobotContainer();

	void Reset();

	frc2::CommandPtr GetAutonomousCommand();

	/// Contains all subsystems to easily pass into other sections of the code
	Subsystems &m_subsystems;

private:
	frc2::CommandXboxController m_driverController{
		OperatorConstants::kDriverControllerPort
	}, m_shooterController{
		OperatorConstants::kShooterControllerPort
	};

	/// @brief Keeps track of whether we are doing climb
	bool m_climbMode = false;

	frc2::Trigger m_climbModeTrigger;

	frc::SendableChooser<autos::AutoPreset> m_autoChooser;

	frc::SlewRateLimiter<units::meters_per_second> m_driveLimiter;
	frc::SlewRateLimiter<units::degrees_per_second> m_turnLimiter;

	/// @brief Configures all controller bindings
	void ConfigureBindings();
};
