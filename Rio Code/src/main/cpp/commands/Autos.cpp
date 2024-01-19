// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc2/command/Commands.h>

#include <vector>

#include "commands/Autos.h"
#include "commands/autoCommands/DriveAutos.h"

#include "commands/ExampleCommand.h"

frc2::CommandPtr autos::ExampleAuto(ExampleSubsystem* subsystem)
{
	std::vector<frc2::CommandPtr> autoSequence;

	auto autoCmd = subsystem->ExampleMethodCommand();

	autoSequence.push_back(std::move(autoCmd));

	return frc2::cmd::Sequence(std::move(autoSequence));
}

frc2::CommandPtr autos::TestAuto(RobotDrive *driveSubsystem, Odometry *odometrySubsystem)
{
	DriveAutos driveAutos(driveSubsystem, odometrySubsystem);

	std::vector<frc2::CommandPtr> autoSequence;

	autoSequence.push_back(std::move(driveAutos.DriveDistance(4_m)));
	// autoSequence.push_back(std::move(driveAutos.TurnByAngle(-90_deg)));
	// autoSequence.push_back(std::move(driveAutos.DriveDistance(1_m)));
	// autoSequence.push_back(std::move(driveAutos.FacePoint(0_m, 0_m)));
	// autoSequence.push_back(std::move(driveAutos.DriveDistance(units::meter_t(std::sqrt(5)))));
	// autoSequence.push_back(std::move(driveAutos.GoToPoint(3_m, 0_m)));
	// autoSequence.push_back(std::move(driveAutos.GoToPoint(1_m, 1_m)));

	return frc2::cmd::Sequence(std::move(autoSequence));
}