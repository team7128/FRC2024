// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Autos.h"

#include <frc2/command/Commands.h>

#include <vector>

#include "subsystems/Subsystems.h"

#include "commands/ShooterCommands.h"

frc2::CommandPtr autos::InitAuto()
{
	Subsystems &subsystems = Subsystems::GetInstance();
	std::vector<frc2::CommandPtr> initCommands;

	initCommands.push_back(subsystems.ampRampSub.HomeCmd());
	initCommands.push_back(subsystems.intakeSub.m_liftSub.HomeCmd());

	return frc2::cmd::Parallel(std::move(initCommands));
}

frc2::CommandPtr autos::BasicAuto()
{
	Subsystems &subsystems = Subsystems::GetInstance();
	std::vector<frc2::CommandPtr> autoSequence;
	// autoSequence.push_back(InitAuto());

	autoSequence.push_back(ShootSequence(1.0));
	autoSequence.push_back(subsystems.robotDriveSub.DriveDistanceCmd(2.5_m));

	return frc2::cmd::Sequence(std::move(autoSequence));
}

frc2::CommandPtr autos::TestAuto()
{
	Subsystems &subsystems = Subsystems::GetInstance();
	std::vector<frc2::CommandPtr> autoSequence;
	autoSequence.push_back(InitAuto());

	// autoSequence.push_back(subsystems.robotDriveSub.DriveDistanceCmd(2_m));
	// autoSequence.push_back(subsystems.robotDriveSub.TurnByAngleCmd(90_deg));

	// autoSequence.push_back(subsystems.robotDriveSub.DriveDistanceCmd(3_m));
	// autoSequence.push_back(subsystems.robotDriveSub.DriveDistanceCmd(-3_m));

	// autoSequence.push_back(subsystems.robotDriveSub.GoToPointCmd(2_m, 0_m));
	// autoSequence.push_back(subsystems.robotDriveSub.GoToPointCmd(0_m, 0_m));
	// autoSequence.push_back(subsystems.robotDriveSub.GoToPointCmd(2_m, 0_m));
	// autoSequence.push_back(subsystems.robotDriveSub.GoToPointCmd(0_m, 0_m));
	// autoSequence.push_back(subsystems.robotDriveSub.FacePointCmd(2_m, 0_m));

	autoSequence.push_back(subsystems.robotDriveSub.TurnByAngleCmd(30_deg));
	autoSequence.push_back(subsystems.robotDriveSub.DriveDistanceCmd(8_m));
	autoSequence.push_back(subsystems.robotDriveSub.TurnByAngleCmd(-30_deg));
	autoSequence.push_back(subsystems.robotDriveSub.DriveDistanceCmd(3_m));
	autoSequence.push_back(subsystems.robotDriveSub.FacePointCmd(9_m, 7_m));
	autoSequence.push_back(subsystems.robotDriveSub.DriveDistanceCmd(2_m));
	autoSequence.push_back(subsystems.robotDriveSub.GoToPointCmd(13_m, 4_m));
	autoSequence.push_back(subsystems.robotDriveSub.GoToPointCmd(11_m, 9_m));

	// autoSequence.push_back(subsystems.robotDriveSub.GoToPointCmd(9_m, 4_m, 2_m));
	// autoSequence.push_back(subsystems.robotDriveSub.GoToPointCmd(13_m, 6_m));
	// autoSequence.push_back(subsystems.robotDriveSub.GoToPointCmd(9_m, 4_m, 2_m));
	// autoSequence.push_back(subsystems.robotDriveSub.GoToPointCmd(11_m, 1_m));
	// autoSequence.push_back(subsystems.robotDriveSub.GoToPointCmd(9_m, 4_m, 2_m));
	// autoSequence.push_back(subsystems.robotDriveSub.GoToPointCmd(3_m, 3_m));
	// autoSequence.push_back(subsystems.robotDriveSub.GoToPointCmd(9_m, 4_m, 2_m));

	return frc2::cmd::Sequence(std::move(autoSequence));
}