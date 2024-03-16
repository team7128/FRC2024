// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Autos.h"

#include <frc/DriverStation.h>

#include <frc2/command/Commands.h>

#include <vector>

#include "subsystems/Subsystems.h"

#include "commands/ShooterCommands.h"

frc2::CommandPtr autos::BasicAuto(StartLocation startLocation)
{
	Subsystems &subsystems = Subsystems::GetInstance();
	std::vector<frc2::CommandPtr> autoSequence;

	autoSequence.push_back(ShootSequence(1));

	if (startLocation == StartLocation::Center)
	{
		autoSequence.push_back(subsystems.robotDriveSub.DriveDistanceCmd(1.33_m));
	}
	else
	{
		autoSequence.push_back(subsystems.robotDriveSub.DriveDistanceCmd(1.43_m));

		if (startLocation == StartLocation::Left)
		{
			autoSequence.push_back(subsystems.robotDriveSub.TurnByAngleCmd(-60_deg));
		}
		else
		{
			autoSequence.push_back(subsystems.robotDriveSub.TurnByAngleCmd(60_deg));
		}

		autoSequence.push_back(subsystems.robotDriveSub.DriveDistanceCmd(1.43_m));
	}

	return frc2::cmd::Sequence(std::move(autoSequence));
}

frc2::CommandPtr autos::CompAuto(AutoPreset preset)
{
	Subsystems &subsystems = Subsystems::GetInstance();
	std::vector<frc2::CommandPtr> autoSequence;

	if (preset == AutoPreset::None)
		return frc2::cmd::None();

	if (preset == AutoPreset::Mobility)
		return subsystems.robotDriveSub.DriveDistanceCmd(0.8_m);

	if (preset == AutoPreset::SpeakerShot)
		return ShootSequence(1);

	if (preset == AutoPreset::SpeakerCenter)
	{
		autoSequence.push_back(ShootSequence(1));
		autoSequence.push_back(subsystems.robotDriveSub.DriveDistanceCmd(1.33_m));
	}

	if (preset == AutoPreset::SpeakerLeft)
	{
		autoSequence.push_back(ShootSequence(1));
		autoSequence.push_back(subsystems.robotDriveSub.DriveDistanceCmd(0.5_m));
		autoSequence.push_back(subsystems.robotDriveSub.TurnByAngleCmd(-30_deg));
		autoSequence.push_back(subsystems.robotDriveSub.DriveDistanceCmd(1.43_m));
	}

	if (preset == AutoPreset::SpeakerRight)
	{
		autoSequence.push_back(ShootSequence(1));
		autoSequence.push_back(subsystems.robotDriveSub.DriveDistanceCmd(0.5_m));
		autoSequence.push_back(subsystems.robotDriveSub.TurnByAngleCmd(30_deg));
		autoSequence.push_back(subsystems.robotDriveSub.DriveDistanceCmd(1.43_m));
	}
	
	if (preset == AutoPreset::Custom)
	{
		// ===== Set up custom auto here: =====
		// Append to the autoSequence vector



	}

	return frc2::cmd::Sequence(std::move(autoSequence));
}

frc2::CommandPtr autos::TestAuto()
{
	Subsystems &subsystems = Subsystems::GetInstance();
	std::vector<frc2::CommandPtr> autoSequence;

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