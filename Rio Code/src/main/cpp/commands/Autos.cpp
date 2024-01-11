// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Autos.h"

#include <frc2/command/Commands.h>

#include "commands/ExampleCommand.h"

#include <vector>

frc2::CommandPtr autos::ExampleAuto(ExampleSubsystem* subsystem)
{
	std::vector<frc2::CommandPtr> autoSequence;

	auto autoCmd = subsystem->ExampleMethodCommand();

	autoSequence.push_back(std::move(autoCmd));

	return frc2::cmd::Sequence(std::move(autoSequence));

	return frc2::cmd::Sequence(subsystem->ExampleMethodCommand(),
		ExampleCommand(subsystem).ToPtr());
}
