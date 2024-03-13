// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>

namespace autos {
	enum StartLocation
	{
		Center,
		Left,
		Right
	};

	enum AutoPreset
	{
		None,
		Mobility,
		SpeakerCenter,
		SpeakerLeft,
		SpeakerRight,
		Custom
	};

	frc2::CommandPtr BasicAuto(StartLocation);

	frc2::CommandPtr CompAuto(AutoPreset preset);

	frc2::CommandPtr TestAuto();
}  // namespace autos
