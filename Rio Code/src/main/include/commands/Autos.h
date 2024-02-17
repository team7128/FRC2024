// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>

namespace autos {
	// Initialize (home) all subsystems
	frc2::CommandPtr InitAuto();

	frc2::CommandPtr TestAuto();
}  // namespace autos
