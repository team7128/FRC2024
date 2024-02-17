#pragma once

#include <memory>

#include "RobotDrive.h"
#include "Shooter.h"
#include "Intake.h"
#include "AmpRamp.h"

struct Subsystems
{
public:
	static Subsystems& GetInstance();

	RobotDrive robotDriveSub;
	Shooter shooterSub;
	Intake intakeSub;
	AmpRamp ampRampSub;
};