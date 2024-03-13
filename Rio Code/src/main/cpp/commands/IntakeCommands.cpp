#include "commands/IntakeCommands.h"

#include <frc2/command/StartEndCommand.h>
#include <frc2/command/WaitCommand.h>

#include "subsystems/Subsystems.h"

frc2::CommandPtr IntakeDeploySequence()
{
	Subsystems &subsystems = Subsystems::GetInstance();

	return subsystems.intakeSub.m_liftSub.DeployCmd()
		.AndThen(subsystems.intakeSub.m_rollerSub.EnableCmd(1.0));
}

frc2::CommandPtr IntakeStowSequence()
{
	Subsystems &subsystems = Subsystems::GetInstance();

	return subsystems.intakeSub.m_liftSub.StowCmd()
		.AlongWith(frc2::WaitCommand(400_ms).ToPtr()
			.AndThen(subsystems.intakeSub.m_rollerSub.EnableTimedCmd(0.7, 400_ms)));
}