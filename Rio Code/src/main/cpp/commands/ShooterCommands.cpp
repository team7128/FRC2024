#include "commands/ShooterCommands.h"

#include <frc2/command/WaitCommand.h>

#include "subsystems/Subsystems.h"

frc2::CommandPtr ShootSequence(double speed)
{
    Subsystems &subsystems = Subsystems::GetInstance();

    return subsystems.shooterSub.EnableTimedCmd(speed, 1.5_s)
		.DeadlineWith(
			frc2::WaitCommand(300_ms).ToPtr().AndThen(
				subsystems.intakeSub.m_rollerSub.EnableCmd(-1.0)
		));
}