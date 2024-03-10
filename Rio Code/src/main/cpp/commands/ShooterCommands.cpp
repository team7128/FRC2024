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

frc2::CommandPtr AmpSequence(double speed)
{
    Subsystems &subsystems = Subsystems::GetInstance();

	// Deploy AmpRamp
	// At the same time, spin up the shooter wheels
	// After a short delay, push the note into the shooter
	// Afterwards, stow the AmpRamp
	return subsystems.ampRampSub.DeployCmd()
		.AlongWith(subsystems.shooterSub.EnableTimedCmd(speed, 2.0_s)
		.DeadlineWith(frc2::WaitCommand(800_ms).ToPtr()
			.AndThen(subsystems.intakeSub.m_rollerSub.EnableCmd(-1.0))))
		.AndThen(subsystems.ampRampSub.StowCmd());
}