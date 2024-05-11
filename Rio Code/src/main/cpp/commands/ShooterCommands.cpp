#include "commands/ShooterCommands.h"

#include <frc2/command/WaitCommand.h>

#include "subsystems/Subsystems.h"

frc2::CommandPtr ShootSequence(double speed)
{
    Subsystems &subsystems = Subsystems::GetInstance();

    return subsystems.shooterSub.EnableTimedCmd(speed, 2_s)
		.DeadlineWith(
			frc2::WaitCommand(subsystems.shooterSub.GetRampTime(speed)).ToPtr().AndThen(
				subsystems.intakeSub.m_rollerSub.EnableCmd(-1.0)
		));
}

frc2::CommandPtr SpeakerShotSequence()
{
    Subsystems &subsystems = Subsystems::GetInstance();

    return subsystems.shooterSub.EnableSpeakerTimedCmd(2_s)
		.DeadlineWith(
			frc2::WaitCommand(1.5_s).ToPtr().AndThen(
				subsystems.intakeSub.m_rollerSub.EnableCmd(-1.0)
		));
}

frc2::CommandPtr AmpSequence()
{
    Subsystems &subsystems = Subsystems::GetInstance();

	// Deploy AmpRamp
	// At the same time, spin up the shooter wheels
	// After a short delay, push the note into the shooter
	// Afterwards, stow the AmpRamp
	return subsystems.ampRampSub.DeployCmd()
		.AlongWith(subsystems.shooterSub.EnableAmpTimedCmd(2.0_s)
			.DeadlineWith(frc2::WaitCommand(500_ms).ToPtr()
				.AndThen(subsystems.intakeSub.m_rollerSub.EnableCmd(-1.0))))
		.AndThen(frc2::WaitCommand(1.5_s).ToPtr())
		.AndThen(subsystems.ampRampSub.StowCmd());
}