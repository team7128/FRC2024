#include "commands/GeneralCommands.h"

#include "subsystems/Subsystems.h"

frc2::CommandPtr HomeAllCmd()
{
	Subsystems &subsystems = Subsystems::GetInstance();

	return subsystems.ampRampSub.HomeCmd()
		.AlongWith(subsystems.intakeSub.m_liftSub.HomeCmd());
}