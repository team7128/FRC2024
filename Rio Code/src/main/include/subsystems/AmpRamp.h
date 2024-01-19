#pragma once

#include <frc/DoubleSolenoid.h>

#include <frc2/command/SubsystemBase.h>

class AmpRamp : public frc2::SubsystemBase
{
public:
	AmpRamp();

	frc2::CommandPtr ExtendCmd();
	frc2::CommandPtr RetractCmd();

private:
	frc::DoubleSolenoid m_rampSolenoidLeft, m_rampSolenoidRight;
};