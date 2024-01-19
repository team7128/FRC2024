#include "subsystems/AmpRamp.h"

#include <frc2/command/InstantCommand.h>

#include "Constants.h"

using namespace HardwareConstants;

AmpRamp::AmpRamp()
	: m_rampSolenoidLeft(frc::PneumaticsModuleType::CTREPCM, kAmpSolenoidChannels[0], kAmpSolenoidChannels[1]),
	m_rampSolenoidRight(frc::PneumaticsModuleType::CTREPCM, kAmpSolenoidChannels[2], kAmpSolenoidChannels[3])
{}

frc2::CommandPtr AmpRamp::ExtendCmd()
{
	return this->RunOnce([this] {
		this->m_rampSolenoidLeft.Set(frc::DoubleSolenoid::kForward);
		this->m_rampSolenoidRight.Set(frc::DoubleSolenoid::kForward);
	});
}

frc2::CommandPtr AmpRamp::RetractCmd()
{
	return this->RunOnce([this] {
		this->m_rampSolenoidLeft.Set(frc::DoubleSolenoid::kReverse);
		this->m_rampSolenoidRight.Set(frc::DoubleSolenoid::kReverse);
	});
}