#include "subsystems/Intake.h"

#include "Constants.h"

using namespace HardwareConstants;

Intake::Rollers::Rollers()
	: m_intakeMotorTop(3, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
	m_intakeMotorBottom(4, rev::CANSparkMaxLowLevel::MotorType::kBrushless)
{
	m_intakeMotorBottom.Follow(m_intakeMotorTop, true);
}

void Intake::Rollers::Enable(double speed)
{
	m_intakeMotorTop.Set(speed);
}

void Intake::Rollers::Disable()
{
	m_intakeMotorTop.StopMotor();
}

frc2::CommandPtr Intake::Rollers::EnableTimedCmd(double speed, units::second_t time)
{
	return this->Run([this, speed] { this->Enable(speed); }).WithTimeout(time).AndThen(std::move(DisableCmd()));
}

frc2::CommandPtr Intake::Rollers::DisableCmd()
{
	return this->Run([this] { this->Disable(); });
}

Intake::Deployer::Deployer()
	: m_deployerSolenoidLeft(frc::PneumaticsModuleType::CTREPCM, kIntakeSolenoidChannels[0], kIntakeSolenoidChannels[1]),
	m_deployerSolenoidRight(frc::PneumaticsModuleType::CTREPCM, kIntakeSolenoidChannels[2], kIntakeSolenoidChannels[3])
{}

frc2::CommandPtr Intake::Deployer::DeployCmd()
{
	return this->RunOnce([this] {
		this->m_deployerSolenoidLeft.Set(frc::DoubleSolenoid::kForward);
		this->m_deployerSolenoidRight.Set(frc::DoubleSolenoid::kForward);
	});
}

frc2::CommandPtr Intake::Deployer::RetractCmd()
{
	return this->RunOnce([this] {
		this->m_deployerSolenoidLeft.Set(frc::DoubleSolenoid::kReverse);
		this->m_deployerSolenoidRight.Set(frc::DoubleSolenoid::kReverse);
	});
}