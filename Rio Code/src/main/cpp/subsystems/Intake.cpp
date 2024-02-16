#include "subsystems/Intake.h"

#include "Constants.h"

using namespace HardwareConstants;

Intake::Rollers::Rollers() :
	m_intakeMotor(kIntakeRollerTalonID)
{}

void Intake::Rollers::Enable(double speed)
{
	m_intakeMotor.Set(speed);
}

void Intake::Rollers::Disable()
{
	m_intakeMotor.StopMotor();
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
{}

frc2::CommandPtr Intake::Deployer::DeployCmd()
{
	return this->RunOnce([this] {
	});
}

frc2::CommandPtr Intake::Deployer::RetractCmd()
{
	return this->RunOnce([this] {
	});
}