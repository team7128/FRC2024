#include "subsystems/Shooter.h"

#include <frc2/command/RunCommand.h>
#include <frc2/command/InstantCommand.h>

#include "Constants.h"

using namespace HardwareConstants;

Shooter::Shooter()
	: m_shooterLeft(kShooterSparkIDs[0], rev::CANSparkMaxLowLevel::MotorType::kBrushless),
	m_shooterRight(kShooterSparkIDs[1], rev::CANSparkMaxLowLevel::MotorType::kBrushless)
{
	m_shooterRight.Follow(m_shooterLeft, true);
}

void Shooter::Enable(double speed)
{
	m_shooterLeft.Set(speed);
}

void Shooter::Disable()
{
	m_shooterLeft.StopMotor();
	m_shooterRight.StopMotor();
}

frc2::CommandPtr Shooter::EnableCmd(double speed)
{
	return this->RunEnd([this, speed] { this->Enable(speed); }, [this] { this->Disable(); });
}

frc2::CommandPtr Shooter::EnableTimedCmd(double speed, units::second_t time)
{
	return this->Run([this, speed] { this->Enable(speed); }).WithTimeout(time).AndThen(std::move(DisableCmd()));
}

frc2::CommandPtr Shooter::DisableCmd()
{
	return this->Run([this] { this->Disable(); });
}