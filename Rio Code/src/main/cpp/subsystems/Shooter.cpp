#include "subsystems/Shooter.h"

#include <frc2/command/RunCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>

#include "Constants.h"

Shooter::Shooter() :
	m_shooterLeft(CANConstants::kShooterSparkIDs[0], rev::CANSparkMaxLowLevel::MotorType::kBrushless),
	m_shooterRight(CANConstants::kShooterSparkIDs[1], rev::CANSparkMaxLowLevel::MotorType::kBrushless)
{
	// Reversing the shooter so positive shoots the note out
	m_shooterLeft.SetInverted(true);

	// Right shooter NEO follows the left NEO, but inverted
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
	return this->Run([this, speed] { this->Enable(speed); });
}

frc2::CommandPtr Shooter::EnableTimedCmd(double speed, units::second_t time)
{
	return this->Run([this, speed] { this->Enable(speed); }).WithTimeout(time);
}

frc2::CommandPtr Shooter::DisableCmd()
{
	return this->Run([this] { this->Disable(); });
}