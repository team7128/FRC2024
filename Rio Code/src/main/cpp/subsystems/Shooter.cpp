#include "subsystems/Shooter.h"

#include <frc/Preferences.h>

#include <frc2/command/RunCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>

#include "Constants.h"

using namespace ShooterConstants;

Shooter::Shooter() :
	m_shooterLeft(CANConstants::kShooterSparkIDs[0], rev::CANSparkMaxLowLevel::MotorType::kBrushless),
	m_shooterRight(CANConstants::kShooterSparkIDs[1], rev::CANSparkMaxLowLevel::MotorType::kBrushless),
	m_speakerSpeed(kSpeakerSpeedDefault),
	m_ampSpeed(kAmpSpeedDefault)
{
	// Reversing the shooter so positive shoots the note out
	m_shooterLeft.SetInverted(true);

	// Right shooter NEO follows the left NEO, but inverted
	m_shooterRight.Follow(m_shooterLeft, true);

	frc::Preferences::InitDouble(kSpeakerSpeedKey, m_speakerSpeed);
	frc::Preferences::InitDouble(kAmpSpeedKey, m_ampSpeed);
}

void Shooter::Periodic()
{
	if (frc::Preferences::GetDouble(kSpeakerSpeedKey, m_speakerSpeed) != m_speakerSpeed)
	{
		m_speakerSpeed = frc::Preferences::GetDouble(kSpeakerSpeedKey);
	}
	
	if (frc::Preferences::GetDouble(kAmpSpeedKey, m_ampSpeed) != m_ampSpeed)
	{
		m_ampSpeed = frc::Preferences::GetDouble(kAmpSpeedKey);
	}
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
	return this->Run([this, speed] { this->Enable(speed); }).AndThen(DisableCmd());
}

frc2::CommandPtr Shooter::EnableTimedCmd(double speed, units::second_t time)
{
	return this->Run([this, speed] { this->Enable(speed); }).WithTimeout(time).AndThen(DisableCmd());
}

frc2::CommandPtr Shooter::DisableCmd()
{
	return this->RunOnce([this] { this->Disable(); });
}

frc2::CommandPtr Shooter::EnableSpeakerCmd()
{
	return this->Run([this] { this->Enable(this->m_speakerSpeed); });
}

frc2::CommandPtr Shooter::EnableSpeakerTimedCmd(units::second_t time)
{
	return this->Run([this] { this->Enable(this->m_speakerSpeed); }).WithTimeout(time);
}

frc2::CommandPtr Shooter::EnableAmpCmd()
{
	return this->Run([this] { this->Enable(this->m_ampSpeed); });
}

frc2::CommandPtr Shooter::EnableAmpTimedCmd(units::second_t time)
{
	return this->Run([this] { this->Enable(this->m_ampSpeed); }).WithTimeout(time);
}