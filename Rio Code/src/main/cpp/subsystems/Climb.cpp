#include "subsystems/Climb.h"

#include "Constants.h"

#include <frc/Preferences.h>

using namespace ClimbConstants;

Climb::Climb() :
    m_leftClimbMotor(CANConstants::kClimbVictorIDs[0]),
    m_rightClimbMotor(CANConstants::kClimbVictorIDs[1]),
	m_climbSpeed(kClimbSpeedDefault)
{
    m_leftClimbMotor.SetInverted(true);

	frc::Preferences::InitDouble(kCLimbSpeedKey, m_climbSpeed);
}

void Climb::Periodic()
{
	if (frc::Preferences::GetDouble(kCLimbSpeedKey, m_climbSpeed) != m_climbSpeed)
	{
		m_climbSpeed = frc::Preferences::GetDouble(kCLimbSpeedKey);
	}
}

void Climb::Drive(double speed)
{
    m_rightClimbMotor.Set(speed);
	m_leftClimbMotor.Set(speed);
}

frc2::CommandPtr Climb::DriveCmd(double speed)
{
	return this->Run([this, speed] { this->Drive(speed); });
}

frc2::CommandPtr Climb::UpCmd()
{
	return DriveCmd(m_climbSpeed);
}

frc2::CommandPtr Climb::DownCmd()
{
	return DriveCmd(-m_climbSpeed);
}

frc2::CommandPtr Climb::StopCmd()
{
    return this->Run([this] {
        this->m_leftClimbMotor.StopMotor();
        this->m_rightClimbMotor.StopMotor();
    });
}