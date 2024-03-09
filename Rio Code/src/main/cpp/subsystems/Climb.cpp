#include "subsystems/Climb.h"

#include "Constants.h"

Climb::Climb() :
    m_leftClimbMotor(CANConstants::kClimbVictorIDs[0]),
    m_rightClimbMotor(CANConstants::kClimbVictorIDs[1])
{
    m_leftClimbMotor.SetInverted(true);
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

frc2::CommandPtr Climb::StopCmd()
{
    return this->Run([this] {
        this->m_leftClimbMotor.StopMotor();
        this->m_rightClimbMotor.StopMotor();
    });
}