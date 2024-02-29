#include "subsystems/Climb.h"

#include "Constants.h"

Climb::Climb() :
    m_leftClimbMotor(CANConstants::kClimbVictorIDs[0]),
    m_rightClimbMotor(CANConstants::kClimbVictorIDs[1])
{
    m_rightClimbMotor.SetInverted(true);
}

void Climb::Drive(double leftSpeed, double rightSpeed)
{
    m_leftClimbMotor.Set(leftSpeed);
    m_rightClimbMotor.Set(rightSpeed);
}

frc2::CommandPtr Climb::StopCmd()
{
    return this->Run([this] {
        this->m_leftClimbMotor.StopMotor();
        this->m_rightClimbMotor.StopMotor();
    });
}