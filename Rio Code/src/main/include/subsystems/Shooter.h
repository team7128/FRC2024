#pragma once

#include <rev/CANSparkMax.h>

#include <frc2/command/SubsystemBase.h>

class Shooter : public frc2::SubsystemBase
{
public:
	Shooter();

	void Enable(double speed);
	void Disable();

	frc2::CommandPtr EnableCmd(double speed);
	frc2::CommandPtr EnableTimedCmd(double speed, units::second_t time);
	frc2::CommandPtr DisableCmd();

private:
	rev::CANSparkMax m_shooterLeft, m_shooterRight;	
};