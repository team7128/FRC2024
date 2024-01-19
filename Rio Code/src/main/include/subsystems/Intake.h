#pragma once

#include <frc/DoubleSolenoid.h>

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>

#include <rev/CANSparkMax.h>

namespace Intake
{
	class Rollers : public frc2::SubsystemBase
	{
	public:
		Rollers();

		void Enable(double speed);
		void Disable();

		frc2::CommandPtr EnableTimedCmd(double speed, units::second_t time);
		frc2::CommandPtr DisableCmd();

	private:
		rev::CANSparkMax m_intakeMotorTop, m_intakeMotorBottom;
	};

	class Deployer : public frc2::SubsystemBase
	{
	public:
		Deployer();

		frc2::CommandPtr DeployCmd();
		frc2::CommandPtr RetractCmd();

	private:
		frc::DoubleSolenoid m_deployerSolenoidLeft, m_deployerSolenoidRight;
	};
}