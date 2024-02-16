#pragma once

#include <frc/DoubleSolenoid.h>

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>

#include <rev/CANSparkMax.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>

class Intake
{
private:
	class Rollers : public frc2::SubsystemBase
	{
	public:
		Rollers();

		/**
		 * @brief Enables intake rollers
		 * 
		 * @param speed Roller speed. Positive is to intake game piece, negative to feed it out.
		*/
		void Enable(double speed);
		void Disable();

		frc2::CommandPtr EnableTimedCmd(double speed, units::second_t time);
		frc2::CommandPtr DisableCmd();

	private:
		ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_intakeMotor;
	};

	class Deployer : public frc2::SubsystemBase
	{
	public:
		Deployer();

		frc2::CommandPtr DeployCmd();
		frc2::CommandPtr RetractCmd();

	private:
	};
	
public:
	Rollers m_rollerSub;
	Deployer m_deployerSub;
};