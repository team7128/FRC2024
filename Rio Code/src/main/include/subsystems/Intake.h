#pragma once

#include <optional>

#include <frc/Encoder.h>
#include <frc/DigitalInput.h>
#include <frc/controller/ArmFeedforward.h>

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/ProfiledPIDSubsystem.h>
#include <frc2/command/CommandPtr.h>

#include <rev/CANSparkMax.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>
#include <ctre/phoenix/motorcontrol/can/WPI_VictorSPX.h>

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

		frc2::CommandPtr EnableCmd(double speed);
		frc2::CommandPtr EnableTimedCmd(double speed, units::second_t time);
		frc2::CommandPtr DisableCmd();

	private:
		ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_intakeMotor;
	};

	class Lift : public frc2::ProfiledPIDSubsystem<units::degrees>
	{
		using State = frc::TrapezoidProfile<units::degrees>::State;

	public:
		Lift();

		/**
		 * @brief Manually drives the intake
		 * 
		 * @param speed Intake speed. Positive is deploy, negative is stow.
		 */
		void Drive(double speed);

		/**
		 * @brief Automatically moves the intake to deploy position
		 * 
		 * @return Deploy command
		 */
		frc2::CommandPtr DeployCmd();
		/**
		 * @brief Automatically moves the intake to stow position
		 * 
		 * @return Stow command
		 */
		frc2::CommandPtr StowCmd();
		/**
		 * @brief Automatically moves intake to climb position.
		 * Moves the intake into the climb position and disables automatic PID control.
		 * 
		 * @return Climb command 
		 */
		frc2::CommandPtr ClimbCmd();
		/**
		 * @brief Stops the intake from moving
		 * 
		 * @return Disable command 
		 */
		frc2::CommandPtr DisableCmd();
		/**
		 * @brief Homes the intake.
		 * IMPORTANT: This must run first before any other intake functions.
		 * 
		 * @return Home command 
		 */
		frc2::CommandPtr HomeCmd();

		virtual units::degree_t GetMeasurement() override;
		virtual void UseOutput(double output, State setpoint) override;

	private:
		ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_liftMotor;
		frc::Encoder m_encoder;
		frc::DigitalInput m_limitSwitch;
	};
	
public:
	Rollers m_rollerSub;
	Lift m_liftSub;
};