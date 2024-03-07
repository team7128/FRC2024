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

		void DriveRaw(double speed);

		frc2::CommandPtr DisableCmd();
		frc2::CommandPtr DeployCmd();
		frc2::CommandPtr StowCmd();
		frc2::CommandPtr HomeCmd();

		virtual units::degree_t GetMeasurement() override;
		virtual void UseOutput(double output, State setpoint) override;

	private:
		ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_liftMotor;
		frc::Encoder m_encoder;
		frc::DigitalInput m_limitSwitch;
		frc::ArmFeedforward m_feedforward;
		/// @brief Holds the home command that runs at the start of the match
		std::optional<frc2::CommandPtr> m_homeCmd;
	};
	
public:
	Rollers m_rollerSub;
	Lift m_liftSub;
};