#pragma once

#include <optional>

#include <frc/DoubleSolenoid.h>
#include <frc/DigitalInput.h>

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>

#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>

#include <units/angle.h>

class AmpRamp : public frc2::SubsystemBase
{
public:
	AmpRamp();

	void Drive(double speed);
	frc2::CommandPtr StopCmd();

	frc2::CommandPtr DeployCmd();
	frc2::CommandPtr StowCmd();

	frc2::CommandPtr HomeCmd();

private:
	frc::DigitalInput m_limitSwitch;
	ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_motorController;
	/// @brief Holds the homing command that will run at the start of the match
	std::optional<frc2::CommandPtr> m_homeCmd;

	frc2::CommandPtr GoToAngleCmd(units::degree_t angle, units::degree_t deadzone);
};