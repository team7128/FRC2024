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

	/**
	 * @brief Deploys the amp ramp.
	 * Creates a command to raise the amp ramp.
	 * 
	 * @return Deploy command.
	 */
	frc2::CommandPtr DeployCmd();
	/**
	 * @brief Stows the amp ramp.
	 * Creates a command to lower the amp ramp.
	 * 
	 * @return Stow command.
	 */
	frc2::CommandPtr StowCmd();

	/**
	 * @brief Homes the amp ramp.
	 * Creates a command to home the amp ramp.
	 * Called automatically at the start of the match.
	 * Takes priority over all other amp commands.
	 * 
	 * @return Home command. 
	 */
	frc2::CommandPtr HomeCmd();

private:
	/// Limit switch used for homing
	frc::DigitalInput m_limitSwitch;

	/// Talon motor controller for the VersaPlanetary that drives the amp ramp
	ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_motorController;

	/**
	 * @brief Moves the amp ramp to a specific angle.
	 * 
	 * @param angle Target angle.
	 * @param deadzone Acceptable error to stop at.
	 * @return Go to angle command.
	 */
	frc2::CommandPtr GoToAngleCmd(units::degree_t angle, units::degree_t deadzone);
};