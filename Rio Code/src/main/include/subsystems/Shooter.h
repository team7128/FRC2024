#pragma once

#include <rev/CANSparkMax.h>

#include <frc2/command/SubsystemBase.h>

class Shooter : public frc2::SubsystemBase
{
public:
	Shooter();

	/**
	 * @brief Runs the shooter wheels.
	 * 
	 * @param speed Speed to run the shooter wheels at.
	 */
	void Enable(double speed);
	/**
	 * @brief Disables the shooter wheels.
	 * Wheels are set to coast when disabled.
	 */
	void Disable();

	/**
	 * @brief Runs the shooter wheels.
	 * Creates a command to run the shooter wheels at a set speed.
	 * 
	 * @param speed Speed to run the shooter wheels at.
	 * @return Command to run the wheels. 
	 */
	frc2::CommandPtr EnableCmd(double speed);
	/**
	 * @brief Enables the shooter wheels for a set time.
	 * 
	 * @param speed Speed to run the shooter wheels at.
	 * @param time How long to run the shooter for.
	 * @return Command to run the wheels.
	 */
	frc2::CommandPtr EnableTimedCmd(double speed, units::second_t time);
	/**
	 * @brief Disables the shooter wheels.
	 * Wheels are set to coast when disabled.
	 * 
	 * @return Command to disable the wheels.
	 */
	frc2::CommandPtr DisableCmd();

	frc2::CommandPtr EnableSpeakerCmd();
	frc2::CommandPtr EnableSpeakerTimedCmd(units::second_t time);
	frc2::CommandPtr EnableAmpCmd();
	frc2::CommandPtr EnableAmpTimedCmd(units::second_t time);

	double GetRampTime(double speed);

private:
	/// Spark MAXes to run the shooter NEOs
	rev::CANSparkMax m_shooterLeft, m_shooterRight;	

	double m_speakerSpeed, m_ampSpeed;
	double m_rampTime;

	void UpdateParams();
};