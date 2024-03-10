#include "subsystems/AmpRamp.h"

#include <frc/shuffleboard/Shuffleboard.h>

#include <frc2/command/InstantCommand.h>

#include "Constants.h"

using namespace AmpRampConstants;

AmpRamp::AmpRamp() :
	m_limitSwitch(DIOConstants::kAmpRampSwitchPort),
	m_motorController(CANConstants::kAmpRampTalonID)
{
	// Set the Talon to use the VersaPlanetary encoder
	m_motorController.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder);
	m_motorController.Config_kP(0, kP);

	// AmpRamp encoder count shuffleboard
	auto &tab = frc::Shuffleboard::GetTab("Main");
	tab.AddNumber("AmpRamp encoder value", [this] {
		return this->m_motorController.GetSelectedSensorPosition();
	});
	// adds AmpRamp encoder angle to shuffleboard
	tab.AddNumber("AmpRamp encoder angle", [this] {
		return this->m_motorController.GetSelectedSensorPosition() / HardwareConstants::kVersaEncoderCPR * 360.f;
	});
	tab.Add("AmpRamp limit switch", m_limitSwitch);
}

void AmpRamp::Drive(double speed)
{
	m_motorController.Set(speed);
}

frc2::CommandPtr AmpRamp::StopCmd()
{
	return this->Run([this] { this->m_motorController.StopMotor(); });
}

//deploy AmpRamp
frc2::CommandPtr AmpRamp::DeployCmd()
{
	return GoToAngleCmd(kDeployAngle, 2_deg);
}

//pull in AmpRamp
frc2::CommandPtr AmpRamp::StowCmd()
{
	return GoToAngleCmd(kStowAngle, 2_deg);
}

//move AmpRamp to "home", reset encoder
frc2::CommandPtr AmpRamp::HomeCmd()
{
	return this->Run([this] { this->m_motorController.Set(khomeSpeed); })	// Back away from the switch in case it is already pressed
		.Until([this] { return this->m_limitSwitch.Get(); })
		.AndThen(this->Run([this] { this->m_motorController.Set(-khomeSpeed); })	// Lower until the limit switch is hit
		.Until([this] { return !this->m_limitSwitch.Get(); }))
		.AndThen([this] {
			this->m_motorController.StopMotor();
			this->m_motorController.SetSelectedSensorPosition(kHomeAngle / 360_deg * HardwareConstants::kVersaEncoderCPR);
	});
	//.WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelIncoming);	// Ensure that the home command has priority over other commands
}

frc2::CommandPtr AmpRamp::GoToAngleCmd(units::degree_t angle, units::degree_t deadzone)
{
	return this->RunOnce([this, angle] {
		this->m_motorController.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::Position, angle / 360_deg * HardwareConstants::kVersaEncoderCPR);
	});
	//.Until([this, deadzone] { return std::abs(this->m_motorController.GetClosedLoopError()) < deadzone / 360_deg * HardwareConstants::kVersaEncoderCPR; });
}