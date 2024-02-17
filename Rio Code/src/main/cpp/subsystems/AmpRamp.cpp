#include "subsystems/AmpRamp.h"

#include <frc2/command/InstantCommand.h>

#include "Constants.h"

using namespace AmpRampConstants;

AmpRamp::AmpRamp() :
	m_limitSwitch(DIOConstants::kAmpRampSwitchPort),
	m_motorController(CANConstants::kAmpRampTalonID)
{
	m_motorController.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder);
}

frc2::CommandPtr AmpRamp::DeployCmd()
{
	return GoToAngleCmd(kDeployAngle, 2_deg);
}

frc2::CommandPtr AmpRamp::StowCmd()
{
	return GoToAngleCmd(kStowAngle, 2_deg);
}

frc2::CommandPtr AmpRamp::HomeCmd()
{
	return this->Run([this] { this->m_motorController.Set(-0.2); })
		.Until([this] { return this->m_limitSwitch.Get(); })
		.AndThen([this] { this->m_motorController.SetSelectedSensorPosition(kHomeAngle.convert<units::radians>().value()); });
}

frc2::CommandPtr AmpRamp::GoToAngleCmd(units::degree_t angle, units::degree_t deadzone)
{
	return this->Run([this, angle] {
		this->m_motorController.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::Position, angle / 360.0_deg * HardwareConstants::kVersaEncoderCPR);
	}).Until([this, deadzone] { return this->m_motorController.GetClosedLoopError() < deadzone / 360.0_deg * HardwareConstants::kVersaEncoderCPR; });
}