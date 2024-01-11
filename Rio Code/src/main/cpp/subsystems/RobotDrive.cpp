#include "subsystems/RobotDrive.h"

#include <numbers>

#include "Constants.h"

#include <frc2/command/RunCommand.h>

RobotDrive::RobotDrive()
	: m_motorFR(HardwareConstants::kDrivebaseMotorIDs[0]),
	m_motorBR(HardwareConstants::kDrivebaseMotorIDs[0]),
	m_motorFL(HardwareConstants::kDrivebaseMotorIDs[0]),
	m_motorBL(HardwareConstants::kDrivebaseMotorIDs[0]),
	m_diffDrive(m_motorFL, m_motorFR)
{
	m_motorBR.Follow(m_motorFR);
	m_motorBL.Follow(m_motorFL);

	m_motorFL.SetInverted(true);
}

frc2::CommandPtr RobotDrive::ArcadeDrive(frc::XboxController &controller)
{
	return this->Run(
		[this, &controller] {
			this->m_diffDrive.ArcadeDrive(
				controller.GetLeftY(),
				controller.GetRightX()
			);
		}
	);
}

frc2::CommandPtr RobotDrive::Stop()
{
	return this->RunOnce(
		[this] {
			this->m_diffDrive.StopMotor();
		}
	);
}