#include "subsystems/RobotDrive.h"

#include <numbers>

#include "Constants.h"

RobotDrive::RobotDrive()
	: m_motorFR(HardwareConstants::kDrivebaseMotorIDs[0]),
	m_motorBR(HardwareConstants::kDrivebaseMotorIDs[0]),
	m_motorFL(HardwareConstants::kDrivebaseMotorIDs[0]),
	m_motorBL(HardwareConstants::kDrivebaseMotorIDs[0]),
	m_diffDrive(m_motorFL, m_motorFR),
	m_diffDriveKinematics(RobotConstants::kWheelbaseWidth)
{
	m_motorBR.Follow(m_motorFR);
	m_motorBL.Follow(m_motorFL);

	m_motorFL.SetInverted(true);
}

void RobotDrive::ArcadeDrive(units::meters_per_second_t velocity, units::degrees_per_second_t rotational, bool squareInputs)
{
	auto wheelSpeeds = m_diffDriveKinematics.ToWheelSpeeds({ velocity, 0_mps, rotational });
	wheelSpeeds.Desaturate(RobotConstants::kMaxWheelSpeed);

	// Using tank drive to simply provide wheel speeds, instead of converting back to chassis speeds for arcade
	m_diffDrive.TankDrive(
		wheelSpeeds.left / RobotConstants::kMaxWheelSpeed,
		wheelSpeeds.right / RobotConstants::kMaxWheelSpeed,
		squareInputs
	);
	// wheelSpeeds.left /= RobotConstants::kMaxWheelSpeed.value();
	// wheelSpeeds.right /= RobotConstants::kMaxWheelSpeed.value();
	// auto chassisSpeed = m_diffDriveKinematics.ToChassisSpeeds(wheelSpeeds);
	// m_diffDrive.ArcadeDrive(chassisSpeed.vx.value(), chassisSpeed.om)
}

void RobotDrive::Stop()
{
	m_diffDrive.StopMotor();
}