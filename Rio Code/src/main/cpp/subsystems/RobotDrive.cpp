#include "subsystems/RobotDrive.h"

#include <numbers>

#include <frc/shuffleboard/Shuffleboard.h>

#include "Constants.h"

using namespace HardwareConstants;
using namespace RobotConstants;

RobotDrive::RobotDrive()
	: m_motorFR(kDrivebaseMotorIDs[0]),
	m_motorBR(kDrivebaseMotorIDs[1]),
	m_motorFL(kDrivebaseMotorIDs[2]),
	m_motorBL(kDrivebaseMotorIDs[3]),
	m_diffDrive(m_motorFL, m_motorFR),
	m_diffDriveKinematics(kWheelbaseWidth)
{
	m_motorFR.SetInverted(true);
	m_motorBR.SetInverted(true);

	m_motorBR.Follow(m_motorFR);
	m_motorBL.Follow(m_motorFL);

	auto &tab = frc::Shuffleboard::GetTab("Main");
	tab.Add(m_diffDrive);
}

void RobotDrive::ArcadeDrive(units::meters_per_second_t velocity, units::degrees_per_second_t rotational, bool squareInputs)
{
	auto wheelSpeeds = m_diffDriveKinematics.ToWheelSpeeds({ velocity, 0_mps, rotational });
	wheelSpeeds.Desaturate(kMaxWheelSpeed);

	// wpi::outs() << "Wheel speeds: [" << units::to_string(wheelSpeeds.left) << ", " << units::to_string(wheelSpeeds.right) << "]\n";

	// Using tank drive to simply provide wheel speeds, instead of converting back to chassis speeds for arcade
	m_diffDrive.TankDrive(
		wheelSpeeds.left / kMaxWheelSpeed,
		wheelSpeeds.right / kMaxWheelSpeed,
		squareInputs
	);
}

void RobotDrive::Stop()
{
	m_diffDrive.StopMotor();
}