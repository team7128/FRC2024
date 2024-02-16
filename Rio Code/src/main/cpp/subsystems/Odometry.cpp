#include "subsystems/Odometry.h"

#include <frc/shuffleboard/Shuffleboard.h>

#include <wpi/raw_ostream.h>

#include <numbers>

#include <units/math.h>

#include "Constants.h"

using namespace HardwareConstants;
using namespace RobotConstants;

Odometry::Odometry()
	: m_leftEncoder(kDrivebaseEncoderPorts[0], kDrivebaseEncoderPorts[1]),
	m_rightEncoder(kDrivebaseEncoderPorts[2], kDrivebaseEncoderPorts[3]),
	m_gyro(frc::SPI::Port::kMXP)
{
	// Set up encoder distance per pulse (wheel circumference / pulses per revolution)
	m_leftEncoder.SetDistancePerPulse(kWheelDiameter.value() * std::numbers::pi / kDrivebaseEncoderCountsPerRev);
	m_rightEncoder.SetDistancePerPulse(kWheelDiameter.value() * std::numbers::pi / kDrivebaseEncoderCountsPerRev);
	
	m_leftEncoder.SetReverseDirection(true);

	// Initialize all position variables to 0
	ResetPosition();

	auto &mainTab = frc::Shuffleboard::GetTab("Main");

	mainTab.AddString("Position", [this] {
		return "[" + std::to_string(this->m_position[0].value()) + ", " + std::to_string(this->m_position[1].value()) + "]";
	});

	// mainTab.AddDouble("Angle", [this] { return this->m_angle.value(); });

	mainTab.Add("Left encoder", m_leftEncoder);
	mainTab.Add("Right encoder", m_rightEncoder);

	mainTab.Add("Gyro", m_gyro);
}

void Odometry::Periodic()
{
	auto leftWheelDistance = units::meter_t(m_leftEncoder.GetDistance()),
		rightWheelDistance = units::meter_t(m_rightEncoder.GetDistance());
	auto prevDistance = m_centerDistance;
	m_centerDistance = (leftWheelDistance + rightWheelDistance) / 2;
	auto distanceDriven = m_centerDistance - prevDistance;

	auto prevAngle = m_gyroAngle;
	m_gyroAngle = m_gyro.GetRotation2d().Degrees();
	auto angleChange = m_gyroAngle - prevAngle;

	m_angle += angleChange;

	// wpi::outs() << "Position: [" + std::to_string(this->m_position[0].value()) + ", " + std::to_string(this->m_position[1].value()) + "]; ";
	// wpi::outs() << "Angle: " << units::to_string(m_angle) << "; ";
	// wpi::outs() << "Angle change: " << units::to_string(angleChange) << "\n";
	// wpi::outs().flush();

	// if (angleChange == 0_deg)
	// {
		// Handling edge case of angle change being 0, which would result in a divide by 0 otherwise
		m_position[0] += distanceDriven * units::math::sin(m_angle);
		m_position[1] += distanceDriven * units::math::cos(m_angle);
	// }
	// else
	// {
		/**
		 * Obtained by integrating velocity equations of x and y
		 * x' = cos(theta0 + theta1 * t)
		 * y' = sin(theta0 + theta1 * t)
		 * over 0 < t < 1
		 * 
		 * where theta0 is the intial angle, and theta1 is the angle change
		*/
	// 	m_position[0] += distanceDriven / angleChange.convert<units::radian>().value() * (units::math::sin(m_angle + angleChange) - units::math::cos(m_angle));
	// 	m_position[1] += distanceDriven / angleChange.convert<units::radian>().value() * (units::math::cos(m_angle) - units::math::cos(m_angle + angleChange));
	// }

	m_wheelVelocities[0] = units::meters_per_second_t(m_leftEncoder.GetRate());
	m_wheelVelocities[1] = units::meters_per_second_t(m_rightEncoder.GetRate());
	m_velocity = (m_wheelVelocities[0] + m_wheelVelocities[1]) / 2;

	m_angularVelocity = units::degrees_per_second_t(m_gyro.GetRate());
};

void Odometry::UpdatePosition(units::meter_t x, units::meter_t y, units::degree_t angle)
{
	m_position[0] = x;
	m_position[1] = y;
	m_angle = angle;
}

void Odometry::ResetEncoders()
{
	m_leftEncoder.Reset();
	m_rightEncoder.Reset();
	m_centerDistance = 0_m;
}

void Odometry::ResetGyro()
{
	m_gyro.Reset();
	m_gyroAngle = 0_deg;
}

void Odometry::ResetPosition()
{
	ResetEncoders();
	ResetGyro();
	UpdatePosition(0_m, 0_m, 0_deg);
}