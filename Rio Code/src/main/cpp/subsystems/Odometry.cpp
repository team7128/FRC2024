#include "subsystems/Odometry.h"

#include <numbers>

#include <units/math.h>

#include "Constants.h"

Odometry::Odometry()
	: m_leftEncoder(HardwareConstants::kDrivebaseEncoderPorts[0], HardwareConstants::kDrivebaseEncoderPorts[1]),
	m_rightEncoder(HardwareConstants::kDrivebaseEncoderPorts[2], HardwareConstants::kDrivebaseEncoderPorts[3]),
	m_gyro(frc::SPI::Port::kMXP),
	m_position{ 0_m, 0_m },
	m_angle(0)
{
	// Set up encoder distance per pulse (wheel circumference / pulses per revolution)
	m_leftEncoder.SetDistancePerPulse(RobotDimensions::kWheelDiameter.value() * std::numbers::pi / HardwareConstants::kDrivebaseEncoderCountsPerRev);
	m_rightEncoder.SetDistancePerPulse(RobotDimensions::kWheelDiameter.value() * std::numbers::pi / HardwareConstants::kDrivebaseEncoderCountsPerRev);
}

void Odometry::Periodic()
{
	units::meter_t leftWheelDistance = units::meter_t(m_leftEncoder.GetDistance());
	units::meter_t rightWheelDistance = units::meter_t(m_rightEncoder.GetDistance());
	units::meter_t centerlineDistance = (leftWheelDistance + rightWheelDistance) / 2;

	units::degree_t angleChange = units::radian_t(m_gyro.GetYaw());

	if (angleChange == 0_deg)
	{
		// Handling edge case of angle change being 0, which would result in a divide by 0 otherwise
		m_position[0] += centerlineDistance * units::math::sin(m_angle);
		m_position[1] += centerlineDistance * units::math::cos(m_angle);
	}
	else
	{
		/**
		 * Obtained by integrating velocity equations of x and y
		 * x' = cos(theta0 + theta1 * t)
		 * y' = sin(theta0 + theta1 * t)
		 * over 0 < t < 1
		 * 
		 * where theta0 is the intial angle, and theta1 is the angle change
		*/
		m_position[0] += centerlineDistance / angleChange.convert<units::radian>().value() * (units::math::sin(m_angle + angleChange) - units::math::cos(m_angle));
		m_position[1] += centerlineDistance / angleChange.convert<units::radian>().value() * (units::math::cos(m_angle) - units::math::cos(m_angle + angleChange));
	}

	m_angle += angleChange;

	m_leftEncoder.Reset();
	m_rightEncoder.Reset();
	m_gyro.Reset();
}

void Odometry::UpdatePosition(units::meter_t x, units::meter_t y, units::degree_t angle)
{
	m_position[0] = x;
	m_position[1] = y;
	m_angle = angle;
}