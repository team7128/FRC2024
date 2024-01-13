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
	m_leftEncoder.SetDistancePerPulse(RobotConstants::kWheelDiameter.value() * std::numbers::pi / HardwareConstants::kDrivebaseEncoderCountsPerRev);
	m_rightEncoder.SetDistancePerPulse(RobotConstants::kWheelDiameter.value() * std::numbers::pi / HardwareConstants::kDrivebaseEncoderCountsPerRev);
}

void Odometry::Periodic()
{
	// m_lastCenterDistance = m_centerDistance;
	// m_lastAngle = m_angle;

	auto leftWheelDistance = units::meter_t(m_leftEncoder.GetDistance()),
		rightWheelDistance = units::meter_t(m_rightEncoder.GetDistance());
	auto prevDistance = m_centerDistance;
	m_centerDistance = (leftWheelDistance + rightWheelDistance) / 2;
	// units::meter_t distanceDriven = GetDistanceDriven();
	auto distanceDriven = m_centerDistance - prevDistance;

	auto prevAngle = m_angle;
	m_angle = m_gyro.GetRotation2d().Degrees();
	// units::degree_t angleChange = GetAngleTurned();
	auto angleChange = m_angle - prevAngle;

	if (angleChange == 0_deg)
	{
		// Handling edge case of angle change being 0, which would result in a divide by 0 otherwise
		m_position[0] += distanceDriven * units::math::sin(m_angle);
		m_position[1] += distanceDriven * units::math::cos(m_angle);
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
		m_position[0] += distanceDriven / angleChange.convert<units::radian>().value() * (units::math::sin(m_angle + angleChange) - units::math::cos(m_angle));
		m_position[1] += distanceDriven / angleChange.convert<units::radian>().value() * (units::math::cos(m_angle) - units::math::cos(m_angle + angleChange));
	}

	units::meters_per_second_t leftWheelVel{ m_leftEncoder.GetRate() },
		rightWheelVel{ m_rightEncoder.GetRate() };
	m_velocity = (leftWheelVel + rightWheelVel) / 2;

	m_angularVelocity = units::degrees_per_second_t(m_gyro.GetRate());
}

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
	// m_lastCenterDistance = 0_m;
}