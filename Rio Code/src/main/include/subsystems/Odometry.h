#pragma once

#include <frc2/command/SubsystemBase.h>

#include <frc/Encoder.h>

#include <AHRS.h>
#include <frc/SPI.h>

#include <units/length.h>
#include <units/velocity.h>
#include <units/angle.h>
#include <units/angular_velocity.h>

class Odometry : public frc2::SubsystemBase
{
public:
	Odometry();

	void Periodic() override;

	void UpdatePosition(units::meter_t x, units::meter_t y, units::degree_t angle);
	void ResetEncoders();

	inline units::meter_t GetX() const { return m_position[0]; }
	inline units::meter_t GetY() const { return m_position[1]; }

	inline units::meter_t GetCenterDistance() const { return m_centerDistance; }
	inline units::degree_t GetAngle() const { return m_angle; }

	// inline units::meter_t GetDistanceDriven() const { return m_centerDistance - m_lastCenterDistance; }
	// inline units::degree_t GetAngleTurned() const { return m_angle - m_lastAngle; }

	inline units::meters_per_second_t GetVelocity() const { return m_velocity; }
	inline units::degrees_per_second_t GetAngularVelocity() const { return m_angularVelocity; }

private:
	// Encoders for base
	frc::Encoder m_leftEncoder, m_rightEncoder;

	/**
	 * NavX gyro
	*/
	AHRS m_gyro;

	units::meter_t m_position[2];
	units::degree_t m_angle; //, m_lastAngle;

	units::meter_t m_centerDistance; //, m_lastCenterDistance;

	units::meters_per_second_t m_velocity;
	units::degrees_per_second_t m_angularVelocity;
};