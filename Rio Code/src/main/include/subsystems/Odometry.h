#pragma once

#include <frc2/command/SubsystemBase.h>

#include <frc/Encoder.h>

#include <AHRS.h>
#include <frc/SPI.h>

#include <units/length.h>
#include <units/angle.h>

class Odometry : public frc2::SubsystemBase
{
public:
	Odometry();

	void Periodic() override;

	void UpdatePosition(units::meter_t x, units::meter_t y, units::degree_t angle);

	inline units::meter_t GetX() const { return m_position[0]; }
	inline units::meter_t GetaY() const { return m_position[1]; }

	inline units::degree_t GetAngle() const { return m_angle; }

private:
	//encoders for base
	frc::Encoder m_leftEncoder, m_rightEncoder;

	/**
	 * NavX gyro
	*/
	AHRS m_gyro;

	units::meter_t m_position[2];
	units::degree_t m_angle;
};