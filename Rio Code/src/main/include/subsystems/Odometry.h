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

	/**
	 * Sets the robot position
	*/
	void UpdatePosition(units::meter_t x, units::meter_t y, units::degree_t angle);
	
	/// Resets encoder positions to 0
	void ResetEncoders();
	/// Resets gyro angle to 0
	void ResetGyro();
	/// Resets robot position and all sensors to 0
	void ResetPosition();

	inline units::meter_t GetX() const { return m_position[0]; }
	inline units::meter_t GetY() const { return m_position[1]; }
	inline units::degree_t GetAngle() const { return m_angle; }

	/**
	 * Distance driven by robot
	 * 
	 * Returns the distance driven by the robot centerpoint since the last call to ResetEncoders()
	 * 
	 * @returns Distance driven, in meters
	*/
	inline units::meter_t GetCenterDistance() const { return m_centerDistance; }

	inline units::meters_per_second_t GetVelocity() const { return m_velocity; }
	inline units::degrees_per_second_t GetAngularVelocity() const { return m_angularVelocity; }

	/**
	 * Returns wheel velocity of given wheel
	 * 
	 * 0 for left wheel
	 * 1 for right wheel
	*/
	inline units::meters_per_second_t GetWheelVelocity(int wheel) const { return m_wheelVelocities[wheel]; }

private:
	// Encoders for base
	frc::Encoder m_leftEncoder, m_rightEncoder;

	/// NavX gyro
	AHRS m_gyro;

	units::meter_t m_position[2];
	units::degree_t m_angle;

	units::meter_t m_centerDistance;
	/// Keeps track of the gyro angle separate from the robot's angle to allow for offset
	units::degree_t m_gyroAngle;

	units::meters_per_second_t m_velocity;
	units::meters_per_second_t m_wheelVelocities[2];
	units::degrees_per_second_t m_angularVelocity;
};