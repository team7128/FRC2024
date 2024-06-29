#pragma once

#include <frc/XboxController.h>
#include <frc/Encoder.h>

#include <frc/controller/ProfiledPIDController.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/smartdashboard/Field2d.h>

#include <frc/simulation/EncoderSim.h>
#include <frc/simulation/DifferentialDrivetrainSim.h>
#include <hal/SimDevice.h>
#include <hal/simulation/SimDeviceData.h>

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/ProfiledPIDCommand.h>

#include <ctre/phoenix/motorcontrol/can/WPI_VictorSPX.h>

#include <AHRS.h>

#include <units/velocity.h>
#include <units/angular_velocity.h>

#include "Constants.h"

class RobotDrive : public frc2::SubsystemBase
{
public:
	// RobotDrive(Odometry *odometrySubsystem);
	RobotDrive();

	virtual void Periodic() override;
	virtual void SimulationPeriodic() override;

	/**
	 * Command factory to provide the default human-driven arcade drive behaviour
	 * 
	 * @param velocity Forward/back velocity in meters/second
	 * @param rotational Rotational velocity, in degrees/second. CCW is positive.
	*/
	void ArcadeDrive(units::meters_per_second_t velocity, units::degrees_per_second_t rotational, bool squareInputs);

	/**
	 * Stops the drivebase
	*/
	void Stop();

	/**
	 * Resets position and odometry components
	 * 
	 * IMPORTANT: Only call this when nothing else is running, as it will reset encoders and gyro and will likely break any commands using them directly.
	*/
	void ResetPosition();

	/**
	 * @brief Externally change the positon of the drivebase
	 * 
	 * @param position 
	 */
	inline void UpdatePosition(frc::Pose2d position) { m_position = position; }

	/// @brief Position getter
	inline frc::Pose2d GetPosition() const { return m_position; }

	/**
	 * @brief Updates configurable parameters.
	 * 
	 * Currently only whether to use encoders.
	 */
	void UpdateParams();

	/**
	 * @brief Drive in a straight line for a set distance.
	 * Creates a command to drive forward/backwards by a specified distance.
	 *
	 * @param distance The distance to drive. Can be positive or negative.
	 * @return The distance drive command.
	 */
	frc2::CommandPtr DriveDistanceCmd(units::meter_t distance);

	/**
	 * @brief Face a specific angle.
	 * Creates a command to face a specific direction relative to the field.
	 * 
	 * @param angle Direction to face.
	 * @return Turn command.
	 */
	frc2::CommandPtr TurnToAngleCmd(units::degree_t angle);
	/**
	 * @brief Turn drivebase by an angle.
	 * Creates a command to turn by a set angle.
	 * 
	 * @param angle Angle to turn by. Positive is left.
	 * @return Turn command.
	 */
	frc2::CommandPtr TurnByAngleCmd(units::degree_t angle);
	/**
	 * @brief Turn to face a point on the field.
	 * Creates a command to face a specific point on the field.
	 * 
	 * @param fieldX Field point X coordinate.
	 * @param fieldY Field point Y coordinate.
	 * @return Face point command.
	 */
	frc2::CommandPtr FacePointCmd(units::meter_t fieldX, units::meter_t fieldY);

	/**
	 * @brief Go to a specified point on the field.
	 * Creates a command to navigate to a point on the field. Turns first, then drives to the target point.
	 * Optinally, can stop some distance before the target point.
	 * 
	 * @param fieldX Target point X coordinate.
	 * @param fieldY Target point Y coordinate.
	 * @param stopDistance Optinal distance to stop before the target point.
	 * @return Go to point command.
	 */
	frc2::CommandPtr GoToPointCmd(units::meter_t fieldX, units::meter_t fieldY, units::meter_t stopDistance = 0_m);
	
	/**
	 * @brief Configures drive controller.
	 * Sets the deadzone of a drive controller.
	 * 
	 * @param controller A ProfiledPIDController with meters as units.
	 */
	static void ConfigureDriveController(frc::ProfiledPIDController<units::meters> &controller);
	/**
	 * @brief Configures a turn controller.
	 * Sets the deadzone of a turn controller and enables continuous values.
	 * 
	 * @param controller A ProfiledPIDController with degrees as units.
	 */
	static void ConfigureTurnController(frc::ProfiledPIDController<units::degrees> &controller);

	/**
	 * @brief Holds all components for position tracking
	 * Holds drivebase encoders and gyro. Also holds the relevant sim objects.
	 * Has basic functions for getting values and resetting component readings.
	 */
	struct OdometryComponents
	{
		bool useEncoders;
		AHRS m_gyro;
		frc::Encoder m_leftEncoder, m_rightEncoder;

		frc::sim::EncoderSim m_leftEncoderSim{ m_leftEncoder },
			m_rightEncoderSim{ m_rightEncoder };
		
		HAL_SimDeviceHandle m_gyroSim = HALSIM_GetSimDeviceHandle("navX-Sensor[4]");
		hal::SimDouble m_angleSim = HALSIM_GetSimValueHandle(m_gyroSim, "Yaw");

		// Velocity-estimated position when encoders are disabled
		units::meters_per_second_t leftSpeed, rightSpeed;
		units::meter_t leftDistance, rightDistance;

		OdometryComponents();

		void Periodic();
		void Reset();

		inline units::degree_t GetAngle() { return -units::degree_t(m_gyro.GetYaw()); }

		inline units::meter_t GetLeftDistance() const { return useEncoders ? units::meter_t(m_leftEncoder.GetDistance()) : leftDistance; }
		inline units::meter_t GetRightDistance() const { return useEncoders ? units::meter_t(m_rightEncoder.GetDistance()) : rightDistance; }
		inline units::meter_t GetAvgDistance() const { return (GetRightDistance() + GetLeftDistance()) / 2; }

		inline units::meters_per_second_t GetLeftSpeed() const { return useEncoders ? units::meters_per_second_t(m_leftEncoder.GetRate()) : leftSpeed; }
		inline units::meters_per_second_t GetRightSpeed() const { return useEncoders ? units::meters_per_second_t(m_rightEncoder.GetRate()) : rightSpeed; }
		inline units::meters_per_second_t GetAvgSpeed() const { return (GetRightSpeed() + GetRightSpeed()) / 2; }
	} m_odometryComponents;

private:
	/// Victor SPX motor controllers
	ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_motorFR, m_motorBR, m_motorFL, m_motorBL;

	/// Differential drive for controlling motors
	frc::DifferentialDrive m_diffDrive;

	/// Handles limiting wheel speeds to reasonable values
	frc::DifferentialDriveKinematics m_diffDriveKinematics;

	/// Position tracking
	frc::Pose2d m_position;
	frc::DifferentialDriveOdometry m_odometry;

	/// Sim drivebase
	frc::sim::DifferentialDrivetrainSim m_driveSim{
		frc::DCMotor::CIM(2),
		7.29,
		0.1_kg_sq_m,
		50_kg,
		DriveConstants::kWheelDiameter / 2,
		DriveConstants::kWheelbaseWidth / 2
	};

	frc::Field2d m_field;

	int ToSensorUnits(double value);

	// HERE BE DRAGONS
	// All the classes required to implement drivebase auto commands

	class DriveDistanceCmd_t : public frc2::CommandHelper<frc2::ProfiledPIDCommand<units::meters>, DriveDistanceCmd_t>
	{
	public:
		DriveDistanceCmd_t(units::meter_t distance, RobotDrive *drive);

		virtual void Initialize() override;

		virtual bool IsFinished() override;
	
	private:
		RobotDrive *m_robotDriveSub;
	};

	class TurnToAngleCmd_t : public frc2::CommandHelper<frc2::ProfiledPIDCommand<units::degrees>, TurnToAngleCmd_t>
	{
	public:
		TurnToAngleCmd_t(units::degree_t angle, RobotDrive *drive);

		virtual void Initialize() override;

		virtual bool IsFinished() override;

	protected:
		RobotDrive *m_robotDriveSub;
	};

	class TurnByAngleCmd_t : public frc2::CommandHelper<frc2::ProfiledPIDCommand<units::degrees>, TurnByAngleCmd_t>
	{
	public:
		TurnByAngleCmd_t(units::degree_t angle, RobotDrive *drive);

		virtual void Initialize() override;

		virtual bool IsFinished() override;

	private:
		RobotDrive *m_robotDriveSub;
	};

	class GoToPointCmd_t : public frc2::CommandHelper<frc2::Command, GoToPointCmd_t>
	{
	public:
		GoToPointCmd_t(units::meter_t fieldX, units::meter_t fieldY, units::meter_t stopDistance, RobotDrive *drive);

		virtual void Initialize() override;
		virtual void Execute() override;

		virtual bool IsFinished() override;

	private:
		units::meter_t m_targetX, m_targetY;
		bool m_facingCorrect = false;

		frc::ProfiledPIDController<units::meters> m_driveController;
		frc::ProfiledPIDController<units::degrees> m_turnController;

		RobotDrive *m_robotDriveSub;
	};
};