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
	inline void UpdatePosition(frc::Pose2d position) { m_position = position; }
	inline frc::Pose2d GetPosition() const { return m_position; }

	frc2::CommandPtr DriveDistanceCmd(units::meter_t distance);

	frc2::CommandPtr TurnToAngleCmd(units::degree_t angle);
	frc2::CommandPtr TurnByAngleCmd(units::degree_t angle);
	frc2::CommandPtr FacePointCmd(units::meter_t fieldX, units::meter_t fieldY);

	frc2::CommandPtr GoToPointCmd(units::meter_t fieldX, units::meter_t fieldY, units::meter_t stopDistance = 0_m);
	
	static void ConfigureDriveController(frc::ProfiledPIDController<units::meters> &controller);
	static void ConfigureTurnController(frc::ProfiledPIDController<units::degrees> &controller);

	struct OdometryComponents
	{
		AHRS m_gyro;
		frc::Encoder m_leftEncoder, m_rightEncoder;

		frc::sim::EncoderSim m_leftEncoderSim{ m_leftEncoder },
			m_rightEncoderSim{ m_rightEncoder };
		
		HAL_SimDeviceHandle m_gyroSim = HALSIM_GetSimDeviceHandle("navX-Sensor[4]");
		hal::SimDouble m_angleSim = HALSIM_GetSimValueHandle(m_gyroSim, "Yaw");

		OdometryComponents();

		void Reset();

		inline units::degree_t GetAngle() { return -units::degree_t(m_gyro.GetYaw()); }

		inline units::meter_t GetLeftDistance() const { return units::meter_t(m_leftEncoder.GetDistance()); }
		inline units::meter_t GetRightDistance() const { return units::meter_t(m_rightEncoder.GetDistance()); }
		inline units::meter_t GetAvgDistance() const { return (GetLeftDistance() + GetRightDistance()) / 2; }

		inline units::meters_per_second_t GetLeftSpeed() const { return units::meters_per_second_t(m_leftEncoder.GetRate()); }
		inline units::meters_per_second_t GetRightSpeed() const { return units::meters_per_second_t(m_rightEncoder.GetRate()); }
		inline units::meters_per_second_t GetAvgSpeed() const { return (GetLeftSpeed() + GetRightSpeed()) / 2; }
	} m_odometryComponents;

private:
	/// Victor SPX motor controllers
	ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_motorFR, m_motorBR, m_motorFL, m_motorBL;

	/// Differential drive for controlling motors
	frc::DifferentialDrive m_diffDrive;

	/// Handles limiting wheel speeds to reasonable values
	frc::DifferentialDriveKinematics m_diffDriveKinematics;

	frc::Pose2d m_position;
	frc::DifferentialDriveOdometry m_odometry;

	frc::sim::DifferentialDrivetrainSim m_driveSim{
		frc::DCMotor::CIM(2),
		7.29,
		0.1_kg_sq_m,
		50_kg,
		DriveConstants::kWheelDiameter / 2,
		DriveConstants::kWheelbaseWidth / 2
	};

	frc::Field2d m_field;

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