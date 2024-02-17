#include "subsystems/Intake.h"

#include "Constants.h"

using namespace IntakeConstants;

Intake::Rollers::Rollers() :
	m_intakeMotor(CANConstants::kIntakeRollerTalonID)
{}

void Intake::Rollers::Enable(double speed)
{
	m_intakeMotor.Set(speed);
}

void Intake::Rollers::Disable()
{
	m_intakeMotor.StopMotor();
}

frc2::CommandPtr Intake::Rollers::EnableTimedCmd(double speed, units::second_t time)
{
	return this->Run([this, speed] { this->Enable(speed); }).WithTimeout(time).AndThen(std::move(DisableCmd()));
}

frc2::CommandPtr Intake::Rollers::DisableCmd()
{
	return this->Run([this] { this->Disable(); });
}

Intake::Lift::Lift() :
	frc2::ProfiledPIDSubsystem<units::degrees>(
		frc::ProfiledPIDController<units::degrees>(
			kP, 0.0, 0.0, { kMaxVel, kMaxAccel }
		),
		0_deg
	),
	m_liftMotor(CANConstants::kIntakeLiftVictorID),
	m_limitSwitch(DIOConstants::kIntakeLimitSwitchPort),
	m_encoder(DIOConstants::kIntakeLiftEncoderPorts[0], DIOConstants::kIntakeLiftEncoderPorts[1]),
	m_feedforward(kS, kG, kV)
{
	m_encoder.SetDistancePerPulse(360.0 * kLiftRatio / kLiftEncoderCPR);
}

units::degree_t Intake::Lift::GetMeasurement()
{
	return units::degree_t(m_encoder.GetDistance());
}

void Intake::Lift::UseOutput(double output, State setpoint)
{
	m_liftMotor.SetVoltage(m_feedforward.Calculate(kHomeAngle - units::degree_t(m_encoder.GetDistance()), units::degrees_per_second_t(output)));
}

frc2::CommandPtr Intake::Lift::DeployCmd()
{
	return this->RunOnce([this] { this->SetGoal(kDeployAngle); });
}

frc2::CommandPtr Intake::Lift::StowCmd()
{
	// Run to 0 degrees, as this is where the intake homes to
	return this->RunOnce([this] { this->SetGoal(0_deg); });
}

frc2::CommandPtr Intake::Lift::HomeCmd()
{
	return this->RunOnce([this] { this->Disable(); })	// Disable automatic PID control
		.AndThen(this->RunEnd(
			[this] { this->m_liftMotor.Set(-0.2); },	// Run the motor slowly
			[this] {									// Reset the encoder and resume PID control
				this->m_encoder.Reset();
				this->Enable();
			}
		).Until([this] { return m_limitSwitch.Get(); })	// Stop once we hit the limit switch
	);
}