#include "subsystems/Intake.h"

#include <frc/shuffleboard/Shuffleboard.h>

#include "Constants.h"

using namespace IntakeConstants;

Intake::Rollers::Rollers() :
	m_intakeMotor(CANConstants::kIntakeRollerTalonID)
{
	m_intakeMotor.SetInverted(true);
}

void Intake::Rollers::Enable(double speed)
{
	m_intakeMotor.Set(speed);
}

void Intake::Rollers::Disable()
{
	m_intakeMotor.StopMotor();
}

// creates a command to enable intake rollers
frc2::CommandPtr Intake::Rollers::EnableCmd(double speed)
{
	return this->Run([this, speed] { this->Enable(speed); });
}

frc2::CommandPtr Intake::Rollers::EnableTimedCmd(double speed, units::second_t time)
{
	return this->Run([this, speed] { this->Enable(speed); }).WithTimeout(time);
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
	m_encoder(DIOConstants::kIntakeLiftEncoderPorts[0], DIOConstants::kIntakeLiftEncoderPorts[1])
{
	m_encoder.SetDistancePerPulse(-360.0 * kLiftRatio / kLiftEncoderCPR);
	m_controller.SetTolerance(5_deg);

	auto &tab = frc::Shuffleboard::GetTab("Main");
	tab.Add("Intake encoder", m_encoder);
	tab.Add("Intake limit switch", m_limitSwitch);
}

void Intake::Lift::Drive(double speed)
{
	m_liftMotor.Set(speed);
}

frc2::CommandPtr Intake::Lift::DisableCmd()
{
	return this->Run([this] { this->Disable(); });
}

units::degree_t Intake::Lift::GetMeasurement()
{
	return units::degree_t(m_encoder.GetDistance());
}

void Intake::Lift::UseOutput(double output, State setpoint)
{
	m_liftMotor.Set(-output);
}

frc2::CommandPtr Intake::Lift::DeployCmd()
{
	return this->Run([this] {
		this->SetGoal(kDeployAngle);
		this->Enable();
	}).Until([this] { return this->m_controller.AtGoal(); });
}

frc2::CommandPtr Intake::Lift::StowCmd()
{
	// Run to 0 degrees, as this is where the intake homes to
	return this->Run([this] {
			this->SetGoal(0_deg);
			this->Enable();
		}).Until([this] { return this->m_controller.AtGoal(); });
}

frc2::CommandPtr Intake::Lift::ClimbCmd()
{
	// Move intake to climb position, then disable automatic PID control so the intake is not fighting the chain once it is hanging
	return this->Run([this] {
			this->SetGoal(kClimbAngle);
			this->Enable();
		}).Until([this] { return this->m_controller.AtGoal(); })
		.AndThen(this->RunOnce([this] { this->Disable(); }));
}

frc2::CommandPtr Intake::Lift::HomeCmd()
{
	return this->RunOnce([this] { this->Disable(); })	// Disable automatic PID control
		.AndThen(this->Run([this] { Drive(-kHomeSpeed); })	// Drive the intake up until no longer contacting the limit switch
			.Until([this] { return this->m_limitSwitch.Get(); }))
		.AndThen(this->Run([this] { this->m_liftMotor.Set(kHomeSpeed); })	// Run the motor slowly
			.Until([this] { return !m_limitSwitch.Get(); }))	// Stop once we hit the limit switch
		.AndThen(this->RunOnce([this] {	// Once the limit switch is hit
			this->m_encoder.Reset();	// Reset encoders
			this->SetGoal(0_deg);		// Set goal to homed position
			this->Enable();				// Resume PID control
		}));
		// .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelIncoming);	// Ensure the home command has priority over others
}
				