#include "subsystems/Intake.h"

#include <frc/shuffleboard/Shuffleboard.h>

#include <frc2/command/WaitUntilCommand.h>

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
	return this->Run([this, speed] { this->Enable(speed); }).FinallyDo([this] { this->Disable(); });
}

frc2::CommandPtr Intake::Rollers::EnableTimedCmd(double speed, units::second_t time)
{
	return this->Run([this, speed] { this->Enable(speed); }).WithTimeout(time).AndThen(DisableCmd());
}

frc2::CommandPtr Intake::Rollers::DisableCmd()
{
	return this->RunOnce([this] { this->Disable(); });
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
	Disable();
	m_liftMotor.Set(speed);
}

frc2::CommandPtr Intake::Lift::DriveTimedCmd(double speed, units::second_t time)
{
	return this->RunOnce([this] { this->Disable(); }).AndThen(this->Run([this, speed] { this->Drive(speed); }).WithTimeout(time));
}

frc2::CommandPtr Intake::Lift::DisableCmd()
{
	return this->RunOnce([this] { this->m_liftMotor.StopMotor(); });
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
	return GoToAngleCmd(kDeployAngle);
}

frc2::CommandPtr Intake::Lift::StowCmd()
{
	// Run to 0 degrees, as this is where the intake homes to
	return GoToAngleCmd(0_deg);
}

frc2::CommandPtr Intake::Lift::ClimbCmd()
{
	// Move intake to climb position, then disable automatic PID control so the intake is not fighting the chain once it is hanging
	return GoToAngleCmd(kClimbAngle);
}

frc2::CommandPtr Intake::Lift::ClimbIdleCmd()
{
	return GoToAngleCmd(kClimbIdleAngle);
}

frc2::CommandPtr Intake::Lift::HomeCmd()
{
	return this->RunOnce([this] { this->Disable(); })	// Disable automatic PID control
		.AndThen(this->Run([this] { Drive(-kHomeSpeed); })	// Drive the intake up until no longer contacting the limit switch
			.Until([this] { return this->m_limitSwitch.Get(); }))
		.AndThen(this->Run([this] { this->m_liftMotor.Set(kHomeSpeed); })	// Run the motor slowly
			.Until([this] { return !this->m_limitSwitch.Get(); }))	// Stop once we hit the limit switch
		.AndThen(this->RunOnce([this] {	// Once the limit switch is hit
			this->m_encoder.Reset();	// Reset encoders
			// this->SetGoal(0_deg);		// Set goal to homed position
			// this->Enable();				// Resume PID control
		}));
		// .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelIncoming);	// Ensure the home command has priority over others
}

frc2::CommandPtr Intake::Lift::GoToAngleCmd(units::degree_t angle)
{
	return this->RunOnce([this, angle] {
			this->SetGoal(angle);
			this->Enable();
		}); // .AndThen(frc2::WaitUntilCommand([this] { return this->m_controller.AtGoal(); }).ToPtr());
}