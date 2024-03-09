#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>

#include <ctre/phoenix/motorcontrol/can/WPI_VictorSPX.h>

class Climb : public frc2::SubsystemBase
{
public:
    Climb();

    /**
     * @brief Drives the climb arms with the specified speeds
     * For all speeds, negative is down, positive is up
     * 
     * @param speed Speed of climb arms
     */
    void Drive(double speed);

    /**
     * @brief Constructs a command to move the climb arms
     * 
     * @param speed Climb arms speed
     * @return The climb drive command
     */
    frc2::CommandPtr DriveCmd(double speed);

    /**
     * @brief Generates a command to stop the climb motors
     * 
     * @return A CommandPtr for the stop command 
     */
    frc2::CommandPtr StopCmd();

private:
    ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_leftClimbMotor, m_rightClimbMotor;
};