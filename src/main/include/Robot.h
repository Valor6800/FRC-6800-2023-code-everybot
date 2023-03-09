#pragma once
#include <frc/TimedRobot.h>
#include <frc2/command/Command.h>
#include <frc/DataLogManager.h>
#include <RobotContainer.h>
#include <wpi/DataLog.h>

class Robot : public frc::TimedRobot {
public:

	void RobotInit() override;
	void RobotPeriodic() override;
	void DisabledInit() override;
	void DisabledPeriodic() override;
	void AutonomousInit() override;
	void AutonomousPeriodic() override;
	void TeleopInit() override;
	void TeleopPeriodic() override;
	void TestPeriodic() override;

private:
    // The robot's subsystems and commands are defined here...
    frc2::Command* m_autonomousCommand = nullptr;
    frc::DataLogManager m_dataLogManager;
};