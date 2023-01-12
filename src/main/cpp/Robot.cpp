// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include "frc/XboxController.h"
#include <frc/TimedRobot.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/PWMSparkMax.h>

/**
 * This is a demo program showing the use of the DifferentialDrive class.
 * Runs the motors with tank steering.
 */
class Robot : public frc::TimedRobot {
  frc::PWMSparkMax m_leftMotor{0};
  frc::PWMSparkMax m_rightMotor{1};
  frc::DifferentialDrive m_robotDrive{m_leftMotor, m_rightMotor};
  frc::XboxController controller{0}; //error was here (I did not use {0})
  frc::Timer m_timer;

 public:
  void RobotInit() override {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.SetInverted(true);
  }

  void TeleopPeriodic() override {
    //get the speed and roataion from the controller
    //Left Joystic - Y Axis for speed
    double speed = controller.GetLeftY();
    //Right Joystic - X Axis for rotation
    double rotation = controller.GetRightX();

    //Use ArcadeDrive to eliminate usage of multiple if/else statements
    m_robotDrive.ArcadeDrive(speed, rotation);
  }
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
