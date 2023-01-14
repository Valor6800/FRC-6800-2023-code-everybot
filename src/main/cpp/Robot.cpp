// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include "frc/XboxController.h"
#include <frc/TimedRobot.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/Timer.h>
#include <frc/TimedRobot.h>
#include <frc/motorcontrol/MotorControllerGroup.h>

/**
 * This is a demo program showing the use of the DifferentialDrive class.
 * Runs the motors with tank steering.
 */
class Robot : public frc::TimedRobot {
  //acceleration variable
  const double kAcceleration = 0.01;
  //current speed
  double currentSpeed = 0;

  //creating 4 motors as it is in the robot
  frc::PWMSparkMax m_leftMotor1{0};
  frc::PWMSparkMax m_leftMotor2{1};
  frc::PWMSparkMax m_rightMotor1{2};
  frc::PWMSparkMax m_rightMotor2{3};
  //create a MotorControllerGroup to combine all left and right motors
  frc::MotorControllerGroup m_leftMotors{m_leftMotor1, m_leftMotor2};
  frc::MotorControllerGroup m_rightMotors{m_rightMotor1, m_rightMotor2};
  //Drive variable that is using motor groups
  frc::DifferentialDrive m_robotDrive{m_leftMotors, m_rightMotors};
  frc::XboxController controller{0}; 

 public:
  void RobotInit() override {
    //right motor must be inverted for it to go forward
    m_rightMotors.SetInverted(true);
  }

  void TeleopPeriodic() override {
    //get the speed and roataion from the controller
    //Left Joystic - Y Axis for speed
    double speed = controller.GetLeftY();
    //Right Joystic - X Axis for rotation
    double rotation = controller.GetRightX();

    //calculates current speed with acceleration
    currentSpeed += speed * kAcceleration;

    //Use ArcadeDrive to eliminate usage of multiple if/else statements
    //also speed * kMaxSpeed to create acceleration for the robot
    m_robotDrive.ArcadeDrive(currentSpeed, rotation);

    //code to check the speed of the robot without acceleration
    //m_robotDrive.ArcadeDrive(speed, rotation);
  }
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
