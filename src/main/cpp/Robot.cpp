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
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/CANSparkMax.h>
//hi
/**
 * This is a demo program showing the use of the DifferentialDrive class.
 * Runs the motors with tank steering.
 */
class Robot : public frc::TimedRobot {
  //acceleration feature
  double Acceleration = 0.3;
  double currentSpeed = 0;

  //creating 4 motors as it is in the robot
  rev::CANSparkMax m_leftMotor1{1, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftMotor2{2, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightMotor1{3, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightMotor2{4, rev::CANSparkMax::MotorType::kBrushless};
  //create a MotorControllerGroup to combine all left and right motors
  frc::MotorControllerGroup m_leftMotors{m_leftMotor1, m_leftMotor2};
  frc::MotorControllerGroup m_rightMotors{m_rightMotor1, m_rightMotor2};

  //Drive variable that is using motor groups
  frc::DifferentialDrive m_robotDrive{m_leftMotors, m_rightMotors};
  frc::XboxController controller{0}; 

 public:
  void RobotInit() override {
    m_leftMotor1.RestoreFactoryDefaults();
    m_leftMotor2.RestoreFactoryDefaults();
    m_rightMotor1.RestoreFactoryDefaults();
    m_rightMotor2.RestoreFactoryDefaults();
    //right motor must be inverted for it to go forward
    m_rightMotors.SetInverted(true);
  }

  void TeleopPeriodic() override {
    //get the speed and roataion from the controller
    //Left Joystic - Y Axis for speed
    double speed = controller.GetLeftY();
    //Right Joystic - X Axis for rotation
    double rotation = controller.GetRightX();

    //creating current speed by * speed and acceleratiton
    currentSpeed += speed * Acceleration;

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
