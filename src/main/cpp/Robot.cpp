// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include "frc/XboxController.h"
#include <frc/TimedRobot.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/Timer.h>
#include <frc/Notifier.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/CANSparkMax.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/RobotState.h>
#define WPILIB_NO_UNITS

  //auto options
   frc::SendableChooser<std::string> m_chooser;
   const std::string kAutoDefault = "NONE";
   const std::string kAutoFowrard1 = "Forward_3sec";
   const std::string kAutoFowrard2 = "Forward_with_turn";
   const std::string kAutoFowrard3 = "Turn_and_go";
   const std::string kAutoFowrard4 = "Basic_Start";

/**
 * This is a demo program showing the use of the DifferentialDrive class.
 * Runs the motors with tank steering.
 */
class Robot : public frc::TimedRobot {

  //creating 4 motors as it is in the robot
  rev::CANSparkMax m_leftMotor1{1, rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax m_leftMotor2{2, rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax m_rightMotor1{3, rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax m_rightMotor2{4, rev::CANSparkMax::MotorType::kBrushed};

  //2 motors for the intake
  rev::CANSparkMax m_leftIntake{12, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightIntake{5, rev::CANSparkMax::MotorType::kBrushless};

  //create a MotorControllerGroup to combine all left and right motors
  frc::MotorControllerGroup m_leftMotors{m_leftMotor1, m_leftMotor2};
  frc::MotorControllerGroup m_rightMotors{m_rightMotor1, m_rightMotor2};

  //Drive variable that is using motor groups
  frc::DifferentialDrive m_robotDrive{m_leftMotors, m_rightMotors};
  frc::XboxController controller{0}; 

  //AUTO
  frc::Timer m_timer;
  bool move_forward;
  
  //frc::Notifier notifier();

 public:
  void RobotInit() override {
    //make motors reset to 0
    m_leftMotor1.RestoreFactoryDefaults();
    m_leftMotor2.RestoreFactoryDefaults();
    m_rightMotor1.RestoreFactoryDefaults();
    m_rightMotor2.RestoreFactoryDefaults();

    m_leftIntake.RestoreFactoryDefaults();
    m_rightIntake.RestoreFactoryDefaults();

    //m_rightIntake.RestoreFactoryDefaults();
    //right motor must be inverted for it to go forward
    m_rightMotors.SetInverted(true);
    
    //give auto options (Now only 2)
    m_chooser.SetDefaultOption(kAutoDefault, kAutoDefault);
    m_chooser.AddOption(kAutoFowrard1, kAutoFowrard1);
    m_chooser.AddOption(kAutoFowrard2, kAutoFowrard2);
    m_chooser.AddOption(kAutoFowrard3, kAutoFowrard3);
    m_chooser.AddOption(kAutoFowrard4, kAutoFowrard4);
    frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

    //give a 0 as initial number for intake speed
    frc::SmartDashboard::PutNumber("Intake Speed", 0.4); //check if it will give me an option to change a number
  }

  void TeleopPeriodic() override {
    //spped + rotatation
    double speed = controller.GetLeftY();
    double rotation = controller.GetRightX();
    //Arcade drive method
    m_robotDrive.ArcadeDrive(speed, rotation);

    //get number from smartDashboard (if not picked, it will be 0.5)
    //int intakenum = frc::SmartDashboard::GetNumber("Intake Speed", 0.5);
    //initial speed of the intake
    double intakeSpeed = frc::SmartDashboard::GetNumber("Intake Speed", 0.4);

    //check for left and right trigger
    if (controller.GetLeftTriggerAxis() > 0) {
        m_leftIntake.Set(-intakeSpeed);
        m_rightIntake.Set(intakeSpeed);
    }
    else if (controller.GetRightTriggerAxis() > 0) {
        m_leftIntake.Set(intakeSpeed);
        m_rightIntake.Set(-intakeSpeed);
    }
    else {

        m_leftIntake.Set(0);
        m_rightIntake.Set(0);
    }
    
  }

    //*************************************************AUTONOMUS PART*************************************************************
  void AutonomousInit() override {
    //m_robotState = frc::RobotState::Autonomous; //develop even further
    m_timer.Reset();
    m_timer.Start();
  }
  

  void AutonomousPeriodic() override {
    //NOTE ON AUTO: it cant go 15% of motor capacity and lower
    std::string selectedOption = frc::SmartDashboard::GetString("Auto Modes", "NONE");

     //if (selectedOption == "Forward_3sec") {
      if(m_timer.Get() < 3_s){
        m_robotDrive.ArcadeDrive(0.5, 0.0, false); 
      }
      else if(m_timer.Get() < 5_s){
        m_robotDrive.ArcadeDrive(0.2, 0.0, false); 
      }
      else if(m_timer.Get() < 7_s){
        m_robotDrive.TankDrive(0.50, 0.0, false);
      }
      else if (m_timer.Get() < 9_s){
        m_robotDrive.ArcadeDrive(0.2, 0.0, false);
      }
      else if (m_timer.Get() < 14.5_s){
        m_robotDrive.ArcadeDrive(0.0, 0.0, false);
        //m_robotDrive = frc::RobotState::IsTeleop();  
      }
    }
  //**************************************************************************************************************************
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
