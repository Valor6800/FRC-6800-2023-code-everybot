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
#include <units/time.h>
#define WPILIB_NO_UNITS

  //auto options
   frc::SendableChooser<std::string> m_chooser;
   std::string kAutoDefault = "NONE";
   std::string kAutoFowrard1 = "Red_Mid";
   std::string kAutoFowrard2 = "Red_Right";
   std::string kAutoFowrard3 = "Red_Left";

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
  frc::Timer m_waittimer;

 public:
  void RobotInit() override {
    //make motors reset to 0
    m_leftMotor1.RestoreFactoryDefaults();
    m_leftMotor2.RestoreFactoryDefaults();
    m_rightMotor1.RestoreFactoryDefaults();
    m_rightMotor2.RestoreFactoryDefaults();

    m_leftIntake.RestoreFactoryDefaults();
    m_rightIntake.RestoreFactoryDefaults();

    //right motor must be inverted for it to go forward
    m_rightMotors.SetInverted(true);
    
    //give auto options (Now only 2)
    m_chooser.SetDefaultOption("NONE", kAutoDefault);
    m_chooser.AddOption("Red_Mid", kAutoFowrard1);
    m_chooser.AddOption("Red_Right", kAutoFowrard2);
    m_chooser.AddOption("Red_Left", kAutoFowrard3);
    frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

    //give a 0 as initial number for intake speed
    frc::SmartDashboard::PutNumber("Intake Speed", 0.4);
    frc::SmartDashboard::PutNumber("Delay (Sec)", 0);
  }

  void TeleopPeriodic() override {
    //spped + rotatation
    double speed = controller.GetLeftY();
    double rotation = controller.GetRightX();

    //Arcade drive method
    m_robotDrive.ArcadeDrive(speed, rotation);

    //initial speed of the intake
    double intakeSpeed = frc::SmartDashboard::GetNumber("Intake Speed", 0.4);

    //check for left and right trigger
    if (controller.GetLeftTriggerAxis() > 0) {
        //intake in
        m_leftIntake.Set(-intakeSpeed);
        m_rightIntake.Set(intakeSpeed);
    }
    else if (controller.GetRightTriggerAxis() > 0) {
        //intake out
        m_leftIntake.Set(intakeSpeed);
        m_rightIntake.Set(-intakeSpeed);
    }
    else {
        //otherwise set them to 0
        m_leftIntake.Set(0);
        m_rightIntake.Set(0);
    }
    
  }

    //*************************************************AUTONOMUS PART*************************************************************
  void AutonomousInit() override {
    m_timer.Reset();
    m_timer.Start();
    //m_waittimer.Reset();
    //m_waittimer.Start();
  }

  void AutonomousPeriodic() override {
    bool wait = true;
    int x = frc::SmartDashboard::GetNumber("Delay (Sec)", 0);
    double intakeSpeed = frc::SmartDashboard::GetNumber("Intake Speed", 0.8);
    std::string selectedOption = m_chooser.GetSelected();
    //--------------------------DELAY SYSTEM--------------------------------------
    //  if(!wait){
        units::unit_t<units::time::second, double, units::linear_scale> secondsX(x);
    //    //std::this_thread::sleep_for(std::chrono::seconds(seconds));
    //     if(m_timer.Get() < secondsX) {
    //      m_robotDrive.ArcadeDrive(0.0, 0.0, false);
    //    }
    //    else if(m_timer.Get() == secondsX){
    //       m_robotDrive.ArcadeDrive(0.0, 0.0, false);
    //      wait = false;
    //    }
    //    else{
    //     m_robotDrive.ArcadeDrive(0.0, 0.0, false);
    //    }
    //  }

    //---------------------------------------------------------------------------
    //if(wait == false){
    if (selectedOption == "Red_Left") {
      //ROBOT MUST FACE THE DRIVER
      if(m_timer.Get() < secondsX){
        m_robotDrive.ArcadeDrive(0.0, 0.0, false);
      }
      else if(m_timer.Get() < 0.2_s + secondsX){ //+0.75s is perfect time to turn 90 degrees
        m_robotDrive.ArcadeDrive(-0.6, 0.0, false); 
      }
      else if (m_timer.Get() < 0.85_s + secondsX){
        m_robotDrive.TankDrive(0.7, -0.7, false);
      }
      else if (m_timer.Get() < 1.85_s + secondsX){
        m_robotDrive.ArcadeDrive(-0.5, 0, false);
      }
      else if (m_timer.Get() < 2.3_s + secondsX){
        m_robotDrive.TankDrive(0.72, -0.72, false);
      }
      else if (m_timer.Get() < 3.7_s + secondsX){
        m_robotDrive.ArcadeDrive(-0.3, 0, false);
      }
      else if(m_timer.Get() < 15_s + secondsX){
        m_robotDrive.ArcadeDrive(0, 0, false); 
      }
    }
    else if(selectedOption == "Red_Mid"){
      //ROBOT MUST FACE THE FIELD
      if(m_timer.Get() < secondsX){
        m_robotDrive.ArcadeDrive(0.0, 0.0, false);
      }
      else if(m_timer.Get() < 0.75_s + secondsX){ //0.48 if nothing else is wokring is perfect to turn right 90 degrees
        m_robotDrive.TankDrive(-0.55, 0.55, false); 
      }
      else if(m_timer.Get() < 2_s + secondsX){
          m_robotDrive.ArcadeDrive(-0.55, 0.0, false); 
      }
      else if(m_timer.Get() < 2.5_s + secondsX){
          m_robotDrive.ArcadeDrive(0, 0.0, false); 
      }
      else if(m_timer.Get() < 3.2_s + secondsX){ 
        m_robotDrive.TankDrive(0.65, -0.65, false); 
      }
      else if(m_timer.Get() < 3.6_s + secondsX){ 
        m_robotDrive.ArcadeDrive(0, 0, false);
        //m_robotDrive.ArcadeDrive(-.9, 0, false);
      }
      else if(m_timer.Get() < 4.4_s + secondsX){ 
        m_robotDrive.ArcadeDrive(-.9, 0, false);
      }
      else if(m_timer.Get() < 5.65_s + secondsX){ 
        m_robotDrive.ArcadeDrive(0, 0, false);
        //m_robotDrive.TankDrive(0.-7, 0.7, false); 
      }
      else if(m_timer.Get() < 6.7_s + secondsX){ 
        m_robotDrive.TankDrive(-0.6, 0.6, false); 
      }
      else if(m_timer.Get() < 7_s + secondsX){ 
        m_robotDrive.TankDrive(0, 0, false); 
      }
      else if(m_timer.Get() < 9_s + secondsX){ 
        m_robotDrive.ArcadeDrive(-.5, 0, false); 
      }
      else if(m_timer.Get() < 10_s + secondsX){ 
        m_robotDrive.ArcadeDrive(0, 0, false); 
      }
      else if(m_timer.Get() < 10.75_s + secondsX){ 
        m_robotDrive.TankDrive(-0.6, 0.6, false); 
      }
      else if(m_timer.Get() < 15_s + secondsX){
        m_robotDrive.ArcadeDrive(0, 0, false); 
      }
    }
    else if(selectedOption == "Red_Right"){
      //ROBOT MUST FACE THE DRIVER
      if(m_timer.Get() < secondsX){
        m_robotDrive.ArcadeDrive(0.0, 0.0, false);
      }
      else if(m_timer.Get() < 0.2_s + secondsX){
        m_robotDrive.ArcadeDrive(-0.6, 0.0, false); 
      }
      else if (m_timer.Get() < 0.5_s + secondsX){
        m_robotDrive.ArcadeDrive(0.0, 0.0, false); 
          m_leftIntake.Set(intakeSpeed);
          m_rightIntake.Set(-intakeSpeed);
      }
      else if(m_timer.Get() < 1.3_s + secondsX){
          m_robotDrive.TankDrive(-.9, .9, false); //worked on 0.76 and 0.79
          m_leftIntake.Set(0);
          m_rightIntake.Set(0);
      }
      else if(m_timer.Get() < 1.5_s + secondsX){
          m_robotDrive.TankDrive(0, 0.0, false);
      }
      else if(m_timer.Get() < 3_s + secondsX){
          m_robotDrive.ArcadeDrive(-0.65, 0.0, false); 
          m_leftIntake.Set(intakeSpeed);
          m_rightIntake.Set(-intakeSpeed);
      }
      else if(m_timer.Get() < 5_s + secondsX){
          m_robotDrive.ArcadeDrive(0.0, 0.0, false); 
      }
      else if(m_timer.Get() < 6_s + secondsX){
          m_robotDrive.TankDrive(-.8, .8, false); 
      }
      else if(m_timer.Get() < 6.1_s + secondsX){
          m_robotDrive.TankDrive(0, 0, false); 
          m_leftIntake.Set(0);
          m_rightIntake.Set(0); 
      }
      else if(m_timer.Get() < 7.35_s + secondsX){
          m_robotDrive.ArcadeDrive(-0.75, 0.0, false); 
      }
      else if(m_timer.Get() < 8.1_s + secondsX){
          m_robotDrive.ArcadeDrive(0, 0.0, false); 
          m_leftIntake.Set(intakeSpeed);
          m_rightIntake.Set(-intakeSpeed);
      }
      else if(m_timer.Get() < 9.1_s + secondsX){
          m_leftIntake.Set(0);
          m_rightIntake.Set(0);
      }
      else if(m_timer.Get() < 15_s + secondsX){
          m_robotDrive.TankDrive(0, 0.0, false);
      }
    }
    //}
    
  }
  //**************************************************************************************************************************
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
