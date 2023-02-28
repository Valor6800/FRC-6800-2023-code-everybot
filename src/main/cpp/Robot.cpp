#include "frc/XboxController.h"
#include <frc/TimedRobot.h>
#include <frc/RobotState.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/Timer.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/CANSparkMax.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <rev/MotorFeedbackSensor.h>
#include <frc/PowerDistribution.h>
#include <networktables/NetworkTableInstance.h>

  //auto options
   frc::SendableChooser<std::string> m_chooser;
   const std::string kAutoOptions[] = { "NONE", "Red_Mid_BR", "Red_left_BR", "Red_left_PushGo", "Red_right_BR", "Red_Mid_ScoreTwo", "Red_right_PushGo", "Blue_Mid_BR", "Blue_left_BR", "Blue_left_PushGo", "Blue_right_BR", "Blue_Mid_ScoreTwo", "Blue_right_PushGo"};


  static const int ARM_CURRENT_LIMIT_A = 20;
  static const double ARM_OUTPUT_POWER = 0.4;
  static const int INTAKE_CURRENT_LIMIT_A = 25;
  static const int INTAKE_HOLD_CURRENT_LIMIT_A = 5;
  static const double INTAKE_OUTPUT_POWER = 1.0;
  static const double INTAKE_HOLD_POWER = 0.07;
  static const double ARM_EXTEND_TIME_S = 2.0;
  static const double AUTO_THROW_TIME_S = 0.375;
  static const double AUTO_DRIVE_TIME = 6.0;
  static const double AUTO_DRIVE_SPEED = -0.25;


class Robot : public frc::TimedRobot {

  //Replace/DELETE
  rev::CANSparkMax m_rightIntake{11, rev::CANSparkMax::MotorType::kBrushed}; //CHANGE IDS FOR INTAKE
  rev::CANSparkMax m_leftIntake{2, rev::CANSparkMax::MotorType::kBrushed};
  
  //Motorcontroller for intake (1 for arm, 1 for intake itself)
  rev::CANSparkMax m_leftIntake{2, rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax m_leftIntake{2, rev::CANSparkMax::MotorType::kBrushed};

  //4 main drive motor cotnrollers
  rev::CANSparkMax m_leftMotor1{12, rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax m_leftMotor2{5, rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax m_rightMotor1{4, rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax m_rightMotor2{1, rev::CANSparkMax::MotorType::kBrushed};

  //create a MotorControllerGroup to combine all left and right motors
  frc::MotorControllerGroup m_leftMotors{m_leftMotor1, m_leftMotor2};
  frc::MotorControllerGroup m_rightMotors{m_rightMotor1, m_rightMotor2};

  //Combine both motor groups
  frc::DifferentialDrive m_robotDrive{m_leftMotors, m_rightMotors};
  frc::XboxController controller{0}; 
  //Operator controller
  frc::XboxController controllerOP{1};

  //Timer for Auto
  frc::Timer m_timer;

 public:
  void RobotInit() override {
    //reset all motors before start
    //need to add more as intake is developed
    m_leftIntake.RestoreFactoryDefaults();
    m_rightIntake.RestoreFactoryDefaults();
    m_leftMotor1.RestoreFactoryDefaults();
    m_leftMotor2.RestoreFactoryDefaults();
    m_rightMotor1.RestoreFactoryDefaults();
    m_rightMotor2.RestoreFactoryDefaults();

    //right motor must be inverted for it to go forward
    m_rightMotors.SetInverted(true);
    
    //list of Auto Options
    m_chooser.SetDefaultOption("NONE", kAutoOptions[0]);
    m_chooser.AddOption("Red_Mid_BR", kAutoOptions[1]);
    m_chooser.AddOption("Red_left_BR", kAutoOptions[2]);
    m_chooser.AddOption("Red_left_PushGo", kAutoOptions[3]);
    m_chooser.AddOption("Red_right_BR", kAutoOptions[4]);
    m_chooser.AddOption("Red_Mid_ScoreTwo", kAutoOptions[5]);
    m_chooser.AddOption("Red_right_PushGo", kAutoOptions[6]);
    m_chooser.AddOption("Blue_Mid_BR", kAutoOptions[7]);
    m_chooser.AddOption("Blue_left_BR", kAutoOptions[8]);
    m_chooser.AddOption("Blue_left_PushGo", kAutoOptions[9]);
    m_chooser.AddOption("Blue_right_BR", kAutoOptions[10]);
    m_chooser.AddOption("Blue_Mid_ScoreTwo", kAutoOptions[11]);
    m_chooser.AddOption("Blue_right_PushGo", kAutoOptions[12]);
    frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

    //give a 0 as initial number for intake speed
    frc::SmartDashboard::PutNumber("Intake Speed", 0.4);
    //initial delay is 0 sec
    frc::SmartDashboard::PutNumber("Delay (Sec)", 0);

    //create one more to limit a specific motor

    frc::SmartDashboard::PutNumber("Rotation Sesitivity", 1);
    frc::SmartDashboard::PutNumber("Speed Sesitivity", 1);

    frc::SmartDashboard::PutNumber("Left Motor Limit", 65); //initially 100%
    frc::SmartDashboard::PutNumber("Right Motor Limit", 100);
  }

  void TeleopPeriodic() override {
//---------------------------------------------------VOLTAGE COLLECTOR--------------------------------------------
 /*
    frc::PowerDistribution pdp;
    nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();

    double voltage = 0;

    voltage = pdp.GetVoltage();
    frc::SmartDashboard::PutNumber("Voltage", voltage);
*/
//----------------------------------------------------------------------------------------------------------------------

    //limtations on speed and rotation
    double m_rot = frc::SmartDashboard::GetNumber("Rotation Sesitivity", 0.7);
    double Totsens = frc::SmartDashboard::GetNumber("Speed Sesitivity", 0.7);

    double sensLeft = frc::SmartDashboard::GetNumber("Left Motor Limit", 65); //65%?
    double sensRight = frc::SmartDashboard::GetNumber("Right Motor Limit", 100);

    m_leftMotor1.SetSmartCurrentLimit(sensLeft);
    m_leftMotor2.SetSmartCurrentLimit(sensLeft);
    m_rightMotor1.SetSmartCurrentLimit(sensRight);
    m_rightMotor2.SetSmartCurrentLimit(sensRight);
    m_leftMotor1.EnableVoltageCompensation(12.3);
    m_leftMotor2.EnableVoltageCompensation(12.3);
    m_rightMotor1.EnableVoltageCompensation(12.3);
    m_rightMotor2.EnableVoltageCompensation(12.3);
    //add one more for intake

    // Drive the robot forward when the A button is pressed
    // Otherwise, set the motors to zero
    if(controller.GetAButton() > 0){
     m_robotDrive.TankDrive(0.7, 0.7);
    }else{
      m_robotDrive.TankDrive(0, 0);
    }

    

    //motor speed and rotation variables from controller for ArcadeDrive
    double speed = controller.GetLeftY();
    double rotation = controller.GetRightX();


    m_robotDrive.ArcadeDrive(speed * Totsens, rotation * m_rot);

    //initial speed of the intake will be 40%
    double intakeSpeed = frc::SmartDashboard::GetNumber("Intake Speed", 0.4);


    //INTAKE CODE HERE:

    //use left + right triggers (controllerOP)
    
  }
    //*************************************************AUTONOMUS PART BELLOW*************************************************************
  void AutonomousInit() override {
    m_timer.Reset();
    m_timer.Start();
  }

  void AutonomousPeriodic() override {
    //initial values for the auto
    int x = frc::SmartDashboard::GetNumber("Delay (Sec)", 0);
    //inital speed of the intake is 40% for the sake of testing
    double intakeSpeed = frc::SmartDashboard::GetNumber("Intake Speed", 0.4);
    std::string selectedOption = m_chooser.GetSelected();
    //convertion of double into seconds
    units::unit_t<units::time::second, double, units::linear_scale> secondsX(x);


    if(selectedOption == "Red_Mid_BR"){
      //MUST FACE THE DRIVER
      if(m_timer.Get() < secondsX){
        m_robotDrive.ArcadeDrive(0.0, 0.0, false);
      }
      else if(m_timer.Get() < 0.15_s + secondsX){
        m_robotDrive.TankDrive(0.6, 0.6, false); 
      }
      else if(m_timer.Get() < 1.3_s + secondsX){
        m_robotDrive.TankDrive(-0.7, -0.7, false); //initially 0.9 
      }
      else if(m_timer.Get() < 1.35_s + secondsX){
        m_robotDrive.TankDrive(0.8, 0.8, false); 
      }
      else if(m_timer.Get() < 1.7_s + secondsX){
        m_robotDrive.TankDrive(0, 0, false); 
      }
    }
    
    else if(selectedOption == "Blue_mid_BR"){
      //MUST FACE THE DRIVER
      if(m_timer.Get() < secondsX){
        m_robotDrive.ArcadeDrive(0.0, 0.0, false);
      }
      else if(m_timer.Get() < 0.15_s + secondsX){
        m_robotDrive.TankDrive(0.6, 0.6, false); 
      }
      else if(m_timer.Get() < 1.3_s + secondsX){
        m_robotDrive.TankDrive(-0.7, -0.7, false); //initially 0.9 
      }
      else if(m_timer.Get() < 1.35_s + secondsX){
        m_robotDrive.TankDrive(0.8, 0.8, false); 
      }
      else if(m_timer.Get() < 1.7_s + secondsX){
        m_robotDrive.TankDrive(0, 0, false); 
      }
    }

    else if(selectedOption == "Red_Mid_ScoreTwo"){
      //MUST FACE THE DRIVER
      if(m_timer.Get() < secondsX){
        m_robotDrive.ArcadeDrive(0.0, 0.0, false);
      }
      else if(m_timer.Get() < 0.15_s + secondsX){
        m_robotDrive.TankDrive(0.6, 0.6, false); 
      }
      else if(m_timer.Get() < 1.3_s + secondsX){
        m_robotDrive.TankDrive(-0.53, -0.53, false); 
      }
      else if(m_timer.Get() < 1.35_s + secondsX){
        m_robotDrive.TankDrive(0.8, 0.8, false); 
      }
      else if(m_timer.Get() < 1.7_s + secondsX){
        m_robotDrive.TankDrive(0, 0, false); 
      }
      else if(m_timer.Get() < 3.3_s + secondsX){
        m_robotDrive.TankDrive(-0.3, -0.3, false); 
      }
      else if(m_timer.Get() < 3.9_s + secondsX){
        m_robotDrive.TankDrive(-0.6, -0.6, false);
      }
      else if(m_timer.Get() < 15_s + secondsX){
        m_robotDrive.TankDrive(0, 0, false);
      }
    }

    else if(selectedOption == "Red_left_BR"){
     //ROBOT MUST FACE THE DRIVER

      //What this Auto is doing(TankDrive ONLY):

      //1. facing driver it goes a little forward with object and release it
      //2. going backwards
      //3. tiurn left - 90 degrees
      //4. A little forward
      //5. turn right - 90 degrees
      //6. jump onto the bridge
      if(m_timer.Get() < secondsX){                     // PERFECT VOLTAGE - 12.3 - 12.5
        m_robotDrive.ArcadeDrive(0.0, 0.0, false);
      }
      else if(m_timer.Get() < 0.15_s + secondsX){
        m_robotDrive.TankDrive(-0.6, -0.6, false); 
      }
      else if(m_timer.Get() < 1.5_s + secondsX){
        m_robotDrive.TankDrive(0.9, 0.9, false);  //this is going between 0.15 and 1.2
      }
      else if(m_timer.Get() < 1.7_s + secondsX){
        m_robotDrive.TankDrive(0, 0, false); 
      }
      else if(m_timer.Get() < 2_s + secondsX){
        m_robotDrive.TankDrive(0.65, -0.65, false); 
      }
      else if(m_timer.Get() < 2.6_s + secondsX){ //0.3sec to turn 90 degrees with speed 0.62
        m_robotDrive.TankDrive(-0.6, -0.6, false); 
      }
      else if(m_timer.Get() < 3.1_s + secondsX){
        m_robotDrive.TankDrive(0, 0, false); 
      }
      else if(m_timer.Get() < 3.4_s + secondsX){
        m_robotDrive.TankDrive(-0.57, 0.57, false); 
      }
      else if(m_timer.Get() < 4.65_s + secondsX){
        m_robotDrive.TankDrive(-0.8, -0.8, false); 
      }
      else if(m_timer.Get() < 4.7_s + secondsX){
        m_robotDrive.TankDrive(0.8, 0.8, false); 
      }
      else if(m_timer.Get() < 6_s + secondsX){
        m_robotDrive.TankDrive(0, 0, false); 
      }
    }

    else if(selectedOption == "Red_left_PushGo"){
      if(m_timer.Get() < secondsX){
        m_robotDrive.ArcadeDrive(0.0, 0.0, false);
      }
      else if(m_timer.Get() < 0.15_s + secondsX){
        m_robotDrive.TankDrive(-0.6, -0.6, false); 
      }
      else if(m_timer.Get() < 1.5_s + secondsX){
        m_robotDrive.TankDrive(0.9, 0.9, false); 
      }
      else if(m_timer.Get() < 1.7_s + secondsX){
        m_robotDrive.TankDrive(0, 0, false); 
      }
    }

    else if(selectedOption == "Red_right_BR"){
      //ROBOT MUST FACE THE DRIVER

      //What this Auto is doing(TankDrive ONLY):

      //1. facing driver it goes a little forward with object and release it
      //2. going backwards
      //3. tiurn left - 90 degrees
      //4. A little forward
      //5. turn right - 90 degrees
      //6. jump onto the bridge
      if(m_timer.Get() < secondsX){                     // PERFECT VOLTAGE - 12.3 - 12.5
        m_robotDrive.ArcadeDrive(0.0, 0.0, false);
      }
      else if(m_timer.Get() < 0.15_s + secondsX){
        m_robotDrive.TankDrive(-0.6, -0.6, false); 
      }
      else if(m_timer.Get() < 1.5_s + secondsX){
        m_robotDrive.TankDrive(0.9, 0.9, false);  //this is going between 0.15 and 1.2
      }
      else if(m_timer.Get() < 1.7_s + secondsX){
        m_robotDrive.TankDrive(0, 0, false); 
      }
      else if(m_timer.Get() < 2_s + secondsX){
        m_robotDrive.TankDrive(-0.7, 0.7, false);  //initially 0.65
      }
      else if(m_timer.Get() < 2.6_s + secondsX){ //0.3sec to turn 90 degrees with speed 0.62
        m_robotDrive.TankDrive(-0.6, -0.6, false); 
      }
      else if(m_timer.Get() < 3.1_s + secondsX){
        m_robotDrive.TankDrive(0, 0, false); 
      }
      else if(m_timer.Get() < 3.4_s + secondsX){
        m_robotDrive.TankDrive(0.57, -0.57, false); 
      }
      else if(m_timer.Get() < 4.65_s + secondsX){
        m_robotDrive.TankDrive(-0.8, -0.8, false); 
      }
      else if(m_timer.Get() < 4.7_s + secondsX){
        m_robotDrive.TankDrive(0.8, 0.8, false); 
      }
      else if(m_timer.Get() < 6_s + secondsX){
        m_robotDrive.TankDrive(0, 0, false); 
      }
    }
  }
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
