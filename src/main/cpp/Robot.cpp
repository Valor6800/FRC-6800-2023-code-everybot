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
   const std::string kAutoOptions[] = { "NONE", "Red_Mid", "Red_Right", "Red_Left", "Red_BR" };


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

  //LEFT + RIGHT INTAKE (Replace with new motor controllers)
  rev::CANSparkMax m_rightIntake{11, rev::CANSparkMax::MotorType::kBrushed}; //CHANGE IDS FOR INTAKE
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
    m_chooser.AddOption("Red_Mid", kAutoOptions[1]);
    m_chooser.AddOption("Red_Right", kAutoOptions[2]);
    m_chooser.AddOption("Red_Left", kAutoOptions[3]);
    m_chooser.AddOption("Red_BR", kAutoOptions[4]);
    frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

    //give a 0 as initial number for intake speed
    frc::SmartDashboard::PutNumber("Intake Speed", 0.4);
    //initial delay is 0 sec
    frc::SmartDashboard::PutNumber("Delay (Sec)", 0);

    //create one more to limit a specific motor

    frc::SmartDashboard::PutNumber("Rotation Power", 1);
    frc::SmartDashboard::PutNumber("Speed Power", 1);

  }

  void TeleopPeriodic() override {
//---------------------------------------------------VOLTAGE COLLECTOR--------------------------------------------
    frc::PowerDistribution pdp;
    nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();

    double voltage = 0;

    voltage = pdp.GetVoltage();
    frc::SmartDashboard::PutNumber("Voltage", voltage);

//----------------------------------------------------------------------------------------------------------------------

    //limtations on speed and rotation
    double m_rot = frc::SmartDashboard::GetNumber("Rotation Power", 1);
    double sens = frc::SmartDashboard::GetNumber("Speed Power", 1);

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

    m_robotDrive.ArcadeDrive(speed * sens, rotation * m_rot);

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


    if (selectedOption == "Red_Left") {
      //ROBOT MUST FACE THE DRIVER

      //What this Auto is doing:
      //1. facing driver it goes a little forward with object and release it
      //2. turn 90 degrees to the left + forward for a few meters
      //3. turn left again facing the bridge
      //4. getting up to the bridge with lower speed 
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
      //ROBOT MUST FACE THE DRIVER

      //What this Auto is doing:
      //1. facing driver it goes a little forward with object and release it
      //2. turn 180 degrees to the left + forward - slow to get through te bridge
      //3. get the object in the middle of the field
      //4. may chose to go both to the left or right.
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
      }
      else if(m_timer.Get() < 4.4_s + secondsX){ 
        m_robotDrive.ArcadeDrive(-.9, 0, false);
      }
      else if(m_timer.Get() < 5.65_s + secondsX){ 
        m_robotDrive.ArcadeDrive(0, 0, false);
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

      //What this Auto is doing:
      //1. facing driver it goes a little forward with object and release it
      //2. turn 180 degrees to the left + forward until obejct is hit
      //3. capture the object and turn 180 degrees
      //4. go back and put the object in different place
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
          m_robotDrive.TankDrive(-.9, .9, false); //worked on 0.76 when charge is 100%
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

    else if(selectedOption == "Red_BR"){
     //ROBOT MUST FACE THE DRIVER

      //What this Auto is doing:
      //1. facing driver it goes a little forward with object and release it
      //2. turn 180 degrees to the left + forward until obejct is hit
      //3. capture the object and turn 180 degrees
      //4. go back and put the object in different place
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
          m_robotDrive.TankDrive(-.9, .9, false); //worked on 0.76 when charge is 100%
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
  }
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
