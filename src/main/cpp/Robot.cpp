#include "Robot.h"
#include "RobotContainer.h"

  //auto options
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoOptions[] = { "NONE", "Red_Mid_BR", "Red_left_BR", "Red_left_PushGo", "Red_right_BR", "Red_Mid_ScoreTwo", "Red_right_PushGo", "Blue_Mid_BR", "Blue_left_BR", "Blue_left_PushGo", "Blue_right_BR", "Blue_Mid_ScoreTwo", "Blue_right_PushGo"};



class Robot : public frc::TimedRobot {

  //4 main drive motor cotnrollers
    rev::CANSparkMax m_leftMotor1{12, rev::CANSparkMax::MotorType::kBrushed};
    rev::CANSparkMax m_leftMotor2{5, rev::CANSparkMax::MotorType::kBrushed};
    rev::CANSparkMax m_rightMotor1{4, rev::CANSparkMax::MotorType::kBrushed};
    rev::CANSparkMax m_rightMotor2{1, rev::CANSparkMax::MotorType::kBrushed};

    //combine left + right motors
    frc::MotorControllerGroup m_leftMotors{m_leftMotor1, m_leftMotor2};
    frc::MotorControllerGroup m_rightMotors{m_rightMotor1, m_rightMotor2};

    //Combine both motor groups
    frc::DifferentialDrive m_robotDrive{m_leftMotors, m_rightMotors};

    //Intake + arm controllers
    rev::CANSparkMax m_intake{3, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax m_arm{8, rev::CANSparkMax::MotorType::kBrushless};

    //Controller for driver
    frc::XboxController controller{0}; 
    //Operator controller
    frc::XboxController controllerOP{1};

    //Timer for Auto
    frc::Timer m_timer;

    bool coneInt = true;
    
    
 public:
  void RobotInit() override {

    //right motor must be inverted for it to go forward
    m_rightMotors.SetInverted(true);
    //reset the intake settings to default
    

    //reset all motors before start
    //need to add more as intake is developed
    m_leftMotor1.RestoreFactoryDefaults();
    m_leftMotor2.RestoreFactoryDefaults();
    m_rightMotor1.RestoreFactoryDefaults();
    m_rightMotor2.RestoreFactoryDefaults();
    
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

    //intake + arm speed
    frc::SmartDashboard::PutNumber("Intake Speed", 0.8);
    frc::SmartDashboard::PutNumber("Arm Speed", 1);

    //initial delay is 0 sec
    frc::SmartDashboard::PutNumber("Delay (Sec)", 0);

    //limintation of speed and rotation sensitivity
    frc::SmartDashboard::PutNumber("Rotation Sesitivity", 1);
    frc::SmartDashboard::PutNumber("Speed Sesitivity", 1);

    //boolean to determine if we picking up a code - initially false
    frc::SmartDashboard::PutBoolean("Cone", false);

    frc::SmartDashboard::PutNumber("Left Motor Limit", 1); //initially 65%
    frc::SmartDashboard::PutNumber("Right Motor Limit", 1);
    
    // In your periodic function, get the battery voltage and send it to SmartDashboard
    //double voltage = pdp.GetVoltage();
    //frc::SmartDashboard::PutNumber("Battery Voltage", voltage);

    //limit motors to a specific voltage
    m_leftMotor1.EnableVoltageCompensation(12.3);
    m_leftMotor2.EnableVoltageCompensation(12.3);
    m_rightMotor1.EnableVoltageCompensation(12.3);
    m_rightMotor2.EnableVoltageCompensation(12.3);

    frc::SmartDashboard::PutNumber("foward tick", 0);
  }

  void TeleopPeriodic() override {

    //limtations on speed and rotation
    double m_rot = frc::SmartDashboard::GetNumber("Rotation Sesitivity", 0.8);
    double Totsens = frc::SmartDashboard::GetNumber("Speed Sesitivity", 0.8);

    //numbers to limit both motor's capacity
     double sensLeft = frc::SmartDashboard::GetNumber("Left Motor Limit", 1); //65%?
     double sensRight = frc::SmartDashboard::GetNumber("Right Motor Limit", 1);

    //initial speed of the intake will be 80% + arm 100%
    double intakeSpeed = frc::SmartDashboard::GetNumber("Intake Speed", 0.8);
    double armSpeed = frc::SmartDashboard::GetNumber("Arm Speed", 1);



    //--------------------------------------DRIVE CODE HERE:--------------------------------------------
    //IMPORTANT: when mototr is - its going forward, and when its + its going backward (Its wierd but important for the auto)

    //motor speed and rotation variables from controller for ArcadeDrive
    double speed = controller.GetLeftY();
    double rotation = controller.GetRightX();

    //main drive function 
    m_robotDrive.ArcadeDrive(speed * Totsens, rotation * m_rot);

    // Drive the robot forward when the A button is pressed
     if(controller.GetAButton() > 0){
      m_robotDrive.TankDrive(-0.7, -0.7);
     }else{
       m_robotDrive.ArcadeDrive(speed * Totsens, rotation * m_rot);
     }

    //-------------------------------------INTAKE CODE HERE:-----------------------------------------------
    //TODO cgange it to make it easier to understand and adapt to
    if (controller.GetYButtonReleased()){coneInt = false;} else { coneInt = true;}
    frc::SmartDashboard::PutBoolean("Cone", coneInt);
    
    //use controllerOP for the second controller
    //TODO move it to controllerOP + modify as needed
     if(controller.GetLeftTriggerAxis() != 0){
       if (coneInt){
        m_intake.Set(-intakeSpeed); //may need to manually change the values
       } else{
        m_intake.Set(intakeSpeed);
       }
       
     }
     else if (controller.GetRightTriggerAxis() != 0) {
      if (coneInt){
        m_intake.Set(intakeSpeed);
       } else{
        m_intake.Set(-intakeSpeed);
       }
     }
     else{
       m_intake.Set(0.0);
     }

     //-------------------------------------ARM CONTROLLER CODE--------------------------------------------
    m_arm.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, 0);
    //m_arm.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, 0);
   // frc::SmartDashboard::PutNumber("foward tick", e_arm.GetPosition());
    if(controller.GetXButton()){
      m_arm.Set(0.6);
    }else if(controller.GetBButton()){
      m_arm.Set(-0.6);
      // m_arm.GetEncoder(rev::CANEncoder::EncoderType::kHallSensor, 42).SetPosition()
    }else{
      m_arm.Set(0.0);
    }
  }
    //*************************************************AUTONOMUS CODE BELLOW*************************************************************
  void AutonomousInit() override {
    m_timer.Reset();
    m_timer.Start();
  }

  void AutonomousPeriodic() override {
    //TODO - Auto is not working somehow. Intake is not checked enough
    //Possible solution - All autos must be 15 seconds even when real auto is 8
    
    //initial values for the auto
    int x = frc::SmartDashboard::GetNumber("Delay (Sec)", 0);
    //inital speed of the intake is 40% for the sake of testing
    double intakeSpeed = frc::SmartDashboard::GetNumber("Intake Speed", 0.4);
    
    //get the selected auto option
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
      if(m_timer.Get() < secondsX){ // PERFECT VOLTAGE - 12.3 - 12.5
        m_robotDrive.ArcadeDrive(0.0, 0.0, false);
      }
      else if(m_timer.Get() < 0.15_s + secondsX){
        m_robotDrive.TankDrive(-0.6, -0.6, false); 
      }
      else if(m_timer.Get() < 1.5_s + secondsX){
        m_robotDrive.TankDrive(0.9, 0.9, false); //this is going between 0.15 and 1.2
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
      if(m_timer.Get() < secondsX){ // PERFECT VOLTAGE - 12.3 - 12.5
        m_robotDrive.ArcadeDrive(0.0, 0.0, false);
      }
      else if(m_timer.Get() < 0.15_s + secondsX){
        m_robotDrive.TankDrive(-0.6, -0.6, false); 
      }
      else if(m_timer.Get() < 1.5_s + secondsX){
        m_robotDrive.TankDrive(0.9, 0.9, false); //this is going between 0.15 and 1.2
      }
      else if(m_timer.Get() < 1.7_s + secondsX){
        m_robotDrive.TankDrive(0, 0, false); 
      }
      else if(m_timer.Get() < 2_s + secondsX){
        m_robotDrive.TankDrive(-0.7, 0.7, false); //initially 0.65
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