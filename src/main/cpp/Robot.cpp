//You can tune the P gain to achieve the desired response of the arm motion. 
//Increasing the P gain will make the arm move faster but can also cause overshoot and oscillations. 
//The I and D gains can be added to improve the response and stability of the arm motion. 
//You can experiment with different values to find the best performance for your robot.

#include "Robot.h"
#include "RobotContainer.h"

  //auto options
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoOptions[] = { "NONE", "R_Left", "R_Mid", "R_Right", "R_Left_BR", "R_Right_BR"};



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

     // set the Motion Magic parameters
    const double kMaxVelocity = 1000.0; // adjust as needed
    const double kMaxAcceleration = 5000.0; // adjust as needed
    

    bool coneInt = true;
    
    
 public:
  void RobotInit() override {
    //Magic Motion Variables
    //m_arm.GetPIDController().SetSmartMotionMaxVelocity(kMaxVelocity, 0);
    //m_arm.GetPIDController().SetSmartMotionMaxAccel(kMaxAcceleration, 0);
    //m_arm.GetPIDController().SetSmartMotionAllowedClosedLoopError(0, 0);

    //right motor must be inverted for it to go forward
    m_rightMotors.SetInverted(true);    

    //reset all motors before start
    m_leftMotor1.RestoreFactoryDefaults();
    m_leftMotor2.RestoreFactoryDefaults();
    m_rightMotor1.RestoreFactoryDefaults();
    m_rightMotor2.RestoreFactoryDefaults();
    
    //list of Auto Options
    m_chooser.SetDefaultOption("NONE", kAutoOptions[0]);
    m_chooser.AddOption("R_Left", kAutoOptions[1]);
    m_chooser.AddOption("R_Mid", kAutoOptions[2]);
    m_chooser.AddOption("R_Right", kAutoOptions[3]);
    m_chooser.AddOption("R_Left_BR", kAutoOptions[4]);
    m_chooser.AddOption("R_Right_BR", kAutoOptions[5]);
    frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

    //intake + arm speed
    frc::SmartDashboard::PutNumber("Intake Speed", 0.8);
    frc::SmartDashboard::PutNumber("Arm Speed", 1);

    //initial delay is 0 sec
    frc::SmartDashboard::PutNumber("Delay (Sec)", 0);

    //limintation of speed and rotation sensitivity
    frc::SmartDashboard::PutNumber("Rotation Sesitivity", 0.9);
    frc::SmartDashboard::PutNumber("Speed Sesitivity", 0.85);

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
  }

  void TeleopPeriodic() override {

    //limit edges of the arm
    constexpr int kUpperLimitSwitchPort = 0; //change as needed
    constexpr int kLowerLimitSwitchPort = 1;
    frc::DigitalInput m_upperLimitSwitch(kUpperLimitSwitchPort);
    frc::DigitalInput m_lowerLimitSwitch(kLowerLimitSwitchPort);

    //limtations on speed and rotation
    double m_rot = frc::SmartDashboard::GetNumber("Rotation Sesitivity", 0.9);
    double Totsens = frc::SmartDashboard::GetNumber("Speed Sesitivity", 0.85);

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

    //-------------------------------------INTAKE CODE-----------------------------------------------
    //TODO cgange it to make it easier to understand and adapt to
    if (controller.GetYButton()){coneInt = false;} else { coneInt = true;}
    frc::SmartDashboard::PutBoolean("Cone", coneInt);

    //  double kArmAxis = controllerOP.GetLeftY();

    //  // convert the joystick input to a target position for Motion Magic
    //  const double kTargetRange = 1000.0; // adjust as needed
    //  double targetPosition = armSpeed * kTargetRange;

    //  // set the target position for Motion Magic
    // m_arm.GetPIDController().SetReference(targetPosition, ControlType::kSmartMotion, 0);
    
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
    //m_arm.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, 0);
    //m_arm.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, 0);
    //frc::SmartDashboard::PutNumber("foward tick", e_arm.GetPosition());
    if(controller.GetXButton())
    {
      m_arm.Set(armSpeed);
    }
    else if(controller.GetBButton())
    {
      m_arm.Set(-armSpeed);
      //m_arm.GetEncoder(rev::CANEncoder::EncoderType::kHallSensor, 42).SetPosition();
    }
    // else if (m_upperLimitSwitch.Get() || m_lowerLimitSwitch.Get())
    // {
    //  m_arm.Set(0.0);
    // }
    else
    {
      m_arm.Set(0.0);
    }
  }
    //*************************************************AUTONOMUS CODE*************************************************************
  void AutonomousInit() override {
    m_timer.Reset();
    m_timer.Start();
  }

  void AutonomousPeriodic() override {
    //TODO - Auto is not working somehow. Motors are not checked frequently enough
    //Possible solution - All autos must be 15 seconds even when real auto is 8

    //initial values for the auto
    int x = frc::SmartDashboard::GetNumber("Delay (Sec)", 0);
    //inital speed of the intake is 40% for the sake of testing
    double intakeSpeed = frc::SmartDashboard::GetNumber("Intake Speed", 0.4);
    
    //get the selected auto option
    std::string selectedOption = m_chooser.GetSelected();
    //convertion of double into seconds
    units::unit_t<units::time::second, double, units::linear_scale> secondsX(x);


    if(selectedOption == "R_Mid"){
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
      else{
        m_robotDrive.TankDrive(0, 0, false);
      }
    }

    // else if(selectedOption == "Red_Mid_ScoreTwo"){
    //   //MUST FACE THE DRIVER
    //   if(m_timer.Get() < secondsX){
    //     m_robotDrive.ArcadeDrive(0.0, 0.0, false);
    //   }
    //   else if(m_timer.Get() < 0.15_s + secondsX){
    //     m_robotDrive.TankDrive(0.6, 0.6, false); 
    //   }
    //   else if(m_timer.Get() < 1.3_s + secondsX){
    //     m_robotDrive.TankDrive(-0.53, -0.53, false); 
    //   }
    //   else if(m_timer.Get() < 1.35_s + secondsX){
    //     m_robotDrive.TankDrive(0.8, 0.8, false); 
    //   }
    //   else if(m_timer.Get() < 1.7_s + secondsX){
    //     m_robotDrive.TankDrive(0, 0, false); 
    //   }
    //   else if(m_timer.Get() < 3.3_s + secondsX){
    //     m_robotDrive.TankDrive(-0.3, -0.3, false); 
    //   }
    //   else if(m_timer.Get() < 3.9_s + secondsX){
    //     m_robotDrive.TankDrive(-0.6, -0.6, false);
    //   }
    //   else if(m_timer.Get() < 15_s + secondsX){
    //     m_robotDrive.TankDrive(0, 0, false);
    //   }
    // }

    else if(selectedOption == "R_Left_BR"){
     //ROBOT MUST FACE THE DRIVER
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

    else if(selectedOption == "R_Left"){
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
      else{
        m_robotDrive.TankDrive(0, 0, false);
      }
    }

    else if(selectedOption == "Red_right_BR"){
      //ROBOT MUST FACE THE DRIVER
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