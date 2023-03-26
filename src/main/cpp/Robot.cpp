//You can tune the P gain to achieve the desired response of the arm motion. 
//Increasing the P gain will make the arm move faster but can also cause overshoot and oscillations. 
//The I and D gains can be added to improve the response and stability of the arm motion. 
//You can experiment with different values to find the best performance for your robot.

#include "Robot.h"
#include "RobotContainer.h"

  //auto options
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoOptions[] = { "NONE", "R_Left", "R_Mid", "R_Right", "R_Left_BR", "R_Right_BR", "TEST"};



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

    rev::SparkMaxRelativeEncoder m_encoder = m_arm.GetEncoder();

    //Controller for driver
    frc::XboxController controller{0}; 
    //Operator controller
    frc::XboxController controllerOP{1};

    //Timer for Auto
    frc::Timer m_timer;

    //NavX
    AHRS m_navx{frc::SPI::Port::kMXP};

     // set the int parameters
    const double kP = 0.05;
    const double kI = 0.0;
    const double kD = 0.0;
    const double kF = 0.0;
    const double kToleranceDegrees = 2.0;

    // create a PID controller
    frc2::PIDController m_autoControlPID{kP, kI, kD};

     // Output value for the PID controller
    double m_output = 0.0;
    bool button_pressed = false;
    int motor_direction = 0;
    int motor_position = 0;
    double m_upperLimit = 90.0;  // example upper limit in degrees
    double m_lowerLimit = 0.0;  // example lower limit in degrees
    bool coneInt = true;
    
    
 public:
  void RobotInit() override {

     m_arm.RestoreFactoryDefaults();  // reset motor controller to default settings
     m_encoder.SetPosition(0.0);  // reset encoder to zero position
    

    // Set up the PID controller
    m_autoControlPID.SetP(kP);
    m_autoControlPID.SetI(kI);
    m_autoControlPID.SetD(kD);
    m_autoControlPID.SetTolerance(kToleranceDegrees);

    // Set the controller to be continuous (because it is an angle controller)
    m_navx.Calibrate();

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
    m_chooser.AddOption("TEST", kAutoOptions[6]);
    frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

    //intake + arm speed
    frc::SmartDashboard::PutNumber("Intake Speed", 1);
    frc::SmartDashboard::PutNumber("Arm Speed", 0.5);

    //used to count NEO motor rotations to fully open an arm
    frc::SmartDashboard::PutNumber("Arm Rotation", 0);

    //initial delay is 0 sec
    frc::SmartDashboard::PutNumber("Delay (Sec)", 0);

    //limintation of speed and rotation sensitivity
    frc::SmartDashboard::PutNumber("Rotation Sesitivity", 0.9);
    frc::SmartDashboard::PutNumber("Speed Sesitivity", 1);

    //boolean to determine if we picking up a code - initially false
    frc::SmartDashboard::PutBoolean("Cone", false);

    frc::SmartDashboard::PutNumber("Left Motor Limit", 1); //initially 65%
    frc::SmartDashboard::PutNumber("Right Motor Limit", 1);

    //limit motors to a specific voltage
    m_leftMotor1.EnableVoltageCompensation(12.3);
    m_leftMotor2.EnableVoltageCompensation(12.3);
    m_rightMotor1.EnableVoltageCompensation(12.3);
    m_rightMotor2.EnableVoltageCompensation(12.3);
  }

  void TeleopPeriodic() override {

    // Read button input from joystick
    bool button_state = false;

    if (controller.GetXButtonReleased())
    {
      button_state = !button_state;
    } 

    //limtations on speed and rotation
    double m_rot = frc::SmartDashboard::GetNumber("Rotation Sesitivity", 0.9);
    double Totsens = frc::SmartDashboard::GetNumber("Speed Sesitivity", 1);

    //numbers to limit both motor's capacity
    double sensLeft = frc::SmartDashboard::GetNumber("Left Motor Limit", 1); //65%? 0.65
    double sensRight = frc::SmartDashboard::GetNumber("Right Motor Limit", 1);

    //initial speed of the intake will be 80% + arm 100%
    double intakeSpeed = frc::SmartDashboard::GetNumber("Intake Speed", 1);
    double armSpeed = frc::SmartDashboard::GetNumber("Arm Speed", 0.5);



    //--------------------------------------DRIVE CODE HERE:--------------------------------------------
    //IMPORTANT: when mototr is - its going forward, and when its + its going backward (Its wierd but important for the auto)

    //motor speed and rotation variables from controller for ArcadeDrive
    double speed = controller.GetLeftY(); 
    double SQRTspeed = sqrt(speed);
    double rotation = controller.GetRightX(); 
    double SQRTrotation = sqrt(rotation);

    //main drive function 
    m_robotDrive.ArcadeDrive(SQRTspeed * Totsens, SQRTrotation * m_rot);

    // Drive the robot forward when the A button is pressed
     if(controller.GetAButton() > 0){
      m_robotDrive.TankDrive(-0.7, -0.7);
     }
     else
     {
       m_robotDrive.ArcadeDrive(speed * Totsens, rotation * m_rot);
     }

    //-------------------------------------INTAKE CODE-----------------------------------------------
    if (controller.GetYButtonReleased())      
    {
      coneInt = !coneInt;
    } 

    if(!coneInt){

    }
    else{

    }

    frc::SmartDashboard::PutBoolean("Cone", coneInt);

    // Get the number of rotations of the motor
    double numRotations = m_encoder.GetPosition() / 42.0;
    if(controller.GetLeftStickButtonReleased() > 0){
      m_encoder.SetPosition(0);
    }
    frc::SmartDashboard::PutNumber("Arm Rotation", numRotations);
    
    //TODO move it to controllerOP + modify as needed
    if(controllerOP.GetLeftTriggerAxis() != 0){
      if (coneInt){
      m_intake.Set(-intakeSpeed); //may need to manually change the values
      } else{
      m_intake.Set(intakeSpeed * 0.3); //only 30% are availiable
      }
      
    }
     //TODO move it to controllerOP + modify as needed
    else if (controllerOP.GetRightTriggerAxis() != 0) 
    {
    if (coneInt){
      m_intake.Set(intakeSpeed);
      } else{
      m_intake.Set(-intakeSpeed * 0.3);
      }
     }
     else
     {
       m_intake.Set(0.0);
     }



    //-------------------------------------ARM CONTROLLER CODE--------------------------------------------
    double armPosition = m_encoder.GetPosition();  // get current arm position from encoder
    double armSpeed = controllerOP.GetLeftY();  // read joystick Y axis for arm speed control

    // calculate proportional speed reduction as arm approaches limit
    double limitError = 0.0;  // distance to limit from current position
    double limitThreshold = 0.1;  // distance at which speed reduction starts
    double maxSpeedReduction = 0.5;  // maximum speed reduction factor (0.0 to 1.0)
    if (armPosition > m_upperLimit - limitThreshold) {
        limitError = m_upperLimit - armPosition;
    } else if (armPosition < m_lowerLimit + limitThreshold) {
        limitError = m_lowerLimit - armPosition;
    }
    double speedReduction = std::min(1.0, limitError / limitThreshold * maxSpeedReduction);

    // apply speed reduction to arm speed
    if (armSpeed > 0.0) {
        armSpeed *= (1.0 - speedReduction);  // reduce speed if moving up
    } else if (armSpeed < 0.0) {
        armSpeed *= (1.0 + speedReduction);  // reduce speed if moving down
    }

    // check if limit is reached and stop motor if necessary
    bool limitReached = false;
    if (armPosition >= m_upperLimit) {
        armSpeed = 0.0;
        limitReached = true;
    } else if (armPosition <= m_lowerLimit) {
        armSpeed = 0.0;
        limitReached = true;
    }

    // apply arm speed to motor controller
    m_armMotor.Set(armSpeed);

    // update dashboard with arm position and limit status
    frc::SmartDashboard::PutNumber("Arm Position", armPosition);
    frc::SmartDashboard::PutBoolean("Limit Reached", limitReached);
  }
    //*************************************************AUTONOMUS CODE*************************************************************
  void AutonomousInit() override {
    m_autoControlPID.Reset();
    m_navx.Reset();
    m_timer.Reset();
    m_timer.Start();
  }

  void AutonomousPeriodic() override {
    //TODO - Auto is not working somehow. Motors are not checked frequently enough
    //Possible solution - All autos must be 15 seconds even when real auto is 8


    // Get the yaw angle from the navX-MXP sensor
    double yaw = m_navx.GetYaw();
    //set to default
    m_autoControlPID.SetSetpoint(0);
     // Log data to SmartDashboard
    frc::SmartDashboard::PutNumber("Yaw", yaw);
    frc::SmartDashboard::PutNumber("Output", m_output);


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

  else if (selectedOption == "TEST"){
    //change
    if(m_timer.Get() < 0_s){
      m_robotDrive.TankDrive(-0.6,-0.6, false);
    }
    else if(m_timer.Get() < 1.8_s){
      m_robotDrive.TankDrive(-0.6,-0.6, false);
    }
    else if(m_timer.Get() < 2_s){
      m_robotDrive.TankDrive(0.8,0.8, false);
    }
    else if(m_timer.Get() < 15_s){
      //Auto Balance Logic
      if (yaw < -kToleranceDegrees) {
        m_robotDrive.ArcadeDrive(0.5, 0.0); // move forward
      } else if (yaw > kToleranceDegrees) {
          m_robotDrive.ArcadeDrive(-0.5, 0.0); // move backward
      } else {
        m_robotDrive.ArcadeDrive(0.0, 0.0); // stop robot
      }
    }
    else{
      m_robotDrive.TankDrive(0,0,false);
    }
  }
  else if(selectedOption == "NONE"){
    if(m_timer.Get() < 1_s){
      m_robotDrive.TankDrive(0,0);
    }
    else if(m_timer.Get() < 15_s){
      m_robotDrive.TankDrive(0,0);
    }
    else{
      m_robotDrive.TankDrive(0,0);
    }
  }
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