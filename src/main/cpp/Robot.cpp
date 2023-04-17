//You can tune the P gain to achieve the desired response of the arm motion. 
//Increasing the P gain will make the arm move faster but can also cause overshoot and oscillations. 
//The I and D gains can be added to improve the response and stability of the arm motion. 
//You can experiment with different values to find the best performance for your robot.

#include "Robot.h"
#include "RobotContainer.h"
#include "PID.h"

  //auto options
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoOptions[] = { "NONE", "R_Left", "R_Mid", "R_Right", "IntakeBR", "IntakeGOBR", "TEST", "IntakeGO", "IntakeGOHARD"};

class Robot : public frc::TimedRobot { 

  //4 main drive motor cotnrollers
    rev::CANSparkMax m_leftMotor1{12, rev::CANSparkMax::MotorType::kBrushed}; //12
    rev::CANSparkMax m_leftMotor2{5, rev::CANSparkMax::MotorType::kBrushed}; //5
    rev::CANSparkMax m_rightMotor1{4, rev::CANSparkMax::MotorType::kBrushed}; //4
    rev::CANSparkMax m_rightMotor2{4, rev::CANSparkMax::MotorType::kBrushed}; //1
    //Intake + arm controllers
    rev::CANSparkMax m_intake{3, rev::CANSparkMax::MotorType::kBrushless}; //3
    rev::CANSparkMax m_arm{8, rev::CANSparkMax::MotorType::kBrushless}; //8
    

    rev::SparkMaxRelativeEncoder m_encoder = m_arm.GetEncoder();
    //combine left + right motors
    frc::MotorControllerGroup m_leftMotors{m_leftMotor1, m_leftMotor2};
    frc::MotorControllerGroup m_rightMotors{m_rightMotor1, m_rightMotor2};
    //Combine both motor groups
    frc::DifferentialDrive m_robotDrive{m_leftMotors, m_rightMotors};
    //Controller for driver
    frc::XboxController controller{0}; 
    //Operator controller
    frc::XboxController controllerOP{1};
    //Timer for Auto
    frc::Timer m_timer;
    //NavX sensor with the function to set voltage
    AHRS m_navx{frc::SPI::Port::kMXP};


    void setArmMotorVoltageLimit(double voltageLimit)
    {
      voltageLimit = fmax(0.0, fmin(voltageLimit, 12.0));
      m_arm.EnableVoltageCompensation(voltageLimit);
    }

    void setIntakeMotorVoltageLimit(double voltageLimit)
    {
      voltageLimit = fmax(0.0, fmin(voltageLimit, 12.0));
      m_intake.EnableVoltageCompensation(voltageLimit);
    }
    
 public:
  void RobotInit() override {
    //reset the position
    m_encoder.SetPosition(0.0);  
    // Set the controller to be continuous (because it is an angle controller)
    m_navx.Calibrate();
    double m_angle = m_navx.GetAngle();
    
    // Set up the PID controller
    m_autoControlPID.SetP(kP);
    m_autoControlPID.SetI(kI);
    m_autoControlPID.SetD(kD);
    m_autoControlPID.SetTolerance(kToleranceDegrees);

    //right motor must be inverted for it to go forward
    m_rightMotors.SetInverted(true);
    m_arm.SetInverted(true);

    //reset all motors before start
    m_leftMotor1.RestoreFactoryDefaults();
    m_leftMotor2.RestoreFactoryDefaults();
    m_rightMotor1.RestoreFactoryDefaults();
    m_rightMotor2.RestoreFactoryDefaults();
    m_arm.RestoreFactoryDefaults();
    m_intake.RestoreFactoryDefaults();
    
    //list of Auto Options
    m_chooser.SetDefaultOption("NONE", kAutoOptions[0]);
    m_chooser.AddOption("R_Left", kAutoOptions[1]);
    m_chooser.AddOption("R_Mid", kAutoOptions[2]);
    m_chooser.AddOption("R_Right", kAutoOptions[3]);
    m_chooser.AddOption("IntakeBR", kAutoOptions[4]);
    m_chooser.AddOption("IntakeGOBR", kAutoOptions[5]);
    m_chooser.AddOption("TEST", kAutoOptions[6]);
    m_chooser.AddOption("IntakeGO", kAutoOptions[7]);
    m_chooser.AddOption("IntakeGOHARD", kAutoOptions[8]);


    frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
    frc::SmartDashboard::PutNumber("Arm Voltage", 0);
    //intake + arm speed
    frc::SmartDashboard::PutNumber("Intake Speed", 1);
    frc::SmartDashboard::PutNumber("Arm Speed", 0.9);
    //used to count NEO motor rotations to fully open an arm
    frc::SmartDashboard::PutNumber("Arm Rotation", 0);
    //initial delay is 0 sec
    frc::SmartDashboard::PutNumber("Delay (Sec)", 0);
    //limintation of speed and rotation sensitivity
    frc::SmartDashboard::PutNumber("Rotation Sesitivity", 0.75);
    frc::SmartDashboard::PutNumber("Speed Sesitivity", 0.85);
    //boolean to determine if we picking up a code - initially false
    frc::SmartDashboard::PutBoolean("Cone", false);
    //motor limit - simmilar to the sensitivity level
    frc::SmartDashboard::PutNumber("Left Motor Limit", 1); //initially 65%
    frc::SmartDashboard::PutNumber("Right Motor Limit", 1);
    //get the horizontal angle of the robot (must be 0 at start)
    frc::SmartDashboard::PutNumber("Hor Angle", m_angle);

    //limit motors to a specific voltage
    m_leftMotor1.EnableVoltageCompensation(12.3);
    m_leftMotor2.EnableVoltageCompensation(12.3);
    m_rightMotor1.EnableVoltageCompensation(12.3);
    m_rightMotor2.EnableVoltageCompensation(12.3);
    m_arm.SetSmartCurrentLimit(12);
    m_arm.EnableVoltageCompensation(12.3);
    m_intake.EnableVoltageCompensation(12.3);

    setArmMotorVoltageLimit(12); //change input voltage for the arm to 12
    setIntakeMotorVoltageLimit(12); //change voltage input for the intake to 12

    //check if those are working
    m_leftMotor1.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_rightMotor1.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_rightMotor2.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_leftMotor2.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_arm.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast); //change to coast mode
  }

  void TeleopPeriodic() override {
    //callibrate the sensor before the game
    m_navx.Calibrate();
    //limtations on speed and rotation
    double m_rot = frc::SmartDashboard::GetNumber("Rotation Sesitivity", 0.75);
    double Totsens = frc::SmartDashboard::GetNumber("Speed Sesitivity", 0.85);
    //numbers to limit both motor's capacity
    double sensLeft = frc::SmartDashboard::GetNumber("Left Motor Limit", 1); //65%? 0.65
    double sensRight = frc::SmartDashboard::GetNumber("Right Motor Limit", 1);
    //initial speed of the intake will be 80% + arm 100%
    double intakeSpeed = frc::SmartDashboard::GetNumber("Intake Speed", 1);
    double armSpeed = frc::SmartDashboard::GetNumber("Arm Speed", 0.9);


    //--------------------------------------------------------DRIVE CODE HERE:--------------------------------------------------------------
    //- is forward, and + is back
    
    // Get the yaw angle from the navX-MXP sensor
    double pitch = m_navx.GetRoll();
    //set to default
    m_autoControlPID.SetSetpoint(0);


    //track the numbers
    frc::SmartDashboard::PutNumber("Yaw2", pitch);
    frc::SmartDashboard::PutNumber("Output2", m_output);


    //motor speed and rotation variables from controller for ArcadeDrive
    double speed = controller.GetLeftY(); 
    double rotation = controller.GetRightX();
    //sqrt the results for the smoothest speed
    double SQRTspeed = sqrt(speed); 
    double SQRTrotation = sqrt(rotation);

    //main drive function 
    m_robotDrive.ArcadeDrive(speed * Totsens, rotation * m_rot);

    //---------------------------------------------------------INTAKE CODE----------------------------------------------------------------
    //controller - driver
    //controllerOP - operator

    //driver switches the cone mode as operator dont have anymore buttons to work with
    if (controller.GetYButtonReleased())      
    {
      coneInt = !coneInt;
    }
    // Get the number of rotations of the motor
    double m_armRot = m_encoder.GetPosition() / 42.0;
    if(controllerOP.GetRightStickButtonReleased() > 0)
    {
      m_encoder.SetPosition(0);
    }


    frc::SmartDashboard::PutBoolean("Cone", coneInt);
    frc::SmartDashboard::PutNumber("Arm Rotation", m_armRot);
    

    if(controllerOP.GetLeftTriggerAxis() != 0)
    {
      if (coneInt)
      {
      m_intake.Set(-intakeSpeed); //may need to manually change the values
      }
      else
      {
      m_intake.Set(intakeSpeed * 0.55); //only 45% are availiable
      }
      
    }

    else if (controllerOP.GetRightTriggerAxis() != 0) 
    {
      if (coneInt)
      {
          m_intake.Set(intakeSpeed);
        }
        else
        {
          m_intake.Set(-intakeSpeed * 0.55);
      }
     }
     else
     {
       m_intake.Set(0.0);
     }



    //-------------------------------------ARM CONTROLLER CODE--------------------------------------------

    //this logic is moving the arm into max arm position + automatic hold
    if(controllerOP.GetYButtonPressed())
    {
      m_arm.Set(-armSpeed);
    }
    //This logic is moving the arm into lowest point of the arm + automatic hold
    if (controllerOP.GetAButtonPressed())
    {
      m_arm.Set(armSpeed * 0.5);
    }
    //Reset the NEO motor rotation number
    if(controllerOP.GetLeftStickButtonReleased())
    {
      m_encoder.SetPosition(0);
    }


    // Set up a boolean variable to keep track of whether the motor has reached the setpoint
    bool atSetpoint = false;
    if(controllerOP.GetXButton())
    {
      // If the motor has reached the setpoint, stop it
      if (atSetpoint) {
        m_arm.Set(0.0);
      }

      // If the position is greater than the setpoint, move the motor backwards
      else if (m_armRot > -0.40)
      {
        m_arm.Set(-0.3); 
      }

      // If the position is less than the setpoint, move the motor forwards
      else if (m_armRot < -0.50)
      {
        m_arm.Set(0.3); 
      }

      // If the position is exactly at the setpoint, stop the motor and set atSetpoint to true
      else
      {
        m_arm.Set(0.0);
        atSetpoint = true;
      }
    }   

    //right and left bumpers will be the manual control of the arm
    if(controllerOP.GetRightBumper() != 0)
    {
      m_arm.Set(-armSpeed);
    }
    //right and left bumpers will be the manual control of the arm
    else if (controllerOP.GetLeftBumper() != 0)
    {
      m_arm.Set(armSpeed);
    }
    else if (controllerOP.GetLeftBumperReleased() || controllerOP.GetRightBumperReleased())
    {
      m_arm.Set(0);
    }

    //Current hold button Not working yet
    if(controllerOP.GetBButtonPressed())
    {
      double holdnum = m_armRot;
      if(m_armRot > holdnum)
      {
        m_arm.Set(-armSpeed * 0.3);
      }
      else if(m_armRot < holdnum)
      {
        m_arm.Set(armSpeed * 0.3);
      }
      else
      {
        m_arm.Set(0);
      }
    }
  }
  //*************************************************AUTONOMUS CODE*************************************************************
  void AutonomousInit() override {
    //check if those are working
    m_leftMotor1.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_rightMotor1.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_rightMotor2.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_leftMotor2.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_autoControlPID.Reset();
    m_navx.Reset();
    m_navx.ZeroYaw();
    //timer for auto
    m_timer.Reset();
    m_timer.Start();
  }

  void AutonomousPeriodic() override {
    //All autos must be 15 seconds even when real auto is 8
    double yaw = m_navx.GetRoll();
    //inital speed of the intake is 40% for the sake of testing
    double recangle = frc::SmartDashboard::GetNumber("Hor Angle", 0);
    //initial values for the auto
    int x = frc::SmartDashboard::GetNumber("Delay (Sec)", 0);
    double intakeSpeed = frc::SmartDashboard::GetNumber("Intake Speed", 0.4);
    //set to default
    m_autoControlPID.SetSetpoint(0);
    //get the selected auto option
    std::string selectedOption = m_chooser.GetSelected();


    frc::SmartDashboard::PutNumber("Yaw", yaw);
    frc::SmartDashboard::PutNumber("Output", m_output);    
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

  else if (selectedOption == "TEST"){ //use with 12.5 voltage
     coneInt = true;
    //ROBOT MUST FACE THE DRIVER
    if(m_timer.Get() < secondsX){ // PERFECT VOLTAGE - 12.3 - 12.5
      m_robotDrive.TankDrive(0, 0, false);
    }
    else if(m_timer.Get() < 2_s + secondsX){
      m_robotDrive.TankDrive(0, 0, false);
      m_arm.Set(-0.3);
    }
    else if(m_timer.Get() < 3_s + secondsX){
      m_intake.Set(-intakeSpeed); //change to - when cube
    }
    else if(m_timer.Get() < 4_s + secondsX){
      m_arm.Set(0.3);
      m_intake.Set(0); //cone should go out
    }
    else if(m_timer.Get() < 4.5_s + secondsX){
      m_robotDrive.TankDrive(0, 0, false);
    }
    else if(m_timer.Get() < 6.5_s + secondsX){
      m_robotDrive.TankDrive(0.5, 0.5, false); //should go backwards
      m_arm.Set(0.3);
    }
     else if(m_timer.Get() < 9_s + secondsX){
       if(recangle > -70){
       m_robotDrive.TankDrive(-0.45, 0.45, false);
       }
       else{
         m_robotDrive.TankDrive(0, 0, false);
       }
     }
    else if(m_timer.Get() < 15_s + secondsX){ 
      m_robotDrive.TankDrive(0, 0, false);
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
  else if(selectedOption == "R_Left"){
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
      m_robotDrive.TankDrive(-0.65, 0.65, false); 
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
    else if(m_timer.Get() < 5_s + secondsX){
      m_robotDrive.TankDrive(-0.4, -0.4, false); //should go backwards
    }
    else if(m_timer.Get() < 15_s){
      if (yaw < -7.6) {
        m_robotDrive.TankDrive(-0.3,-0.3, false);
      } else if (yaw > -7.6) {
         m_robotDrive.TankDrive(0,0, false);
      }
      else{
        m_robotDrive.TankDrive(0,0, false);
      }
    }
    else{
      m_robotDrive.TankDrive(0, 0, false);
    }
  }

  else if(selectedOption == "IntakeBR"){
    coneInt = true;
    //ROBOT MUST FACE THE DRIVER
    if(m_timer.Get() < secondsX){ // PERFECT VOLTAGE - 12.3 - 12.5
      m_robotDrive.TankDrive(0, 0, false);
    }
    else if(m_timer.Get() < 2_s + secondsX){
      m_robotDrive.TankDrive(0, 0, false);
      m_arm.Set(-0.3);
    }
    else if(m_timer.Get() < 3_s + secondsX){
      m_intake.Set(-intakeSpeed); //change to - when cube
    }
    else if(m_timer.Get() < 4_s + secondsX){
      m_arm.Set(0.3);
      m_intake.Set(0); //cone should go out
    }
    else if(m_timer.Get() < 7_s + secondsX){
      m_robotDrive.TankDrive(0.4, 0.4, false); //should go backwards
      m_arm.Set(0.3);
    }
    else if(m_timer.Get() < 15_s){
      if (yaw < -7.6) {
        m_robotDrive.TankDrive(0.3,0.3, false);
      } else if (yaw > -7.6) {
         m_robotDrive.TankDrive(0,0, false);
      }
      else{
        m_robotDrive.TankDrive(0,0, false);
      }
    }
  }

  else if(selectedOption == "IntakeGOBR"){
    coneInt = true;
    //ROBOT MUST FACE THE DRIVER
    if(m_timer.Get() < secondsX){ // PERFECT VOLTAGE - 12.3 - 12.5
      m_robotDrive.TankDrive(0, 0, false);
    }
    else if(m_timer.Get() < 2_s + secondsX){
      m_robotDrive.TankDrive(0, 0, false);
      m_arm.Set(-0.4);
    }
    else if(m_timer.Get() < 3_s + secondsX){
      m_intake.Set(intakeSpeed); //cone should go out
    }
    else if(m_timer.Get() < 4_s + secondsX){
      m_arm.Set(0.4);
      m_intake.Set(0); //cone should go out
    }
    else if(m_timer.Get() < 7_s + secondsX){
      m_robotDrive.TankDrive(0.45, 0.45, false); //should go backwards
    }
    else if (m_timer.Get() < 15_s + secondsX){
       //Auto Balance Logic
      if (yaw < -kToleranceDegrees) {
        m_robotDrive.TankDrive(-0.3, -0.3, false); 
      } else if (yaw > kToleranceDegrees) {
          m_robotDrive.TankDrive(0.35, 0.35, false);
      } else {
        m_robotDrive.TankDrive(0, 0, false); 
      }
    }
    else{
      m_robotDrive.TankDrive(0, 0, false); 
    }
  }
  else if(selectedOption == "IntakeGO"){
    coneInt = true;
    //ROBOT MUST FACE THE DRIVER
    if(m_timer.Get() < secondsX){ // PERFECT VOLTAGE - 12.3 - 12.5
      m_robotDrive.TankDrive(0, 0, false);
    }
    else if(m_timer.Get() < 2_s + secondsX){
      m_robotDrive.TankDrive(0, 0, false);
      m_arm.Set(-0.4);
    }
    else if(m_timer.Get() < 3_s + secondsX){
      m_intake.Set(intakeSpeed); //cone should go out
    }
    else if(m_timer.Get() < 4_s + secondsX){
      m_arm.Set(0.4);
      m_intake.Set(0); //cone should go out
    }
    else if(m_timer.Get() < 5.5_s + secondsX){
      m_robotDrive.TankDrive(0.7, 0.7); //should go backwards
    }
    else if(m_timer.Get() < 15_s + secondsX){
      m_robotDrive.TankDrive(0, 0);
    }
    else{
      m_robotDrive.TankDrive(0, 0);
    }
  }
  else if(selectedOption == "IntakeGOHARD"){
    coneInt = true;
    //ROBOT MUST FACE THE DRIVER
    if(m_timer.Get() < secondsX)
    { // PERFECT VOLTAGE - 12.3 - 12.5
      m_robotDrive.TankDrive(0, 0, false);
    }
    else if(m_timer.Get() < 2_s + secondsX)
    {
      m_robotDrive.TankDrive(0, 0, false);
      m_arm.Set(-0.4);
    }
    else if(m_timer.Get() < 3_s + secondsX)
    {
      m_intake.Set(intakeSpeed); //cone should go out
    }
    else if(m_timer.Get() < 4_s + secondsX)
    {
      m_arm.Set(0.4);
      m_intake.Set(0); //cone should go out
    }
    else if(m_timer.Get() < 6_s + secondsX)
    {
      m_robotDrive.TankDrive(0.7, 0.7); //should go backwards
    }
    else if(m_timer.Get() < 15_s + secondsX)
    {
      m_robotDrive.TankDrive(0, 0);
    }
    else
    {
      m_robotDrive.TankDrive(0, 0);
    }
  }
}
};

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif