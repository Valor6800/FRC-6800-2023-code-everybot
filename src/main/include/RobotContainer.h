#pragma once
#include "frc/XboxController.h"
#include <frc/TimedRobot.h>
#include <frc/RobotState.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/Timer.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/CANSparkMax.h>
#include "rev/CANSparkMax.h"
#include <frc/smartdashboard/SendableChooser.h>
#include <rev/MotorFeedbackSensor.h>
#include <frc/PowerDistribution.h>
#include <networktables/NetworkTableInstance.h>
#include <iostream>
#include <rev/CANEncoder.h>
#include <rev/SparkMaxRelativeEncoder.h>
#include <rev/RelativeEncoder.h>
#include <rev/CANDigitalInput.h>
#include <rev/AbsoluteEncoder.h>
#include <rev/CANPIDController.h>
#include <frc/SPI.h>
#include <frc/encoder.h>
#include <frc/DigitalInput.h>      // DigitalInput class
#include <frc/Solenoid.h>          // Solenoid class
#include <frc/DoubleSolenoid.h>    // DoubleSolenoid class
#include <frc/Compressor.h>        // Compressor class
#include <frc/SPI.h>               // SPI class
#include <frc/ADXRS450_Gyro.h>     // ADXRS450 gyro class
#include <frc/AnalogGyro.h>        // Analog gyro class
#include <frc2/command/PIDCommand.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc2/command/PIDSubsystem.h>
#include <frc2/command/CommandHelper.h>
#include <AHRS.h>
#include <frc/AnalogInput.h>
#include <frc/AnalogGyro.h>
#include <frc/AnalogPotentiometer.h>
#include <frc/AnalogTrigger.h>
#include <frc/AnalogTriggerOutput.h>
#include <frc/AnalogAccelerometer.h>
#include "rev/ControlType.h"
#include "rev/SparkMaxPIDController.h"
#include <frc/DigitalInput.h>
#include <frc/RobotBase.h>
#include <math.h>
#include <cmath>
#include <algorithm>


#include <frc2/command/Command.h>
#include <frc/DataLogManager.h>
#include <RobotContainer.h>
#include <wpi/DataLog.h>

using namespace rev;