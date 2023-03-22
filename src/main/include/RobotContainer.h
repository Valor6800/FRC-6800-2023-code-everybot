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
#include "rev/ControlType.h"
#include "rev/SparkMaxPIDController.h"
#include <frc/DigitalInput.h>


#include <frc2/command/Command.h>
#include <frc/DataLogManager.h>
#include <RobotContainer.h>
#include <wpi/DataLog.h>

using namespace rev;