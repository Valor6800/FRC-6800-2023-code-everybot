#pragma once
#include "RobotContainer.h"

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



// Output value for the PID controller
double m_output = 0.0;
bool coneInt = true;

// Create a PowerDistributionPanel object
frc::PowerDistribution pdp;