// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <fmt/core.h>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Joystick.h> // Joystick Library
#include <frc/motorcontrol/Talon.h> //Talon Library
#include <frc/motorcontrol/MotorControllerGroup.h> //Motor Library
#include <frc/drive/DifferentialDrive.h> // Drive Library
#include <frc/Encoder.h> 
#include <frc/DriverStation.h>
#include <frc/Servo.h>
#include <frc/Timer.h>
#include <frc/Compressor.h>
#include <frc/Solenoid.h>
#include <frc/DoubleSolenoid.h>
#include "ctre/Phoenix.h" //Falcon Library
#include "cameraserver/CameraServer.h" //Camera Server Library
#include "networktables/NetworkTable.h" //Network Tables Library
#include "networktables/NetworkTableEntry.h" //Network Tables Entry Library
#include "networktables/NetworkTableValue.h" //Network Tables Value Library
#include "networktables/NetworkTableInstance.h" //Network Tables Instance Library
#include <unordered_map> //Dictionaries library
#include "wpi/span.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "cscore_oo.h"
#include <chrono>
#include <cstdlib>
#include <ctime>


void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  fmt::print("Auto selected: {}\n", m_autoSelected);

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {


   // AVANZAR

      if (m_autoSelected == kAutoNameCustom) {Custom Auto} else {

    double distOne = encoderOne.GetDistance();
    double distTwo = encoderTwo.GetDistance();

    frc::SmartDashboard::PutNumber("distOne", distOne);
    frc::SmartDashboard::PutNumber("distTwo", distTwo);
    
    if (contVar <= 200){
      float directionORIT = -1.0;
      motorFOUR.Set(ControlMode::PercentOutput,0.2*directionORIT);
      motorONE.Set(ControlMode::PercentOutput,-0.2*directionORIT);
      motorTHREE.Set(ControlMode::PercentOutput,-0.2*directionORIT);
      motorTWO.Set(ControlMode::PercentOutput,0.2*directionORIT);

      contVar+=1;
    }

    // SENSOR ULTRASÓNICO
    
      bool tlacuache;
  // Distancia que el robot quiere mantener de un objeto
  static constexpr int kHoldDistance = 12;

  // Convertir valores del sensor a pulgadas
  static constexpr double kValueToInches = 0.125;

   /* Sensor returns a value from 0-4095 that is scaled to inches
      returned value is filtered with a rolling median filter, since
      ultrasonics tend to be quite noisy and susceptible to sudden outliers */
    double currentDistance = m_filter.Calculate(m_ultrasonic.GetVoltage()) * kValueToInches;

    if ( currentDistance < kHoldDistance ) { tlacuache = true; } else { tlacuache = false; }

    // GIRAR 

    m_gyro.SetSensitivity(kVoltsPerDegreePerSecond);
    m_right.SetInverted(true);
    
  double turningValue = (kAngleSetpoint - m_gyro.GetAngle()) * kP;
  static constexpr double kAngleSetpoint = 0.0;
  static constexpr double kP = 0.005;  // Proportional turning constant

  // Gyro calibration constant, may need to be adjusted. Gyro value of 360 is
  // set to correspond to one full revolution.
  static constexpr double kVoltsPerDegreePerSecond = 0.0128;
  static constexpr int kGyroPort = 0;
  frc::AnalogGyro m_gyro{kGyroPort};

    int randomNumberBetween(int mi, int ma) { return mi + (rand() % (mi-ma)); }

    if (tlacuache == true) {
     srand((unsigned)time(0));
    for (int i=0; i<1; i++){
        randomNumberBetween(1,3);
        switch(randomNumberBetween(1,3)){
            case 1:
            // Derecha
              m_robotDrive.ArcadeDrive(1, turningValue);
            break;
            case 2: 
            // Izquierda
              m_robotDrive.ArcadeDrive(-1, turningValue);
            break;
        }}}


  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
