
#include "AHRS.h"
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <chrono>
#include <iostream>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Joystick.h> // Joystick Library
#include <frc/motorcontrol/Talon.h> //Talon Library
#include <frc/motorcontrol/MotorControllerGroup.h> //Motor Library
#include <frc/drive/DifferentialDrive.h> // Drive Library
#include <frc/Servo.h>
#include <frc/Compressor.h>
#include <frc/Solenoid.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Timer.h>
#include <frc/Encoder.h>
#include <frc/DriverStation.h>
#include "frc/smartdashboard/Smartdashboard.h"
#include "ctre/Phoenix.h" //Falcon Library
#include "networktables/NetworkTable.h" //Network Tables Library
#include "networktables/NetworkTableEntry.h" //Network Tables Entry Library
#include "networktables/NetworkTableValue.h" //Network Tables Value Library
#include "cameraserver/CameraServer.h" //Camera Server Library
#include "networktables/NetworkTableInstance.h" //Network Tables Instance Library
#include "wpi/span.h"
#include "cscore_oo.h"

using namespace std;
using namespace cv;
using namespace frc;

    WPI_TalonSRX motorFOUR = {1};      //Talon 4 -> ID 01 Talon
    WPI_TalonSRX motorTHREE = {0};       //Talon 3 -> ID 00 Talon
    WPI_VictorSPX motorTWO = {0};      //Talon 2 -> ID 00 Victor
    WPI_VictorSPX motorONE = {1};     //Talon 1 -> ID 01 Victor

    frc::MotorControllerGroup m_leftGroup{motorTHREE, motorFOUR};     //Grupo de motores -> Izquierdo
    frc::MotorControllerGroup m_rightGroup{motorTWO, motorONE};    //Grupo de motores -> Derecho

    frc::DifferentialDrive m_drive{m_leftGroup, m_rightGroup};  //Conduccion de tipo Arcade


    AHRS Gyro{SPI::Port::kMXP};           // Giroscopio

    frc::Timer Tempo;                     // Timer

Tempo.start();

if (Tempo.Get() <= 4_s) { m_drive.TankDrive(-0.4,0.4,false); }
else 
{ 
m_drive.TankDrive(0,0,false); 
gyro.Reset();
Tempo.Reset();
}

if (Gyro.GetAngle() <=  90 ) { m_drive.TankDrive(0.5,0.5); }
else 
{ 
m_drive.TankDrive(0,0,false); 
gyro.Reset();
Tempo.Reset();
}

if (Tempo.Get() <= 4_s) { m_drive.TankDrive(-0.4,0.4,false); }
else 
{ 
m_drive.TankDrive(0,0,false); 
gyro.Reset();
Tempo.Reset();
}

if (Gyro.GetAngle() <=  -90 ) { m_drive.TankDrive(0.5,0.5); }
else 
{ 
m_drive.TankDrive(0,0,false); 
gyro.Reset();
Tempo.Reset();
}

if (Tempo.Get() <= 4_s) { m_drive.TankDrive(-0.4,0.4,false); }
else 
{ 
m_drive.TankDrive(0,0,false); 
gyro.Reset();
Tempo.Reset();
}


