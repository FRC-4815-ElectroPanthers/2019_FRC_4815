/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */                           
/*------------------------------------------*/
#pragma once

#include <string>

#include <frc/WPILib.h>
#include <ctre/Phoenix.h>
#include <rev/CANSparkMax.h>

class Robot : public frc::TimedRobot
{
  public:
    void RobotInit() override;
    void RobotPeriodic() override;
    void AutonomousInit() override;
    void AutonomousPeriodic() override;
    void TeleopInit() override;
    void TeleopPeriodic() override;
    void TestPeriodic() override;

//Up goes right
//Down goes left
//Right goes straight
//Left goes backwards

//Up goes forward
//Down goes backward
//Right turns right
//Left turns left

//Right stick strafes left and right
  private:
    frc::SendableChooser<std::string> m_chooser;
    const std::string kAutoNameDefault = "Default";
    const std::string kAutoNameCustom = "My Auto";
    std::string m_autoSelected;
    
    frc::XboxController Controller{0};
    frc::XboxController ControllerA{1};

    rev::CANSparkMax talonTL{4, rev::CANSparkMaxLowLevel::MotorType::kBrushed};
    rev::CANSparkMax talonTR{2, rev::CANSparkMaxLowLevel::MotorType::kBrushed};
    rev::CANSparkMax talonBL{3, rev::CANSparkMaxLowLevel::MotorType::kBrushed};
    rev::CANSparkMax talonBR{1, rev::CANSparkMaxLowLevel::MotorType::kBrushed};
    frc::MecanumDrive mDrive{talonTL,talonBL,talonTR,talonBR};
    
    rev::CANSparkMax elevator{5, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
    rev::CANPIDController elevatorPID{elevator};
    rev::CANEncoder elevatorEncoder{elevator};
    //limitswitch declarations

    TalonSRX gripperR{6};
    TalonSRX gripperL{7};
    
    cs::UsbCamera camera1;
    cs::UsbCamera camera2;

    double prevElevatorPos;
    double kP = 0.1,
           kI = 1e-4, 
           kD = 1, 
           kIz = 0, 
           kFF = 0;
    double gripperBall = 0,
           gripperHatch = 0,
           gripperClose = 0;
    double elevatorLvl1 = 0,
           elevatorLvl2 = 0,
           elevatorLvl3 = 0,
           elevator0 = 0;

    double Deadband(double, double);
};
