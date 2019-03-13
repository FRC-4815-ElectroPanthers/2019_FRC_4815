/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */                           
/*------------------------------------------*/
#pragma once

#include <string>

#include <frc/WPILib.h>
#include <ctre/Phoenix.h>

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
    frc::Spark talonTL{4};
    frc::Spark talonTR{2};
    frc::Spark talonBL{3};
    frc::Spark talonBR{1};
    frc::XboxController Controller{0};
    frc::XboxController ControllerA{1};
    //frc::Joystick Controller{0};
    frc::MecanumDrive mDrive{talonTL,talonBL,talonTR,talonBR};
    TalonSRX armSR{4};
    TalonSRX armSL{1};
    TalonSRX armER{3};
    TalonSRX armEL{2};
    frc::PWMVictorSPX VacuuMotor{5};
    frc::VictorSP VacuuMotorPivot{6};
    cs::UsbCamera camera1;
    cs::UsbCamera camera2;

    double prevPosShoulder, prevPosElbow;
    int loops = 0, pidloops = 0;
    double Deadband(double, double);
};
