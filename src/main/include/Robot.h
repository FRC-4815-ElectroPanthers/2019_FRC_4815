/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */                           
/*------------------------------------------*/
#pragma once

#include <string>

// #include <frc/TimedRobot.h>
// #include <frc/smartdashboard/SendableChooser.h>
// #include <frc/Spark.h>
// #include <frc/XboxController.h>
// //#include <frc/>
// #include <frc/drive/MecanumDrive.h>
// #include <frc/VictorSP.h>
#include <frc/WPILib.h>
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
    frc::VictorSP VacuuMotor{5};
    frc::VictorSP VacuuMotorPivot{6};
};
