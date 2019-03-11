/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  mDrive.SetRightSideInverted(false);

  armSR.ConfigFactoryDefault();
  armSL.ConfigFactoryDefault();
  armER.ConfigFactoryDefault();
  armEL.ConfigFactoryDefault();

  int shoulderAbsPos = armSR.GetSelectedSensorPosition() & 0xFFF;
  int elbowAbsPos = armER.GetSelectedSensorPosition() & 0xFFF;

  armSR.SetSelectedSensorPosition(shoulderAbsPos);
  armER.SetSelectedSensorPosition(elbowAbsPos);

  armSR.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative);
  armER.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative);

  armSR.SetSensorPhase(false);
  armER.SetSensorPhase(false);

  armSR.ConfigNominalOutputForward(0);
	armSR.ConfigNominalOutputReverse(0);
  armSR.ConfigPeakOutputForward(1);
	armSR.ConfigPeakOutputReverse(-1);

  armER.ConfigNominalOutputForward(0);
	armER.ConfigNominalOutputReverse(0);
  armER.ConfigPeakOutputForward(1);
	armER.ConfigPeakOutputReverse(-1);

  armSR.Config_kF(0, 0.0);
	armSR.Config_kP(0, 0.1);
	armSR.Config_kI(0, 0.0);
	armSR.Config_kD(0, 0.0);

  armER.Config_kF(0, 0.0);
	armER.Config_kP(0, 0.1);
	armER.Config_kI(0, 0.0);
	armER.Config_kD(0, 0.0);

  armSL.Follow(armSR);
  armEL.Follow(armER);

  armSR.SetInverted(false);
  armSL.SetInverted(InvertType::OpposeMaster);
  armER.SetInverted(false);
  armEL.SetInverted(InvertType::OpposeMaster);
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
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  mDrive.DriveCartesian(
    Controller.GetX(frc::GenericHID::kRightHand),
    -1*Controller.GetY(frc::GenericHID::kLeftHand),
    Controller.GetX(frc::GenericHID::kLeftHand)
  );
  if (ControllerA.GetBumper(frc::GenericHID::kRightHand)){
    VacuuMotor.Set(1);
  }else {
    VacuuMotor.Set(0);
  }
  //VacuuMotorPivot.Set(ControllerA.GetX(frc::GenericHID::kLeftHand));

  armSR.Set(ControlMode::PercentOutput, ControllerA.GetY(frc::GenericHID::kRightHand));
  //armSL.Set(ControlMode::PercentOutput, -1*ControllerA.GetY(frc::GenericHID::kRightHand));

  armER.Set(ControlMode::PercentOutput, ControllerA.GetY(frc::GenericHID::kLeftHand));
  //armEL.Set(ControlMode::PercentOutput, -1*ControllerA.GetY(frc::GenericHID::kLeftHand));
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
